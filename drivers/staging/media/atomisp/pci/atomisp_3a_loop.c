// SPDX-License-Identifier: GPL-2.0
/*
 * In-kernel auto white balance and auto exposure for atomisp.
 *
 * The atomisp firmware performs no 3A algorithm of its own and the legacy
 * userspace clients that did are gone; simple camera apps (Cheese,
 * GStreamer) therefore get a fixed-gain pipeline, which on the OV sensors
 * of this platform shows as a strong color cast.
 *
 * This implements a measurement-driven 3A loop on top of the existing
 * driver plumbing:
 *  - AWB: per-frame gray-world gains computed from the s3a AWB channel
 *    sums and pushed through the wb_config ISP parameter (the same path
 *    as ATOMISP_IOC_S_ISP_WHITE_BALANCE).
 *  - AE: paced proportional control of the sensor's exposure and analogue
 *    gain controls towards a target luma measured from the 256-bin Y
 *    histogram of the s3a statistics.
 *
 * atomisp_3a_loop_frame_done() is called from atomisp_buf_done() with
 * isp->mutex held. It consumes one s3a buffer per frame exactly like the
 * ATOMISP_IOC_G_3A_STAT ioctl does and recycles it to the free list, so
 * no buffers are starved from the firmware.
 *
 * Note: ATOMISP_IOC_G_3A_STAT itself runs without isp->mutex (pre-existing
 * staging driver behaviour); concurrent use of that ioctl while this loop
 * is active has the same benign race as two concurrent ioctl callers.
 */

#include <linux/kernel.h>
#include <linux/list.h>

#include <media/v4l2-ctrls.h>

#include <linux/list.h>
#include <linux/math.h>
#include <linux/string.h>

#include "atomisp_cmd.h"
#include "atomisp_internal.h"
#include "ia_css_3a.h"

/*
 * WB gains are N.3 fixed point, 8192 == 1.0x (the atomisp_wb_config
 * convention used by the firmware).
 */
#define AWB_UNIT		8192
#define AWB_MIN_GAIN		(AWB_UNIT / 2)	/* 0.5x */
#define AWB_MAX_GAIN		(AWB_UNIT * 4)	/* 4.0x */
#define AWB_APPLY_THRESHOLD	64	/* ~0.8% total drift before re-applying */

/* AE: 8-bit luma target with a clamped proportional controller. */
#define AE_TARGET_LUMA		110
#define AE_MIN_FACTOR		179	/* at most 0.7x darker per step */
#define AE_MAX_FACTOR		366	/* at most 1.43x brighter per step */
#define AE_DEADBAND		13	/* ~5% luma error: skip adjustment */
#define AE_PACE_FRAMES		8	/* adjust at most every N frames */
#define AE_OUTPUT_OFFSET	51	/* BH hist domain vs YUYV output luma */

static void atomisp_3a_awb(struct atomisp_sub_device *asd, u64 gr, u64 r,
			   u64 b, u64 gb)
{
	struct atomisp_wb_config wb;
	u32 new_r, new_b;
	u64 green;

	if (!r || !b || !gr || !gb)
		return;
	green = (gr + gb) / 2;
	new_r = clamp_t(u64, green * AWB_UNIT / r, AWB_MIN_GAIN, AWB_MAX_GAIN);
	new_b = clamp_t(u64, green * AWB_UNIT / b, AWB_MIN_GAIN, AWB_MAX_GAIN);

	/* Hysteresis: only push an ISP parameter update on meaningful drift. */
	if (abs((int)new_r - (int)asd->awb_r) +
	    abs((int)new_b - (int)asd->awb_b) < AWB_APPLY_THRESHOLD)
		return;

	memset(&wb, 0, sizeof(wb));
	wb.integer_bits = 3;
	wb.gr = AWB_UNIT;
	wb.gb = AWB_UNIT;
	wb.r = new_r;
	wb.b = new_b;

	memcpy(&asd->params.css_param.wb_config, &wb, sizeof(wb));
	asd->params.config.wb_config = &asd->params.css_param.wb_config;
	asd->params.css_param.update_flag.wb_config =
		(struct atomisp_wb_config *)&asd->params.css_param.wb_config;
	asd->params.css_update_params_needed = true;

	asd->awb_r = new_r;
	asd->awb_b = new_b;

	dev_dbg(asd->isp->dev, "3A: AWB applied r=%u b=%u\n", new_r, new_b);
}

static void atomisp_3a_ae(struct atomisp_sub_device *asd,
			  struct ia_css_3a_statistics *stat)
{
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *exp_c, *gain_c;
	struct v4l2_control ctrl;
	struct ia_css_3a_rgby_output *hist;
	u64 luma_sum = 0, count = 0;
	unsigned int mean, factor, i;
	s32 exp, gain, new_exp, new_gain;

	if (!asd->ae_en)
		return;
	if (++asd->ae_pace < AE_PACE_FRAMES)
		return;
	asd->ae_pace = 0;

	/*
	 * ae_y in the per-cell data is only filled by the S3A1 kernel and
	 * is always zero with the S3A2 firmware used on this platform, so
	 * measure mean luma from the 256-bin Y histogram filled by
	 * ia_css_s3a_hmem_decode() instead.
	 */
	hist = stat->rgby_data;
	if (!hist)
		return;
	for (i = 1; i < 256; i++) {
		count += hist[i].y;
		luma_sum += (u64)i * hist[i].y;
	}
	/*
	 * Bin 0 holds zero-luma pixels and is corrected by
	 * ia_css_s3a_hmem_decode() by subtracting the pixels outside the
	 * 3A area; that subtraction can underflow (unsigned wrap) when
	 * the histogrammed area exceeds the grid. Only trust it when sane.
	 */
	if (hist[0].y <= 0x7fffffff)
		count += hist[0].y;
	if (!count)
		return;
	mean = luma_sum / count;
	if (!mean)
		return;

	/*
	 * The BH histogram measures a pre-processing domain that sits a
	 * fixed additive offset below the YUYV output luma (measured on
	 * OV8858: output = hist + 51 across the 90..154 range).
	 */
	mean += AE_OUTPUT_OFFSET;

	factor = clamp_t(u32, AE_TARGET_LUMA * 256 / mean,
			 AE_MIN_FACTOR, AE_MAX_FACTOR);
	if (abs((int)factor - 256) < AE_DEADBAND)
		return;

	if (!asd->isp->inputs[asd->input_curr].sensor)
		return;
	handler = asd->isp->inputs[asd->input_curr].sensor->ctrl_handler;
	exp_c = v4l2_ctrl_find(handler, V4L2_CID_EXPOSURE);
	gain_c = v4l2_ctrl_find(handler, V4L2_CID_ANALOGUE_GAIN);
	if (!exp_c || !gain_c)
		return;

	exp = v4l2_ctrl_g_ctrl(exp_c);
	gain = v4l2_ctrl_g_ctrl(gain_c);
	if (exp <= 0 || gain <= 0)
		return;

	new_exp = clamp_t(s64, (s64)exp * factor / 256,
			  exp_c->minimum, exp_c->maximum);
	new_gain = clamp_t(s64, (s64)gain * factor / 256,
			   gain_c->minimum, gain_c->maximum);
	if (new_exp == exp && new_gain == gain)
		return;

	ctrl.id = V4L2_CID_EXPOSURE;
	ctrl.value = new_exp;
	if (v4l2_s_ctrl(NULL, handler, &ctrl))
		return;

	ctrl.id = V4L2_CID_ANALOGUE_GAIN;
	ctrl.value = new_gain;
	v4l2_s_ctrl(NULL, handler, &ctrl);

	dev_dbg(asd->isp->dev, "3A: AE applied exp=%d gain=%d (y=%u)\n",
		new_exp, new_gain, mean);
}

void atomisp_3a_loop_frame_done(struct atomisp_sub_device *asd)
{
	struct atomisp_s3a_buf *s3a_buf;
	struct ia_css_3a_statistics *stat = asd->params.s3a_user_stat;
	struct ia_css_3a_output *cells;
	unsigned int count, i;
	u64 r = 0, gr = 0, b = 0, gb = 0;

	if ((!asd->awb_en && !asd->ae_en) ||
	    asd->params.s3a_output_bytes == 0 || !stat || !stat->data)
		return;
	if (list_empty(&asd->s3a_stats_ready))
		return;

	s3a_buf = list_first_entry(&asd->s3a_stats_ready,
				   struct atomisp_s3a_buf, list);

	/* Translate the oldest ready buffer, exactly like G_3A_STAT does. */
	if (s3a_buf->s3a_map)
		ia_css_translate_3a_statistics(stat, s3a_buf->s3a_map);
	else if (ia_css_get_3a_statistics(stat, s3a_buf->s3a_data))
		return; /* translation failed: leave the buffer queued */

	count = stat->grid.width * stat->grid.height;
	count = min_t(unsigned int, count,
		      asd->params.s3a_output_bytes / sizeof(*cells));
	cells = stat->data;
	for (i = 0; i < count; i++) {
		r += cells[i].awb_r;
		gr += cells[i].awb_gr;
		b += cells[i].awb_b;
		gb += cells[i].awb_gb;
	}

	atomisp_3a_awb(asd, gr, r, b, gb);
	atomisp_3a_ae(asd, stat);
	/* Recycle the consumed buffer to the free list for the firmware. */
	list_del_init(&s3a_buf->list);
	list_add_tail(&s3a_buf->list, &asd->s3a_stats);
}
