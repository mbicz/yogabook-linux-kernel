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
 * Both the statistics ioctl and frame completion are serialized by isp->mutex.
 */

#include <linux/kernel.h>
#include <linux/list.h>

#include <media/v4l2-ctrls.h>

#include <linux/math.h>
#include <linux/math64.h>
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
/*
 * Small, frequent steps keep the preview free of visible brightness jumps:
 * close half of the error per step, at most 8%, every second frame (the
 * sensor applies new exposure a frame later). A full 2x change still settles
 * in about a third of a second at 30 fps.
 */
#define AE_MIN_FACTOR		236	/* at most 0.92x darker per step */
#define AE_MAX_FACTOR		277	/* at most 1.08x brighter per step */
#define AE_DEADBAND		13	/* ~5% luma error: skip adjustment */
#define AE_PACE_FRAMES		2	/* adjust at most every N frames */
/*
 * Analogue gain is spent only once exposure time is exhausted. The OV
 * sensors here use 128 = 1.0x, but OV8858 reports a control minimum of 0,
 * so never use the minimum itself as the 1.0x unit (divide by zero).
 */
#define AE_GAIN_UNIT		128

/*
 * Raw chromaticity (R/G, B/G; AWB_CHROMA_UNIT = 1.0) of a white surface seen
 * by the OV2740 under Planckian light, 2319K..6302K: the inverses of the
 * neutral gains in the AIQB-derived tuning of libcamera's simple IPA
 * ov2740.yaml (colourGains). The s3a statistics are taken after the OB stage
 * (default_ob_config), so they are already black-level corrected.
 *
 * Gray-world on the whole frame fails whenever the scene is not grey on
 * average: clipped windows and lamps read R = G = B and pull the gains towards
 * 1.0x (a strong green cast on this sensor), coloured objects pull towards
 * their complement. Instead:
 *  - drop clipped and near-black cells,
 *  - let only cells whose own chromaticity is near the white curve vote,
 *  - keep the result within AWB_TRANSVERSE of the curve (module-to-module and
 *    illuminant variation is real, a green or magenta "white" is not),
 *  - and converge smoothly.
 */
#define AWB_CHROMA_UNIT		4096
static const struct {
	s32 r, b;
} awb_locus[] = {
	{ 4054, 1465 },	/* 2319K */
	{ 3247, 1720 },	/* 2854K */
	{ 2927, 1690 },	/* 2884K */
	{ 2618, 1834 },	/* 3239K */
	{ 2293, 2172 },	/* 3865K */
	{ 2155, 2161 },	/* 4136K */
	{ 2089, 2492 },	/* 4939K */
	{ 1813, 2918 },	/* 6302K */
};

/*
 * Per-sensor colour tuning. The locus above is the OV2740's; other sensors
 * scale it per channel (4096 = 1.0) to their own white response. The CCM is
 * pushed to the ISP (CCM2, YCgCo -> RGB) at the start of every stream so that
 * switching cameras switches tuning.
 */
struct atomisp_3a_tuning {
	const char *sensor;		/* sensor subdev name prefix */
	s32 locus_r, locus_b;		/* locus scale, 4096 = 1.0 */
	struct ia_css_cc_config ccm;
};

static const struct atomisp_3a_tuning atomisp_3a_tunings[] = {
	{
		/* 4136K CCM from the AIQB-derived libcamera ov2740.yaml */
		.sensor = "ov2740", .locus_r = 4096, .locus_b = 4096,
		.ccm = { 11, { 2048, -5467, 4042, 2048, 4665, -19,
			       2048, -5977, -3906 } },
	}, {
		/* No published OV8858 CCM: plain YCgCo -> RGB, no hue errors. */
		.sensor = "ov8858", .locus_r = 4096, .locus_b = 4096,
		.ccm = { 12, { 4096, -4096, 4096, 4096, 4096, 0,
			       4096, -4096, -4096 } },
	},
};

static const struct atomisp_3a_tuning *
atomisp_3a_tuning(struct atomisp_sub_device *asd)
{
	struct v4l2_subdev *sd = asd->isp->inputs[asd->input_curr].sensor;
	unsigned int i;

	for (i = 0; sd && i < ARRAY_SIZE(atomisp_3a_tunings); i++)
		if (!strncmp(sd->name, atomisp_3a_tunings[i].sensor,
			     strlen(atomisp_3a_tunings[i].sensor)))
			return &atomisp_3a_tunings[i];
	return &atomisp_3a_tunings[0];
}

/* Per-pixel limits in the 13-bit ISP Bayer domain. */
#define AWB_SAT_LEVEL		6800	/* ~83% of 8191: clipped, no colour info */
#define AWB_DARK_LEVEL		48	/* green below this is noise */
#define AWB_GREY_TOL		410	/* grey-candidate cell: within 0.10 of curve */
#define AWB_TRANSVERSE		205	/* result: at most 0.05 off the curve */
#define AWB_MIN_GREY_PCT	5	/* fewer grey voters: use all usable cells */
#define AWB_SPEED_SHIFT		3	/* move 1/8 of the way per frame */

/* Move (cr, cb) to the closest point of the curve; return squared distance. */
static s64 atomisp_3a_awb_project(const struct atomisp_3a_tuning *t,
				  s32 *cr, s32 *cb)
{
	s64 best = S64_MAX, pr = *cr, pb = *cb, rr = *cr, rb = *cb;
	unsigned int i;

	for (i = 0; i + 1 < ARRAY_SIZE(awb_locus); i++) {
		s64 ar = (s64)awb_locus[i].r * t->locus_r / 4096;
		s64 ab = (s64)awb_locus[i].b * t->locus_b / 4096;
		s64 dr = (s64)awb_locus[i + 1].r * t->locus_r / 4096 - ar;
		s64 db = (s64)awb_locus[i + 1].b * t->locus_b / 4096 - ab;
		s64 len = dr * dr + db * db;
		s64 k = (pr - ar) * dr + (pb - ab) * db;
		s64 qr, qb, d;

		k = clamp_t(s64, k, 0, len);
		qr = ar + div64_s64(dr * k, len);
		qb = ab + div64_s64(db * k, len);
		d = (pr - qr) * (pr - qr) + (pb - qb) * (pb - qb);
		if (d < best) {
			best = d;
			rr = qr;
			rb = qb;
		}
	}
	*cr = rr;
	*cb = rb;
	return best;
}

static s32 atomisp_3a_awb_ratio(u64 num, u64 den)
{
	return min_t(u64, div64_u64(num * AWB_CHROMA_UNIT, den),
		     4 * AWB_CHROMA_UNIT);
}

static void atomisp_3a_awb(struct atomisp_sub_device *asd,
			   struct ia_css_3a_statistics *stat,
			   struct ia_css_3a_output *cells, unsigned int count)
{
	unsigned int bqs = stat->grid.bqs_per_grid_cell;
	u64 px = (u64)bqs * bqs;
	u64 sat = AWB_SAT_LEVEL * px, dark = AWB_DARK_LEVEL * px;
	u64 ar = 0, ag = 0, ab = 0, gr = 0, gg = 0, gb = 0;
	unsigned int i, usable = 0, grey = 0;
	const struct atomisp_3a_tuning *tun = atomisp_3a_tuning(asd);
	struct atomisp_wb_config wb;
	s32 all_r, all_b;
	bool first = !asd->awb_r || !asd->awb_b;
	s32 cr, cb, qr, qb;
	s64 d2;
	u32 new_r, new_b;

	if (!asd->awb_en || !px)
		return;

	for (i = 0; i < count; i++) {
		const struct ia_css_3a_output *c = &cells[i];
		u64 g = ((u64)c->awb_gr + c->awb_gb) / 2;
		s32 pr, pb;

		if (c->awb_r > sat || c->awb_gr > sat ||
		    c->awb_b > sat || c->awb_gb > sat)
			continue;	/* clipped: R = G = B whatever the light */
		if (g < dark)
			continue;	/* noise */
		ar += c->awb_r;
		ag += g;
		ab += c->awb_b;
		usable++;

		pr = atomisp_3a_awb_ratio(c->awb_r, g);
		pb = atomisp_3a_awb_ratio(c->awb_b, g);
		if (atomisp_3a_awb_project(tun, &pr, &pb) >
		    (s64)AWB_GREY_TOL * AWB_GREY_TOL)
			continue;	/* too coloured to be a white surface */
		gr += c->awb_r;
		gg += g;
		gb += c->awb_b;
		grey++;
	}
	if (!usable)
		return;	/* everything clipped or dark: keep the current gains */
	all_r = atomisp_3a_awb_ratio(ar, ag);
	all_b = atomisp_3a_awb_ratio(ab, ag);
	if (grey * 100 < usable * AWB_MIN_GREY_PCT) {
		gr = ar;
		gg = ag;
		gb = ab;
	}

	cr = atomisp_3a_awb_ratio(gr, gg);
	cb = atomisp_3a_awb_ratio(gb, gg);
	qr = cr;
	qb = cb;
	d2 = atomisp_3a_awb_project(tun, &qr, &qb);
	if (d2 > (s64)AWB_TRANSVERSE * AWB_TRANSVERSE) {
		s64 d = int_sqrt64(d2);

		cr = qr + div64_s64((s64)(cr - qr) * AWB_TRANSVERSE, d);
		cb = qb + div64_s64((s64)(cb - qb) * AWB_TRANSVERSE, d);
	}
	if (cr <= 0 || cb <= 0)
		return;

	new_r = clamp_t(u32, AWB_UNIT * AWB_CHROMA_UNIT / cr,
			AWB_MIN_GAIN, AWB_MAX_GAIN);
	new_b = clamp_t(u32, AWB_UNIT * AWB_CHROMA_UNIT / cb,
			AWB_MIN_GAIN, AWB_MAX_GAIN);

	/* Converge smoothly instead of jumping with every frame's content. */
	if (asd->awb_r && asd->awb_b) {
		new_r = (s32)asd->awb_r +
			(((s32)new_r - (s32)asd->awb_r) >> AWB_SPEED_SHIFT);
		new_b = (s32)asd->awb_b +
			(((s32)new_b - (s32)asd->awb_b) >> AWB_SPEED_SHIFT);
	}

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

	if (first) {
		asd->params.css_param.yuv2rgb_cc_config = tun->ccm;
		asd->params.config.yuv2rgb_cc_config =
			&asd->params.css_param.yuv2rgb_cc_config;
		asd->params.css_param.update_flag.yuv2rgb_cc_config =
			(struct atomisp_cc_config *)
			&asd->params.css_param.yuv2rgb_cc_config;
	}

	memcpy(&asd->params.css_param.wb_config, &wb, sizeof(wb));
	asd->params.config.wb_config = &asd->params.css_param.wb_config;
	asd->params.css_param.update_flag.wb_config =
		(struct atomisp_wb_config *)&asd->params.css_param.wb_config;
	asd->params.css_update_params_needed = true;

	asd->awb_r = new_r;
	asd->awb_b = new_b;

	dev_dbg(asd->isp->dev,
		"3A: AWB %s applied r=%u b=%u (white %d/%d, all %d/%d, %u grey of %u usable of %u cells)\n",
		tun->sensor, new_r, new_b, cr, cb, all_r, all_b, grey, usable,
		count);
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
	s64 total;

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
	 * The BH histogram mean is used directly as the luma estimate. An
	 * additive offset between the histogram domain and the YUYV output luma
	 * was measured on OV8858 (51) and applied unconditionally, but on the
	 * front OV2740 that offset does not hold: the AE then settled its
	 * target 110 on hist+51, i.e. an output luma of only ~60, leaving the
	 * image dark and (through the ISP low-light chroma processing)
	 * effectively achromatic. Measuring the histogram as-is converges the
	 * output luma to the target on both sensors.
	 */

	factor = AE_TARGET_LUMA * 256 / mean;
	if (abs((int)factor - 256) < AE_DEADBAND)
		return;
	factor = clamp_t(u32, 256 + ((int)factor - 256) / 2,
			 AE_MIN_FACTOR, AE_MAX_FACTOR);

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

	/*
	 * Scale the total exposure (time x gain) once, then spend it on
	 * exposure time first and analogue gain only beyond the longest
	 * exposure: less noise, and the step is exactly 'factor'.
	 */
	total = div_s64((s64)exp * gain * factor, 256);
	new_exp = clamp_t(s64, div_s64(total, max(gain_c->minimum, AE_GAIN_UNIT)),
			  max(exp_c->minimum, 1), exp_c->maximum);
	new_gain = clamp_t(s64, div_s64(total, new_exp),
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
	unsigned int count;

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

	if (asd->awb_en) {
		count = stat->grid.width * stat->grid.height;
		count = min_t(unsigned int, count,
			      asd->params.s3a_output_bytes / sizeof(*cells));
		cells = stat->data;
		atomisp_3a_awb(asd, stat, cells, count);
	}
	atomisp_3a_ae(asd, stat);
	/* Recycle the consumed buffer to the free list for the firmware. */
	list_del_init(&s3a_buf->list);
	list_add_tail(&s3a_buf->list, &asd->s3a_stats);
}
