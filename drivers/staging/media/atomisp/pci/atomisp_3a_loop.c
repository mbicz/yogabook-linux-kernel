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
#include "isp/kernels/gc/gc_2/ia_css_gc2_table.host.h"
#include "isp/kernels/ob/ob_1.0/ia_css_ob.host.h"
#include "isp/kernels/wb/wb_1.0/ia_css_wb.host.h"

/*
 * WB gains are N.3 fixed point, 8192 == 1.0x (the atomisp_wb_config
 * convention used by the firmware).
 */
#define AWB_UNIT		8192
#define AWB_MIN_GAIN		(AWB_UNIT / 2)	/* 0.5x */
#define AWB_MAX_GAIN		(AWB_UNIT * 4)	/* 4.0x */
#define AWB_APPLY_THRESHOLD	64	/* ~0.8% total drift before re-applying */

/* AE: gamma-encoded 8-bit luma target with a clamped controller. */
#define AE_TARGET_LUMA		110
/*
 * Average metering alone exposes a dim room with a bright window for the
 * room. The white-balanced R and B channels carry gains above 1x and clip
 * first; a clipped channel then turns near-whites pink or yellow through
 * the colour matrix. So also limit the share of pixels at full scale in
 * the brightest white-balanced channel, but never darken below a floor.
 * All three are re-read every adjustment, so they can be tuned live.
 */
#define AE_CLIP_BIN		248	/* of 255, linear, after WB gains */
static unsigned int ae_target = AE_TARGET_LUMA;
module_param(ae_target, uint, 0644);
MODULE_PARM_DESC(ae_target, "AE target mean output luma, 8-bit (default 110)");
static unsigned int ae_clip_permille = 10;
module_param(ae_clip_permille, uint, 0644);
MODULE_PARM_DESC(ae_clip_permille,
		 "AE highlight limit: permille of pixels at full scale in the brightest white-balanced channel (default 10)");
static unsigned int ae_min_luma = 50;
module_param(ae_min_luma, uint, 0644);
MODULE_PARM_DESC(ae_min_luma,
		 "AE does not darken below this mean output luma to protect highlights (default 50)");
static unsigned int ae_tone_strength = 100;
module_param(ae_tone_strength, uint, 0644);
MODULE_PARM_DESC(ae_tone_strength,
		 "Shadow lift while highlights hold AE below target, percent of the missing brightness (0 off, default 100)");
static unsigned int ae_clip_deadband_permille = 5;
module_param(ae_clip_deadband_permille, uint, 0644);
MODULE_PARM_DESC(ae_clip_deadband_permille,
		 "AE highlight deadband around the clip limit, permille (default 5)");
#define TONE_A_MAX		(4 * 256)	/* Q8: shadows gain at most 5x */
/*
 * The caller slews the lift by TONE_SLEW per update, so the hysteresis only
 * has to suppress rounding noise; a larger step would deadlock the ramp
 * (the caller would move less than the step and the upload never happen).
 */
#define TONE_A_STEP		2		/* Q8: re-upload beyond ~0.8% change */
/*
 * Small, frequent steps keep the preview free of visible brightness jumps:
 * at most 8%, every second frame (the sensor applies new exposure a frame
 * later). Steps are halved once the scene is known to clip, because the
 * highlight limit then reacts to exposure at ~3.4 permille per 1% and a full
 * step would overshoot the limit band by more than the band itself. A full
 * 2x change still settles in about a third of a second at 30 fps.
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
 * The share of clipped pixels is a hard knee in exposure: below it nothing
 * clips, above it the slope is a property of the scene (measured at ~341
 * permille per 100% exposure on a ceiling-lit room, ~1200-1900 on synthetic
 * steep ones). Estimate that slope online instead of assuming it, so the
 * controller can size its step to cross the limit band exactly once.
 */
#define SLOPE_MIN		8	/* permille clip per 100% exposure */
#define SLOPE_MAX		4096	/* keep a noisy sample from stalling AE */
#define SLOPE_INIT		341	/* the fitted slope, until one is measured */
#define MEAN_SHIFT		2	/* luma EMA, Q8 so small errors survive */
#define AE_CLIP_STEP		4	/* permille: minimum darken near the limit */
#define TONE_SLEW		8	/* Q8: tone lift slew per update */

/*
 * Raw chromaticity (R/G, B/G; AWB_CHROMA_UNIT = 1.0) of neutral surfaces.
 * Statistics follow optical-black correction. Reject clipped/dark cells and
 * coloured objects, constrain the result to the sensor's own white curve,
 * and converge smoothly rather than applying whole-frame grey-world.
 */
#define AWB_CHROMA_UNIT		4096
struct atomisp_awb_point {
	s32 r, b;
};

/*
 * Lenovo 01ov2740-yeti.aiqb ("OV2740 V11 tuning"), the front module's own
 * factory tuning from the same S100043 firmware as the rear one.
 * CMC_ChromaticityResponse (15), average responses: Q10 converted to Q12,
 * ordered warm to cool.
 */
static const struct atomisp_awb_point ov2740_awb_locus[] = {
	{ 3856, 1400 }, { 3128, 1520 }, { 2452, 1844 }, { 2244, 2052 },
	{ 1776, 3008 },
};

/*
 * Lenovo 00ov8858-yeti.aiqb, Lenovo_OV8858_IQ3.2PRC_20160428.
 * CMC_ChromaticityResponse (15), average responses: Q10 R/G and B/G,
 * converted to Q12 and ordered warm to cool. The high/low module-variation
 * envelopes are not extra illuminants.
 */
static const struct atomisp_awb_point ov8858_awb_locus[] = {
	{ 4344, 1680 }, { 4084, 1716 }, { 3720, 1784 }, { 3516, 1836 },
	{ 3344, 1892 }, { 3140, 1972 }, { 2836, 2140 }, { 2600, 2328 },
	{ 2484, 2448 }, { 2360, 2604 }, { 2248, 2772 }, { 2104, 3040 },
	{ 2036, 3188 }, { 1976, 3352 }, { 1880, 3628 }, { 1804, 3884 },
};

struct atomisp_ccm_point {
	struct atomisp_awb_point white;
	struct ia_css_cc_config ccm;
};

/*
 * Same AIQB, CMC_ColorMatrices (18): accurate RGB matrices, Q16.
 * Compose with YCgCo -> RGB, then round to CCM2 Q11. Q11 keeps every
 * coefficient within the ISP's [-8192, 8191] range. Neutral row sums are 1.
 * The calibration white responses in this record use Q8, converted to Q12.
 */
static const struct atomisp_ccm_point ov8858_ccms[] = {
	{ { 3568, 1808 },	/* A, incandescent */
	  { 11, { 2048, -1629, 3653, 2048, 6923, 258,
		  2048, -5699, -5570 } } },
	{ { 2688, 2080 },	/* F11, TL84 */
	  { 11, { 2048, -3921, 3813, 2048, 5889, 25,
		  2048, -5045, -4445 } } },
	{ { 2336, 2656 },	/* D50 */
	  { 11, { 2048, -4800, 3843, 2048, 6117, 606,
		  2048, -5066, -4063 } } },
	{ { 2064, 2976 },	/* D65 */
	  { 11, { 2048, -5160, 3953, 2048, 5904, 636,
		  2048, -4641, -3682 } } },
};

/*
 * Same front AIQB, CMC_ColorMatrices (18): accurate RGB matrices, folded and
 * rounded exactly like the rear ones. Light sources by Intel CMC enum.
 */
static const struct atomisp_ccm_point ov2740_ccms[] = {
	{ { 3152, 1584 },	/* 1, A, incandescent */
	  { 11, { 2048, -2283, 3696, 2048, 6198, 42,
		  2048, -5604, -4390 } } },
	{ { 2576, 1984 },	/* 19, F11, TL84 */
	  { 11, { 2048, -3726, 3673, 2048, 5460, -140,
		  2048, -4980, -3811 } } },
	{ { 2128, 1936 },	/* 17, F9 fluorescent */
	  { 11, { 2048, -5292, 4393, 2048, 5330, -338,
		  2048, -4055, -3629 } } },
	{ { 1792, 3008 },	/* 6, D65 */
	  { 11, { 2048, -5331, 4148, 2048, 5027, 499,
		  2048, -4857, -3514 } } },
};

struct atomisp_3a_tuning {
	const char *sensor;
	const struct atomisp_awb_point *locus;
	unsigned int num_locus;
	struct ia_css_cc_config ccm;
	const struct atomisp_ccm_point *ccms;
	unsigned int num_ccms;
	u32 black_level; /* u0.16 override; zero keeps the ISP default. */
};

static const struct atomisp_3a_tuning atomisp_3a_tunings[] = {
	{
		.sensor = "ov2740",
		.locus = ov2740_awb_locus,
		.num_locus = ARRAY_SIZE(ov2740_awb_locus),
		.ccms = ov2740_ccms,
		.num_ccms = ARRAY_SIZE(ov2740_ccms),
	}, {
		.sensor = "ov8858",
		.locus = ov8858_awb_locus,
		.num_locus = ARRAY_SIZE(ov8858_awb_locus),
		.ccms = ov8858_ccms,
		.num_ccms = ARRAY_SIZE(ov8858_ccms),
		.black_level = 1024,
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

/* Per-pixel limits in the ISP Bayer domain, [0, 8191]. */
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

	for (i = 0; i + 1 < t->num_locus; i++) {
		s64 ar = t->locus[i].r;
		s64 ab = t->locus[i].b;
		s64 dr = t->locus[i + 1].r - ar;
		s64 db = t->locus[i + 1].b - ab;
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

/*
 * Interpolate the rear's CCM at the applied (smoothed) white balance, not the
 * noisy instantaneous estimate. Clamp to the nearest calibrated segment at
 * either end; do not extrapolate matrices beyond the measured illuminants.
 */
static void atomisp_3a_ccm(const struct atomisp_3a_tuning *t, u32 r, u32 b,
			   struct ia_css_cc_config *ccm)
{
	s64 pr, pb;
	s64 best = S64_MAX, weight = 0;
	unsigned int segment = 0, i, j;

	if (!t->ccms) {
		*ccm = t->ccm;
		return;
	}
	pr = (u64)AWB_UNIT * AWB_CHROMA_UNIT / r;
	pb = (u64)AWB_UNIT * AWB_CHROMA_UNIT / b;

	for (i = 0; i + 1 < t->num_ccms; i++) {
		s64 ar = t->ccms[i].white.r;
		s64 ab = t->ccms[i].white.b;
		s64 dr = t->ccms[i + 1].white.r - ar;
		s64 db = t->ccms[i + 1].white.b - ab;
		s64 len = dr * dr + db * db;
		s64 k = clamp_t(s64, (pr - ar) * dr + (pb - ab) * db, 0, len);
		s64 qr = ar + div64_s64(dr * k, len);
		s64 qb = ab + div64_s64(db * k, len);
		s64 distance = (pr - qr) * (pr - qr) + (pb - qb) * (pb - qb);

		if (distance < best) {
			best = distance;
			segment = i;
			weight = div64_s64(k * AWB_CHROMA_UNIT, len);
		}
	}

	ccm->fraction_bits = 11;
	for (j = 0; j < ARRAY_SIZE(ccm->matrix); j++) {
		s32 a = t->ccms[segment].ccm.matrix[j];
		s32 z = t->ccms[segment + 1].ccm.matrix[j];

		ccm->matrix[j] = a + div_s64((s64)(z - a) * weight,
					     AWB_CHROMA_UNIT);
	}
}

/*
 * Global tone lift: y = (1 + a) x / (1 + a x) on the linear input of the RGB
 * gamma stage, composed with the stock sRGB-like curve. Black and full scale
 * stay where they are; shadows gain up to (1 + a)x and highlights compress
 * slightly. a == 0 restores the stock curve. Tables are only re-uploaded
 * when a moves noticeably.
 */
static void atomisp_3a_tone_fill(struct atomisp_sub_device *asd, u32 a)
{
	struct atomisp_css_params *p = &asd->params.css_param;
	const u16 *base = default_g_gamma_table.data.vamem_2;
	struct ia_css_rgb_gamma_table *t = &p->g_gamma_table;
	unsigned int i;

	t->vamem_type = IA_CSS_VAMEM_TYPE_2;
	for (i = 0; i < IA_CSS_VAMEM_2_RGB_GAMMA_TABLE_SIZE; i++) {
		u32 x = i << 8;			/* Q16 linear input */
		u32 y = div_u64((u64)x * (256 + a),
				256 + (u32)(((u64)a * x) >> 16));
		u32 j = y >> 8, f = y & 0xff;

		t->data.vamem_2[i] = j >= IA_CSS_VAMEM_2_RGB_GAMMA_TABLE_SIZE - 1 ?
			base[IA_CSS_VAMEM_2_RGB_GAMMA_TABLE_SIZE - 1] :
			base[j] + (((base[j + 1] - base[j]) * f) >> 8);
	}
	p->r_gamma_table = *t;
	p->b_gamma_table = *t;
	asd->tone_a = a;
}

static void atomisp_3a_tone(struct atomisp_sub_device *asd, u32 a)
{
	struct atomisp_css_params *p = &asd->params.css_param;

	if (abs((int)a - (int)asd->tone_a) < TONE_A_STEP && (a || !asd->tone_a))
		return;
	atomisp_3a_tone_fill(asd, a);
	asd->params.config.r_gamma_table = &p->r_gamma_table;
	asd->params.config.g_gamma_table = &p->g_gamma_table;
	asd->params.config.b_gamma_table = &p->b_gamma_table;
	p->update_flag.r_gamma_table = (struct atomisp_rgb_gamma_table *)&p->r_gamma_table;
	p->update_flag.g_gamma_table = (struct atomisp_rgb_gamma_table *)&p->g_gamma_table;
	p->update_flag.b_gamma_table = (struct atomisp_rgb_gamma_table *)&p->b_gamma_table;
	asd->params.css_update_params_needed = true;
}

/*
 * Install the selected sensor's parameters before the first frame, even with
 * AWB disabled. Keep the persistent cache in step with CSS so later parameter
 * commits cannot restore the previous sensor's white balance or colour matrix.
 *
 * The OV8858 pedestal is 16/1024, not the OV2740 default of 64/1024.
 * Subtracting 64 folds the rear's dark Bayer values and solarizes colours.
 */
void atomisp_3a_loop_init(struct atomisp_sub_device *asd)
{
	const struct atomisp_3a_tuning *tun = atomisp_3a_tuning(asd);
	struct ia_css_ob_config *ob = &asd->params.css_param.ob_config;
	struct ia_css_isp_config config;

	if (asd->copy_mode)
		return;

	asd->params.css_param.wb_config = default_wb_config;
	atomisp_3a_ccm(tun, default_wb_config.r, default_wb_config.b,
		       &asd->params.css_param.yuv2rgb_cc_config);
	*ob = default_ob_config;
	if (tun->black_level) {
		ob->level_gr = tun->black_level;
		ob->level_r = tun->black_level;
		ob->level_b = tun->black_level;
		ob->level_gb = tun->black_level;
	}
	/* Recovery also reaches this path without the normal stream-on ioctl. */
	asd->awb_r = 0;
	asd->awb_b = 0;
	/* A new stream starts on the stock tone curve. */
	atomisp_3a_tone_fill(asd, 0);
	/*
	 * The highlight servo learns from one scene only: a new stream (or a
	 * sensor switch) must not inherit the previous one's clip slope, EMA
	 * or knee, or AE would step as if the old scene were still there.
	 */
	asd->ae_slope = SLOPE_INIT;
	asd->ae_mean_f = 0;
	asd->ae_last_exp = 0;
	asd->ae_prev_exp = 0;
	asd->ae_last_clip = 0;
	asd->ae_knee_exp = 0;

	memset(&config, 0, sizeof(config));
	config.ob_config = ob;
	config.wb_config = &asd->params.css_param.wb_config;
	config.yuv2rgb_cc_config = &asd->params.css_param.yuv2rgb_cc_config;
	config.r_gamma_table = &asd->params.css_param.r_gamma_table;
	config.g_gamma_table = &asd->params.css_param.g_gamma_table;
	config.b_gamma_table = &asd->params.css_param.b_gamma_table;
	ia_css_stream_set_isp_config(asd->stream_env[ATOMISP_INPUT_STREAM_GENERAL].stream,
				     &config);
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

	if (first || tun->ccms) {
		struct ia_css_cc_config ccm;

		atomisp_3a_ccm(tun, new_r, new_b, &ccm);
		if (first || memcmp(&asd->params.css_param.yuv2rgb_cc_config,
				    &ccm, sizeof(ccm))) {
			asd->params.css_param.yuv2rgb_cc_config = ccm;
			asd->params.config.yuv2rgb_cc_config =
				&asd->params.css_param.yuv2rgb_cc_config;
			asd->params.css_param.update_flag.yuv2rgb_cc_config =
				(struct atomisp_cc_config *)
				&asd->params.css_param.yuv2rgb_cc_config;
		}
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
	const u16 *gamma = default_g_gamma_table.data.vamem_2;
	u64 luma_sum = 0, count = 0;
	u64 chan[3] = { 0 }, clipped[3] = { 0 };
	unsigned int mean, factor, i, clip = 0, target, limit, floor;
	unsigned int db, lo, hi;
	s32 exp, gain, new_exp, new_gain, mf, err, cap, f, mean_f;
	bool forced;
	u32 want, tone;
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
	/*
	 * BH bins are linear, while the target is output luma. Apply the
	 * production gamma curve to each bin before averaging; gamma(mean)
	 * would overestimate the output for scenes with mixed illumination.
	 */
	for (i = 1; i < 256; i++) {
		count += hist[i].y;
		luma_sum += (u64)gamma[i] * hist[i].y;
		chan[0] += hist[i].r;
		chan[1] += hist[i].g;
		chan[2] += hist[i].b;
		if (i >= AE_CLIP_BIN) {
			clipped[0] += hist[i].r;
			clipped[1] += hist[i].g;
			clipped[2] += hist[i].b;
		}
	}
	/*
	 * Black pixels are part of the scene, not missing samples. Excluding
	 * them makes the clip fraction rise as a dark scene gets darker.
	 * The decoder can underflow bin 0 while removing grid padding; only
	 * add each channel's zero count when that subtraction was sane.
	 */
	if (hist[0].r <= 0x7fffffff)
		chan[0] += hist[0].r;
	if (hist[0].g <= 0x7fffffff)
		chan[1] += hist[0].g;
	if (hist[0].b <= 0x7fffffff)
		chan[2] += hist[0].b;
	/* Permille of pixels at full scale in the most-clipped channel. */
	for (i = 0; i < 3; i++)
		if (chan[i])
			clip = max_t(unsigned int, clip,
				     div64_u64(clipped[i] * 1000, chan[i]));
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
	/* The ISP2400 gamma table is u0.12; normalize to full-range 8-bit Y. */
	mean = div64_u64(luma_sum * 255, count * 4095);
	/* A black frame must still raise exposure, or AE never recovers. */
	if (!mean)
		mean = 1;

	/* clip is a permille, so anything above 1000 is meaningless. */
	target = clamp_t(unsigned int, READ_ONCE(ae_target), 16, 240);
	limit = min_t(unsigned int, READ_ONCE(ae_clip_permille), 1000);
	floor = min_t(unsigned int, READ_ONCE(ae_min_luma), target);
	db = min_t(unsigned int, READ_ONCE(ae_clip_deadband_permille), 1000);
	lo = limit > db ? limit - db : 0;
	hi = limit + db;

	if (!asd->isp->inputs[asd->input_curr].sensor)
		return;
	handler = asd->isp->inputs[asd->input_curr].sensor->ctrl_handler;
	exp_c = v4l2_ctrl_find(handler, V4L2_CID_EXPOSURE);
	gain_c = v4l2_ctrl_find(handler, V4L2_CID_ANALOGUE_GAIN);
	if (!exp_c || !gain_c)
		return;
	exp = v4l2_ctrl_g_ctrl(exp_c);
	gain = v4l2_ctrl_g_ctrl(gain_c);
	if (exp <= 0 || gain < 0)
		return;
	/* OV8858 permits zero gain, which cannot recover by multiplication. */
	if (!gain) {
		ctrl.id = V4L2_CID_ANALOGUE_GAIN;
		ctrl.value = max(gain_c->minimum, AE_GAIN_UNIT);
		if (v4l2_s_ctrl(NULL, handler, &ctrl))
			return;
		gain = ctrl.value;
	}

	/*
	 * Mean luma in Q8: at 4 luma per step, plain integer smoothing would
	 * quantise away exactly the error AE has to correct.
	 */
	if (!asd->ae_mean_f)
		asd->ae_mean_f = (s32)mean << 8;
	else
		asd->ae_mean_f += (((s32)mean << 8) - asd->ae_mean_f) >> MEAN_SHIFT;
	mean_f = (asd->ae_mean_f + 128) >> 8;
	if (mean_f < 1)
		mean_f = 1;

	/*
	 * Estimate the clip slope online. Both terms must describe the same
	 * frame: ae_last_clip was measured with ae_last_exp, so the ratio of
	 * that exposure to the one before it is the matching denominator.
	 * The control read above is one update newer than the statistics, so
	 * it must not be used here.
	 */
	if (asd->ae_prev_exp > 0 && asd->ae_last_exp > 0) {
		s32 r = ((s32)asd->ae_last_exp << 8) / asd->ae_prev_exp - 256;

		if (r > 4 && (s32)clip > (s32)asd->ae_last_clip) {
			s32 dc = (s32)clip - (s32)asd->ae_last_clip;
			s32 s = clamp_t(s32, (dc << 8) / r, SLOPE_MIN, SLOPE_MAX);

			asd->ae_slope = (asd->ae_slope + s) / 2;
		}
	}
	/*
	 * Extrapolate back to the exposure where clipping starts. clip is
	 * monotone in exposure, so observing none at an exposure above the
	 * last estimate raises it: the knee is a lower bound that must keep
	 * moving up, or a transient clip would freeze AE dark for good.
	 */
	if (clip > 0 && asd->ae_slope > 0 && asd->ae_last_exp > 0) {
		s32 kk = asd->ae_last_exp * ((s32)asd->ae_slope - (s32)clip) /
			 (s32)asd->ae_slope;

		if (kk >= 1)
			asd->ae_knee_exp = kk;
	} else if (!clip && asd->ae_last_exp > asd->ae_knee_exp) {
		asd->ae_knee_exp = asd->ae_last_exp;
	}

	mf = target * 256 / mean_f;
	err = (s32)mf - 256;
	/* Cap brightening at the knee, but never use the cap to darken. */
	cap = mf;
	if (asd->ae_knee_exp > 0 && asd->ae_slope > hi) {
		s64 e_allow = div_s64((s64)asd->ae_knee_exp * asd->ae_slope,
				      (s64)asd->ae_slope - (s64)hi);
		s64 fcap = exp > 0 ? div_s64(256 * e_allow, exp) : 0;

		fcap = max_t(s64, fcap, 256);
		if (fcap < cap)
			cap = fcap;
	}

	/*
	 * Three regimes, one actuator each:
	 *  - above the band: drive clip back into the band, not mean to target
	 *    (mean is unreachable without clipping there);
	 *  - inside the band: hold exposure and let tone lift the shadows;
	 *  - below the band: ordinary mean servo, capped at the knee.
	 * Halving only near the knee: far from it the knee cap does not bind
	 * and halving would merely quadruple the settle time.
	 */
	forced = false;
	/* A highlight must not drive the rest of the scene below the floor. */
	if (mean_f < floor) {
		f = (s32)floor * 256 / mean_f;
		forced = true;
	} else if ((s32)clip > (s32)hi) {
		f = 256 + (256 * ((s32)hi - (s32)clip)) / (s32)asd->ae_slope;
		f = min(f, 256 - AE_CLIP_STEP);
		f = max(f, (s32)floor * 256 / mean_f);
		forced = true;
	} else if ((s32)clip >= (s32)lo) {
		f = err >= 0 ? 256 : mf;	/* hold; tone carries the deficit */
	} else if (err > AE_DEADBAND) {
		f = min(mf, cap);
	} else if (err < -AE_DEADBAND) {
		f = mf;
	} else {
		f = 256;
	}
	f = clamp_t(s32, f, AE_MIN_FACTOR, AE_MAX_FACTOR);
	if (!forced && f != 256 && ((s32)clip > 0 || cap < mf)) {
		f = clamp_t(s32, 256 + (f - 256) / 2, AE_MIN_FACTOR,
			    AE_MAX_FACTOR);
		if (abs(f - 256) < 2)
			f = 256;
	}
	factor = (u32)f;

	/*
	 * When highlights hold AE below target, lift the shadows by the
	 * missing brightness instead of exposing longer. The lift acts before
	 * the RGB gamma curve, roughly a square root, so an output deficit of
	 * target/mean needs about (target/mean)^2 of linear shadow gain. The
	 * histogram is measured before gamma too, so the lift never feeds back
	 * into 'mean'. The lift is slewed, because a step change would itself
	 * be a visible brightness jump.
	 */
	want = 0;
	if (mean_f < (s32)target && ((s32)clip >= (s32)lo || cap < mf)) {
		u64 deficit = (u64)target * target * 256 /
			      ((u64)mean_f * mean_f) - 256;

		want = (u32)min_t(u64, deficit * READ_ONCE(ae_tone_strength) / 100,
				   TONE_A_MAX);
	}
	tone = asd->tone_a;
	if (want > tone)
		tone = min(want, tone + TONE_SLEW);
	else if (want < tone)	/* u32: tone - TONE_SLEW would wrap below 8 */
		tone = tone > want + TONE_SLEW ? tone - TONE_SLEW : want;
	atomisp_3a_tone(asd, tone);

	/* Remember this frame's exposure and clip for the slope estimator. */
	asd->ae_prev_exp = asd->ae_last_exp;
	asd->ae_last_exp = exp;
	asd->ae_last_clip = clip;

	if (factor == 256)
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
			   max(gain_c->minimum, AE_GAIN_UNIT), gain_c->maximum);
	if (new_exp == exp && new_gain == gain)
		return;

	ctrl.id = V4L2_CID_EXPOSURE;
	ctrl.value = new_exp;
	if (v4l2_s_ctrl(NULL, handler, &ctrl))
		return;

	ctrl.id = V4L2_CID_ANALOGUE_GAIN;
	ctrl.value = new_gain;
	v4l2_s_ctrl(NULL, handler, &ctrl);

	dev_dbg(asd->isp->dev, "3A: AE applied exp=%d gain=%d (y=%u clip=%u/1000 tone=%u)\n",
		new_exp, new_gain, mean, clip, asd->tone_a);
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
