// SPDX-License-Identifier: GPL-2.0
/*
 * Support for Intel Camera Imaging ISP subsystem.
 * Copyright (c) 2015, Intel Corporation.
 */

#include "ia_css_types.h"
#include "sh_css_defs.h"
#ifndef IA_CSS_NO_DEBUG
#include "ia_css_debug.h"
#endif

#include "ia_css_aa2.host.h"

/* YUV Anti-Aliasing configuration. */
const struct ia_css_aa_config default_aa_config = {
	0 /* 0 = ineffective; 8191 averages Bayer channels and kills chroma */
};

/* Bayer Anti-Aliasing configuration. */
const struct ia_css_aa_config default_baa_config = {
	0 /* 0 = ineffective; 8191 averages Bayer channels and kills chroma */
};
