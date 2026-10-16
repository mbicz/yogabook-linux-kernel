// SPDX-License-Identifier: GPL-2.0
/*
 * Intel CHT Whiskey Cove PMIC operation region driver
 * Copyright (C) 2017 Hans de Goede <hdegoede@redhat.com>
 *
 * Based on various non upstream patches to support the CHT Whiskey Cove PMIC:
 * Copyright (C) 2013-2015 Intel Corporation. All rights reserved.
 */

#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include "intel_pmic.h"

#define CHT_WC_V1P05A_CTRL		0x6e3b
#define CHT_WC_V1P15_CTRL		0x6e3c
#define CHT_WC_V1P05A_VSEL		0x6e3d
#define CHT_WC_V1P15_VSEL		0x6e3e
#define CHT_WC_V1P8A_CTRL		0x6e56
#define CHT_WC_V1P8SX_CTRL		0x6e57
#define CHT_WC_VDDQ_CTRL		0x6e58
#define CHT_WC_V1P2A_CTRL		0x6e59
#define CHT_WC_V1P2SX_CTRL		0x6e5a
#define CHT_WC_V1P8A_VSEL		0x6e5b
#define CHT_WC_VDDQ_VSEL		0x6e5c
#define CHT_WC_V2P8SX_CTRL		0x6e5d
#define CHT_WC_V3P3A_CTRL		0x6e5e
#define CHT_WC_V3P3SD_CTRL		0x6e5f
#define CHT_WC_VSDIO_CTRL		0x6e67
#define CHT_WC_V3P3A_VSEL		0x6e68
#define CHT_WC_VPROG1A_CTRL		0x6e90
#define CHT_WC_VPROG1B_CTRL		0x6e91
#define CHT_WC_VPROG1F_CTRL		0x6e95
#define CHT_WC_VPROG2D_CTRL		0x6e99
#define CHT_WC_VPROG3A_CTRL		0x6e9a
#define CHT_WC_VPROG3B_CTRL		0x6e9b
#define CHT_WC_VPROG4A_CTRL		0x6e9c
#define CHT_WC_VPROG4B_CTRL		0x6e9d
#define CHT_WC_VPROG4C_CTRL		0x6e9e
#define CHT_WC_VPROG4D_CTRL		0x6e9f
#define CHT_WC_VPROG5A_CTRL		0x6ea0
#define CHT_WC_VPROG5B_CTRL		0x6ea1
#define CHT_WC_VPROG6A_CTRL		0x6ea2
#define CHT_WC_VPROG6B_CTRL		0x6ea3
#define CHT_WC_VPROG1A_VSEL		0x6ec0
#define CHT_WC_VPROG1B_VSEL		0x6ec1
#define CHT_WC_V1P8SX_VSEL		0x6ec2
#define CHT_WC_V1P2SX_VSEL		0x6ec3
#define CHT_WC_V1P2A_VSEL		0x6ec4
#define CHT_WC_VPROG1F_VSEL		0x6ec5
#define CHT_WC_VSDIO_VSEL		0x6ec6
#define CHT_WC_V2P8SX_VSEL		0x6ec7
#define CHT_WC_V3P3SD_VSEL		0x6ec8
#define CHT_WC_VPROG2D_VSEL		0x6ec9
#define CHT_WC_VPROG3A_VSEL		0x6eca
#define CHT_WC_VPROG3B_VSEL		0x6ecb
#define CHT_WC_VPROG4A_VSEL		0x6ecc
#define CHT_WC_VPROG4B_VSEL		0x6ecd
#define CHT_WC_VPROG4C_VSEL		0x6ece
#define CHT_WC_VPROG4D_VSEL		0x6ecf
#define CHT_WC_VPROG5A_VSEL		0x6ed0
#define CHT_WC_VPROG5B_VSEL		0x6ed1
#define CHT_WC_VPROG6A_VSEL		0x6ed2
#define CHT_WC_VPROG6B_VSEL		0x6ed3

/*
 * Regulator support is based on the non upstream patch:
 * "regulator: whiskey_cove: implements Whiskey Cove pmic VRF support"
 * https://github.com/intel-aero/meta-intel-aero/blob/master/recipes-kernel/linux/linux-yocto/0019-regulator-whiskey_cove-implements-WhiskeyCove-pmic-V.patch
 */
static const struct pmic_table power_table[] = {
	{
		.address = 0x0,
		.reg = CHT_WC_V1P8A_CTRL,
		.bit = 0x01,
	}, /* V18A */
	{
		.address = 0x04,
		.reg = CHT_WC_V1P8SX_CTRL,
		.bit = 0x07,
	}, /* V18X */
	{
		.address = 0x08,
		.reg = CHT_WC_VDDQ_CTRL,
		.bit = 0x01,
	}, /* VDDQ */
	{
		.address = 0x0c,
		.reg = CHT_WC_V1P2A_CTRL,
		.bit = 0x07,
	}, /* V12A */
	{
		.address = 0x10,
		.reg = CHT_WC_V1P2SX_CTRL,
		.bit = 0x07,
	}, /* V12X */
	{
		.address = 0x14,
		.reg = CHT_WC_V2P8SX_CTRL,
		.bit = 0x07,
	}, /* V28X */
	{
		.address = 0x18,
		.reg = CHT_WC_V3P3A_CTRL,
		.bit = 0x01,
	}, /* V33A */
	{
		.address = 0x1c,
		.reg = CHT_WC_V3P3SD_CTRL,
		.bit = 0x07,
	}, /* V3SD */
	{
		.address = 0x20,
		.reg = CHT_WC_VSDIO_CTRL,
		.bit = 0x07,
	}, /* VSD */
/*	{
		.address = 0x24,
		.reg = ??,
		.bit = ??,
	}, ** VSW2 */
/*	{
		.address = 0x28,
		.reg = ??,
		.bit = ??,
	}, ** VSW1 */
/*	{
		.address = 0x2c,
		.reg = ??,
		.bit = ??,
	}, ** VUPY */
/*	{
		.address = 0x30,
		.reg = ??,
		.bit = ??,
	}, ** VRSO */
	{
		.address = 0x34,
		.reg = CHT_WC_VPROG1A_CTRL,
		.bit = 0x07,
	}, /* VP1A */
	{
		.address = 0x38,
		.reg = CHT_WC_VPROG1B_CTRL,
		.bit = 0x07,
	}, /* VP1B */
	{
		.address = 0x3c,
		.reg = CHT_WC_VPROG1F_CTRL,
		.bit = 0x07,
	}, /* VP1F */
	{
		.address = 0x40,
		.reg = CHT_WC_VPROG2D_CTRL,
		.bit = 0x07,
	}, /* VP2D */
	{
		.address = 0x44,
		.reg = CHT_WC_VPROG3A_CTRL,
		.bit = 0x07,
	}, /* VP3A */
	{
		.address = 0x48,
		.reg = CHT_WC_VPROG3B_CTRL,
		.bit = 0x07,
	}, /* VP3B */
	{
		.address = 0x4c,
		.reg = CHT_WC_VPROG4A_CTRL,
		.bit = 0x07,
	}, /* VP4A */
	{
		.address = 0x50,
		.reg = CHT_WC_VPROG4B_CTRL,
		.bit = 0x07,
	}, /* VP4B */
	{
		.address = 0x54,
		.reg = CHT_WC_VPROG4C_CTRL,
		.bit = 0x07,
	}, /* VP4C */
	{
		.address = 0x58,
		.reg = CHT_WC_VPROG4D_CTRL,
		.bit = 0x07,
	}, /* VP4D */
	{
		.address = 0x5c,
		.reg = CHT_WC_VPROG5A_CTRL,
		.bit = 0x07,
	}, /* VP5A */
	{
		.address = 0x60,
		.reg = CHT_WC_VPROG5B_CTRL,
		.bit = 0x07,
	}, /* VP5B */
	{
		.address = 0x64,
		.reg = CHT_WC_VPROG6A_CTRL,
		.bit = 0x07,
	}, /* VP6A */
	{
		.address = 0x68,
		.reg = CHT_WC_VPROG6B_CTRL,
		.bit = 0x07,
	}, /* VP6B */
/*	{
		.address = 0x6c,
		.reg = ??,
		.bit = ??,
	}  ** VP7A */
};

static int intel_cht_wc_pmic_get_power(struct regmap *regmap, int reg,
		int bit, u64 *value)
{
	int data;

	if (regmap_read(regmap, reg, &data))
		return -EIO;

	*value = (data & bit) ? 1 : 0;
	return 0;
}

static int intel_cht_wc_pmic_update_power(struct regmap *regmap, int reg,
		int bitmask, bool on)
{
	return regmap_update_bits(regmap, reg, bitmask, on ? 1 : 0);
}

static int intel_cht_wc_exec_mipi_pmic_seq_element(struct regmap *regmap,
						   u16 i2c_client_address,
						   u32 reg_address,
						   u32 value, u32 mask)
{
	struct device *dev = regmap_get_device(regmap);
	u32 address;

	if (i2c_client_address > 0xff || reg_address > 0xff) {
		dev_warn(dev, "warning addresses too big client 0x%x reg 0x%x\n",
			 i2c_client_address, reg_address);
		return -ERANGE;
	}

	address = (i2c_client_address << 8) | reg_address;

	return regmap_update_bits(regmap, address, mask, value);
}

/*
 * System thermistor registers and current-source conversion are documented by
 * Lenovo's Yoga Book Android driver, intel_soc_pmic_wc_opregion.c. Request a
 * fresh thermal-group conversion: firmware does not necessarily leave periodic
 * sampling enabled. The opregion mutex serializes these ADC transactions.
 *
 * Keep interrupt masks, monitor periods, alert thresholds and power rails
 * unchanged. Completion is polled while the ADC interrupt remains masked.
 */
#define CHT_WC_ADC_REQUEST	0x4f02
#define CHT_WC_ADC_IRQ		0x6e08
#define CHT_WC_ADC_THERMAL	BIT(3)
#define CHT_WC_ADC_DONE		BIT(2)

static const struct pmic_table thermal_table[] = {
	{ .address = 0x00, .reg = 0x4f39 }, /* TMP0: system thermistor 0 */
	{ .address = 0x0c, .reg = 0x4f3b }, /* TMP1: system thermistor 1 */
	{ .address = 0x18, .reg = 0x4f3d }, /* TMP2: system thermistor 2 */
};

static int intel_cht_wc_pmic_get_raw_temp(struct regmap *regmap, int reg)
{
	static const unsigned int resistance_lsb[] = {
		0, 260420, 130210, 65100, 32550, 16280,
		8140, 4070, 2030, 0, 260420, 130210,
	};
	unsigned int value, high, low, source, raw;
	int ret;

	ret = regmap_read_poll_timeout(regmap, CHT_WC_ADC_REQUEST, value,
				      !(value & BIT(0)), 2000, 1000000);
	if (ret)
		return ret;

	ret = regmap_write(regmap, CHT_WC_ADC_IRQ, CHT_WC_ADC_DONE);
	if (ret)
		return ret;
	ret = regmap_write(regmap, CHT_WC_ADC_REQUEST, CHT_WC_ADC_THERMAL);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(regmap, CHT_WC_ADC_IRQ, value,
				      value & CHT_WC_ADC_DONE, 2000, 1000000);
	if (ret)
		return ret;

	ret = regmap_read(regmap, reg - 1, &high);
	if (ret)
		return ret;
	ret = regmap_read(regmap, reg, &low);
	if (ret)
		return ret;

	source = high >> 4;
	if (source >= ARRAY_SIZE(resistance_lsb) || !resistance_lsb[source])
		return -ENODATA;

	raw = ((high & 0x0f) << 8) | low;
	if (!raw || raw == 0x0fff)
		return -ENODATA;

	/* PMIC LPAT uses resistance in ohms, temperatures in decikelvin. */
	return raw * resistance_lsb[source] / 1000;
}

static const struct intel_pmic_opregion_data intel_cht_wc_pmic_opregion_data = {
	.get_power		= intel_cht_wc_pmic_get_power,
	.update_power		= intel_cht_wc_pmic_update_power,
	.get_raw_temp		= intel_cht_wc_pmic_get_raw_temp,
	.thermal_table		= thermal_table,
	.thermal_table_count	= ARRAY_SIZE(thermal_table),
	.exec_mipi_pmic_seq_element = intel_cht_wc_exec_mipi_pmic_seq_element,
	.lpat_raw_to_temp	= acpi_lpat_raw_to_temp,
	.power_table		= power_table,
	.power_table_count	= ARRAY_SIZE(power_table),
};

static int intel_cht_wc_pmic_opregion_probe(struct platform_device *pdev)
{
	struct intel_soc_pmic *pmic = dev_get_drvdata(pdev->dev.parent);

	return intel_pmic_install_opregion_handler(&pdev->dev,
			ACPI_HANDLE(pdev->dev.parent),
			pmic->regmap,
			&intel_cht_wc_pmic_opregion_data);
}

static const struct platform_device_id cht_wc_opregion_id_table[] = {
	{ .name = "cht_wcove_region" },
	{},
};

static struct platform_driver intel_cht_wc_pmic_opregion_driver = {
	.probe = intel_cht_wc_pmic_opregion_probe,
	.driver = {
		.name = "cht_whiskey_cove_pmic",
	},
	.id_table = cht_wc_opregion_id_table,
};
builtin_platform_driver(intel_cht_wc_pmic_opregion_driver);
