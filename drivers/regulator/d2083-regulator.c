/*
* d2083-regulator.c: Regulator driver for Dialog D2083
*
* Copyright(c) 2011 Dialog Semiconductor Ltd.
*
* Author: Dialog Semiconductor Ltd. D. Chen
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/d2083/core.h>
#include <linux/d2083/pmic.h>
#include <linux/d2083/d2083_reg.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#if 0 //def CONFIG_MACH_RHEA_SS_IVORY
#include <linux/broadcom/pmu_chip.h>
#include <asm/uaccess.h>
#endif /* CONFIG_MACH_RHEA_SS_IVORY */

#ifdef CONFIG_KONA_AVS
#include <plat/kona_avs.h>
#endif

struct regl_register_map {
	u8 pm_reg;
	u8 mctl_reg;
	u8 dsm_opmode;
};

struct d2083_reg_info {
	u32 *v_table;
	u32 num_voltages;
};


#define DRIVER_NAME                 "d2083-regulator"

#define REGULATOR_REGISTER_ADJUST   (D2083_LDO13_REG - D2083_LDO12_REG - 1)             /* = 4 */
#define MCTL_REGISTER_ADJUST        (D2083_LDO1_MCTL_REG - D2083_BUCK4_MCTL_REG - 1)    /* = -25 */

#define mV_to_uV(v)                 ((v) * 1000)
#define uV_to_mV(v)                 ((v) / 1000)
#define MAX_MILLI_VOLT              (3300)
#define get_regulator_reg(n)        (regulator_register_map[n].pm_reg)
#define get_regulator_mctl_reg(n)   (regulator_register_map[n].mctl_reg)
#define get_regulator_dsm_opmode(n) (regulator_register_map[n].dsm_opmode)

#define D2083_DEFINE_REGL(_name, _pm_reg, _mctl_reg) \
	[D2083_##_name] = \
		{ \
			.pm_reg = _pm_reg, \
			.mctl_reg = _mctl_reg, \
		}

extern struct d2083 *d2083_regl_info;


static struct regl_register_map regulator_register_map[] = {
	D2083_DEFINE_REGL(BUCK_1,  D2083_BUCK1_TURBO_REG,   D2083_BUCK1_MCTL_REG  ), /* 0 */
	D2083_DEFINE_REGL(BUCK_2,  D2083_BUCK2_REG,   D2083_BUCK2_MCTL_REG  ), /* 1 */
	D2083_DEFINE_REGL(BUCK_3,  D2083_BUCK3_REG,   D2083_BUCK3_MCTL_REG  ), /* 2 */
	D2083_DEFINE_REGL(BUCK_4,  D2083_BUCK4_REG,   D2083_BUCK4_MCTL_REG  ), /* 3 */
	D2083_DEFINE_REGL(LDO_1,   D2083_LDO1_REG,    D2083_LDO1_MCTL_REG   ), /* 4 */
	D2083_DEFINE_REGL(LDO_2,   D2083_LDO2_REG,    D2083_LDO2_MCTL_REG   ), /* 5 */
	D2083_DEFINE_REGL(LDO_3,   D2083_LDO3_REG,    D2083_LDO3_MCTL_REG   ), /* 6 */
	D2083_DEFINE_REGL(LDO_4,   D2083_LDO4_REG,    D2083_LDO4_MCTL_REG   ), /* 7 */
	D2083_DEFINE_REGL(LDO_5,   D2083_LDO5_REG,    D2083_LDO5_MCTL_REG   ), /* 8 */
	D2083_DEFINE_REGL(LDO_6,   D2083_LDO6_REG,    D2083_LDO6_MCTL_REG   ), /* 9 */
	D2083_DEFINE_REGL(LDO_7,   D2083_LDO7_REG,    D2083_LDO7_MCTL_REG   ), /* 10 */
	D2083_DEFINE_REGL(LDO_8,   D2083_LDO8_REG,    D2083_LDO8_MCTL_REG   ), /* 11 */
	D2083_DEFINE_REGL(LDO_9,   D2083_LDO9_REG,    D2083_LDO9_MCTL_REG   ), /* 12 */
	D2083_DEFINE_REGL(LDO_10,  D2083_LDO10_REG,   D2083_LDO10_MCTL_REG  ), /* 13 */
	D2083_DEFINE_REGL(LDO_11,  D2083_LDO11_REG,   D2083_LDO11_MCTL_REG  ), /* 14 */
	D2083_DEFINE_REGL(LDO_12,  D2083_LDO12_REG,   D2083_LDO12_MCTL_REG  ), /* 15 */
	D2083_DEFINE_REGL(LDO_13,  D2083_LDO13_REG,   D2083_LDO13_MCTL_REG  ), /* 16 */
	D2083_DEFINE_REGL(LDO_14,  D2083_LDO14_REG,   D2083_LDO14_MCTL_REG  ), /* 17 */
	D2083_DEFINE_REGL(LDO_15,  D2083_LDO15_REG,   D2083_LDO15_MCTL_REG  ), /* 18 */
	D2083_DEFINE_REGL(LDO_16,  D2083_LDO16_REG,   D2083_LDO16_MCTL_REG  ), /* 19 */
	D2083_DEFINE_REGL(LDO_17,  D2083_LDO17_REG,   D2083_LDO17_MCTL_REG  ), /* 20 */
	D2083_DEFINE_REGL(LDO_AUD, D2083_LDO_AUD_REG, D2083_LDO_AUD_MCTL_REG)  /* 24 */
};

#define MICRO_VOLT(x)	((x)*1000L*1000L)
#define _DEFINE_REGL_VOLT(vol, regVal)	{ MICRO_VOLT(vol), regVal }

static const int core_vol_list[][2] = {
	_DEFINE_REGL_VOLT(0.500, 0),
	_DEFINE_REGL_VOLT(0.525, 1),
	_DEFINE_REGL_VOLT(0.550, 2),
	_DEFINE_REGL_VOLT(0.575, 3),
	_DEFINE_REGL_VOLT(0.600, 4),
	_DEFINE_REGL_VOLT(0.625, 5),
	_DEFINE_REGL_VOLT(0.650, 6),
	_DEFINE_REGL_VOLT(0.675, 7),
	_DEFINE_REGL_VOLT(0.700, 8),
	_DEFINE_REGL_VOLT(0.725, 9),
	_DEFINE_REGL_VOLT(0.750, 0xA),
	_DEFINE_REGL_VOLT(0.775, 0xB),
	_DEFINE_REGL_VOLT(0.800, 0xC),
	_DEFINE_REGL_VOLT(0.825, 0xD),
	_DEFINE_REGL_VOLT(0.850, 0xE),
	_DEFINE_REGL_VOLT(0.875, 0xF),
	_DEFINE_REGL_VOLT(0.900, 0x10),
	_DEFINE_REGL_VOLT(0.925, 0x11),
	_DEFINE_REGL_VOLT(0.950, 0x12),
	_DEFINE_REGL_VOLT(0.975, 0x13),
	_DEFINE_REGL_VOLT(1.000, 0x14),
	_DEFINE_REGL_VOLT(1.025, 0x15),
	_DEFINE_REGL_VOLT(1.050, 0x16),
	_DEFINE_REGL_VOLT(1.075, 0x17),
	_DEFINE_REGL_VOLT(1.100, 0x18),
	_DEFINE_REGL_VOLT(1.125, 0x19),
	_DEFINE_REGL_VOLT(1.150, 0x1A),
	_DEFINE_REGL_VOLT(1.175, 0x1B),
	_DEFINE_REGL_VOLT(1.200, 0x1C),
	_DEFINE_REGL_VOLT(1.225, 0x1D),
	_DEFINE_REGL_VOLT(1.250, 0x1E),
	_DEFINE_REGL_VOLT(1.275, 0x1F),
	_DEFINE_REGL_VOLT(1.300, 0x20),
	_DEFINE_REGL_VOLT(1.325, 0x21),
	_DEFINE_REGL_VOLT(1.350, 0x22),
	_DEFINE_REGL_VOLT(1.375, 0x23),
	_DEFINE_REGL_VOLT(1.400, 0x24),
	_DEFINE_REGL_VOLT(1.425, 0x25),
	_DEFINE_REGL_VOLT(1.450, 0x26),
	_DEFINE_REGL_VOLT(1.475, 0x27),
	_DEFINE_REGL_VOLT(1.500, 0x28),
	_DEFINE_REGL_VOLT(1.525, 0x29),
	_DEFINE_REGL_VOLT(1.550, 0x2A),
	_DEFINE_REGL_VOLT(1.575, 0x2B),
	_DEFINE_REGL_VOLT(1.600, 0x2C),
	_DEFINE_REGL_VOLT(1.625, 0x2D),
	_DEFINE_REGL_VOLT(1.650, 0x2E),
	_DEFINE_REGL_VOLT(1.675, 0x2F),
	_DEFINE_REGL_VOLT(1.700, 0x30),
	_DEFINE_REGL_VOLT(1.725, 0x31),
	_DEFINE_REGL_VOLT(1.750, 0x32),
	_DEFINE_REGL_VOLT(1.775, 0x33),
	_DEFINE_REGL_VOLT(1.800, 0x34),
	_DEFINE_REGL_VOLT(1.825, 0x35),
	_DEFINE_REGL_VOLT(1.850, 0x36),
	_DEFINE_REGL_VOLT(1.875, 0x37),
	_DEFINE_REGL_VOLT(1.900, 0x38),
	_DEFINE_REGL_VOLT(1.925, 0x39),
	_DEFINE_REGL_VOLT(1.950, 0x3A),
	_DEFINE_REGL_VOLT(1.975, 0x3B),
	_DEFINE_REGL_VOLT(2.000, 0x3C),
	_DEFINE_REGL_VOLT(2.025, 0x3D),
	_DEFINE_REGL_VOLT(2.050, 0x3E),
	_DEFINE_REGL_VOLT(2.075, 0x3F),
};

static const int ldo_vol_list[][2] = {
	_DEFINE_REGL_VOLT(1.200, 0),
	_DEFINE_REGL_VOLT(1.250, 1),
	_DEFINE_REGL_VOLT(1.300, 2),
	_DEFINE_REGL_VOLT(1.350, 3),
	_DEFINE_REGL_VOLT(1.400, 4),
	_DEFINE_REGL_VOLT(1.450, 5),
	_DEFINE_REGL_VOLT(1.500, 6),
	_DEFINE_REGL_VOLT(1.550, 7),
	_DEFINE_REGL_VOLT(1.600, 8),
	_DEFINE_REGL_VOLT(1.650, 9),
	_DEFINE_REGL_VOLT(1.700, 0xA),
	_DEFINE_REGL_VOLT(1.750, 0xB),
	_DEFINE_REGL_VOLT(1.800, 0xC),
	_DEFINE_REGL_VOLT(1.850, 0xD),
	_DEFINE_REGL_VOLT(1.900, 0xE),
	_DEFINE_REGL_VOLT(1.950, 0xF),
	
	_DEFINE_REGL_VOLT(2.000, 0x10),
	_DEFINE_REGL_VOLT(2.050, 0x11),
	_DEFINE_REGL_VOLT(2.100, 0x12),
	_DEFINE_REGL_VOLT(2.150, 0x13),
	_DEFINE_REGL_VOLT(2.200, 0x14),
	_DEFINE_REGL_VOLT(2.250, 0x15),
	_DEFINE_REGL_VOLT(2.300, 0x16),
	_DEFINE_REGL_VOLT(2.350, 0x17),
	_DEFINE_REGL_VOLT(2.400, 0x18),
	_DEFINE_REGL_VOLT(2.450, 0x19),
	_DEFINE_REGL_VOLT(2.500, 0x1A),
	_DEFINE_REGL_VOLT(2.550, 0x1B),
	_DEFINE_REGL_VOLT(2.600, 0x1C),
	_DEFINE_REGL_VOLT(2.650, 0x1D),
	_DEFINE_REGL_VOLT(2.700, 0x1E),
	_DEFINE_REGL_VOLT(2.750, 0x1F),
	_DEFINE_REGL_VOLT(2.800, 0x20),
	_DEFINE_REGL_VOLT(2.850, 0x21),
	_DEFINE_REGL_VOLT(2.900, 0x22),
	_DEFINE_REGL_VOLT(2.950, 0x23),
	
	_DEFINE_REGL_VOLT(3.000, 0x24),
	_DEFINE_REGL_VOLT(3.050, 0x25),
	_DEFINE_REGL_VOLT(3.100, 0x26),
	_DEFINE_REGL_VOLT(3.150, 0x27),
	_DEFINE_REGL_VOLT(3.200, 0x28),
	_DEFINE_REGL_VOLT(3.250, 0x29),
	_DEFINE_REGL_VOLT(3.330, 0x2A),
};

u32 d2083_ldo_v_table[] = {
	1200000,			/* 0x00 */
	1250000,			/* 0x01 */
	1300000,			/* 0x02 */
	1350000,			/* 0x03 */
	1400000,			/* 0x04 */
	1450000,			/* 0x05 */
	1500000,			/* 0x06 */
	1550000,			/* 0x07 */
	1600000,			/* 0x08 */
	1650000,			/* 0x09 */
	1700000,			/* 0x0A */
	1750000,			/* 0x0B */
	1800000,			/* 0x0C */
	1850000,			/* 0x0D */
	1900000,			/* 0x0E */
	1950000,			/* 0x0F */
	2000000,			/* 0x10 */
	2050000,			/* 0x11 */
	2100000,			/* 0x12 */
	2150000,			/* 0x13 */
	2200000,			/* 0x14 */
	2250000,			/* 0x15 */
	2300000,			/* 0x16 */
	2350000,			/* 0x17 */
	2400000,			/* 0x18 */
	2450000,			/* 0x19 */
	2500000,			/* 0x1A */
	2550000,			/* 0x1B */
	2600000,			/* 0x1C */
	2650000,			/* 0x1D */
	2700000,			/* 0x1E */
	2750000,			/* 0x1F */
	2800000,			/* 0x20 */
	2850000,			/* 0x21 */
	2900000,			/* 0x22 */
	2950000,			/* 0x23 */
	3000000,			/* 0x24 */
	3050000,			/* 0x25 */
	3100000,			/* 0x26 */
	3150000,			/* 0x27 */
	3200000,			/* 0x28 */
	3250000,			/* 0x29 */
	3300000,			/* 0x2A */
};

u32 d2083_buck12_v_table[] = {
	500000,			/* 0x00 */
	525000,			/* 0x01 */
	550000,			/* 0x02 */
	575000,			/* 0x03 */
	600000,			/* 0x04 */
	625000,			/* 0x05 */
	650000,			/* 0x06 */
	675000,			/* 0x07 */
	700000,			/* 0x08 */
	725000,			/* 0x09 */
	750000,			/* 0x0A */
	775000,			/* 0x0B */
	800000,			/* 0x0C */
	825000,			/* 0x0D */
	850000,			/* 0x0E */
	875000,			/* 0x0F */
	900000,			/* 0x10 */
	925000,			/* 0x11 */
	950000,			/* 0x12 */
	975000,			/* 0x13 */
	1000000,			/* 0x14 */
	1025000,			/* 0x15 */
	1050000,			/* 0x16 */
	1075000,			/* 0x17 */
	1100000,			/* 0x18 */
	1125000,			/* 0x19 */
	1150000,			/* 0x1A */
	1175000,			/* 0x1B */
	1200000,			/* 0x1C */
	1225000,			/* 0x1D */
	1250000,			/* 0x1E */
	1275000,			/* 0x1F */
	1300000,			/* 0x20 */
	1325000,			/* 0x21 */
	1350000,			/* 0x22 */
	1375000,			/* 0x23 */
	1400000,			/* 0x24 */
	1425000,			/* 0x25 */
	1450000,			/* 0x26 */
	1475000,			/* 0x27 */
	1500000,			/* 0x28 */
	1525000,			/* 0x29 */
	1550000,			/* 0x2A */
	1575000,			/* 0x2B */
	1600000,			/* 0x2C */
	1625000,			/* 0x2D */
	1650000,			/* 0x2E */
	1675000,			/* 0x2F */
	1700000,			/* 0x30 */
	1725000,			/* 0x31 */
	1750000,			/* 0x32 */
	1775000,			/* 0x33 */
	1800000,			/* 0x34 */
	1825000,			/* 0x35 */
	1850000,			/* 0x36 */
	1875000,			/* 0x37 */
	1900000,			/* 0x38 */
	1925000,			/* 0x39 */
	1950000,			/* 0x3A */
	1975000,			/* 0x3B */
	2000000,			/* 0x3C */
	2025000,			/* 0x3D */
	2050000,			/* 0x3E */
	2075000,			/* 0x3F */
};

u32 d2083_buck3_v_table[] = {
	925000,			/* 0x00 */
	950000,			/* 0x01 */
	975000,			/* 0x02 */
	1000000,			/* 0x03 */
	1025000,			/* 0x04 */
	1050000,			/* 0x05 */
	1075000,			/* 0x06 */
	1100000,			/* 0x07 */
	1125000,			/* 0x08 */
	1150000,			/* 0x09 */
	1175000,			/* 0x0A */
	1200000,			/* 0x0B */
	1225000,			/* 0x0C */
	1250000,			/* 0x0D */
	1275000,			/* 0x0E */
	1300000,			/* 0x0F */
	1325000,			/* 0x10 */
	1350000,			/* 0x11 */
	1375000,			/* 0x12 */
	1400000,			/* 0x13 */
	1425000,			/* 0x14 */
	1450000,			/* 0x15 */
	1475000,			/* 0x16 */
	1500000,			/* 0x17 */
	1525000,			/* 0x18 */
	1550000,			/* 0x19 */
	1575000,			/* 0x1A */
	1600000,			/* 0x1B */
	1625000,			/* 0x1C */
	1650000,			/* 0x1D */
	1675000,			/* 0x1E */
	1700000,			/* 0x1F */
	1725000,			/* 0x20 */
	1750000,			/* 0x21 */
	1775000,			/* 0x22 */
	1800000,			/* 0x23 */
	1825000,			/* 0x24 */
	1850000,			/* 0x25 */
	1875000,			/* 0x26 */
	1900000,			/* 0x27 */
	1925000,			/* 0x28 */
	1950000,			/* 0x29 */
	1975000,			/* 0x2A */
	2000000,			/* 0x2B */
	2025000,			/* 0x2C */
	2050000,			/* 0x2D */
	2075000,			/* 0x2E */
	
	2100000,			/* 0x2F */
	2125000,			/* 0x30 */
	2150000,			/* 0x31 */
	2175000,			/* 0x32 */
	2200000,			/* 0x33 */
	2225000,			/* 0x34 */
	2250000,			/* 0x35 */
	2275000,			/* 0x36 */
	2300000,			/* 0x37 */
	2325000,			/* 0x38 */
	2350000,			/* 0x39 */
	2375000,			/* 0x3A */
	2400000,			/* 0x3B */
	2425000,			/* 0x3C */
	2450000,			/* 0x3D */
	2475000,			/* 0x3E */
	2500000,			/* 0x3F */

};


u32 d2083_buck4_v_table[] = {
	1200000,			/* 0x00 */
	1225000,			/* 0x01 */
	1250000,			/* 0x02 */
	1275000,			/* 0x03 */
	1300000,			/* 0x04 */
	1325000,			/* 0x05 */
	1350000,			/* 0x06 */
	1375000,			/* 0x07 */
	1400000,			/* 0x08 */
	1425000,			/* 0x09 */
	1450000,			/* 0x0A */
	1475000,			/* 0x0B */
	1500000,			/* 0x0C */
	1525000,			/* 0x0D */
	1550000,			/* 0x0E */
	1575000,			/* 0x0F */
	1600000,			/* 0x10 */
	1625000,			/* 0x11 */
	1650000,			/* 0x12 */
	1675000,			/* 0x13 */
	1700000,			/* 0x14 */
	1725000,			/* 0x15 */
	1750000,			/* 0x16 */
	1775000,			/* 0x17 */
	1800000,			/* 0x18 */
	1825000,			/* 0x19 */
	1850000,			/* 0x1A */
	1875000,			/* 0x1B */
	1900000,			/* 0x1C */
	1925000,			/* 0x1D */
	1950000,			/* 0x1E */
	1975000,			/* 0x1F */
	2000000,			/* 0x20 */
	2025000,			/* 0x21 */
	2050000,			/* 0x22 */
	2075000,			/* 0x23 */
	2100000,			/* 0x24 */
	2125000,			/* 0x25 */
	2150000,			/* 0x26 */
	2175000,			/* 0x27 */
	2200000,			/* 0x28 */
	2225000,			/* 0x29 */
	2250000,			/* 0x2A */
	2275000,			/* 0x2B */
	2300000,			/* 0x2C */
	2325000,			/* 0x2D */
	2350000,			/* 0x2E */
	2375000,			/* 0x2F */
	2400000,			/* 0x30 */
	2425000,			/* 0x31 */
	2450000,			/* 0x32 */
	2475000,			/* 0x33 */
	2500000,			/* 0x34 */
	2525000,			/* 0x35 */
	2550000,			/* 0x36 */
	2575000,			/* 0x37 */
	2600000,			/* 0x38 */
	2625000,			/* 0x39 */
	2650000,			/* 0x3A */
	2675000,			/* 0x3B */
	2700000,			/* 0x3C */
	2725000,			/* 0x3D */
	2750000,			/* 0x3E */
	2775000,			/* 0x3F */
	2800000,			/* 0x40 */
	2825000,			/* 0x41 */
	2850000,			/* 0x42 */
	2875000,			/* 0x43 */
	2900000,			/* 0x44 */
	2925000,			/* 0x45 */
	2950000,			/* 0x46 */
	2975000,			/* 0x47 */
	3000000,			/* 0x48 */
	3025000,			/* 0x49 */
	3050000,			/* 0x4A */
	3075000,			/* 0x4B */
	3100000,			/* 0x4C */
	3125000,			/* 0x4D */
	3150000,			/* 0x4E */
	3175000,			/* 0x4F */
	3200000,			/* 0x50 */
	3225000,			/* 0x51 */
	3250000,			/* 0x52 */
	3275000,			/* 0x53 */
	3300000,			/* 0x54 */
};


static struct d2083_reg_info d2083_regulator_info[D2083_NUMBER_OF_REGULATORS] = {
	[D2083_BUCK_1] = {
		.v_table = d2083_buck12_v_table,
		.num_voltages = ARRAY_SIZE(d2083_buck12_v_table),
	},
	[D2083_BUCK_2] = {
		.v_table = d2083_buck12_v_table,
		.num_voltages = ARRAY_SIZE(d2083_buck12_v_table),
	},
	[D2083_BUCK_3] = {
		.v_table = d2083_buck3_v_table,
		.num_voltages = ARRAY_SIZE(d2083_buck3_v_table),
	},
	[D2083_BUCK_4] = {
		.v_table = d2083_buck4_v_table,
		.num_voltages = ARRAY_SIZE(d2083_buck4_v_table),
	},

	[D2083_LDO_1] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_2] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_3] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_4] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_5] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_6] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_7] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_8] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_9] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_10] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_11] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_12] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_13] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_14] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_15] = {
	.v_table = d2083_ldo_v_table,
	.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_16] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_17] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},
	[D2083_LDO_AUD] = {
		.v_table = d2083_ldo_v_table,
		.num_voltages = ARRAY_SIZE(d2083_ldo_v_table),
	},

};



#if 1   // Samsung spec-out issue for regulator enable/disable time -> speed up
static int mctl_status=0;   // default is disable

static int is_mode_control_enabled(void)
{
	// 0 : mctl disable, 1 : mctl enable
	return mctl_status;
}
#else
static inline bool is_mode_control_enabled(struct d2083 *d2083)
{
    u8 reg_val;
	bool ret;

    d2083_reg_read(d2083, D2083_POWERCONT_REG, &reg_val);
	ret = (reg_val & D2083_POWERCONT_MCTRLEN);
    
    return ret;
}
#endif

//Needed for sysparms
static struct regulator_dev *d2083_rdev[D2083_NUMBER_OF_REGULATORS];

static inline int get_global_mctl_mode(struct d2083 *d2083)
{
	u8 reg_val;

	d2083_reg_read(d2083, D2083_STATUSA_REG, &reg_val);

	// Remove "NOT" operation
	return ((reg_val & D2083_STATUSA_MCTL) >> D2083_STATUSA_MCTL_SHIFT);
}


static unsigned int get_regulator_mctl_mode(struct d2083 *d2083, int regulator_id)
{
	//TODO: Make sure we can assume we have always current value in the buffer and do this:
	u8 reg_val, regulator_mctl_reg = get_regulator_mctl_reg(regulator_id);
	int ret = 0;

	ret = d2083_reg_read(d2083, regulator_mctl_reg, &reg_val);
	reg_val &= D2083_REGULATOR_MCTL0;
	reg_val >>= MCTL0_SHIFT;

	return reg_val;
}


static int set_regulator_mctl_mode(struct d2083 *d2083, int regulator_id, u8 mode)
{
	u8 reg_val, mctl_reg;
	int ret = 0;

	if(regulator_id < 0 || regulator_id >= D2083_NUMBER_OF_REGULATORS)
		return -EINVAL;
	if(mode > REGULATOR_MCTL_TURBO)
		return -EINVAL;

	mctl_reg = get_regulator_mctl_reg(regulator_id);
	ret = d2083_reg_read(d2083, mctl_reg, &reg_val);
	if(ret < 0)
		return ret;

	reg_val &= ~(D2083_REGULATOR_MCTL0 | D2083_REGULATOR_MCTL2);
	reg_val |= ((mode << MCTL0_SHIFT) | ( mode << MCTL2_SHIFT));
	ret = d2083_reg_write(d2083, mctl_reg, reg_val);
	dlg_info("[REGULATOR] %s. reg_val = 0x%X\n", __func__, reg_val);

	return ret;
}


static int d2083_register_regulator(struct d2083 *d2083, int reg,
								struct regulator_init_data *initdata);

/*
void dump_MCTL_REG(void)
{
	u8 reg_val;
	int i;

	dlg_info("BUCK / LDO reg\n");
	for(i=D2083_BUCK1_REG; i<=D2083_LDO_AUD_REG; i++){
		d2083_reg_read(d2083_regl_info,i,&reg_val);
		dlg_info("addr[0x%x], val[0x%x]\n", i, reg_val);
	}

	dlg_info("MCTL reg\n");
	for(i=D2083_LDO1_MCTL_REG; i<=D2083_BUCK1_TURBO_REG; i++){
		d2083_reg_read(d2083_regl_info,i,&reg_val);
		dlg_info("addr[0x%x], val[0x%x]\n", i, reg_val);
	}
}
EXPORT_SYMBOL(dump_MCTL_REG);
*/

void set_MCTL_enabled(void)
{
	u8 reg_val;

	if(d2083_regl_info==NULL){
		dlg_err("MCTRL_EN bit is not set\n");
		return;
	}
		
	d2083_reg_read(d2083_regl_info, D2083_POWERCONT_REG, &reg_val);
	reg_val |= D2083_POWERCONT_MCTRLEN;
	d2083_reg_write(d2083_regl_info, D2083_POWERCONT_REG, reg_val);

	mctl_status=1;
}
EXPORT_SYMBOL(set_MCTL_enabled);



#if 1	// Normal voltage setting for sleep
#define CSR_VAL_ECO_SS_1G		0x18 /*1.08V -> 1.10V*/
#define CSR_VAL_ECO_TT_1G		0x14 /*0.98V -> 1.00V*/
#define CSR_VAL_ECO_FF_1G		0x14 /*0.98V -> 1.00V*/

void d2083_set_sType(u32 s_type)
{
	u8 val;
	u8 mV_val = 0x00;

	if(s_type == SILICON_TYPE_SLOW)
		mV_val = CSR_VAL_ECO_SS_1G;
	else if(s_type == SILICON_TYPE_TYPICAL)
		mV_val = CSR_VAL_ECO_TT_1G;
	else if(s_type == SILICON_TYPE_FAST)
		mV_val = CSR_VAL_ECO_FF_1G;
	else
		mV_val = CSR_VAL_ECO_SS_1G;

	d2083_reg_read(d2083_regl_info, D2083_BUCK1_REG, &val);
	val &= ~D2083_MAX_VSEL;
	d2083_reg_write(d2083_regl_info, D2083_BUCK1_REG, (val | mV_val));
	

	d2083_reg_read(d2083_regl_info, D2083_BUCK1_REG, &val);
	printk("Buck1 normal --> s_type = [%d], read reg[0x%x] <--, mV_val[0x%x]\n", 
			s_type, val, mV_val);
}

EXPORT_SYMBOL(d2083_set_sType);
#endif	// Normal voltage setting for sleep


int __init d2083_platform_regulator_init(struct d2083 *d2083)
{
	int i;
	u8 reg_val=0;

#if 1	//todo
	struct d2083_regl_init_data *regl_data = d2083->pdata->regulator_data;

	if(regl_data == NULL)
		return -1;

	for(i = D2083_BUCK_1; i < D2083_NUMBER_OF_REGULATORS; i++) {
		d2083_register_regulator(d2083, i, (regl_data + i)->initdata);
		
		regulator_register_map[i].dsm_opmode = d2083->pdata->regl_map[i].default_pm_mode;

		if(i >= D2083_BUCK_1) {
			d2083_reg_write(d2083, get_regulator_mctl_reg(i),
			d2083->pdata->regl_map[i].dsm_opmode);
		}
	}
#else
	for(i = D2083_BUCK_1; i < D2083_NUMBER_OF_REGULATORS; i++) {
		if (d2083->pdata != NULL &&
		d2083->pdata->regulator_data != NULL &&
		d2083->pdata->regulator_data[i].supply != NULL) {

			d2083_regulators_init_data[i].consumer_supplies = &d2083->pdata->regulator_data[i];
			d2083_regulators_init_data[i].num_consumer_supplies = 1;

			dev_info(d2083->dev, "No consumers for regulator %s\n",
			d2083_regulators_init_data[i].constraints.name);
		}

		d2083_register_regulator(d2083, i, &(d2083_regulators_init_data[i]));
		regulator_register_map[i].dsm_opmode = d2083->pdata->regl_map[i].default_pm_mode;

		if(i >= D2083_BUCK_1) {
			d2083_reg_write(d2083, get_regulator_mctl_reg(i),
			d2083->pdata->regl_map[i].dsm_opmode);
		}
	}
#endif

	// ******************************************************************************

	// set MISC_MCTL
	/*
	reg_val = (D2083_MISC_MCTL3_DIGICLK | D2083_MISC_MCTL2_DIGICLK |
	D2083_MISC_MCTL1_DIGICLK | D2083_MISC_MCTL0_DIGICLK |
	D2083_MISC_MCTL3_BBAT | D2083_MISC_MCTL2_BBAT |
	D2083_MISC_MCTL1_BBAT | D2083_MISC_MCTL0_BBAT);
	*/
	reg_val = 0x0F;
	d2083_reg_write(d2083_regl_info, D2083_MISC_MCTL_REG, reg_val);

	// 	GPADC MTCL default
	reg_val = 0x55;	// 0x55 -> 01 | 01 | 01 | 01 (01 -> On if already enabled)
	d2083_reg_write(d2083_regl_info, D2083_GPADC_MCTL_REG, reg_val);

	d2083_reg_write(d2083_regl_info, D2083_BUCK1_TURBO_REG, D2083_BUCK1_NM_VOLT);
	d2083_reg_write(d2083_regl_info, D2083_BUCK1_RETENTION_REG, D2083_BUCK1_RETENTION_VOLT);

	d2083_reg_write(d2083_regl_info, D2083_BUCK3_REG, 0x4C);	// BUCK3 : 1.23v -> 1.225v

	// set AUD_LDO to 2.5v
	d2083_reg_write(d2083_regl_info, D2083_LDO_AUD_REG, 0x5A);
	// set LDO to 1.8v
	d2083_reg_write(d2083_regl_info, D2083_LDO1_REG, 0x4C);

	// sim_vcc(LDO11) -> default 1.8v
	//d2083_reg_write(d2083_regl_info, D2083_LDO11_REG, 0xC);

	// LDO_AUD pull-down disable
	d2083_reg_write(d2083_regl_info, D2083_PULLDOWN_REG_D, (1<<0));

	// ******************************************************************************

	return 0;
}



static int d2083_regulator_val_to_mvolts(unsigned int val, int regulator_id, struct regulator_dev *rdev)
{
	struct regulation_constraints *constraints;
	int min_mV, mvolts;

	constraints =  rdev->constraints;
	min_mV = uV_to_mV(constraints->min_uV);

	if ( (regulator_id >= D2083_BUCK_1) && (regulator_id <= D2083_BUCK_4) )
		return ((val * D2083_BUCK_VOLT_STEP) + min_mV );

	mvolts = (val * D2083_LDO_VOLT_STEP) + min_mV;
	if(mvolts > MAX_MILLI_VOLT)
		mvolts = MAX_MILLI_VOLT;

	return mvolts;
}


static unsigned int d2083_regulator_mvolts_to_val(int mV, int regulator_id, struct regulator_dev *rdev)
{
	struct regulation_constraints *constraints =  rdev->constraints;
	int min_mV = uV_to_mV(constraints->min_uV);

	if ( (regulator_id >= D2083_BUCK_1) && (regulator_id <= D2083_BUCK_4) )
		return ((mV - min_mV) / D2083_BUCK_VOLT_STEP);

	return ((mV - min_mV) / D2083_LDO_VOLT_STEP);
}

static int d2083_regulator_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV,unsigned *selector)
{
	struct d2083 *d2083 = rdev_get_drvdata(rdev);
	int mV_val;
	int min_mV = uV_to_mV(min_uV);
	int max_mV = uV_to_mV(max_uV);
	unsigned int reg_num, regulator_id = rdev_get_id(rdev);
	u8 val;	
	int ret = 0;
	*selector = -1;

	// Neet to implement parameter *selector : reference v_table / num_voltages

	/* before we do anything check the lock bit */
	ret = d2083_reg_read(d2083, D2083_SUPPLY_REG, &val);
	if(val & D2083_SUPPLY_VLOCK)
		d2083_clear_bits(d2083, D2083_SUPPLY_REG, D2083_SUPPLY_VLOCK);

	mV_val =  d2083_regulator_mvolts_to_val(min_mV, regulator_id, rdev);

	/* Sanity check for maximum value */
	if (d2083_regulator_val_to_mvolts(mV_val, regulator_id, rdev) > max_mV)
		return -EINVAL;

	reg_num = get_regulator_reg(regulator_id);

	ret = d2083_reg_read(d2083, reg_num, &val);
	val &= ~D2083_MAX_VSEL;
	
	d2083_reg_write(d2083, reg_num, (val | mV_val));

	/* For BUCKs enable the ramp */
	if (regulator_id <= D2083_BUCK_4)
		d2083_set_bits(d2083, D2083_SUPPLY_REG, (D2083_SUPPLY_VBUCK1GO << regulator_id));

	*selector = regulator_id;
	
	return ret;
}
//EXPORT_SYMBOL_GPL(d2083_regulator_set_voltage);


static int d2083_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct d2083 *d2083 = rdev_get_drvdata(rdev);
	unsigned int reg_num, regulator_id = rdev_get_id(rdev);
	int ret;
	u8 val;

	dlg_info("[[%s]], regulator_id[%d]\n", __func__, regulator_id);


	reg_num = get_regulator_reg(regulator_id);
	ret = d2083_reg_read(d2083, reg_num, &val);
	val &= D2083_MAX_VSEL;
	ret = mV_to_uV(d2083_regulator_val_to_mvolts(val, regulator_id, rdev));

	return ret;
}
//EXPORT_SYMBOL_GPL(d2083_regulator_get_voltage);


static int d2083_regulator_enable(struct regulator_dev *rdev)
{
	struct d2083 *d2083 = rdev_get_drvdata(rdev);
	u8 reg_val;
	int ret = 0;
	unsigned int regulator_id = rdev_get_id(rdev);
	unsigned int reg_num;

	if (regulator_id >= D2083_NUMBER_OF_REGULATORS)
		return -EINVAL;

	if(!is_mode_control_enabled()){
		reg_num = get_regulator_reg(regulator_id);

		d2083_reg_read(d2083, reg_num, &reg_val);
		reg_val |= (1<<6);
		ret = d2083_reg_write(d2083, reg_num,reg_val);
	}else {
		reg_num = get_regulator_mctl_reg(regulator_id);

		ret = d2083_reg_read(d2083, reg_num, &reg_val);
		if(ret < 0) {
			dlg_err("I2C read error\n");
			return ret;
		}

		reg_val &= ~(D2083_REGULATOR_MCTL1 | D2083_REGULATOR_MCTL2 | D2083_REGULATOR_MCTL3);
		reg_val |= (D2083_REGULATOR_MCTL1_ON | D2083_REGULATOR_MCTL2_ON | D2083_REGULATOR_MCTL3_ON);

		switch(get_regulator_dsm_opmode(regulator_id)) {
		case D2083_REGULATOR_LPM_IN_DSM :
			reg_val &= ~(D2083_REGULATOR_MCTL0);
			reg_val |= (D2083_REGULATOR_MCTL0_SLEEP);
			break;
		case D2083_REGULATOR_OFF_IN_DSM :
			reg_val &= ~(D2083_REGULATOR_MCTL0);
			break;
		case D2083_REGULATOR_ON_IN_DSM :
			reg_val &= ~(D2083_REGULATOR_MCTL0);
			reg_val |= (D2083_REGULATOR_MCTL0_ON);
			break;
		}

		ret |= d2083_reg_write(d2083, reg_num, reg_val);
	}

	return ret;
}
//EXPORT_SYMBOL_GPL(d2083_regulator_enable);


static int d2083_regulator_disable(struct regulator_dev *rdev)
{
	struct d2083 *d2083 = rdev_get_drvdata(rdev);
	unsigned int regulator_id = rdev_get_id(rdev);
	unsigned int reg_num = 0;
	int ret = 0;
	u8 reg_val;

	if (regulator_id >= D2083_NUMBER_OF_REGULATORS)
		return -EINVAL;

	if(!is_mode_control_enabled()) {
		reg_num = get_regulator_reg(regulator_id);

		d2083_reg_read(d2083, reg_num, &reg_val);
		reg_val &= ~(1<<6);
		d2083_reg_write(d2083, reg_num, reg_val);
	} else {
		reg_num = get_regulator_mctl_reg(regulator_id);
		/* 0x00 ==  D2083_REGULATOR_MCTL0_OFF | D2083_REGULATOR_MCTL1_OFF 
		*        | D2083_REGULATOR_MCTL2_OFF | D2083_REGULATOR_MCTL3_OFF 
		*/

		ret = d2083_reg_write(d2083, reg_num, 0x00);
	}

	return ret;
}
//EXPORT_SYMBOL_GPL(d2083_regulator_disable);


static unsigned int d2083_regulator_get_mode(struct regulator_dev *rdev)
{
	struct d2083 *d2083 = rdev_get_drvdata(rdev);
	unsigned int regulator_id = rdev_get_id(rdev);
	unsigned int mode = 0;

	if (regulator_id >= D2083_NUMBER_OF_REGULATORS)
		return -EINVAL;

	mode = get_regulator_mctl_mode(d2083, regulator_id);

	/* Map d2083 regulator mode to Linux framework mode */
	switch(mode) {
	case REGULATOR_MCTL_TURBO:
		mode = REGULATOR_MODE_FAST;
		break;
	case REGULATOR_MCTL_ON:
		mode = REGULATOR_MODE_NORMAL;
		break;
	case REGULATOR_MCTL_SLEEP:
		mode = REGULATOR_MODE_IDLE;
		break;
	case REGULATOR_MCTL_OFF:
		mode = REGULATOR_MODE_STANDBY;
		break;
	default:
		/* unsupported or unknown mode */
		break;
	}

	dlg_info("[REGULATOR] : [%s] >> MODE(%d)\n", __func__, mode);

	return mode;
}
//EXPORT_SYMBOL_GPL(d2083_regulator_get_mode);



/* TODO MW: remove set function ? - mode is HW controled in mctl, we cannot set, what about normal mode ?  */
/*
TODO
Set mode is it software controlled?
if yes then following implementation is useful ??
if no then we dont need following implementation ??
*/
static int d2083_regulator_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct d2083 *d2083 = rdev_get_drvdata(rdev);
	unsigned int regulator_id = rdev_get_id(rdev);
	int ret;
	u8 mctl_mode;

	dlg_info("[REGULATOR] : regulator_set_mode. mode is %d\n", mode);

	switch(mode) {
	case REGULATOR_MODE_FAST :
		mctl_mode = REGULATOR_MCTL_TURBO;
		break;
	case REGULATOR_MODE_NORMAL :
		mctl_mode = REGULATOR_MCTL_ON;
		break;
	case REGULATOR_MODE_IDLE :
		mctl_mode = REGULATOR_MCTL_SLEEP;
		break;
	case REGULATOR_MODE_STANDBY:
		mctl_mode = REGULATOR_MCTL_OFF;
		break;
	default:
		return -EINVAL;
	}

	ret = set_regulator_mctl_mode(d2083, regulator_id, mctl_mode);

	return ret;
}
//EXPORT_SYMBOL_GPL(d2083_regulator_set_mode);


static int d2083_regulator_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	//struct d2083 *d2083 = rdev_get_drvdata(rdev);
	unsigned int regulator_id = rdev_get_id(rdev);

/*
	dlg_info("[REGULATOR][%s] regulator_id[%d], selector[%d]\n", 
					__func__, regulator_id, selector);
*/
	if(regulator_id >= D2083_NUMBER_OF_REGULATORS)
		return -EINVAL;
/*
	dlg_info("[REGULATOR] num_voltage[%d], selector[%d]\n", 
			d2083_regulator_info[regulator_id].num_voltages, 
			d2083_regulator_info[regulator_id].v_table[selector]);
*/
	if(selector >= d2083_regulator_info[regulator_id].num_voltages)
		return -EINVAL;

	return d2083_regulator_info[regulator_id].v_table[selector];
					
}


static int d2083_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct d2083 *d2083 = rdev_get_drvdata(rdev);
	unsigned int reg_num, regulator_id = rdev_get_id(rdev);
	int ret = -EINVAL;
	u8 reg_val = 0;

	if(!is_mode_control_enabled()){

		reg_num = get_regulator_reg(regulator_id);

		ret = d2083_reg_read(d2083, reg_num, &reg_val);
		if(ret < 0) {
			dlg_err("I2C read error. \n");
			return ret;
		}

		/* 0x0 : off, 0x1 : on */
		ret = reg_val & (1<<6);
	} else {
		if (regulator_id >= D2083_NUMBER_OF_REGULATORS)
			return -EINVAL;

		reg_num = get_regulator_mctl_reg(regulator_id);
		ret = d2083_reg_read(d2083, reg_num, &reg_val);
		if(ret < 0) {
			dlg_err("I2C read error. \n");
			return ret;
		}

		/* 0x0 : Off    * 0x1 : On    * 0x2 : Sleep    * 0x3 : n/a */
		ret = ((reg_val & (D2083_REGULATOR_MCTL1|D2083_REGULATOR_MCTL3)) >= 1) ? 1 : 0;
	}

	return ret;
}
//EXPORT_SYMBOL_GPL(d2083_regulator_is_enabled);


static struct regulator_ops d2083_ldo_ops = {
	.set_voltage = d2083_regulator_set_voltage,
	.get_voltage = d2083_regulator_get_voltage,
	.enable = d2083_regulator_enable,
	.disable = d2083_regulator_disable,
	.list_voltage = d2083_regulator_list_voltage,
	.is_enabled = d2083_regulator_is_enabled,
	.get_mode = d2083_regulator_get_mode,
	.set_mode = d2083_regulator_set_mode,
};


static struct regulator_desc d2083_reg[D2083_NUMBER_OF_REGULATORS] = {
	{
		.name = "D2083_BUCK_1",
		.id = D2083_BUCK_1,
		.n_voltages = ARRAY_SIZE(d2083_buck12_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_BUCK_2",
		.id = D2083_BUCK_2,
		.n_voltages = ARRAY_SIZE(d2083_buck12_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_BUCK_3",
		.id = D2083_BUCK_3,
		.n_voltages = ARRAY_SIZE(d2083_buck3_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_BUCK_4",
		.id = D2083_BUCK_4,
		.n_voltages = ARRAY_SIZE(d2083_buck4_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_1",
		.id = D2083_LDO_1,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_2",
		.id = D2083_LDO_2,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_3",
		.id = D2083_LDO_3,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_4",
		.id = D2083_LDO_4,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_5",
		.id = D2083_LDO_5,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_6",
		.id = D2083_LDO_6,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_7",
		.id = D2083_LDO_7,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_8",
		.id = D2083_LDO_8,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_9",
		.id = D2083_LDO_9,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_10",
		.id = D2083_LDO_10,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_11",
		.id = D2083_LDO_11,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_12",
		.id = D2083_LDO_12,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_13",
		.id = D2083_LDO_13,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_14",
		.id = D2083_LDO_14,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_15",
		.id = D2083_LDO_15,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_16",
		.id = D2083_LDO_16,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_17",
		.id = D2083_LDO_17,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "D2083_LDO_AUD",
		.id = D2083_LDO_AUD,
		.n_voltages = ARRAY_SIZE(d2083_ldo_v_table),
		.ops = &d2083_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	}
};




#if 0 //def CONFIG_MACH_RHEA_SS_IVORY 
/****************************************************************/
/*		Regulators IOCTL debugging			*/
/****************************************************************/


int d2083_ioctl_regulator(struct d2083 *d2083, unsigned int cmd, unsigned long arg)
{
	//struct max8986_regl_priv *regl_priv = (struct max8986_regl_priv *) pri_data;
	int ret = -EINVAL;
	int regid;

	dlg_info("Inside %s, IOCTL command is %d\n", __func__, cmd);
	switch (cmd) {
	case BCM_PMU_IOCTL_SET_VOLTAGE:
	{
		pmu_regl_volt regulator;
		if (copy_from_user(&regulator, (pmu_regl_volt *)arg,
			sizeof(pmu_regl_volt)) != 0) 
		{
			return -EFAULT;
		}

		if (regulator.regl_id < 0 
			|| regulator.regl_id >= ARRAY_SIZE(d2083->pdata->regl_map))
			return -EINVAL;
		regid = d2083->pdata->regl_map[regulator.regl_id].reg_id;
		if (regid < 0) 
			return -EINVAL;

		ret = d2083_regulator_set_voltage(d2083_rdev[regid],
		regulator.voltage, regulator.voltage);
		break;
	}
	case BCM_PMU_IOCTL_GET_VOLTAGE:
	{
		pmu_regl_volt regulator;
		int volt;
		if (copy_from_user(&regulator, (pmu_regl_volt *)arg,
		sizeof(pmu_regl_volt)) != 0) 
		{
			return -EFAULT;
		}
		if (regulator.regl_id < 0 
			|| regulator.regl_id >= ARRAY_SIZE(d2083->pdata->regl_map))
			return -EINVAL;
		regid = d2083->pdata->regl_map[regulator.regl_id].reg_id;
		if (regid < 0) 
			return -EINVAL;

		volt = d2083_regulator_get_voltage(d2083_rdev[regid]);
		if (volt > 0) {
			regulator.voltage = volt;
			ret = copy_to_user((pmu_regl_volt *)arg, &regulator,
			sizeof(regulator));
		} else {
			ret = volt;
		}
		break;
	}
	case BCM_PMU_IOCTL_GET_REGULATOR_STATE:
	{
		pmu_regl rmode;
		unsigned int mode;
		if (copy_from_user(&rmode, (pmu_regl *)arg, 
				sizeof(pmu_regl)) != 0)
			return -EFAULT;

		if (rmode.regl_id < 0 
				|| rmode.regl_id >= ARRAY_SIZE(d2083->pdata->regl_map))
			return -EINVAL;
		regid = d2083->pdata->regl_map[rmode.regl_id].reg_id;
		if (regid < 0) 
			return -EINVAL;

		mode = d2083_regulator_get_mode(d2083_rdev[regid]);
		switch (mode) {
		case REGULATOR_MODE_FAST:
			rmode.state = PMU_REGL_TURBO;
			break;
		case REGULATOR_MODE_NORMAL:
			rmode.state = PMU_REGL_ON;
			break;
		case REGULATOR_MODE_STANDBY:
			rmode.state = PMU_REGL_ECO;
			break;
		default:
			rmode.state = PMU_REGL_OFF;
			break;
		};
		ret = copy_to_user((pmu_regl *)arg, &rmode, sizeof(rmode));
		break;
	}
	case BCM_PMU_IOCTL_SET_REGULATOR_STATE:
	{
		pmu_regl rmode;
		unsigned int mode;
		if (copy_from_user(&rmode, (pmu_regl *)arg,
				sizeof(pmu_regl)) != 0)
			return -EFAULT;
		if (rmode.regl_id < 0 
				|| rmode.regl_id >= ARRAY_SIZE(d2083->pdata->regl_map))
			return -EINVAL;
		regid = d2083->pdata->regl_map[rmode.regl_id].reg_id;
		if (regid < 0) 
			return -EINVAL;

		switch (rmode.state) {
		case PMU_REGL_TURBO:
			mode = REGULATOR_MODE_FAST;
			break;
		case PMU_REGL_ON:
			mode = REGULATOR_MODE_NORMAL;
			break;
		case PMU_REGL_ECO:
			mode = REGULATOR_MODE_STANDBY;
			break;
		default:
			mode = REGULATOR_MCTL_OFF;
			break;
		};
		ret = d2083_regulator_set_mode(d2083_rdev[regid], mode);
		break;
	}
	case BCM_PMU_IOCTL_ACTIVATESIM:
	{
		int id = 16;	/*check the status of SIMLDO*/
		pmu_sim_volt sim_volt;
		int value;
		if (copy_from_user(&sim_volt, (int *)arg, sizeof(int)) != 0)
			return -EFAULT;
		regid = d2083->pdata->regl_map[id].reg_id;
		if (regid < 0) 
			return -EINVAL;

		/*check the status of SIMLDO*/
		ret = d2083_regulator_is_enabled(d2083_rdev[regid]);
		if (ret) {
			dlg_info("SIMLDO is activated already\n");
			return -EPERM;
		}
		/* Put SIMLDO in ON State */
		ret = d2083_regulator_enable(d2083_rdev[regid]);
		if (ret)
			return ret;
		/* Set SIMLDO voltage */
		value = 2500000;			// 2.5V
		ret = d2083_regulator_set_voltage(d2083_rdev[regid], value, value);
		break;
	}
	case BCM_PMU_IOCTL_DEACTIVATESIM:
	{
		int id = 16;	/*check the status of SIMLDO*/
		ret = d2083_regulator_is_enabled(d2083_rdev[id]);
		if (!ret) {
			dlg_info("SIMLDFO is already disabled\n");
			return -EPERM;
		}
		regid = d2083->pdata->regl_map[id].reg_id;
		if (regid < 0) 
			return -EINVAL;

		ret = d2083_regulator_disable(d2083_rdev[regid]);
		if (ret)
			return ret;
		break;
	}
	}	/*end of switch*/
	return ret;
}
#endif /* CONFIG_MACH_RHEA_SS_IVORY */
/* IOCTL end */


int d2083_core_reg2volt(int reg)
{
	int volt = -EINVAL;
	int i;

	for (i = 0; i < ARRAY_SIZE(core_vol_list); i++) {
		if (core_vol_list[i][1] == reg) {
			volt = core_vol_list[i][0];
			break;
		}
	}

	dlg_info("[DLG] [%s]-volt[%d]\n", __func__, volt);

	return volt;
}
EXPORT_SYMBOL(d2083_core_reg2volt);


void regulator_has_full_constraints(void);

static int d2083_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	//struct d2083 *d2083 = dev_get_drvdata(&pdev->dev); // platform_get_drvdata(pdev);

	if (pdev->id < D2083_BUCK_1 || pdev->id >= D2083_NUMBER_OF_REGULATORS)
		return -ENODEV;

	/* register regulator */
	rdev = regulator_register(&d2083_reg[pdev->id], &pdev->dev,
				pdev->dev.platform_data,
				dev_get_drvdata(&pdev->dev));

	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			d2083_reg[pdev->id].name);
		return PTR_ERR(rdev);
	}

	d2083_rdev[pdev->id] = rdev;		/* rdev required for IOCTL support */

	regulator_has_full_constraints();

	return 0;
}

static int d2083_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	struct d2083 *d2083 = rdev_get_drvdata(rdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(d2083->pmic.pdev); i++)
		platform_device_unregister(d2083->pmic.pdev[i]);

	regulator_unregister(rdev);

	return 0;
}

static int d2083_register_regulator(struct d2083 *d2083, int reg,
					struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;
	if (reg < D2083_BUCK_1 || reg >= D2083_NUMBER_OF_REGULATORS)
		return -EINVAL;

	if (d2083->pmic.pdev[reg])
		return -EBUSY;

	pdev = platform_device_alloc(DRIVER_NAME, reg);
	if (!pdev)
		return -ENOMEM;

	d2083->pmic.pdev[reg] = pdev;

	initdata->driver_data = d2083;

	pdev->dev.platform_data = initdata;
	pdev->dev.parent = d2083->dev;
	platform_set_drvdata(pdev, d2083);

	ret = platform_device_add(pdev);

	if (ret != 0) {
		dev_err(d2083->dev, "Failed to register regulator %d: %d\n", reg, ret);
		platform_device_del(pdev);
		d2083->pmic.pdev[reg] = NULL;
	}
	return ret;
}


static struct platform_driver d2083_regulator_driver = {
	.probe  = d2083_regulator_probe,
	.remove = d2083_regulator_remove,
	.driver = {
		.name = DRIVER_NAME,
	},
};

static int __init d2083_regulator_init(void)
{
	return platform_driver_register(&d2083_regulator_driver);
}
subsys_initcall(d2083_regulator_init);

static void __exit d2083_regulator_exit(void)
{
	platform_driver_unregister(&d2083_regulator_driver);
}
module_exit(d2083_regulator_exit);

/* Module information */
MODULE_AUTHOR("Dialog Semiconductor Ltd < william.seo@diasemi.com >");
MODULE_DESCRIPTION("D2083 voltage and current regulator driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
