/*****************************************************************************
*  Copyright 2001 - 2011 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
*  http://www.gnu.org/licenses/old-license/gpl-2.0.html (the "GPL").
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*
*****************************************************************************/

#include <linux/version.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#ifdef CONFIG_MFD_D2083
#include <linux/d2083/core.h>
#include <linux/d2083/pmic.h>
#include <linux/d2083/d2083_battery.h>
#else
#include <linux/mfd/bcmpmu.h>
#endif
#include <linux/broadcom/bcmpmu-ponkey.h>
#ifdef CONFIG_KONA_AVS
#include <plat/kona_avs.h>
#endif
#include "pm_params.h"
#if defined(CONFIG_SEC_CHARGING_FEATURE)
#include <linux/spa_power.h>
#endif
#ifdef CONFIG_MFD_D2083
#define PMU_DEVICE_I2C_ADDR	0x49
#else
#define PMU_DEVICE_I2C_ADDR	0x08
#define PMU_DEVICE_I2C_ADDR1	0x0C
#endif
#define PMU_DEVICE_INT_GPIO	29
#define PMU_DEVICE_I2C_BUSNO 2

#define CONFIG_FSA9480_MICROUSB				// TEST config

#if defined(CONFIG_FSA9480_MICROUSB)
#include <linux/fsa9480.h>
#endif


static int vlt_tbl_init;

#if !defined(CONFIG_MFD_D2083)
static struct bcmpmu_rw_data __initdata register_init_data[] = {
	{.map = 0, .addr = 0x01, .val = 0x00, .mask = 0x01},

	/* pmic_set_7sec_mode in pmic_bcm59039.c set 0x0c value according to debug level */
#if 0
	{.map = 0, .addr = 0x0c, .val = 0xdb, .mask = 0xFF},
#endif
#if defined(CONFIG_MACH_RHEA_STONE) || defined(CONFIG_MACH_RHEA_STONE_EDN2X)
	{.map = 0, .addr = 0x13, .val = 0x3d, .mask = 0xFF},
	{.map = 0, .addr = 0x14, .val = 0x79, .mask = 0xFF},
	{.map = 0, .addr = 0x15, .val = 0x20, .mask = 0xFF},
#else
	{.map = 0, .addr = 0x13, .val = 0x43, .mask = 0xFF},
	{.map = 0, .addr = 0x14, .val = 0x7F, .mask = 0xFF},
	{.map = 0, .addr = 0x15, .val = 0x3B, .mask = 0xFF},
#endif /* CONFIG_MACH_RHEA_STONE */
	{.map = 0, .addr = 0x16, .val = 0xF8, .mask = 0xFF},
	{.map = 0, .addr = 0x1D, .val = 0x09, .mask = 0xFF},
	{.map = 0, .addr = 0x40, .val = 0xFF, .mask = 0xFF},
	{.map = 0, .addr = 0x41, .val = 0xFF, .mask = 0xFF},
	{.map = 0, .addr = 0x42, .val = 0xFF, .mask = 0xFF},
	{.map = 0, .addr = 0x43, .val = 0xFF, .mask = 0xFF},
	{.map = 0, .addr = 0x44, .val = 0xFF, .mask = 0xFF},
	{.map = 0, .addr = 0x45, .val = 0xFF, .mask = 0xFF},
	{.map = 0, .addr = 0x46, .val = 0xFF, .mask = 0xFF},
	{.map = 0, .addr = 0x47, .val = 0xFF, .mask = 0xFF},
	{.map = 0, .addr = 0x52, .val = 0x04, .mask = 0x04},
	{.map = 0, .addr = 0x58, .val = 0x05, .mask = 0x0F},
	{.map = 0, .addr = 0x5E, .val = 0x30, .mask = 0xFF},
	/*
	* temp workaround for LDOs, to be revisited once final
		OTP value available
	*/
        {.map = 0, .addr = 0xB1, .val = 0x25, .mask = 0xFF},
	{.map = 0, .addr = 0xB2, .val = 0x04, .mask = 0xFF},
	{.map = 0, .addr = 0xB3, .val = 0x4B, .mask = 0xFF},
	{.map = 0, .addr = 0xB4, .val = 0x27, .mask = 0xFF},
	{.map = 0, .addr = 0xB5, .val = 0x06, .mask = 0xFF}, //HVLDO3 for VDD_SDIO(external SD), 3.0V
	{.map = 0, .addr = 0xB6, .val = 0x07, .mask = 0xFF},
	{.map = 0, .addr = 0xB7, .val = 0x26, .mask = 0xFF},
	{.map = 0, .addr = 0xB8, .val = 0x06, .mask = 0xFF},
	{.map = 0, .addr = 0xB9, .val = 0x07, .mask = 0xFF},
	{.map = 0, .addr = 0xBA, .val = 0x06, .mask = 0xFF},      
	{.map = 0, .addr = 0xBC, .val = 0x07, .mask = 0xFF},
	{.map = 0, .addr = 0xBD, .val = 0x21, .mask = 0xFF},

	{.map = 0, .addr = 0xC1, .val = 0x04, .mask = 0xFF},
	{.map = 0, .addr = 0xAD, .val = 0x11, .mask = 0xFF},
	{.map = 0, .addr = 0x0B, .val = 0x02, .mask = 0xFF},
	{.map = 0, .addr = 0x1B, .val = 0x13, .mask = 0xFF},
	{.map = 0, .addr = 0x1C, .val = 0x13, .mask = 0xFF},
	{.map = 0, .addr = 0x0A, .val = 0x0E, .mask = 0xFF},
	{.map = 0, .addr = 0xA0, .val = 0x01, .mask = 0xFF},
	{.map = 0, .addr = 0xA2, .val = 0x01, .mask = 0xFF},

	/* pmic_set_7sec_mode in pmic_bcm59039.c set 0x0c & 0x0d value accroding to debug level */
#if 0
	{.map = 0, .addr = 0x0C, .val = 0x64, .mask = 0xFF}, //  Smart Reset Change as suggested by Ismael
	{.map = 0, .addr = 0x05, .val = 0xB6, .mask = 0xFF},
#endif
	{.map = 0, .addr = 0x0D, .val = 0x6D, .mask = 0xFF},
	{.map = 0, .addr = 0x0E, .val = 0x41, .mask = 0xFF},

	/*Init SDSR NM, NM2 and LPM voltages to 1.2V
	*/
#if 0
	{.map = 0, .addr = 0xD0, .val = 0x13, .mask = 0xFF},
	{.map = 0, .addr = 0xD1, .val = 0x13, .mask = 0xFF},
	{.map = 0, .addr = 0xD2, .val = 0x13, .mask = 0xFF},
#endif
	{.map = 0, .addr = 0xD0, .val = 0x15, .mask = 0xFF},
        {.map = 0, .addr = 0xD1, .val = 0x15, .mask = 0xFF},
        {.map = 0, .addr = 0xD2, .val = 0x15, .mask = 0xFF},

	/*Init CSR LPM  to 0.9 V
	CSR NM2 to 1.22V
	*/
	{.map = 0, .addr = 0xC1, .val = 0x04, .mask = 0xFF},
	{.map = 0, .addr = 0xC2, .val = 0x14, .mask = 0xFF},

	/*PLLCTRL, Clear Bit 0 to disable PLL when PC2:PC1 = 0b00*/
	{.map = 0, .addr = 0x0A, .val = 0x0E, .mask = 0x0F},
	/*CMPCTRL13, Set bits 4, 1 for BSI Sync. Mode */
	{.map = 0, .addr = 0x1C, .val = 0x13, .mask = 0xFF},
	/*CMPCTRL12, Set bits 4, 1 for NTC Sync. Mode*/
	{.map = 0, .addr = 0x1B, .val = 0x13, .mask = 0xFF},

#ifdef CONFIG_MACH_RHEA_STONE_EDN2X
	{.map = 0, .addr = 0xD9, .val = 0x1A, .mask = 0xFF},
#else
	/*Init ASR LPM to 2.9V - for Rhea EDN10 & EDN00 and 1.8V for EDN2x
	*/
	{.map = 0, .addr = 0xD9, .val = 0x1F, .mask = 0xFF},
	/*Init IOSR NM2 and LPM voltages to 1.8V
	*/
	{.map = 0, .addr = 0xC9, .val = 0x1A, .mask = 0xFF},
	{.map = 0, .addr = 0xCA, .val = 0x1A, .mask = 0xFF},
#endif /*CONFIG_MACH_RHEA_STONE_EDN2X*/

	/* Disable the charging elapsed timer by TCH[2:0]=111b 
	   OTP default value; TCH[2:0] = 010b (5hrs) ,TTR[2:0] = 011b (45mins)
        */
	{.map = 0, .addr = 0x50, .val = 0x3B, .mask = 0xFF},

	/*FGOPMODCTRL, Set bits 4, 1 for FG Sync. Mode*/
	{.map = 1, .addr = 0x42, .val = 0x15, .mask = 0xFF},
	{.map = 1, .addr = 0x43, .val = 0x02, .mask = 0xFF},
};

static struct bcmpmu_temp_map batt_temp_map[] = {
	/*
	* This table is hardware dependent and need to get from platform team
	*/
	/*
   * { adc readings 10-bits,  temperature in Celsius }
	*/
	{932, -400},			/* -40 C */
	{900, -350},			/* -35 C */
	{869, -300},            /* -30 */
	{769, -200},			/* -20 */
	{635, -100},                    /* -10 */
	{574, -50},				/* -5 */
	{509,   0},                    /* 0   */
	{376,  100},                    /* 10  */	
	{277,  200},                    /* 20  */
	{237,  250},                    /* 25  */
	{200,  300},                    /* 30  */	
	{139,  400},                    /* 40  */
	{98 ,  500},                    /* 50  */
	{68 ,  600},                    /* 60  */
	{54 ,  650},                    /* 65  */
	{46 ,  700},            /* 70  */
	{34 ,  800},            /* 80  */
	{28, 850},			/* 85 C */
	{24, 900},			/* 90 C */
	{20, 950},			/* 95 C */
	{16, 1000},			/* 100 C */

};

__weak struct regulator_consumer_supply rf_supply[] = {
	{.supply = "rf"},
};
static struct regulator_init_data bcm59039_rfldo_data = {
	.constraints = {
			.name = "rfldo",
			.min_uV = 1300000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(rf_supply),
	.consumer_supplies = rf_supply,
};

__weak struct regulator_consumer_supply cam_supply[] = {
	{.supply = "cam"},
};
static struct regulator_init_data bcm59039_camldo_data = {
	.constraints = {
			.name = "camldo",
			.min_uV = 1300000,
			.max_uV = 3300000,
			.valid_ops_mask =
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE |
			REGULATOR_CHANGE_VOLTAGE,
			.always_on = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(cam_supply),
	.consumer_supplies = cam_supply,
};


__weak struct regulator_consumer_supply hv1_supply[] = {
	{.supply = "hv1"},
};
static struct regulator_init_data bcm59039_hv1ldo_data = {
	.constraints = {
			.name = "hv1ldo",
			.min_uV = 1300000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS |
				REGULATOR_CHANGE_MODE |
			REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
				REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY,
			.always_on = 1,	// VDD_AUD_2.9V
			},
	.num_consumer_supplies = ARRAY_SIZE(hv1_supply),
	.consumer_supplies = hv1_supply,
};

__weak struct regulator_consumer_supply hv2_supply[] = {
	{.supply = "hv2ldo_uc"},
};
static struct regulator_init_data bcm59039_hv2ldo_data = {
	.constraints = {
			.name = "hv2ldo",
			.min_uV = 1300000,
			.max_uV = 3300000,
			.valid_ops_mask =
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(hv2_supply),
	.consumer_supplies = hv2_supply,
};

__weak struct regulator_consumer_supply hv3_supply[] = {
	{.supply = "hv3"},
};
static struct regulator_init_data bcm59039_hv3ldo_data = {
	.constraints = {
			.name = "hv3ldo",
			.min_uV = 1300000,
			.max_uV = 3000000,
			.valid_ops_mask =
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
			.always_on = 0,	// VDD_SDIO_3.0V for T-flash
			},
	.num_consumer_supplies = ARRAY_SIZE(hv3_supply),
	.consumer_supplies = hv3_supply,
};

__weak struct regulator_consumer_supply hv4_supply[] = {
	{.supply = "hv4"},
	{.supply = "2v9_vibra"},
	{.supply = "dummy"}, /* Add a dummy variable to ensure we can use an array of 3 in rhea_ray.
		  A hack at best to ensure we redefine the supply in board file. */
};
static struct regulator_init_data bcm59039_hv4ldo_data = {
	.constraints = {
			.name = "hv4ldo",
			.min_uV = 1300000,
			.max_uV = 3000000,
			.valid_ops_mask =
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
			.always_on = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(hv4_supply),
	.consumer_supplies = hv4_supply,
};

__weak struct regulator_consumer_supply hv5_supply[] = {
	{.supply = "hv5"},
};
static struct regulator_init_data bcm59039_hv5ldo_data = {
	.constraints = {
			.name = "hv5ldo",
			.min_uV = 1300000,
			.max_uV = 3300000,
			.valid_ops_mask =
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,	// VLCD_3.0
			},
	.num_consumer_supplies = ARRAY_SIZE(hv5_supply),
	.consumer_supplies = hv5_supply,
};

__weak struct regulator_consumer_supply hv6_supply[] = {
	{.supply = "vdd_sdxc"},
};
static struct regulator_init_data bcm59039_hv6ldo_data = {
	.constraints = {
			.name = "hv6ldo",
			.min_uV = 1300000,
			.max_uV = 3000000,
			.valid_ops_mask =
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
			.always_on = 0,	// VDD_SDXC(BB_SDIO)
			},
	.num_consumer_supplies = ARRAY_SIZE(hv6_supply),
	.consumer_supplies = hv6_supply,
};

__weak struct regulator_consumer_supply hv7_supply[] = {
	{.supply = "hv7"},
};
static struct regulator_init_data bcm59039_hv7ldo_data = {
	.constraints = {
			.name = "hv7ldo",
			.min_uV = 1300000,
			.max_uV = 3300000,
			.valid_ops_mask =
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
			.always_on = 0,	// NC
			},
	.num_consumer_supplies = ARRAY_SIZE(hv7_supply),
	.consumer_supplies = hv7_supply,
};

__weak struct regulator_consumer_supply hv8_supply[] = {
	{.supply = "hv8"},
};
static struct regulator_init_data bcm59039_hv8ldo_data = {
	.constraints = {
			.name = "hv8ldo",
			.min_uV = 1300000,
			.max_uV = 3300000,
			.valid_ops_mask =
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,	// VDD_SENSOR_3.0V
			},
	.num_consumer_supplies = ARRAY_SIZE(hv8_supply),
	.consumer_supplies = hv8_supply,
};

__weak struct regulator_consumer_supply hv9_supply[] = {
	{.supply = "hv9"},
};
static struct regulator_init_data bcm59039_hv9ldo_data = {
	.constraints = {
			.name = "hv9ldo",
			.min_uV = 1300000,
			.max_uV = 3300000,
			.valid_ops_mask =
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE |
			REGULATOR_CHANGE_VOLTAGE,
			.always_on = 0,
			},
	.num_consumer_supplies = ARRAY_SIZE(hv9_supply),
	.consumer_supplies = hv9_supply,
};

__weak struct regulator_consumer_supply hv10_supply[] = {
	{.supply = "sim2_vcc"},
};

static struct regulator_init_data bcm59039_hv10ldo_data = {
	.constraints = {
			.name = "hv10ldo",
			.min_uV = 1300000,
			.max_uV = 3300000,
			.valid_ops_mask =
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE |
			REGULATOR_CHANGE_VOLTAGE,
			.always_on = 0,
			},// VSIM2_3.0V
	.num_consumer_supplies = ARRAY_SIZE(hv10_supply),
	.consumer_supplies = hv10_supply,
};

__weak struct regulator_consumer_supply sim_supply[] = {
	{.supply = "sim_vcc"},
};
static struct regulator_init_data bcm59039_simldo_data = {
	.constraints = {
			.name = "simldo",
			.min_uV = 1300000,
			.max_uV = 3300000,
			.valid_ops_mask =
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
/*TODO: We observed that, on Rhearay HW, interrupt from GPIO expander
is not detected by baseband if SIMLDO is disabled. As a temp. workaround
we keep SIMLDO ON by default for Rhearay till the issue is root casued*/
#ifdef CONFIG_MACH_RHEA_RAY_EDN2X
           .always_on = 1,
#endif

			},
	.num_consumer_supplies = ARRAY_SIZE(sim_supply),
	.consumer_supplies = sim_supply,
};

struct regulator_consumer_supply csr_nm_supply[] = {
	{.supply = "csr_nm_uc"},
};
static struct regulator_init_data bcm59039_csr_nm_data = {
	.constraints = {
			.name = "csr_nm",
			.min_uV = 700000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(csr_nm_supply),
	.consumer_supplies = csr_nm_supply,
};

struct regulator_consumer_supply csr_nm2_supply[] = {
	{.supply = "csr_nm2_uc"},
};
static struct regulator_init_data bcm59039_csr_nm2_data = {
	.constraints = {
			.name = "csr_nm2",
			.min_uV = 700000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(csr_nm2_supply),
	.consumer_supplies = csr_nm2_supply,
};

struct regulator_consumer_supply csr_lpm_supply[] = {
	{.supply = "csr_lpm_uc"},
};
static struct regulator_init_data bcm59039_csr_lpm_data = {
	.constraints = {
			.name = "csr_lpm",
			.min_uV = 700000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(csr_lpm_supply),
	.consumer_supplies = csr_lpm_supply,
};


struct regulator_consumer_supply iosr_nm_supply[] = {
	{.supply = "iosr_nm_uc"},
};
static struct regulator_init_data bcm59039_iosr_nm_data = {
	.constraints = {
			.name = "iosr_nm",
			.min_uV = 700000,
			.max_uV = 1800000,
			.valid_ops_mask =
			REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(iosr_nm_supply),
	.consumer_supplies = iosr_nm_supply,
};

struct regulator_consumer_supply iosr_nm2_supply[] = {
	{.supply = "iosr_nm2_uc"},
};
static struct regulator_init_data bcm59039_iosr_nm2_data = {
	.constraints = {
			.name = "iosr_nm2",
			.min_uV = 700000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(iosr_nm2_supply),
	.consumer_supplies = iosr_nm2_supply,
};
struct regulator_consumer_supply iosr_lpm_supply[] = {
	{.supply = "iosr_lmp_uc"},
};
static struct regulator_init_data bcm59039_iosr_lpm_data = {
	.constraints = {
			.name = "iosr_lmp",
			.min_uV = 700000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(iosr_lpm_supply),
	.consumer_supplies = iosr_lpm_supply,
};

struct regulator_consumer_supply sdsr_nm_supply[] = {
	{.supply = "sdsr_nm_uc"},
};

static struct regulator_init_data bcm59039_sdsr_nm_data = {
	.constraints = {
			.name = "sdsr_nm",
			.min_uV = 700000,
			.max_uV = 1800000,
			.valid_ops_mask =
			REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(sdsr_nm_supply),
	.consumer_supplies = sdsr_nm_supply,
};

struct regulator_consumer_supply sdsr_nm2_supply[] = {
	{.supply = "sdsr_nm2_uc"},
};

static struct regulator_init_data bcm59039_sdsr_nm2_data = {
	.constraints = {
			.name = "sdsr_nm2",
			.min_uV = 700000,
			.max_uV = 1800000,
			.valid_ops_mask =
			REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(sdsr_nm2_supply),
	.consumer_supplies = sdsr_nm2_supply,
};

struct regulator_consumer_supply sdsr_lpm_supply[] = {
	{.supply = "sdsr_lpm_uc"},
};

static struct regulator_init_data bcm59039_sdsr_lpm_data = {
	.constraints = {
			.name = "sdsr_lpm",
			.min_uV = 700000,
			.max_uV = 1800000,
			.valid_ops_mask =
			REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(sdsr_lpm_supply),
	.consumer_supplies = sdsr_lpm_supply,
};

struct regulator_consumer_supply asr_nm_supply[] = {
    {.supply = "asr_nm_uc"},
};

static struct regulator_init_data bcm59039_asr_nm_data = {
    .constraints = {
            .name = "asr_nm",
            .min_uV = 700000,
            .max_uV = 2900000,
            .valid_ops_mask =
            REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
#ifdef CONFIG_SOC_CAMERA_POWER_USE_ASR
            .always_on = 0,
#else
            .always_on = 1,
#endif 
            },
    .num_consumer_supplies = ARRAY_SIZE(asr_nm_supply),
    .consumer_supplies = asr_nm_supply,
};

struct regulator_consumer_supply asr_nm2_supply[] = {
    {.supply = "asr_nm2_uc"},
};

static struct regulator_init_data bcm59039_asr_nm2_data = {
    .constraints = {
            .name = "asr_nm2",
            .min_uV = 700000,
            .max_uV = 2900000,
            .valid_ops_mask =
            REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
#ifdef CONFIG_SOC_CAMERA_POWER_USE_ASR
            .always_on = 0,
#else
            .always_on = 1,
#endif
            },
    .num_consumer_supplies = ARRAY_SIZE(asr_nm2_supply),
    .consumer_supplies = asr_nm2_supply,
};

struct regulator_consumer_supply asr_lpm_supply[] = {
    {.supply = "asr_lpm_uc"},
};

static struct regulator_init_data bcm59039_asr_lpm_data = {
    .constraints = {
            .name = "asr_lpm",
            .min_uV = 700000,
            .max_uV = 2900000,
            .valid_ops_mask =
            REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_VOLTAGE,
#ifdef CONFIG_SOC_CAMERA_POWER_USE_ASR
            .always_on = 0,
#else
            .always_on = 1,
#endif
            },
    .num_consumer_supplies = ARRAY_SIZE(asr_lpm_supply),
    .consumer_supplies = asr_lpm_supply,
};

struct bcmpmu_regulator_init_data bcm59039_regulators[BCMPMU_REGULATOR_MAX] = {
	[BCMPMU_REGULATOR_RFLDO] = {
		BCMPMU_REGULATOR_RFLDO, &bcm59039_rfldo_data, 0x01, 0
	},
	[BCMPMU_REGULATOR_CAMLDO] = {
		BCMPMU_REGULATOR_CAMLDO, &bcm59039_camldo_data, 0xAA, BCMPMU_REGL_OFF_IN_DSM
	},
	[BCMPMU_REGULATOR_HV1LDO] =	{	// VDD_AUD_2.9V
		BCMPMU_REGULATOR_HV1LDO, &bcm59039_hv1ldo_data, 0x22, 0
	},
	[BCMPMU_REGULATOR_HV2LDO] =	{	// VDD_USB_3.3V
		BCMPMU_REGULATOR_HV2LDO, &bcm59039_hv2ldo_data, 0x11, BCMPMU_REGL_LPM_IN_DSM
	},
	[BCMPMU_REGULATOR_HV3LDO] = {		// VDD_SDIO_3.0V(T-flash)
		BCMPMU_REGULATOR_HV3LDO, &bcm59039_hv3ldo_data, 0xAA, BCMPMU_REGL_LPM_IN_DSM
	},
	[BCMPMU_REGULATOR_HV4LDO] =	{	// VDD_VIB_2.9V
		BCMPMU_REGULATOR_HV4LDO, &bcm59039_hv4ldo_data, 0xAA, BCMPMU_REGL_ON_IN_DSM
	},
	[BCMPMU_REGULATOR_HV5LDO] = {
		BCMPMU_REGULATOR_HV5LDO, &bcm59039_hv5ldo_data, 0x00, BCMPMU_REGL_LPM_IN_DSM  /* 0x01 -> 0x00 */
	},
	[BCMPMU_REGULATOR_HV6LDO] = {		// VDD_SDXC(BB-SDIO)
		//BCMPMU_REGULATOR_HV6LDO, &bcm59039_hv6ldo_data, 0x11, BCMPMU_REGL_LPM_IN_DSM
		BCMPMU_REGULATOR_HV6LDO, &bcm59039_hv6ldo_data, 0xAA, BCMPMU_REGL_LPM_IN_DSM
	},
	[BCMPMU_REGULATOR_HV7LDO] = {
		BCMPMU_REGULATOR_HV7LDO, &bcm59039_hv7ldo_data, 0xAA, BCMPMU_REGL_OFF_IN_DSM
	},
	[BCMPMU_REGULATOR_HV8LDO] = {		// VDD_SENSOR_3.0V
			BCMPMU_REGULATOR_HV8LDO, &bcm59039_hv8ldo_data, 0x00, BCMPMU_REGL_LPM_IN_DSM
	},
	[BCMPMU_REGULATOR_HV9LDO] = {		// VCAM_IO_1.8V
				BCMPMU_REGULATOR_HV9LDO, &bcm59039_hv9ldo_data, 0xAA, BCMPMU_REGL_OFF_IN_DSM
	},
	[BCMPMU_REGULATOR_HV10LDO] = {
				BCMPMU_REGULATOR_HV10LDO, &bcm59039_hv10ldo_data, 0xAA, BCMPMU_REGL_LPM_IN_DSM
	},

/*TODO: We observed that, on Rhearay HW, interrupt from GPIO expander
is not detected by baseband if SIMLDO is disabled. As a temp. workaround
we keep SIMLDO ON by default for Rhearay till the issue is root casued*/
#ifdef CONFIG_MACH_RHEA_RAY_EDN2X
	[BCMPMU_REGULATOR_SIMLDO] = {
		BCMPMU_REGULATOR_SIMLDO, &bcm59039_simldo_data, 0x00,
			BCMPMU_REGL_LPM_IN_DSM
	},
#else
/*Changed from 0x11 to 0xAA - GCF 27.17.1.4 and 5.1.3 (CSP 542271)*/
	[BCMPMU_REGULATOR_SIMLDO] = {
		BCMPMU_REGULATOR_SIMLDO, &bcm59039_simldo_data, 0xAA,
			BCMPMU_REGL_LPM_IN_DSM
	},
#endif
	[BCMPMU_REGULATOR_CSR_NM] =	{
		BCMPMU_REGULATOR_CSR_NM, &bcm59039_csr_nm_data, 0x11, 0
	},
	[BCMPMU_REGULATOR_CSR_NM2] = {
		BCMPMU_REGULATOR_CSR_NM2, &bcm59039_csr_nm2_data, 0xFF, 0
	},
	[BCMPMU_REGULATOR_CSR_LPM] = {
		BCMPMU_REGULATOR_CSR_LPM, &bcm59039_csr_lpm_data, 0xFF, 0
	},
	[BCMPMU_REGULATOR_IOSR_NM] = {
		BCMPMU_REGULATOR_IOSR_NM, &bcm59039_iosr_nm_data, 0x01, 0
	},
	[BCMPMU_REGULATOR_IOSR_NM2] = {
		BCMPMU_REGULATOR_IOSR_NM2, &bcm59039_iosr_nm2_data, 0xFF, 0
	},
	[BCMPMU_REGULATOR_IOSR_LPM] = {
		BCMPMU_REGULATOR_IOSR_LPM, &bcm59039_iosr_lpm_data, 0xFF, 0
	},
	[BCMPMU_REGULATOR_SDSR_NM] = {
		BCMPMU_REGULATOR_SDSR_NM, &bcm59039_sdsr_nm_data, 0x11, 0
	},
	[BCMPMU_REGULATOR_SDSR_NM2] = {
		BCMPMU_REGULATOR_SDSR_NM2, &bcm59039_sdsr_nm2_data, 0xFF, 0
	},
	[BCMPMU_REGULATOR_SDSR_LPM] = {
		BCMPMU_REGULATOR_SDSR_LPM, &bcm59039_sdsr_lpm_data, 0xFF, 0
	},
#ifdef CONFIG_MACH_RHEA_STONE_EDN2X
    [BCMPMU_REGULATOR_ASR_NM] = {
        BCMPMU_REGULATOR_ASR_NM, &bcm59039_asr_nm_data, 0x01, 0
    },
#elif defined(CONFIG_SOC_CAMERA_POWER_USE_ASR)
    [BCMPMU_REGULATOR_ASR_NM] = {
        BCMPMU_REGULATOR_ASR_NM, &bcm59039_asr_nm_data, 0xAA, BCMPMU_REGL_OFF_IN_DSM
    },
#else
    [BCMPMU_REGULATOR_ASR_NM] = {
        BCMPMU_REGULATOR_ASR_NM, &bcm59039_asr_nm_data, 0x11, 0
    },
#endif /*CONFIG_MACH_RHEA_STONE_EDN2X*/

    [BCMPMU_REGULATOR_ASR_NM2] = {
        BCMPMU_REGULATOR_ASR_NM2, &bcm59039_asr_nm2_data, 0xFF, 0
    },
    [BCMPMU_REGULATOR_ASR_LPM] = {
        BCMPMU_REGULATOR_ASR_LPM, &bcm59039_asr_lpm_data, 0xFF, 0
    },
};

static struct bcmpmu_wd_setting bcm59039_wd_setting = {
  .watchdog_timeout = 127,
};


static struct platform_device bcmpmu_audio_device = {
	.name = "bcmpmu_audio",
	.id = -1,
	.dev.platform_data = NULL,
};

static struct platform_device bcmpmu_em_device = {
	.name = "bcmpmu_em",
	.id = -1,
	.dev.platform_data = NULL,
};

static struct platform_device bcmpmu_otg_xceiv_device = {
	.name = "bcmpmu_otg_xceiv",
	.id = -1,
	.dev.platform_data = NULL,
};

#ifdef CONFIG_BCMPMU_RPC
static struct platform_device bcmpmu_rpc = {
	.name = "bcmpmu_rpc",
	.id = -1,
	.dev.platform_data = NULL,
};
#endif

#ifdef CONFIG_CHARGER_BCMPMU_SPA
static struct platform_device bcmpmu_chrgr_spa_device = {
	.name = "bcmpmu_chrgr_pb",
	.id = -1,
	.dev.platform_data = NULL,
};
#endif

static struct platform_device *bcmpmu_client_devices[] = {
	&bcmpmu_audio_device,
#ifdef CONFIG_CHARGER_BCMPMU_SPA
	&bcmpmu_chrgr_spa_device,
#endif
	&bcmpmu_em_device,
	&bcmpmu_otg_xceiv_device,
#ifdef CONFIG_BCMPMU_RPC
	&bcmpmu_rpc,
#endif
};


static int bcmpmu_exit_platform_hw(struct bcmpmu *bcmpmu)
{
	pr_info("REG: pmu_init_platform_hw called\n");
	return 0;
}
#endif	// CONFIG_MFD_D2083


#if !defined(CONFIG_MFD_D2083)
static struct i2c_board_info pmu_info_map1 = {
	I2C_BOARD_INFO("bcmpmu_map1", PMU_DEVICE_I2C_ADDR1),
};

static struct bcmpmu_adc_setting adc_setting = {
	.tx_rx_sel_addr = 0,
	.tx_delay = 0,
	.rx_delay = 0,
};

static struct bcmpmu_charge_zone chrg_zone[] = {
	{.tl = -50, .th = 600, .v = 3000, .fc = 10, .qc = 100},	/* Zone QC */
	{.tl = -50, .th = -1, .v = 4200, .fc = 100, .qc = 0},	/* Zone LL */
	{.tl = 0, .th = 99, .v = 4200, .fc = 100, .qc = 0},	/* Zone L */
	{.tl = 100, .th = 450, .v = 4200, .fc = 100, .qc = 0},	/* Zone N */
	{.tl = 451, .th = 500, .v = 4200, .fc = 100, .qc = 0},	/* Zone H */
	{.tl = 501, .th = 600, .v = 4200, .fc = 100, .qc = 0},	/* Zone HH */
	{.tl = -50, .th = 600, .v = 0, .fc = 0, .qc = 0},	/* Zone OUT */
};

static struct bcmpmu_voltcap_map batt_voltcap_map[] = {
	/*
	* Battery data for 1300mAH re-measured by Minal 20120717
    * align zero crossing @ 3400mV complying to SS spec
	*/
	/*
	* volt capacity
	*/
	{4159, 100},
	{4107, 95},
	{4070, 90},
	{4017, 85},
	{3979, 80},
	{3953, 75},
	{3924, 70},
	{3897, 65},
	{3859, 60},
	{3823, 55},
	{3803, 50},
	{3790, 46},
	{3780, 41},
	{3775, 36},
	{3770, 31},
	{3749, 26},
	{3713, 21},
	{3689, 16},
	{3673, 11},
	{3663, 10},
	{3648, 9},
	{3628, 7},
	{3605, 6},
	{3578, 5},
	{3546, 4},
	{3511, 3},
	{3470, 2},
	{3425, 1},
	{3400, 0},
};

static int bcmpmu_init_platform_hw(struct bcmpmu *);


static struct bcmpmu_fg_zone fg_zone[FG_TMP_ZONE_MAX+1] = {
/* This table is default data, the real data from board file or device tree*/
/* Battery data for 1200mAH re-measured by Minal 20120601 */
	{.temp = -200,
	 .reset = 0, .fct = 225, .guardband = 100,
	 .esr_vl_lvl = 3291, .esr_vm_lvl = 3388, .esr_vh_lvl = 3588,
	 .esr_vl = 186, .esr_vl_slope = -2183, .esr_vl_offset = 8978,
	 .esr_vm = 172, .esr_vm_slope = 1127, .esr_vm_offset = -1914,
	 .esr_vh = 184, .esr_vh_slope = -771, .esr_vh_offset = 4516,
	 .esr_vf = 183, .esr_vf_slope = -2714, .esr_vf_offset = 11488,
	 .vcmap = &batt_voltcap_map[0],
	 .maplen = ARRAY_SIZE(batt_voltcap_map)},/* -20 */
	{.temp = -150,
	 .reset = 0, .fct = 400, .guardband = 100,
	 .esr_vl_lvl = 3291, .esr_vm_lvl = 3388, .esr_vh_lvl = 3588,
	 .esr_vl = 186, .esr_vl_slope = -2183, .esr_vl_offset = 8978,
	 .esr_vm = 172, .esr_vm_slope = 1127, .esr_vm_offset = -1914,
	 .esr_vh = 184, .esr_vh_slope = -771, .esr_vh_offset = 4516,
	 .esr_vf = 183, .esr_vf_slope = -2714, .esr_vf_offset = 11488,
	 .vcmap = &batt_voltcap_map[0],
	 .maplen = ARRAY_SIZE(batt_voltcap_map)},/* -15 */
	{.temp = -100,
	 .reset = 0, .fct = 576, .guardband = 100,
	 .esr_vl_lvl = 3439, .esr_vm_lvl = 3516, .esr_vh_lvl = 3733,
	 .esr_vl = 186, .esr_vl_slope = -2195, .esr_vl_offset = 8776,
	 .esr_vm = 172, .esr_vm_slope = 700, .esr_vm_offset = -1178,
	 .esr_vh = 184, .esr_vh_slope = -682, .esr_vh_offset = 3681,
	 .esr_vf = 183, .esr_vf_slope = -2171, .esr_vf_offset = 9239,
	 .vcmap = &batt_voltcap_map[0],
	 .maplen = ARRAY_SIZE(batt_voltcap_map)},/* -10 */
	{.temp = -50,
	 .reset = 0, .fct = 701, .guardband = 100,
	 .esr_vl_lvl = 3439, .esr_vm_lvl = 3516, .esr_vh_lvl = 3733,
	 .esr_vl = 186, .esr_vl_slope = -2195, .esr_vl_offset = 8776,
	 .esr_vm = 172, .esr_vm_slope = 700, .esr_vm_offset = -1178,
	 .esr_vh = 184, .esr_vh_slope = -682, .esr_vh_offset = 3681,
	 .esr_vf = 183, .esr_vf_slope = -2171, .esr_vf_offset = 9239,
	 .vcmap = &batt_voltcap_map[0],
	 .maplen = ARRAY_SIZE(batt_voltcap_map)},/* -5 */
	{.temp = 0,
	 .reset = 0, .fct = 826, .guardband = 100,
	 .esr_vl_lvl = 3620, .esr_vm_lvl = 3668, .esr_vh_lvl = 3868,
	 .esr_vl = 186, .esr_vl_slope = -1710, .esr_vl_offset = 6874,
	 .esr_vm = 172, .esr_vm_slope = 1833, .esr_vm_offset = -5951,
	 .esr_vh = 184, .esr_vh_slope = -465, .esr_vh_offset = 2477,
	 .esr_vf = 183, .esr_vf_slope = -1811, .esr_vf_offset = 7683,
	 .vcmap = &batt_voltcap_map[0],
	 .maplen = ARRAY_SIZE(batt_voltcap_map)},/* 0 */
	{.temp = 50,
	 .reset = 0, .fct = 885, .guardband = 100,
	 .esr_vl_lvl = 3529, .esr_vm_lvl = 3711, .esr_vh_lvl = 3763,
	 .esr_vl = 186, .esr_vl_slope = -2301, .esr_vl_offset = 8738,
	 .esr_vm = 172, .esr_vm_slope = -1322, .esr_vm_offset = 5282,
	 .esr_vh = 184, .esr_vh_slope = 1372, .esr_vh_offset = -4714,
	 .esr_vf = 183, .esr_vf_slope = -845, .esr_vf_offset = 3630,
	 .vcmap = &batt_voltcap_map[0],
	 .maplen = ARRAY_SIZE(batt_voltcap_map)},/* 5 */
	{.temp = 100,
	 .reset = 0, .fct = 944, .guardband = 30,
	 .esr_vl_lvl = 3529, .esr_vm_lvl = 3711, .esr_vh_lvl = 3763,
	 .esr_vl = 186, .esr_vl_slope = -2301, .esr_vl_offset = 8738,
	 .esr_vm = 172, .esr_vm_slope = -1322, .esr_vm_offset = 5282,
	 .esr_vh = 184, .esr_vh_slope = 1372, .esr_vh_offset = -4714,
	 .esr_vf = 183, .esr_vf_slope = -845, .esr_vf_offset = 3630,
	 .vcmap = &batt_voltcap_map[0],
	 .maplen = ARRAY_SIZE(batt_voltcap_map)},/* 10 */
	{.temp = 150,
	 .reset = 0, .fct = 1000, .guardband = 30,
	 .esr_vl_lvl = 3803, .esr_vm_lvl = 3993, .esr_vh_lvl = 4033,
	 .esr_vl = 186, .esr_vl_slope = -254, .esr_vl_offset = 1153,
	 .esr_vm = 172, .esr_vm_slope = -560, .esr_vm_offset = 2137,
	 .esr_vh = 184, .esr_vh_slope = 1084, .esr_vh_offset = -4248,
	 .esr_vf = 183, .esr_vf_slope = -506, .esr_vf_offset = 2164,
	 .vcmap = &batt_voltcap_map[0],
	 .maplen = ARRAY_SIZE(batt_voltcap_map)},/* 15 */
	{.temp = 200,
	 .reset = 0, .fct = 1000, .guardband = 30,
	 .esr_vl_lvl = 3803, .esr_vm_lvl = 3993, .esr_vh_lvl = 4033,
	 .esr_vl = 186, .esr_vl_slope = -254, .esr_vl_offset = 1153,
	 .esr_vm = 172, .esr_vm_slope = -560, .esr_vm_offset = 2137,
	 .esr_vh = 184, .esr_vh_slope = 1084, .esr_vh_offset = -4248,
	 .esr_vf = 183, .esr_vf_slope = -506, .esr_vf_offset = 2164,
	 .vcmap = &batt_voltcap_map[0],
	 .maplen = ARRAY_SIZE(batt_voltcap_map)},/* 20 */
};

#ifdef CONFIG_CHARGER_BCMPMU_SPA
static void notify_spa(enum bcmpmu_event_t event, int data)
{
	printk(KERN_INFO "%s event=%d, data=%d\n",
		__func__, event, data);

	switch (event) {
	case BCMPMU_CHRGR_EVENT_CHGR_DETECTION:
		spa_event_handler(SPA_EVT_CHARGER, data);
		break;
	case BCMPMU_CHRGR_EVENT_MBTEMP:
		spa_event_handler(SPA_EVT_TEMP, data);
		break;
	case BCMPMU_CHRGR_EVENT_MBOV:
		spa_event_handler(SPA_EVT_OVP, data);
		break;
	case BCMPMU_CHRGR_EVENT_USBOV:
		spa_event_handler(SPA_EVT_OVP, data);
		break;
	case BCMPMU_CHRGR_EVENT_EOC:
		spa_event_handler(SPA_EVT_EOC, 0);
		break;
	case BCMPMU_CHRGR_EVENT_CAPACITY:
		spa_event_handler(SPA_EVT_CAPACITY, (void *)data);
		break;
	default:
		break;
	}
}
#endif
#endif	// CONFIG_MFD_D2083

#ifdef CONFIG_MFD_D2083
#define mV_to_uV(v)                 ((v) * 1000)
#define uV_to_mV(v)                 ((v) / 1000)
#define MAX_MILLI_VOLT              (3300)


static struct regulator_consumer_supply d2083_buck1_supplies[] = {
	REGULATOR_SUPPLY("csr_nm_uc", NULL),
	REGULATOR_SUPPLY("csr_nm2_uc", NULL),
	REGULATOR_SUPPLY("csr_lpm_uc", NULL),
};

static struct regulator_init_data d2083_buck1 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_BUCK12_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_BUCK12_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_buck1_supplies),
	.consumer_supplies = d2083_buck1_supplies,
};


static struct regulator_consumer_supply d2083_buck2_supplies[] = {
	REGULATOR_SUPPLY("iosr_nm_uc", NULL),
	REGULATOR_SUPPLY("iosr_nm2_uc", NULL),
	REGULATOR_SUPPLY("iosr_lpm_uc", NULL),
};

static struct regulator_init_data d2083_buck2 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_BUCK12_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_BUCK12_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_buck2_supplies),
	.consumer_supplies = d2083_buck2_supplies,
};


static struct regulator_consumer_supply d2083_buck3_supplies[] = {
	REGULATOR_SUPPLY("sdsr_nm_uc", NULL),
	REGULATOR_SUPPLY("sdsr_nm2_uc", NULL),
	REGULATOR_SUPPLY("sdsr_lpm_uc", NULL),
};

static struct regulator_init_data d2083_buck3 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_BUCK3_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_BUCK3_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_buck3_supplies),
	.consumer_supplies = d2083_buck3_supplies,
};

static struct regulator_consumer_supply d2083_buck4_supplies[] = {
	REGULATOR_SUPPLY("buck4", NULL),
};

static struct regulator_init_data d2083_buck4 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_BUCK4_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_BUCK4_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_buck4_supplies),
	.consumer_supplies = d2083_buck4_supplies,
};


// LDO
__weak struct regulator_consumer_supply d2083_ldo1_supplies[] = {
	REGULATOR_SUPPLY("ldo1", NULL),	// Not used
};

static struct regulator_init_data d2083_ldo1 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 0,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo1_supplies),
	.consumer_supplies = d2083_ldo1_supplies,
};

__weak struct regulator_consumer_supply d2083_ldo2_supplies[] = {
	REGULATOR_SUPPLY("rf", NULL),	// VRF_2.7v
};

static struct regulator_init_data d2083_ldo2 = {
	.constraints = {
		.min_uV = 2700000,
		.max_uV = 2700000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo2_supplies),
	.consumer_supplies = d2083_ldo2_supplies,
};


__weak struct regulator_consumer_supply d2083_ldo3_supplies[] = {
	REGULATOR_SUPPLY("hv8", NULL),	// VDD_SENSOR_3.0V
};

static struct regulator_init_data d2083_ldo3 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,	// VDD_SENSOR_3.0V
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo3_supplies),
	.consumer_supplies = d2083_ldo3_supplies,
};

__weak struct regulator_consumer_supply d2083_ldo4_supplies[] = {
	REGULATOR_SUPPLY("hv1", NULL),	// VDD_AUD_2.9V
	REGULATOR_SUPPLY("micbias", NULL),
};

static struct regulator_init_data d2083_ldo4 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY,
		.always_on = 1,	// VDD_AUD_2.9V
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo4_supplies),
	.consumer_supplies = d2083_ldo4_supplies,
};

__weak struct regulator_consumer_supply d2083_ldo5_supplies[] = {
	REGULATOR_SUPPLY("hv2ldo_uc", NULL),	// VDD_USB_3.3V
};

static struct regulator_init_data d2083_ldo5 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo5_supplies),
	.consumer_supplies = d2083_ldo5_supplies,
};


__weak struct regulator_consumer_supply d2083_ldo6_supplies[] = {
	REGULATOR_SUPPLY("cam", NULL),	// VCAM_A_2.8V
};

static struct regulator_init_data d2083_ldo6 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 0,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo6_supplies),
	.consumer_supplies = d2083_ldo6_supplies,
};

__weak struct regulator_consumer_supply d2083_ldo7_supplies[] = {
	REGULATOR_SUPPLY("hv4", NULL),			// VDD_VIB_3.3V
	REGULATOR_SUPPLY("2v9_vibra", NULL),
	REGULATOR_SUPPLY("dummy", NULL),
};

static struct regulator_init_data d2083_ldo7 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 0,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo7_supplies),
	.consumer_supplies = d2083_ldo7_supplies,
};


__weak struct regulator_consumer_supply d2083_ldo8_supplies[] = {
	REGULATOR_SUPPLY("hv5", NULL),	// VLCD_3.0V
};

static struct regulator_init_data d2083_ldo8 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,	// VLCD_3.0
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo8_supplies),
	.consumer_supplies = d2083_ldo8_supplies,
};

__weak struct regulator_consumer_supply d2083_ldo9_supplies[] = {
	REGULATOR_SUPPLY("vdd_sdxc", NULL),	// VDD_SDXC(BB_SDIO)
};

static struct regulator_init_data d2083_ldo9 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		//.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 0,	// VDD_SDXC(BB_SDIO)
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo9_supplies),
	.consumer_supplies = d2083_ldo9_supplies,
};

__weak struct regulator_consumer_supply d2083_ldo10_supplies[] = {
	REGULATOR_SUPPLY("vdd_keyled", NULL),	// KEY_LED_3.3V
};

static struct regulator_init_data d2083_ldo10 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 0,	// TODO: 
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo10_supplies),
	.consumer_supplies = d2083_ldo10_supplies,
};


__weak struct regulator_consumer_supply d2083_ldo11_supplies[] = {
	REGULATOR_SUPPLY("sim_vcc", NULL),	// VSIM1_3.0V
};

static struct regulator_init_data d2083_ldo11 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,

#ifdef CONFIG_MACH_RHEA_RAY_EDN2X
		/*TODO: We observed that, on Rhearay HW, interrupt from GPIO expander
		is not detected by baseband if SIMLDO is disabled. As a temp. workaround
		we keep SIMLDO ON by default for Rhearay till the issue is root casued*/
		.always_on = 0,
#endif
		.always_on = 0,

		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo11_supplies),
	.consumer_supplies = d2083_ldo11_supplies,
};


__weak struct regulator_consumer_supply d2083_ldo12_supplies[] = {
	REGULATOR_SUPPLY("vcc", NULL),	// VDD_EMMC_2.9V
};

static struct regulator_init_data d2083_ldo12 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 1,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo12_supplies),
	.consumer_supplies = d2083_ldo12_supplies,
};

__weak struct regulator_consumer_supply d2083_ldo13_supplies[] = {
	REGULATOR_SUPPLY("hv3", NULL),	// VDD_SDIO_3.0V
	//REGULATOR_SUPPLY("vdd_sdio", NULL),
};

static struct regulator_init_data d2083_ldo13 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 0,	// VDD_SDIO_3.0V for T-flash
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo13_supplies),
	.consumer_supplies = d2083_ldo13_supplies,
};

__weak struct regulator_consumer_supply d2083_ldo14_supplies[] = {
#ifdef CONFIG_MACH_RHEA_SS_NEVISDS
	REGULATOR_SUPPLY("sim2_vcc", NULL),	// VSIM2_3.0V
#else
	REGULATOR_SUPPLY("ldo14", NULL),	// VTOUCH_1.8V
#endif
};

static struct regulator_init_data d2083_ldo14 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 0,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo14_supplies),
	.consumer_supplies = d2083_ldo14_supplies,
};

__weak struct regulator_consumer_supply d2083_ldo15_supplies[] = {
	REGULATOR_SUPPLY("ldo15", NULL),	// VTOUCH_3.3V
};

static struct regulator_init_data d2083_ldo15 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on = 0,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo15_supplies),
	.consumer_supplies = d2083_ldo15_supplies,
};

__weak struct regulator_consumer_supply d2083_ldo16_supplies[] = {
	REGULATOR_SUPPLY("hv9", NULL),	// VCAM_IO_1.8V
};

static struct regulator_init_data d2083_ldo16 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo16_supplies),
	.consumer_supplies = d2083_ldo16_supplies,
};


__weak struct regulator_consumer_supply d2083_ldo17_supplies[] = {
	REGULATOR_SUPPLY("ldo17", NULL),	// VCAM_AF_2.8V
};

static struct regulator_init_data d2083_ldo17 = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldo17_supplies),
	.consumer_supplies = d2083_ldo17_supplies,
};

__weak struct regulator_consumer_supply d2083_ldoaud_supplies[] = {
	REGULATOR_SUPPLY("ldoaud", NULL),	// VLDO_AUD -> d2083 internal use
};

static struct regulator_init_data d2083_ldoaud = {
	.constraints = {
		.min_uV = mV_to_uV(D2083_LDO_VOLT_LOWER),
		.max_uV = mV_to_uV(D2083_LDO_VOLT_UPPER),
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		},
	.num_consumer_supplies = ARRAY_SIZE(d2083_ldoaud_supplies),
	.consumer_supplies = d2083_ldoaud_supplies,
};



static struct d2083_regl_init_data d2083_regulators_init_data[D2083_NUMBER_OF_REGULATORS] = {
	[D2083_BUCK_1] = { D2083_BUCK_1,  &d2083_buck1 },
	[D2083_BUCK_2] = { D2083_BUCK_2,  &d2083_buck2 },
	[D2083_BUCK_3] = { D2083_BUCK_3,  &d2083_buck3 },
	[D2083_BUCK_4] = { D2083_BUCK_4,  &d2083_buck4 },

	[D2083_LDO_1]  = { D2083_LDO_1, &d2083_ldo1 },
	[D2083_LDO_2]  = { D2083_LDO_2, &d2083_ldo2 },
	[D2083_LDO_3]  = { D2083_LDO_3, &d2083_ldo3 },
	[D2083_LDO_4]  = { D2083_LDO_4, &d2083_ldo4 },
	[D2083_LDO_5]  = { D2083_LDO_5, &d2083_ldo5 },
	[D2083_LDO_6]  = { D2083_LDO_6, &d2083_ldo6 },
	[D2083_LDO_7]  = { D2083_LDO_7, &d2083_ldo7 },
	[D2083_LDO_8]  = { D2083_LDO_8, &d2083_ldo8 },
	[D2083_LDO_9]  = { D2083_LDO_9, &d2083_ldo9 },
	[D2083_LDO_10] = { D2083_LDO_10, &d2083_ldo10 },
	[D2083_LDO_11] = { D2083_LDO_11, &d2083_ldo11 },
	[D2083_LDO_12] = { D2083_LDO_12, &d2083_ldo12 },
	[D2083_LDO_13] = { D2083_LDO_13, &d2083_ldo13 },
	[D2083_LDO_14] = { D2083_LDO_14, &d2083_ldo14 },
	[D2083_LDO_15] = { D2083_LDO_15, &d2083_ldo15 },
	[D2083_LDO_16] = { D2083_LDO_16, &d2083_ldo16 },
	[D2083_LDO_17] = { D2083_LDO_17, &d2083_ldo17 },
	
	[D2083_LDO_AUD] = { D2083_LDO_AUD, &d2083_ldoaud },
};



#if 0
/* D2041 regulator mapping */
#define LDO_UNDEFINED	(-1)
struct regulator_consumer_supply d2083_consumers[D2083_NUMBER_OF_REGULATORS] = {
	[D2083_LDO_2 ] = REGULATOR_SUPPLY("rf", NULL),
	[D2083_LDO_6 ] = REGULATOR_SUPPLY("vdd_sdio", NULL),
	[D2083_LDO_7 ] = REGULATOR_SUPPLY("2v9_vibra", NULL),
	[D2083_LDO_9 ] = REGULATOR_SUPPLY("vmmc", NULL),
	[D2083_LDO_11] = REGULATOR_SUPPLY("sim_vcc", NULL),
	[D2083_LDO_13] = REGULATOR_SUPPLY("cam", NULL),
	[D2083_BUCK_1] = REGULATOR_SUPPLY("csr_nm2", NULL),
#if 0	// WS: csr_nm1
	[D2083_BUCK_1_NM1] = REGULATOR_SUPPLY("csr_nm1", NULL),
#endif
	
};
#endif

struct d2083_audio_platform_data audio_pdata = {
	.ina_def_mode =	D2083_IM_FULLY_DIFFERENTIAL,
	.inb_def_mode = D2083_IM_TWO_SINGLE_ENDED,
	.ina_def_preampgain = D2083_PREAMP_GAIN_NEG_6DB, //D2083_PREAMP_GAIN_18DB,
	.inb_def_preampgain = D2083_PREAMP_GAIN_NEG_6DB, //D2083_PREAMP_GAIN_18DB,

	.lhs_def_mixer_in = D2083_MSEL_B1,
	.rhs_def_mixer_in = D2083_MSEL_B2,
	.ihf_def_mixer_in = D2083_MSEL_A2,

	.hs_input_path = D2083_INPUTB,
	.ihf_input_path = D2083_INPUTA,
};


/* Have to syncronize with BCM PMU temperature map.
 * The name of temperature map is "struct bcmpmu_temp_map batt_temp_map[]"
 */
static struct temp2adc_map bcmpmu_batt_temp_map[] = {
/*
 * This table is hardware dependent and need to get from platform team
 */
/*
 * { adc readings 10-bits,  temperature in Celsius }
 */
	{-400,  932},	/* -40 C */
	{-350,  900},	/* -35 C */
	{-300,  869},	/* -30 */
	{-200,  769},	/* -20 */
	{-100,  643},    /* -10 */
	{ -50,  568},	/* -5 */
	{   0,  509},    /* 0   */
	{ 100,  382},    /* 10  */
	{ 200,  275},    /* 20  */
	{ 250,  231},    /* 25  */
	{ 300,  196},    /* 30  */
	{ 400,  138},    /* 40  */
	{ 500,  95 },    /* 50  */
	{ 600,  68 },    /* 60  */
	{ 650,  56 },    /* 65  */
  	{ 700,  47 },    /* 70  */
	{ 800,  34 },	/* 80  */
	{ 850,  28 },	/* 85 C */
	{ 900,  24 },	/* 90 C */
	{ 950,  20 },	/* 95 C */
	{1000,  16 },	/* 100 C */
};


struct d2083_battery_platform_data pbat_pdata = {
       .battery_technology = POWER_SUPPLY_TECHNOLOGY_LION,
       .battery_capacity = 1300,
       .vf_lower    = 250,
       .vf_upper = 510,
       .bcmpmu_temp_map = &bcmpmu_batt_temp_map[0],
       .bcmpmu_temp_map_len = ARRAY_SIZE(bcmpmu_batt_temp_map),
};


struct d2083_platform_data d2083_pdata = {	
#if defined(CONFIG_KONA_PMU_BSC_HS_1625KHZ)
	.i2c_pdata = { ADD_I2C_SLAVE_SPEED(BSC_BUS_SPEED_HS_1625KHZ), },
#else
	.i2c_pdata = { ADD_I2C_SLAVE_SPEED(BSC_BUS_SPEED_400K), },
#endif
	.pbat_platform  = &pbat_pdata,
	.audio_pdata = &audio_pdata,
	.regulator_data = &d2083_regulators_init_data[0],
	.regl_map = {
		/*
		 *		Define initial MCTL value of NEVIS with D2083
		 *	
		 *	[ LDO ]	0x0 : Off	[ BUCK 2,3,4]	0x0 : Off
		 *			0x1 : On					0x1 : On
		 *			0x2 : Sleep - LPM			0x2 : Sleep(Force PFM mode) - LPM
		 *			0x3 : n/a				0x3 : n/a
		 *
		 *	[ BUCK 1 ]	0x0 : Off
		 *				0x1 : On 	(reference VBUCK1     reg[0x002E])
		 *				0x2 : Sleep 	(reference VBUCK1_RET reg[0x0061])
		 *				0x3 : On 	(reference VBUCK1_TUR reg[0x0062])
		 *
		 *		
		 * ---------------------------------------------------------------	
		 * [PC2|PC1]	11	|	10	|	01	|	00
		 * ---------------------------------------------------------------
		 *	[MCTL]	M3	|	M2	|	M1	|	M0
		 * ---------------------------------------------------------------
		 *	0xDE :	11		01		11		10	(TUR, ON , TUR, LPM)
		 *	0xCD :	11		00		11		01	(TUR, OFF, TUR, ON )
		 *
		 *	0x00 :	00		00		00		00	(OFF, OFF, OFF, OFF)
		 *	0x66 :	01		10		01		10	(ON , LPM, ON , LPM)
		 *	0x44 :	01		00		01		00	(ON , OFF, ON , OFF)
		 * ---------------------------------------------------------------
		 *
		 * NEVIS use M3 and M0
		*/
		D2083_MCTL_MODE_INIT(D2083_BUCK_1, 0xDE, D2083_REGULATOR_LPM_IN_DSM),	// VDD_CORE - CSR
		D2083_MCTL_MODE_INIT(D2083_BUCK_2, 0x56, D2083_REGULATOR_MAX), // VDD_IO_1.8V / VRF_1.8V - IOSR
		D2083_MCTL_MODE_INIT(D2083_BUCK_3, 0x56, D2083_REGULATOR_MAX), // VDD_IO_1.2V - TSR
		D2083_MCTL_MODE_INIT(D2083_BUCK_4, 0x44, D2083_REGULATOR_MAX),	// VDD_3G_PAM_3.3V - used.
		
		D2083_MCTL_MODE_INIT(D2083_LDO_1,  0x00, D2083_REGULATOR_MAX),			// Not used.
		D2083_MCTL_MODE_INIT(D2083_LDO_2,  0x56, D2083_REGULATOR_LPM_IN_DSM),	// VRF_2.7V
		D2083_MCTL_MODE_INIT(D2083_LDO_3,  0x55, D2083_REGULATOR_LPM_IN_DSM),	// VDD_SENSOR_3.0V
		D2083_MCTL_MODE_INIT(D2083_LDO_4,  0x44, D2083_REGULATOR_OFF_IN_DSM),	// VDD_AUD_2.9V
		D2083_MCTL_MODE_INIT(D2083_LDO_5,  0x66, D2083_REGULATOR_LPM_IN_DSM),	// VDD_USB_3.3V
		D2083_MCTL_MODE_INIT(D2083_LDO_6,  0x00, D2083_REGULATOR_OFF_IN_DSM),	// VCAM_A_2.8V
		D2083_MCTL_MODE_INIT(D2083_LDO_7,  0x00, D2083_REGULATOR_OFF_IN_DSM),	// VDD_VIB_3.3V
		D2083_MCTL_MODE_INIT(D2083_LDO_8,  0x56, D2083_REGULATOR_LPM_IN_DSM),	// VLCD_3.0V
		D2083_MCTL_MODE_INIT(D2083_LDO_9,  0x66, D2083_REGULATOR_ON_IN_DSM),	// VDD_SDXC
		D2083_MCTL_MODE_INIT(D2083_LDO_10, 0x00, D2083_REGULATOR_OFF_IN_DSM),	// KEY_LED_3.3V
		D2083_MCTL_MODE_INIT(D2083_LDO_11, 0x00, D2083_REGULATOR_LPM_IN_DSM),	// VSIM1_3.0V
		D2083_MCTL_MODE_INIT(D2083_LDO_12, 0x66, D2083_REGULATOR_LPM_IN_DSM),	// VDD_EMMC_3.0V
		D2083_MCTL_MODE_INIT(D2083_LDO_13, 0x00, D2083_REGULATOR_LPM_IN_DSM),	// VDD_SDIO_3.0V
#ifdef CONFIG_MACH_RHEA_SS_NEVISDS
		D2083_MCTL_MODE_INIT(D2083_LDO_14, 0x00, D2083_REGULATOR_LPM_IN_DSM),	// VSIM2_3.0V
#else
		D2083_MCTL_MODE_INIT(D2083_LDO_14, 0x00, D2083_REGULATOR_OFF_IN_DSM),	// Not used
#endif
		D2083_MCTL_MODE_INIT(D2083_LDO_15, 0x54, D2083_REGULATOR_OFF_IN_DSM),	// VTOUCH_3.3V
		D2083_MCTL_MODE_INIT(D2083_LDO_16, 0x00, D2083_REGULATOR_OFF_IN_DSM),	// VCAM_IO_1.8V
		D2083_MCTL_MODE_INIT(D2083_LDO_17, 0x00, D2083_REGULATOR_OFF_IN_DSM),	// VCAM_AF_2.8V
		D2083_MCTL_MODE_INIT(D2083_LDO_AUD,0x00, D2083_REGULATOR_OFF_IN_DSM),	// Internal Audio LDO
	},
};

#else /* CONFIG_MFD_D2083 */
static struct bcmpmu_platform_data bcmpmu_plat_data = {
	.i2c_pdata = { ADD_I2C_SLAVE_SPEED(BSC_BUS_SPEED_400K), },
	.init = bcmpmu_init_platform_hw,
	.exit = bcmpmu_exit_platform_hw,
	.i2c_board_info_map1 = &pmu_info_map1,
	.i2c_adapter_id = PMU_DEVICE_I2C_BUSNO,
	.i2c_pagesize = 256,
	.init_data = &register_init_data[0],
	/* # of registers defined in register_init_data.
	   This value will come from device tree */
	.init_max = ARRAY_SIZE(register_init_data),
	.batt_temp_in_celsius = 1,
	.batt_temp_map = &batt_temp_map[0],
	.batt_temp_map_len = ARRAY_SIZE(batt_temp_map),
	.adc_setting = &adc_setting,
	.num_of_regl = ARRAY_SIZE(bcm59039_regulators),
	.regulator_init_data = &bcm59039_regulators[0],
	.fg_smpl_rate = 2083,
	.fg_slp_rate = 32000,
	.fg_slp_curr_ua = 1220,
	.fg_factor = 820,
	.fg_sns_res = 10,
	.batt_voltcap_map = &batt_voltcap_map[0],
	.batt_voltcap_map_len = ARRAY_SIZE(batt_voltcap_map),
	.batt_impedence = 140,
	.sys_impedence = 35,
	.chrg_1c_rate = 1300,
	.chrg_eoc = 100,
	.support_hw_eoc = 0,
	.chrg_zone_map = &chrg_zone[0],
	.fg_capacity_full = (1300) * 3600,
	.support_fg = 1,
	.support_chrg_maint = 1,
	.wd_setting = &bcm59039_wd_setting,
	.chrg_resume_lvl = 4152, /* 99% = 4160 - (4160-4122)/5 * 1*/
	.fg_support_tc = 1,
	.fg_tc_dn_lvl = 50, /* 5c */
	.fg_tc_up_lvl = 200, /* 20c */
	.fg_zone_settle_tm = 60,
	.fg_zone_info = &fg_zone[0],
	.fg_poll_hbat = 112000,
	.fg_poll_lbat = 1000, // SS PGM poll rate = 1sec
	.fg_lbat_lvl = 3490,  /* <= 2% */
	.fg_fbat_lvl = 4152,  /* >= 99% */
	.fg_low_cal_lvl = 3550,
	.bc = BCMPMU_BC_PMU_BC12,
	.batt_model = "SS,1300mAH",
	.cutoff_volt = 3400,  /* 0% capacity */
	.cutoff_count_max = 3,
	.hard_reset_en = -1,
	.restart_en = -1,
	.pok_hold_deb = -1,
	.pok_shtdwn_dly = -1,
	.pok_restart_dly = -1,
	.pok_restart_deb = -1,
	.ihf_autoseq_dis = 1,
	.pok_lock = 1, /*Keep ponkey locked by default*/
#ifdef CONFIG_CHARGER_BCMPMU_SPA
	.piggyback_chrg = 1,
	.piggyback_chrg_name = "bcm59039_charger",
	.piggyback_notify = notify_spa,
	.piggyback_work = NULL,
	.spafifo = NULL,
	.spalock = NULL,
#endif
};
#endif	/* D2083 end */

/* Add for Ricktek RT8969 */

#if 0 //defined(CONFIG_FSA9480_MICROUSB)	//	// TODO:
/* VBUS_DETECT */
struct d1980_vbus_pdata ttc_dkb_vbus = {
	//.supply         = PM860X_GPIO2_SUPPLY_VBUS,
	//.idpin          = PM860X_IDPIN_NO_USE,
#if 0	// TODO
	.reg_base       = PXA168_U2O_REGBASE,
	.reg_end        = PXA168_U2O_REGBASE + USB_REG_RANGE,
#endif
};

static struct fsa9480_platform_data FSA9480_info = {
	.vbus		= &ttc_dkb_vbus
};
#endif


#ifdef CONFIG_MFD_D2083
static struct i2c_board_info __initdata pmu_info[] = {
	{
		I2C_BOARD_INFO("d2083", PMU_DEVICE_I2C_ADDR),
		.platform_data = &d2083_pdata,
		.irq = gpio_to_irq(PMU_DEVICE_INT_GPIO),
	},
};


/*800 Mhz CSR voltage definitions....*/

#define CSR_VAL_RETN_SS_800M		0x10 /*0.88V -> 0.90V*/
#define CSR_VAL_RETN_TT_800M		0x10 /*0.88V -> 0.90V*/
#define CSR_VAL_RETN_FF_800M		0x10 /*0.88V -> 0.90V*/

#define CSR_VAL_ECO_SS_800M		0x18 /*1.08V -> 1.10V*/
#define CSR_VAL_ECO_TT_800M		0x14 /*0.98V -> 1.00V*/
#define CSR_VAL_ECO_FF_800M		0x14 /*0.98V -> 1.00V*/

#define CSR_VAL_NRML_SS_800M		0x1B /*1.16V -> 1.175V*/
#define CSR_VAL_NRML_TT_800M		0x16 /*1.04V -> 1.050V*/
#define CSR_VAL_NRML_FF_800M		0x14 /*0.98V -> 1.00V*/

#define CSR_VAL_TURBO_SS_800M		0x22 /*1.34V -> 1.350V*/
#define B0_CSR_VAL_TURBO_SS_800M	0x21 /*1.32V -> 1.325V*/
#define CSR_VAL_TURBO_TT_800M		0x1D /*1.22V -> 1.225V*/
#define CSR_VAL_TURBO_FF_800M		0x19 /*1.12V -> 1.125V*/
#else
static struct i2c_board_info __initdata pmu_info[] = {
	{
		I2C_BOARD_INFO("bcmpmu", PMU_DEVICE_I2C_ADDR),
		.platform_data = &bcmpmu_plat_data,
		.irq = gpio_to_irq(PMU_DEVICE_INT_GPIO),
	},
};

/*800 Mhz CSR voltage definitions....*/

#define CSR_VAL_RETN_SS_800M	0x3 /*0.88V*/
#define CSR_VAL_RETN_TT_800M	0x3 /*0.88V*/
#define CSR_VAL_RETN_FF_800M	0x3 /*0.88V*/

#define CSR_VAL_ECO_SS_800M		0xd /*1.08V*/
#define CSR_VAL_ECO_TT_800M		0x8 /*0.98V*/
#define CSR_VAL_ECO_FF_800M		0x8 /*0.98V*/

#define CSR_VAL_NRML_SS_800M	0x11 /*1.16V*/
#define CSR_VAL_NRML_TT_800M	0x0b /*1.04V*/
#define CSR_VAL_NRML_FF_800M	0x8 /*0.98V*/

#define CSR_VAL_TURBO_SS_800M		0x1A /*1.34V*/
#define B0_CSR_VAL_TURBO_SS_800M	0x19 /*1.32V*/
#define CSR_VAL_TURBO_TT_800M		0x14 /*1.22V*/
#define CSR_VAL_TURBO_FF_800M		0x0F /*1.12V*/
#endif



#define PMU_CSR_VLT_TBL_SS_800M	ARRAY_LIST(\
					CSR_VAL_RETN_SS_800M,\
					CSR_VAL_RETN_SS_800M,\
					CSR_VAL_RETN_SS_800M,\
					CSR_VAL_RETN_SS_800M,\
					CSR_VAL_RETN_SS_800M,\
					CSR_VAL_RETN_SS_800M,\
					CSR_VAL_RETN_SS_800M,\
					CSR_VAL_RETN_SS_800M,\
					CSR_VAL_ECO_SS_800M,\
					CSR_VAL_ECO_SS_800M,\
					CSR_VAL_ECO_SS_800M,\
					CSR_VAL_NRML_SS_800M,\
					CSR_VAL_NRML_SS_800M,\
					CSR_VAL_NRML_SS_800M,\
					CSR_VAL_TURBO_SS_800M,\
					CSR_VAL_TURBO_SS_800M)


#define PMU_CSR_VLT_TBL_TT_800M	ARRAY_LIST(\
					CSR_VAL_RETN_TT_800M,\
					CSR_VAL_RETN_TT_800M,\
					CSR_VAL_RETN_TT_800M,\
					CSR_VAL_RETN_TT_800M,\
					CSR_VAL_RETN_TT_800M,\
					CSR_VAL_RETN_TT_800M,\
					CSR_VAL_RETN_TT_800M,\
					CSR_VAL_RETN_TT_800M,\
					CSR_VAL_ECO_TT_800M,\
					CSR_VAL_ECO_TT_800M,\
					CSR_VAL_ECO_TT_800M,\
					CSR_VAL_NRML_TT_800M,\
					CSR_VAL_NRML_TT_800M,\
					CSR_VAL_NRML_TT_800M,\
					CSR_VAL_TURBO_TT_800M,\
					CSR_VAL_TURBO_TT_800M)

#define PMU_CSR_VLT_TBL_FF_800M	ARRAY_LIST(\
					CSR_VAL_RETN_FF_800M,\
					CSR_VAL_RETN_FF_800M,\
					CSR_VAL_RETN_FF_800M,\
					CSR_VAL_RETN_FF_800M,\
					CSR_VAL_RETN_FF_800M,\
					CSR_VAL_RETN_FF_800M,\
					CSR_VAL_RETN_FF_800M,\
					CSR_VAL_RETN_FF_800M,\
					CSR_VAL_ECO_FF_800M,\
					CSR_VAL_ECO_FF_800M,\
					CSR_VAL_ECO_FF_800M,\
					CSR_VAL_NRML_FF_800M,\
					CSR_VAL_NRML_FF_800M,\
					CSR_VAL_NRML_FF_800M,\
					CSR_VAL_TURBO_FF_800M,\
					CSR_VAL_TURBO_FF_800M)

#ifdef CONFIG_MFD_D2083
/*850 Mhz CSR voltage definitions....*/

#if 0	// TEST ONLY
#define CSR_VAL_RETN_SS_850M		0x10 /*0.88V -> 0.90V*/
#define CSR_VAL_RETN_TT_850M		0x24	// 1.4v // 0x10 /*0.88V -> 0.90V*/
#define CSR_VAL_RETN_FF_850M		0x10 /*0.88V -> 0.90V*/

#define CSR_VAL_ECO_SS_850M		0x18 /*1.08V -> 1.10V*/
#define CSR_VAL_ECO_TT_850M		0x2C	// 1.6v	// 0x14 /*0.98V -> 1.00V*/
#define CSR_VAL_ECO_FF_850M		0x14 /*0.98V -> 1.00V*/

#define CSR_VAL_NRML_SS_850M		0x1B /*1.16V -> 1.175V*/
#define CSR_VAL_NRML_TT_850M		0x34	// 1.8v // 0x16 /*1.04V -> 1.050V*/
#define CSR_VAL_NRML_FF_850M		0x14 /*0.98V -> 1.00V*/

#define CSR_VAL_TURBO_SS_850M		0x23 /*1.36V -> 1.375V*/
#define B0_CSR_VAL_TURBO_SS_850M	0x21 /*1.32V -> 1.325V*/
#define CSR_VAL_TURBO_TT_850M		0x3C	// 2.0v	//0x1E /*1.24V -> 1.250V*/
#define CSR_VAL_TURBO_FF_850M		0x1A /*1.14V -> 1.150V*/
#else
#define CSR_VAL_RETN_SS_850M		0x10 /*0.88V -> 0.90V*/
#define CSR_VAL_RETN_TT_850M		0x10 /*0.88V -> 0.90V*/
#define CSR_VAL_RETN_FF_850M		0x10 /*0.88V -> 0.90V*/

#define CSR_VAL_ECO_SS_850M		0x18 /*1.08V -> 1.10V*/
#define CSR_VAL_ECO_TT_850M		0x14 /*0.98V -> 1.00V*/
#define CSR_VAL_ECO_FF_850M		0x14 /*0.98V -> 1.00V*/

#define CSR_VAL_NRML_SS_850M		0x1B /*1.16V -> 1.175V*/
#define CSR_VAL_NRML_TT_850M		0x16 /*1.04V -> 1.050V*/
#define CSR_VAL_NRML_FF_850M		0x14 /*0.98V -> 1.00V*/

#define CSR_VAL_TURBO_SS_850M		0x23 /*1.36V -> 1.375V*/
#define B0_CSR_VAL_TURBO_SS_850M	0x21 /*1.32V -> 1.325V*/
#define CSR_VAL_TURBO_TT_850M		0x1E /*1.24V -> 1.250V*/
#define CSR_VAL_TURBO_FF_850M		0x1A /*1.14V -> 1.150V*/
#endif	// TEST ONLY

#else
/*850 Mhz CSR voltage definitions....*/

#define CSR_VAL_RETN_SS_850M	0x3 /*0.88V*/
#define CSR_VAL_RETN_TT_850M	0x3 /*0.88V*/
#define CSR_VAL_RETN_FF_850M	0x3 /*0.88V*/

#define CSR_VAL_ECO_SS_850M		0xd /*1.08V*/
#define CSR_VAL_ECO_TT_850M		0x8 /*0.98V*/
#define CSR_VAL_ECO_FF_850M		0x8 /*0.98V*/

#define CSR_VAL_NRML_SS_850M	0x11 /*1.16V*/
#define CSR_VAL_NRML_TT_850M	0x0b /*1.04V*/
#define CSR_VAL_NRML_FF_850M	0x8 /*0.98V*/

#define CSR_VAL_TURBO_SS_850M		0x1B /*1.36V*/
#define B0_CSR_VAL_TURBO_SS_850M	0x19 /*1.32V*/
#define CSR_VAL_TURBO_TT_850M		0x15 /*1.24V*/
#define CSR_VAL_TURBO_FF_850M		0x10 /*1.14V*/
#endif


#define PMU_CSR_VLT_TBL_SS_850M	ARRAY_LIST(\
					CSR_VAL_RETN_SS_850M,\
					CSR_VAL_RETN_SS_850M,\
					CSR_VAL_RETN_SS_850M,\
					CSR_VAL_RETN_SS_850M,\
					CSR_VAL_RETN_SS_850M,\
					CSR_VAL_RETN_SS_850M,\
					CSR_VAL_RETN_SS_850M,\
					CSR_VAL_RETN_SS_850M,\
					CSR_VAL_ECO_SS_850M,\
					CSR_VAL_ECO_SS_850M,\
					CSR_VAL_ECO_SS_850M,\
					CSR_VAL_NRML_SS_850M,\
					CSR_VAL_NRML_SS_850M,\
					CSR_VAL_NRML_SS_850M,\
					CSR_VAL_TURBO_SS_850M,\
					CSR_VAL_TURBO_SS_850M)


#define PMU_CSR_VLT_TBL_TT_850M	ARRAY_LIST(\
					CSR_VAL_RETN_TT_850M,\
					CSR_VAL_RETN_TT_850M,\
					CSR_VAL_RETN_TT_850M,\
					CSR_VAL_RETN_TT_850M,\
					CSR_VAL_RETN_TT_850M,\
					CSR_VAL_RETN_TT_850M,\
					CSR_VAL_RETN_TT_850M,\
					CSR_VAL_RETN_TT_850M,\
					CSR_VAL_ECO_TT_850M,\
					CSR_VAL_ECO_TT_850M,\
					CSR_VAL_ECO_TT_850M,\
					CSR_VAL_NRML_TT_850M,\
					CSR_VAL_NRML_TT_850M,\
					CSR_VAL_NRML_TT_850M,\
					CSR_VAL_TURBO_TT_850M,\
					CSR_VAL_TURBO_TT_850M)

#define PMU_CSR_VLT_TBL_FF_850M	ARRAY_LIST(\
						CSR_VAL_RETN_FF_850M,\
						CSR_VAL_RETN_FF_850M,\
						CSR_VAL_RETN_FF_850M,\
						CSR_VAL_RETN_FF_850M,\
						CSR_VAL_RETN_FF_850M,\
						CSR_VAL_RETN_FF_850M,\
						CSR_VAL_RETN_FF_850M,\
						CSR_VAL_RETN_FF_850M,\
						CSR_VAL_ECO_FF_850M,\
						CSR_VAL_ECO_FF_850M,\
						CSR_VAL_ECO_FF_850M,\
						CSR_VAL_NRML_FF_850M,\
						CSR_VAL_NRML_FF_850M,\
						CSR_VAL_NRML_FF_850M,\
						CSR_VAL_TURBO_FF_850M,\
						CSR_VAL_TURBO_FF_850M)

#ifdef CONFIG_MFD_D2083
/*1 Ghz CSR voltage definitions....*/

#define CSR_VAL_RETN_SS_1G		0x10 /*0.88V -> 0.90V*/
#define CSR_VAL_RETN_TT_1G		0x10 /*0.88V -> 0.90V*/
#define CSR_VAL_RETN_FF_1G		0x10 /*0.88V -> 0.90V*/

#define CSR_VAL_ECO_SS_1G		0x18 /*1.08V -> 1.10V*/
#define CSR_VAL_ECO_TT_1G		0x14 /*0.98V -> 1.00V*/
#define CSR_VAL_ECO_FF_1G		0x14 /*0.98V -> 1.00V*/

#define CSR_VAL_NRML_SS_1G		0x1C /*0x1B 1.16V -> 1.175V -> 1.20V*/
#define CSR_VAL_NRML_TT_1G		0x1C /*0x16 1.04V -> 1.050V -> 1.20V*/
#define CSR_VAL_NRML_FF_1G		0x18 /*0x14 0.98V -> 1.00V -> 1.10V*/

#define CSR_VAL_TURBO_SS_1G		0x23 /*1.36V -> 1.375V*/
#define B0_CSR_VAL_TURBO_SS_1G		0x21 /*1.32V -> 1.325V*/
#define CSR_VAL_TURBO_TT_1G		0x23 /*1.36V -> 1.375V*/
#define B0_CSR_VAL_TURBO_TT_1G		0x21 /*1.32V -> 1.325V*/
#define CSR_VAL_TURBO_FF_1G		0x1F /*0x1E 1.24V -> 1.25V -> 1.275V*/
#else
/*1 Ghz CSR voltage definitions....*/

#define CSR_VAL_RETN_SS_1G	0x3 /*0.88V*/
#define CSR_VAL_RETN_TT_1G	0x3 /*0.88V*/
#define CSR_VAL_RETN_FF_1G	0x3 /*0.88V*/

#define CSR_VAL_ECO_SS_1G	0xd /*1.08V*/
#define CSR_VAL_ECO_TT_1G	0x8 /*0.98V*/
#define CSR_VAL_ECO_FF_1G	0x8 /*0.98V*/

#define CSR_VAL_NRML_SS_1G	0x11 /*1.16V*/
#define CSR_VAL_NRML_TT_1G	0x0b /*1.04V*/
#define CSR_VAL_NRML_FF_1G	0x8	/*0.98V*/

#define CSR_VAL_TURBO_SS_1G		0x1B /*1.36V*/
#define B0_CSR_VAL_TURBO_SS_1G	0x19 /*1.32V*/
#define CSR_VAL_TURBO_TT_1G		0x1B /*1.36V*/
#define B0_CSR_VAL_TURBO_TT_1G	0x19 /*1.32V*/
#define CSR_VAL_TURBO_FF_1G		0x15 /*1.24V*/
#endif


#define PMU_CSR_VLT_TBL_SS_1G	ARRAY_LIST(\
						CSR_VAL_RETN_SS_1G,\
						CSR_VAL_RETN_SS_1G,\
						CSR_VAL_RETN_SS_1G,\
						CSR_VAL_RETN_SS_1G,\
						CSR_VAL_RETN_SS_1G,\
						CSR_VAL_RETN_SS_1G,\
						CSR_VAL_RETN_SS_1G,\
						CSR_VAL_RETN_SS_1G,\
						CSR_VAL_ECO_SS_1G,\
						CSR_VAL_ECO_SS_1G,\
						CSR_VAL_ECO_SS_1G,\
						CSR_VAL_NRML_SS_1G,\
						CSR_VAL_NRML_SS_1G,\
						CSR_VAL_NRML_SS_1G,\
						CSR_VAL_TURBO_SS_1G,\
						CSR_VAL_TURBO_SS_1G)

#define PMU_CSR_VLT_TBL_TT_1G	ARRAY_LIST(\
						CSR_VAL_RETN_TT_1G,\
						CSR_VAL_RETN_TT_1G,\
						CSR_VAL_RETN_TT_1G,\
						CSR_VAL_RETN_TT_1G,\
						CSR_VAL_RETN_TT_1G,\
						CSR_VAL_RETN_TT_1G,\
						CSR_VAL_RETN_TT_1G,\
						CSR_VAL_RETN_TT_1G,\
						CSR_VAL_ECO_TT_1G,\
						CSR_VAL_ECO_TT_1G,\
						CSR_VAL_ECO_TT_1G,\
						CSR_VAL_NRML_TT_1G,\
						CSR_VAL_NRML_TT_1G,\
						CSR_VAL_NRML_TT_1G,\
						CSR_VAL_TURBO_TT_1G,\
						CSR_VAL_TURBO_TT_1G)

#define PMU_CSR_VLT_TBL_FF_1G	ARRAY_LIST(\
						CSR_VAL_RETN_FF_1G,\
						CSR_VAL_RETN_FF_1G,\
						CSR_VAL_RETN_FF_1G,\
						CSR_VAL_RETN_FF_1G,\
						CSR_VAL_RETN_FF_1G,\
						CSR_VAL_RETN_FF_1G,\
						CSR_VAL_RETN_FF_1G,\
						CSR_VAL_RETN_FF_1G,\
						CSR_VAL_ECO_FF_1G,\
						CSR_VAL_ECO_FF_1G,\
						CSR_VAL_ECO_FF_1G,\
						CSR_VAL_NRML_FF_1G,\
						CSR_VAL_NRML_FF_1G,\
						CSR_VAL_NRML_FF_1G,\
						CSR_VAL_TURBO_FF_1G,\
						CSR_VAL_TURBO_FF_1G)

u8 csr_vlt_table_ss[A9_FREQ_MAX][SR_VLT_LUT_SIZE] = {
	[A9_FREQ_800_MHZ]	= PMU_CSR_VLT_TBL_SS_800M,
	[A9_FREQ_850_MHZ]	= PMU_CSR_VLT_TBL_SS_850M,
	[A9_FREQ_1_GHZ]		= PMU_CSR_VLT_TBL_SS_1G,
};

u8 csr_vlt_table_tt[A9_FREQ_MAX][SR_VLT_LUT_SIZE] = {
	[A9_FREQ_800_MHZ]	= PMU_CSR_VLT_TBL_TT_800M,
	[A9_FREQ_850_MHZ]	= PMU_CSR_VLT_TBL_TT_850M,
	[A9_FREQ_1_GHZ]		= PMU_CSR_VLT_TBL_TT_1G,
};

u8 csr_vlt_table_ff[A9_FREQ_MAX][SR_VLT_LUT_SIZE] = {
	[A9_FREQ_800_MHZ]	= PMU_CSR_VLT_TBL_FF_800M,
	[A9_FREQ_850_MHZ]	= PMU_CSR_VLT_TBL_FF_850M,
	[A9_FREQ_1_GHZ]		= PMU_CSR_VLT_TBL_FF_1G,
};

#ifdef CONFIG_MFD_D2083	
extern void d2083_set_sType(u32 s_type);
#endif

const u8 *bcmpmu_get_sr_vlt_table(int sr, u32 freq_inx,
						u32 silicon_type)
{
	pr_info("%s:sr = %i, freq_inx = %d,"
			"silicon_type = %d\n", __func__,
			sr, freq_inx, silicon_type);

	//BUG_ON(freq_inx != A9_FREQ_850_MHZ);

#ifdef CONFIG_KONA_AVS
#ifdef CONFIG_MFD_D2083
	d2083_set_sType(silicon_type);
#endif

	switch (silicon_type) {
	case SILICON_TYPE_SLOW:
  		return &csr_vlt_table_ss[freq_inx][0];
	case SILICON_TYPE_TYPICAL:
  		return &csr_vlt_table_tt[freq_inx][0];
	case SILICON_TYPE_FAST:
  		return &csr_vlt_table_ff[freq_inx][0];
	default:
		BUG();
	}
#else
	return csr_vlt_table_ss;
#endif
}

#if !defined(CONFIG_MFD_D2083)
int bcmpmu_init_platform_hw(struct bcmpmu *bcmpmu)
{
	int             i;
	printk(KERN_INFO "%s: called.\n", __func__);

	/* Samsung requirement for PMU restart should be enabled.
	 * Will get configured only 59039C0 or above version
	*/

	if (bcmpmu->rev_info.dig_rev >= BCM59039_CO_DIG_REV) {
		//bcmpmu->pdata->restart_en = 1;
		bcmpmu->pdata->pok_restart_dly = POK_RESTRT_DLY_1SEC;
		bcmpmu->pdata->pok_restart_deb = POK_RESTRT_DEB_8SEC;
		bcmpmu->pdata->pok_lock = 1;
		//bcmpmu->pdata->hard_reset_en = 0;
	}

	for (i = 0; i < ARRAY_SIZE(bcmpmu_client_devices); i++)
		bcmpmu_client_devices[i]->dev.platform_data = bcmpmu;
	platform_add_devices(bcmpmu_client_devices,
			ARRAY_SIZE(bcmpmu_client_devices));

	return 0;
}
#endif	// CONFIG_MFD_D2083

#ifdef CONFIG_MFD_D2083
int d2083_init_platform_hw(void)
{
	printk(KERN_INFO "%s: called.\n", __func__);	
#if 0
	memset(&csr_vlt_table_ss[A9_FREQ_800_MHZ][SR_TURBO_INX_START],
		B0_CSR_VAL_TURBO_SS_800M,
		(SR_TURBO_INX_END - SR_TURBO_INX_START) + 1);

	memset(&csr_vlt_table_ss[A9_FREQ_850_MHZ][SR_TURBO_INX_START],
		B0_CSR_VAL_TURBO_SS_850M,
		(SR_TURBO_INX_END - SR_TURBO_INX_START) + 1);

	memset(&csr_vlt_table_ss[A9_FREQ_1_GHZ][SR_TURBO_INX_START],
		B0_CSR_VAL_TURBO_SS_1G,
		(SR_TURBO_INX_END - SR_TURBO_INX_START) + 1);

	memset(&csr_vlt_table_tt[A9_FREQ_1_GHZ][SR_TURBO_INX_START],
		B0_CSR_VAL_TURBO_TT_1G,
		(SR_TURBO_INX_END - SR_TURBO_INX_START) + 1);
#endif

	vlt_tbl_init = 1;

	return 0;
}
#endif

__init int board_pmu_init(void)
{
	int             ret;
	int             irq;

#ifdef CONFIG_KONA_DT_BCMPMU
	bcmpmu_update_pdata_dt_batt(&bcmpmu_plat_data);
	bcmpmu_update_pdata_dt_pmu(&bcmpmu_plat_data);
#endif
		printk("%s  : %d\n", __FUNCTION__, __LINE__ );
#ifdef CONFIG_MFD_D2083
	d2083_init_platform_hw();
	ret = gpio_request(PMU_DEVICE_INT_GPIO, "d2083-irq");
#else
	ret = gpio_request(PMU_DEVICE_INT_GPIO, "bcmpmu-irq");
#endif
	if (ret < 0) {

		printk(KERN_ERR "%s filed at gpio_request.\n", __func__);
		goto exit;
	}
	ret = gpio_direction_input(PMU_DEVICE_INT_GPIO);
	if (ret < 0) {

		printk(KERN_ERR "%s filed at gpio_direction_input.\n",
				__func__);
		goto exit;
	}
	gpio_set_value(PMU_DEVICE_INT_GPIO,1);
	irq = gpio_to_irq(PMU_DEVICE_INT_GPIO);
#ifdef CONFIG_MFD_D2083
	d2083_pdata.irq_base = irq;
#else
	bcmpmu_plat_data.irq = irq;
#endif

	i2c_register_board_info(PMU_DEVICE_I2C_BUSNO,
				pmu_info, ARRAY_SIZE(pmu_info));
exit:
	return ret;
}


arch_initcall(board_pmu_init);
