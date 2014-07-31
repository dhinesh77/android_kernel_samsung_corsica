/*
 * pmic.h  --  Power Managment Driver for Dialog D2083 PMIC
 *
 * Copyright 2012 Dialog Semiconductor Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_D2083_PMIC_H
#define __LINUX_D2083_PMIC_H

#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>

/*
 * Register values.
 */

enum d2083_regulator_id {
/*
 * DC-DC's
 */
    D2083_BUCK_1    = 0,
    D2083_BUCK_2,   /* 1 */
    D2083_BUCK_3,   /* 2 */
    D2083_BUCK_4,   /* 3 */    
/*
 * LDOs
 */
    D2083_LDO_1,    /* 4 */
    D2083_LDO_2,    /* 5 */
    D2083_LDO_3,    /* 6 */
    D2083_LDO_4,    /* 7 */
    D2083_LDO_5,    /* 8 */
    D2083_LDO_6,    /* 9 */
    D2083_LDO_7,    /* 10 */
    D2083_LDO_8,    /* 11 */
    D2083_LDO_9,    /* 12 */
    D2083_LDO_10,   /* 13 */
    
    D2083_LDO_11,   /* 14 */
    D2083_LDO_12,   /* 15 */
    D2083_LDO_13,   /* 16 */
    D2083_LDO_14,   /* 17 */
    D2083_LDO_15,   /* 18 */
    D2083_LDO_16,   /* 19 */
    D2083_LDO_17,   /* 20 */
    D2083_LDO_AUD,  /* 24 */

    D2083_NUMBER_OF_REGULATORS  /* 25 */
};


//TODO MW: Check Modes when final design is ready
/* Global MCTL state controled by HW (M_CTL1 and M_CTL2 signals) */
enum d2083_mode_control {
    MCTL_0, /* M_CTL1 = 0, M_CTL2 = 0 */
    MCTL_1, /* M_CTL1 = 0, M_CTL2 = 1 */
    MCTL_2, /* M_CTL1 = 1, M_CTL2 = 0 */
    MCTL_3, /* M_CTL1 = 1, M_CTL2 = 1 */
};

/*regualtor DSM settings */
enum d2083_mode_in_dsm {
    D2083_REGULATOR_LPM_IN_DSM = 0,  /* LPM in DSM(deep sleep mode) */
    D2083_REGULATOR_OFF_IN_DSM,      /* OFF in DSM */
    D2083_REGULATOR_ON_IN_DSM,       /* LPM in DSM */
    D2083_REGULATOR_MAX,
};

/* TODO MW: move those to d2083_reg.h - this are bimask specific to registers' map */
/* CONF and EN is same for all */
#define D2083_REGULATOR_CONF            (1<<7)
#define D2083_REGULATOR_EN              (1<<6)

/* TODO MW: description*/
#define D2083_REGULATOR_MCTL3           (3<<6)
#define D2083_REGULATOR_MCTL2           (3<<4)
#define D2083_REGULATOR_MCTL1           (3<<2)
#define D2083_REGULATOR_MCTL0           (3<<0)



/* TODO MW:  figure out more descriptive names to distinguish between global M_CTLx state
 * determined by hardware, and the mode for each regulator configured in BUCKx/LDOx_MCTLy register */

/* MCTL values for regulator bits ... TODO finish description */
#define REGULATOR_MCTL_OFF              0
#define REGULATOR_MCTL_ON               1
#define REGULATOR_MCTL_SLEEP            2
#define REGULATOR_MCTL_TURBO            3 /* Available only for BUCK1 */

#define MCTL3_SHIFT                     6 /* Bits [7:6] in BUCKx/LDOx_MCTLy register */
#define MCTL2_SHIFT                     4 /* Bits [5:4] in BUCKx/LDOx_MCTLy register */
#define MCTL1_SHIFT                     2 /* Bits [3:2] in BUCKx/LDOx_MCTLy register */
#define MCTL0_SHIFT                     0 /* Bits [1:0] in BUCKx/LDOx_MCTLy register */

/* When M_CTL1 = 1, M_CTL2 = 1 (M_CTL3: global Turbo Mode), regultor is: */
#define D2083_REGULATOR_MCTL3_OFF       (REGULATOR_MCTL_OFF     << MCTL3_SHIFT/*MCTL0_SHIFT*/)
#define D2083_REGULATOR_MCTL3_ON        (REGULATOR_MCTL_ON      << MCTL3_SHIFT/*MCTL1_SHIFT*/)
#define D2083_REGULATOR_MCTL3_SLEEP     (REGULATOR_MCTL_SLEEP   << MCTL3_SHIFT/*MCTL2_SHIFT*/)
#define D2083_REGULATOR_MCTL3_TURBO     (REGULATOR_MCTL_TURBO   << MCTL3_SHIFT)

/* When M_CTL1 = 1, M_CTL2 = 0 (M_CTL2: TBD: To Be Defined Mode), regulator is: */ //TODO MW: change name when mode defined
#define D2083_REGULATOR_MCTL2_OFF       (REGULATOR_MCTL_OFF     << MCTL2_SHIFT/*MCTL0_SHIFT*/)
#define D2083_REGULATOR_MCTL2_ON        (REGULATOR_MCTL_ON      << MCTL2_SHIFT/*MCTL1_SHIFT*/)
#define D2083_REGULATOR_MCTL2_SLEEP     (REGULATOR_MCTL_SLEEP   << MCTL2_SHIFT)
#define D2083_REGULATOR_MCTL2_TURBO     (REGULATOR_MCTL_TURBO   << MCTL2_SHIFT/*MCTL3_SHIFT*/)

/* When M_CTL1 = 0, M_CTL2 = 1 (M_CTL1: Normal Mode), regulator is: */
#define D2083_REGULATOR_MCTL1_OFF       (REGULATOR_MCTL_OFF     << MCTL1_SHIFT/*MCTL0_SHIFT*/)
#define D2083_REGULATOR_MCTL1_ON        (REGULATOR_MCTL_ON      << MCTL1_SHIFT)
#define D2083_REGULATOR_MCTL1_SLEEP     (REGULATOR_MCTL_SLEEP   << MCTL1_SHIFT/*MCTL2_SHIFT*/)
#define D2083_REGULATOR_MCTL1_TURBO     (REGULATOR_MCTL_TURBO   << MCTL1_SHIFT/*MCTL3_SHIFT*/)

/* When M_CTL1 = 0, M_CTL2 = 0 (M_CTL0: Sleep Mode), regulator is: */
#define D2083_REGULATOR_MCTL0_OFF       (REGULATOR_MCTL_OFF     << MCTL0_SHIFT)
#define D2083_REGULATOR_MCTL0_ON        (REGULATOR_MCTL_ON      << MCTL0_SHIFT/*MCTL1_SHIFT*/)
#define D2083_REGULATOR_MCTL0_SLEEP     (REGULATOR_MCTL_SLEEP   << MCTL0_SHIFT/*MCTL2_SHIFT*/)
#define D2083_REGULATOR_MCTL0_TURBO     (REGULATOR_MCTL_TURBO   << MCTL0_SHIFT/*MCTL3_SHIFT*/)


/* Maximum value possible for VSEL */
#define D2083_MAX_VSEL                  0x3F  

/* Step Voltage */
#define D2083_BUCK_VOLT_STEP            25
#define D2083_LDO_VOLT_STEP             50

/* Buck Config Validation Macros */
#define D2083_BUCK12_VOLT_UPPER         2075
#define D2083_BUCK12_VOLT_LOWER         500
#define D2083_BUCK3_VOLT_UPPER          2500
#define D2083_BUCK3_VOLT_LOWER          925
#define D2083_BUCK4_VOLT_UPPER          3300	//2800
#define D2083_BUCK4_VOLT_LOWER          1200	//1225

/* Same for all LDO's */
#define D2083_LDO_VOLT_UPPER            3300
#define D2083_LDO_VOLT_LOWER            1200


/* BUCK 1 Retention and Turbo Voltage 
   These values should be changed according to Customer requirement
 */
#define D2083_BUCK1_RETENTION_VOLT 	0x10	// 0.9V       
#define D2083_BUCK1_NM_VOLT 			0x22	// 1.35v

struct d2083;
struct platform_device;
struct regulator_init_data;


struct d2083_pmic {
    /* Number of regulators of each type on this device */
    int max_dcdc;

    /* regulator devices */
    struct platform_device *pdev[D2083_NUMBER_OF_REGULATORS];
};



int             d2083_platform_regulator_init(struct d2083 *d2083);

/*
* Additional support via regulator API
*/
//int             d2083_regulator_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV);              
//int             d2083_regulator_get_voltage(struct regulator_dev *rdev);
//int             d2083_regulator_enable(struct regulator_dev *rdev);
//int             d2083_regulator_disable(struct regulator_dev *rdev);
//unsigned int    d2083_regulator_get_mode(struct regulator_dev *rdev);
//TODO unsigned int     d2083_regulator_set_mode(struct regulator_dev *rdev, unsigned int mode);
//int             d2083_regulator_is_enabled(struct regulator_dev *rdev);


#endif  /* __LINUX_D2083_PMIC_H */

