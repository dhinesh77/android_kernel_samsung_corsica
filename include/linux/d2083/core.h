/*
 * core.h  --  Core Driver for Dialog Semiconductor D2083 PMIC
 *
 * Copyright 2011 Dialog Semiconductor Ltd
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_D2083_CORE_H_
#define __LINUX_D2083_CORE_H_

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/d2083/pmic.h>
#include <linux/d2083/rtc.h>
#include <linux/d2083/audio.h>
#include <linux/power_supply.h>
#include <linux/d2083/d2083_battery.h>
#include <linux/i2c-kona.h>
//#include <plat/bcm_i2c.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif /*CONFIG_HAS_EARLYSUSPEND*/

/*
 * Register values.
 */  

#define I2C						            1

#define D2083_I2C					        "d2083"

//#define D2083_IRQ					        S3C_EINT(9)

/* Module specific error codes */
#define INVALID_REGISTER				    2
#define INVALID_READ					    3
#define INVALID_PAGE					    4

/* Total number of registers in D2083 */
#define D2083_MAX_REGISTER_CNT				(D2083_PAGE1_REG_END+1)


#define D2083_I2C_DEVICE_NAME				"d2083_i2c"
#define D2083_I2C_ADDR					    (0x90 >> 1)


#define __CONFIG_D2083_BATTERY

/*
 * DA1980 Number of Interrupts
 */
enum D2083_IRQ {
	// EVENT_A register IRQ
	D2083_IRQ_EVF = 0,
	D2083_IRQ_ETBAT2,
	D2083_IRQ_EVDD_LOW,
	D2083_IRQ_EVDD_MON,
	D2083_IRQ_EALARM,
	D2083_IRQ_ESEQRDY,
	D2083_IRQ_ETICK,

	// EVENT_B register IRQ
	D2083_IRQ_ENONKEY_LO,
	D2083_IRQ_ENONKEY_HI,
	D2083_IRQ_ENONKEY_HOLDON,
	D2083_IRQ_ENONKEY_HOLDOFF,
	D2083_IRQ_ETBAT1,
	D2083_IRQ_EADCEOM,

	// EVENT_C register IRQ
	D2083_IRQ_ETA,
	D2083_IRQ_ENJIGON,

	// EVENT_D register IRQ
	D2083_IRQ_EGPI0,

	D2083_NUM_IRQ
};

//#ifdef CONFIG_MACH_RHEA_SS_IVORY
#define D2083_IOCTL_REGL_MAPPING_NUM		23	
//#endif /* CONFIG_MACH_RHEA_SS_IVORY */

#define D2083_MCTL_MODE_INIT(_reg_id, _dsm_mode, _default_pm_mode) \
	[_reg_id] = { \
		.reg_id = _reg_id, \
		.dsm_opmode = _dsm_mode, \
		.default_pm_mode = _default_pm_mode, \
	}

// for DEBUGGING and Troubleshooting
#if 1	//defined(DEBUG)
#define dlg_crit(fmt, ...) \
			printk(KERN_CRIT fmt, ##__VA_ARGS__)
#define dlg_err(fmt, ...) \
			printk(KERN_ERR fmt, ##__VA_ARGS__)
#define dlg_warn(fmt, ...) \
			printk(KERN_WARNING fmt, ##__VA_ARGS__)
#define dlg_info(fmt, ...) \
			printk(KERN_INFO fmt, ##__VA_ARGS__)
#else
#define dlg_crit(fmt, ...) 	do { } while(0);
#define dlg_err(fmt, ...)	do { } while(0);
#define dlg_warn(fmt, ...)	do { } while(0);
#define dlg_info(fmt, ...)	do { } while(0);
#endif

typedef u32 (*pmu_platform_callback)(int event, int param);

struct d2083_irq {
	irq_handler_t handler;
	void *data;
};

struct d2083_onkey {
	struct platform_device *pdev;
	struct input_dev *input;
};

//
struct d2083_regl_init_data {
	int regl_id;
	struct regulator_init_data   *initdata;
};


struct d2083_regl_map {
	u8 reg_id;
	u8 dsm_opmode;
	u8 default_pm_mode;
};


/**
 * Data to be supplied by the platform to initialise the D2083.
 *
 * @init: Function called during driver initialisation.  Should be
 *        used by the platform to configure GPIO functions and similar.
 * @irq_high: Set if D2083 IRQ is active high.
 * @irq_base: Base IRQ for genirq (not currently used).
 */
 struct batt_adc_tbl_t {
	u16 *bat_adc;
	u16 *bat_vol;
	u32 num_entries;
};

struct temp2adc_map {
	int temp;
	int adc;
};

struct d2083_battery_platform_data {
	u32	battery_capacity;
	u32	vf_lower;
	u32	vf_upper;
	int battery_technology;
	struct temp2adc_map *bcmpmu_temp_map;
	int bcmpmu_temp_map_len;
};

struct d2083_platform_data {
	struct i2c_slave_platform_data i2c_pdata;
	int i2c_adapter_id;

	int 	(*init)(struct d2083 *d2083);
	int 	(*irq_init)(struct d2083 *d2083);
	int		irq_mode;	/* Clear interrupt by read/write(0/1) */
	int		irq_base;	/* IRQ base number of D2083 */
	struct	d2083_audio_platform_data *audio_pdata;
	struct d2083_regl_init_data	*regulator_data;
	//struct  regulator_consumer_supply *regulator_data;
	struct  d2083_battery_platform_data *pbat_platform;

	pmu_platform_callback pmu_event_cb;
	
	//unsigned char regl_mapping[D2083_IOCTL_REGL_MAPPING_NUM];	/* Regulator mapping for IOCTL */
	struct d2083_regl_map regl_map[D2083_NUMBER_OF_REGULATORS];
};

struct d2083 {
	struct device *dev;

	struct i2c_client *i2c_client;
	struct mutex i2c_mutex;

	int (*read_dev)(struct d2083 * const d2083, char const reg, int const size, void * const dest);
	int (*write_dev)(struct d2083 * const d2083, char const reg, int const size, const u8 *src/*void * const src*/);

	int (*read_dev_bsc)(struct d2083 * const d2083, char const reg, int const size, void * const dest);
	int (*write_dev_bsc)(struct d2083 * const d2083, char const reg, int const size, const u8 *src/*void * const src*/);

	u8 *reg_cache;
	u16 vbat_init_adc[3];
	u16 avarage_vbat_init_adc;

	/* Interrupt handling */
    struct work_struct irq_work;
    struct task_struct *irq_task;
	struct mutex irq_mutex; /* IRQ table mutex */
	struct d2083_irq irq[D2083_NUM_IRQ];
	int chip_irq;

	struct d2083_pmic pmic;
	struct d2083_rtc rtc;
	struct d2083_onkey onkey;
	struct d2083_audio audio;
	struct d2083_battery batt;

    struct d2083_platform_data *pdata;
};

/*
 * d2083 Core device initialisation and exit.
 */
int 	d2083_device_init(struct d2083 *d2083, int irq, struct d2083_platform_data *pdata);
void 	d2083_device_exit(struct d2083 *d2083);

/*
 * d2083 device IO
 */
int 	d2083_clear_bits(struct d2083 * const d2083, u8 const reg, u8 const mask);
int 	d2083_set_bits(struct d2083* const d2083, u8 const reg, u8 const mask);
int     d2083_reg_read(struct d2083 * const d2083, u8 const reg, u8 *dest);
int 	d2083_reg_write(struct d2083 * const d2083, u8 const reg, u8 const val);
int 	d2083_block_read(struct d2083 * const d2083, u8 const start_reg, u8 const regs, u8 * const dest);
int 	d2083_block_write(struct d2083 * const d2083, u8 const start_reg, u8 const regs, u8 * const src);

/*
 * d2083 internal interrupts
 */
int 	d2083_register_irq(struct d2083 *d2083, int irq, irq_handler_t handler, 
				            unsigned long flags, const char *name, void *data);
int 	d2083_free_irq(struct d2083 *d2083, int irq);
int 	d2083_mask_irq(struct d2083 *d2083, int irq);
int 	d2083_unmask_irq(struct d2083 *d2083, int irq);
int 	d2083_irq_init(struct d2083 *d2083, int irq, struct d2083_platform_data *pdata);
int 	d2083_irq_exit(struct d2083 *d2083);

#if defined(CONFIG_MACH_RHEA_SS_IVORY) || defined(CONFIG_MACH_RHEA_SS_NEVIS) || defined(CONFIG_MACH_RHEA_SS_NEVISP) || defined(CONFIG_MACH_RHEA_SS_CORSICA) || defined(CONFIG_MACH_RHEA_SS_NEVISDS)
/* DLG IOCTL interface */
extern int d2083_ioctl_regulator(struct d2083 *d2083, unsigned int cmd, unsigned long arg);
#endif /* CONFIG_MACH_RHEA_SS_IVORY */

/* DLG new prototype */
int d2083_shutdown(struct d2083 *d2083);
void d2083_system_poweroff(void);
extern int d2083_core_reg2volt(int reg);
extern void set_MCTL_enabled(void);

#endif
