/*
 * d2083 Battery/Power module declarations.
  *
 * Copyright(c) 2012 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *
 */


#ifndef __D2083_BATTERY_H__
#define __D2083_BATTERY_H__

#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>


#define D2083_REG_EOM_IRQ
#undef D2083_REG_VDD_MON_IRQ
#undef D2083_REG_VDD_LOW_IRQ
#define D2083_REG_TBAT2_IRQ


#define D2083_MANUAL_READ_RETRIES			(5)
#if 1
#define ADC2TEMP_LUT_SIZE					(22)
#else
#define ADC2TEMP_LUT_SIZE					(10)
#endif
#define ADC2SOC_LUT_SIZE					(14)
#define ADC2VBAT_LUT_SIZE					(10)
#define D2083_ADC_RESOLUTION				(10)

#define AVG_SIZE							(16)
#define AVG_SHIFT							(4)
#define MAX_INIT_TRIES              		(AVG_SIZE + 5)

#define DEGREEK_FOR_DEGREEC_0				(273)

#define C2K(c)								(273 + c)	/* convert from Celsius to Kelvin */
#define K2C(k)								(k - 273)	/* convert from Kelvin to Celsius */

#define BAT_HIGH_TEMPERATURE				C2K(700)
#define BAT_ROOM_TEMPERATURE				C2K(200) //C2K(250)
#define BAT_ROOM_LOW_TEMPERATURE			C2K(100)
#define BAT_LOW_TEMPERATURE					C2K(0)
#define BAT_LOW_MID_TEMPERATURE				C2K(-100)
#define BAT_LOW_LOW_TEMPERATURE				C2K(-200)

#define BAT_CHARGE_START_TIMER				0
#define BAT_CHARGE_RESTART_TIMER			1
#define BAT_CHARGE_BACK_CHG_TIMER		2

#define BAT_CHARGE_TIMER_30MIN				(HZ*60*30)
#define BAT_CHARGE_TIMER_90MIN				(HZ*60*90)
#define BAT_CHARGE_TIMER_5HOUR				(HZ*60*60*5)
#define BAT_CHARGE_TIMER_6HOUR				(HZ*60*60*6)
#define BAT_CHARGE_TIMER_8HOUR				(HZ*60*60*8)
#define	BAT_CHARGE_TIMER_10HOUR				(HZ*60*60*10)

#define BAT_RECHARGE_CHECK_TIMER_30SEC		(HZ*30)

#define BAT_END_OF_CHARGE_NONE				(0 << 0)
#define BAT_END_OF_CHARGE_BY_FULL			(1 << 0)
#define BAT_END_OF_CHARGE_BY_TEMPERATURE	(1 << 1)
#define BAT_END_OF_CHARGE_BY_TIMER			(1 << 2)
#define BAT_END_OF_CHARGE_BY_OVP			(1 << 3) 
#define BAT_END_OF_CHARGE_BY_VF_OPEN		(1 << 4)
#define BAT_END_OF_CHARGE_BY_QUICKSTART		(1 << 5)

#define BAT_CAPACITY_1300MA					(1300)
#define BAT_CAPACITY_1500MA					(1500)
#define BAT_CAPACITY_1800MA					(1800)
#define BAT_CAPACITY_2000MA					(2000)
#define BAT_CAPACITY_4500MA					(4500)
#define BAT_CAPACITY_7000MA					(7000)

#define USED_BATTERY_CAPACITY				BAT_CAPACITY_1300MA

#define D2083_VOLTAGE_MONITOR_START			(HZ*1)
#define D2083_VOLTAGE_MONITOR_NORMAL		(HZ*10)
#define D2083_VOLTAGE_MONITOR_FAST			(HZ*3)
#define D2083_VOLTAGE_MONITOR_PO			(HZ*0.5)

// Unused. #define D2083_VOLTAGE_MONITOR_FAST_USB		(HZ*5)

#define D2083_TEMPERATURE_MONITOR_START		(HZ*1)
#define D2083_TEMPERATURE_MONITOR_NORMAL	(HZ*10)
#define D2083_TEMPERATURE_MONITOR_FAST		(HZ*1)

#define D2083_NOTIFY_INTERVAL				(HZ*10)

#define D2083_SLEEP_MONITOR_WAKELOCK_TIME	(0.35*HZ)

#define BAT_VOLTAGE_ADC_DIVISION			(1700)

#define BAT_VOLTAGE_ADC_HIGH_IDX			(8)  // 4.20 ~ 4.10V ; 0.2
#define BAT_VOLTAGE_ADC_HIMID_IDX			(7)  // 4.09 ~ 4.00V ; 0.1
#define BAT_VOLTAGE_ADC_MIDDLE_IDX			(5)  // 3.99 ~ 3.80V ; 0.1
#define BAT_VOLTAGE_ADC_MIDLOW_IDX			(3)  // 3.79 ~ 3.60V ; 0.09
#define BAT_VOLTAGE_ADC_LOW_IDX             (0)  // 3.59 ~ 3.40V ; 0.3

#define BAT_POWER_OFF_VOLTAGE				(3400)
#define BAT_CHARGING_RESTART_VOLTAGE		(4140)

#define CHARGING_STOP_HIGH_TEMPERATURE		600  // 60 C
#define CHARGING_RESTART_HIGH_TEMPERATURE	400  // 40 C
#define CHARGING_STOP_LOW_TEMPERATURE		-50  // -5 C
#define CHARGING_RESTART_LOW_TEMPERATURE	0

#define D2083_CATEGORY_DEVICE				0
#define D2083_CATEGORY_BATTERY				1

#define SOC_ADJUSET_OFFSET					(2)
#define CHARGE_SPEED_ADC_OFFSET				(2) // (5)

#define D2083_CHARGE_CV_ADC_LEVEL		(3380)
#define FULLY_CHARGED_ADC_LEVEL			(3465)

#define D2083_FIRST_LOW_BAT_NOTIFY			(15)
#define D2083_SECOND_LOW_BAT_NOTIFY			(3)

#define D2083_CAL_HIGH_VOLT					(4200)
#define D2083_CAL_LOW_VOLT					(3400)

#define D2083_BASE_4P2V_ADC                 (3269)
#define D2083_BASE_3P4V_ADC                 (1581)
#define D2083_CAL_MAX_OFFSET				(10)

#define D2083_OFFSET_DIVIDER_VLOW			(75)
#define D2083_OFFSET_DIVIDER_LLOW			(65)
#define D2083_OFFSET_DIVIDER_LOW			(55)

#define D2083_OFFSET_VLOW					(75)
#define D2083_OFFSET_LLOW					(95)
#define D2083_OFFSET_LOW					(115)

#define D2083_DEFAULT_ITER					(3)
#define D2083_LLOW_ITER						(5)
#define D2083_LOW_ITER						(7)


enum {
	D2083_EVENT_TA_ATTACHED = 0,
	D2083_EVENT_TA_DETACHED,
	D2083_EVENT_USB_ATTACHED,
	D2083_EVENT_USB_DETACHED,
	D2083_EVENT_JIG_ATTACHED,
	D2083_EVENT_JIG_DETACHED,			// 5
	
	D2083_EVENT_CHARGE_FULL,
	D2083_EVENT_OVP_CHARGE_STOP,
	D2083_EVENT_OVP_CHARGE_RESTART,		// 10
	D2083_EVENT_SLEEP_MONITOR,

	D2083_EVENT_MAX,
};

enum {
	CHARGER_TYPE_NONE = 0,
	CHARGER_TYPE_TA,
	CHARGER_TYPE_USB,
	CHARGER_TYPE_MAX
};

typedef enum d2083_adc_channel {
	D2083_ADC_VOLTAGE = 0,
	D2083_ADC_TEMPERATURE_1,
	D2083_ADC_TEMPERATURE_2,
	D2083_ADC_VF,
	D2083_ADC_AIN,
	D2083_ADC_TJUNC,
	D2083_ADC_CHANNEL_MAX    
} adc_channel;

typedef enum d2083_adc_mode {
	D2083_ADC_IN_AUTO = 1,
	D2083_ADC_IN_MANUAL,
	D2083_ADC_MODE_MAX
} adc_mode;

struct adc_man_res {
	u16 read_adc;
	u8  is_adc_eoc;
};

struct adc2temp_lookuptbl {
	u16 adc[ADC2TEMP_LUT_SIZE];
	u16 temp[ADC2TEMP_LUT_SIZE];
};

struct adc2vbat_lookuptbl {
	u16 adc[ADC2VBAT_LUT_SIZE];
	u16 vbat[ADC2VBAT_LUT_SIZE];
	u16 offset[ADC2VBAT_LUT_SIZE];
};

struct adc2soc_lookuptbl {
	u16 adc_ht[ADC2SOC_LUT_SIZE];
	u16 adc_rt[ADC2SOC_LUT_SIZE];
	u16 adc_rlt[ADC2SOC_LUT_SIZE];
	u16 adc_lt[ADC2SOC_LUT_SIZE];
	u16 adc_lmt[ADC2SOC_LUT_SIZE];
	u16 adc_llt[ADC2SOC_LUT_SIZE];
	u16 soc[ADC2SOC_LUT_SIZE];
};

struct adc_cont_in_auto {
	u8 adc_preset_val;
	u8 adc_cont_val;
	u8 adc_msb_res;
	u8 adc_lsb_res;
	u8 adc_lsb_mask;
};

struct d2083_charger_data {
	u8  is_charging;
	int	current_charger;
	int	jig_connected; 
	int pmic_vbus_state;
	unsigned int lp_charging;

	struct wake_lock charger_wakeup;

	void	(*enable_charge)(void);
	void	(*disable_charge)(void);
	void	(*enable_back_charge)(void);
};

struct d2083_battery_data {
	u8	health;
	u8 	status;
	u8	end_of_charge;
	u8  vdd_hwmon_level;

	u32	current_level;

	// for voltage
	u32 origin_volt_adc;
	u32 current_volt_adc;
	u32 average_volt_adc;
	u32	current_voltage;
	u32	average_voltage;
	u32 sum_voltage_adc;
	int sum_total_adc;

	// from bootloader
	u32 baseline_voltage;

	// for temperature
	u32	current_temp_adc;
	u32	average_temp_adc; 
	int	current_temperature;
	int	average_temperature;
	u32 sum_temperature_adc;
	
	u32	soc;
	u32 prev_soc;

	u32 anto_soc;
	u32 anto_prev_soc;

	int battery_technology;
	int battery_present;
	u32	capacity;

	u16 vf_adc;
	u32 vf_ohm;
	u32	vf_lower;
	u32	vf_upper; 

	u32 voltage_adc[AVG_SIZE];
	u32 temperature_adc[AVG_SIZE];
	u8	voltage_idx;
	u8 	temperature_idx;

	u8  volt_adc_init_done;
	u8  temp_adc_init_done;
	struct wake_lock sleep_monitor_wakeup;
    struct adc_man_res adc_res[D2083_ADC_CHANNEL_MAX];

	/* These two members are synchronized with BCM PMU temperature map */
	struct temp2adc_map *bcmpmu_temp_map;
	int bcmpmu_temp_map_len;

	u8 vitural_battery_full;
};

struct d2083_battery {
	struct d2083		*pd2083;
	struct device 	*dev;
	struct platform_device *pdev;

	u8 adc_mode;

	struct power_supply	battery;
	struct power_supply	usb;
	struct power_supply	wall;

	struct d2083_charger_data	charger_data;
	struct d2083_battery_data	battery_data;

	struct delayed_work	monitor_volt_work;
	struct delayed_work	monitor_temp_work;
	struct delayed_work	info_notify_work;
	struct delayed_work	charge_timer_work;
	struct delayed_work	recharge_start_timer_work;
	struct delayed_work	sleep_monitor_work;

	struct mutex		meoc_lock;
	struct mutex		lock;
	struct mutex		api_lock;
	
	struct timer_list	charge_timer;
	struct timer_list	recharge_start_timer; 

	int (*d2083_read_adc)(struct d2083_battery *pbat, adc_channel channel);

};

// In order to set function pointer to control charging.
int d2083_register_enable_charge(void (*enable_charge)(void));
int d2083_register_disable_charge(void (*disable_charge)(void));
int d2083_register_enable_back_charge(void (*enable_charge)(void));

void (*d2083_get_external_event_handler(void))(int, int);

#endif /* __D2083_BATTERY_H__ */
