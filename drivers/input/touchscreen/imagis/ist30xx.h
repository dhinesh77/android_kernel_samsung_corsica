/*
 *  Copyright (C) 2010, Imagis Technology Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#ifndef __IST30XX_H__
#define __IST30XX_H__

/*
 * Support F/W ver : IST3000 v2.2~v3.0 (included tag)
 * Release : 2012.09.03 by Ian
 */

#define I2C_BURST_MODE          (1)

#define IST30XX_EVENT_MODE      (1)
#if IST30XX_EVENT_MODE
# define IST30XX_NOISE_MODE     (1)
# define IST30XX_TRACKING_MODE  (1)
#else
# define IST30XX_NOISE_MODE     (0)
# define IST30XX_TRACKING_MODE  (0)
#endif
#define IST30XX_DETECT_TA       (1)
#define IST30XX_USE_KEY         (1)
#define IST30XX_DEBUG           (1)
#define PRINT_TOUCH_EVENT       (1)

#define IST30XX_FACTORY_TEST    (0)
#define IST30XX_USE_ZVALUE      (1)

#define IST30XX_DEV_NAME        "sec_touch"
#define IST30XX_CHIP_ID         (0x30003000)
#define IST30XXA_CHIP_ID        (0x300a300a)

#define IST30XX_DEV_ID          (0xA0 >> 1)
#define IST30XX_FW_DEV_ID       (0xA4 >> 1)

#define IST30XX_ADDR_LEN        (4)
#define IST30XX_DATA_LEN        (4)

#define IST30XX_ISP_CMD_LEN     (3)

#define IST30XX_MAX_MT_FINGERS  (10)

#define IST30XX_MAX_X           (240)
#define IST30XX_MAX_Y           (320)
#define IST30XX_MAX_Z           (255)
#define IST30XX_MAX_W           (15)

#if IST30XX_DEBUG
#define DMSG(x ...) printk(KERN_DEBUG x)
#else
#define DMSG(x ...)
#endif

/* I2C Transfer msg number */
#define WRITE_CMD_MSG_LEN       (1)
#define READ_CMD_MSG_LEN        (2)


#define SEC_TSP_FACTORY_TEST
#define SEC_FAC_TK

#ifdef SEC_TSP_FACTORY_TEST
#define TSP_BUF_SIZE 1024

#define TSP_CMD_STR_LEN 32
#define TSP_CMD_RESULT_STR_LEN 512
#define TSP_CMD_PARAM_NUM 8

#define TSP_CMD(name, func) .cmd_name = name, .cmd_func = func

struct tsp_cmd {
	struct list_head	list;
	const char	*cmd_name;
	void	(*cmd_func)(void *device_data);
};

void fw_update(void *device_data);
void get_config_ver(void *device_data);
void get_fw_ver_bin(void *device_data);
void get_fw_ver_ic(void *device_data);
void get_threshold(void *device_data);
void get_chip_id(void *device_data);
void get_chip_vendor(void *device_data);
void get_chip_name(void *device_data);
void get_raw_value(void *device_data);
void get_diff_value(void *device_data);
void get_baseline_value(void *device_data);
void get_x_num(void *device_data);
void get_y_num(void *device_data);
void run_raw_read(void *device_data);
void run_diff_read(void *device_data);
void run_baseline_read(void *device_data);

void not_support_cmd(void *device_data);

#endif


enum ist30xx_commands {
	CMD_ENTER_UPDATE            = 0x02,
	CMD_EXIT_UPDATE             = 0x03,
	CMD_UPDATE_SENSOR           = 0x04,
	CMD_UPDATE_CONFIG           = 0x05,
	CMD_ENTER_REG_ACCESS        = 0x07,
	CMD_EXIT_REG_ACCESS         = 0x08,
	CMD_SET_TA_MODE             = 0x0A,
	CMD_START_SCAN              = 0x0B,
	CMD_ENTER_FW_UPDATE         = 0x0C,
	CMD_RUN_DEVICE              = 0x0D,

	CMD_CALIBRATE               = 0x11,
	CMD_USE_IDLE                = 0x12,
	CMD_USE_DEBUG               = 0x13,
	CMD_ZVALUE_MODE             = 0x15,
	CMD_CHECK_CALIB             = 0x1A,

	CMD_GET_COORD               = 0x20,

	CMD_GET_CHIP_ID             = 0x30,
	CMD_GET_FW_VER              = 0x31,
	CMD_GET_CHECKSUM            = 0x32,
	CMD_GET_LCD_RESOLUTION      = 0x33,
	CMD_GET_TSP_CHNUM1          = 0x34,
	CMD_GET_PARAM_VER           = 0x35,
	CMD_GET_CALIB_RESULT        = 0x37,
	CMD_GET_TSP_SWAP_INFO       = 0x38,
	CMD_GET_KEY_INFO1           = 0x39,
	CMD_GET_KEY_INFO2           = 0x3A,
	CMD_GET_KEY_INFO3           = 0x3B,
	CMD_GET_TSP_CHNUM2          = 0x3C,
	CMD_GET_TSP_DIRECTION       = 0x3D,

	CMD_GET_TSP_VENDOR          = 0x3E,
	CMD_GET_TSP_PANNEL_TYPE     = 0x40,
};

#define CMD_FW_UPDATE_MAGIC     (0x85FDAE8A)


typedef union {
	struct {
		u32	y       : 10;
		u32	w       : 6;
		u32	x       : 10;
		u32	id      : 4;
		u32	udmg    : 2;
	} bit_field;
	u32 full_field;
} finger_info;


struct ist30xx_status {
	int	power;
	int	update;
	int	calib;
	int	calib_msg;
};

struct ist30xx_fw {
	u32	pre_ver;
	u32	ver;
	u32	index;
	u32	size;
	u32	chksum;
};

#define IST30XX_TAG_MAGIC       "ISTV1TAG"
struct ist30xx_tags {
	char	magic1[8];
	u32	fw_addr;
	u32	fw_size;
	u32	flag_addr;
	u32	flag_size;
	u32	cfg_addr;
	u32	cfg_size;
	u32	sensor1_addr;
	u32	sensor1_size;
	u32	sensor2_addr;
	u32	sensor2_size;
	u32	chksum;
	u32	reserved2;
	char	magic2[8];
};

#include <linux/earlysuspend.h>
#include <linux/leds.h>
#include <mach/gpio.h>

struct ist30xx_data {
	struct i2c_client *	client;
	struct input_dev *	input_dev;
	struct early_suspend	early_suspend;
	struct ist30xx_status	status;
	struct ist30xx_fw	fw;
	struct ist30xx_tags	tags;
	u32			chip_id;
	u32			param_ver;
	u32			num_fingers;
	u32			irq_enabled;
	u32			xy_channel_num;
	u32			tsp_type;
	finger_info		fingers[IST30XX_MAX_MT_FINGERS];
	finger_info		prev_fingers[IST30XX_MAX_MT_FINGERS];
	/* variables for LED*/
	struct led_classdev		led;
	u8				led_brightness;
	
#if defined(SEC_TSP_FACTORY_TEST)
	struct list_head			cmd_list_head;
	unsigned char cmd_state;
	char			cmd[TSP_CMD_STR_LEN];
	int			cmd_param[TSP_CMD_PARAM_NUM];
	char			cmd_result[TSP_CMD_RESULT_STR_LEN];
	struct mutex			cmd_lock;
	bool			cmd_is_running;

	bool ft_flag;
#endif				/* SEC_TSP_FACTORY_TEST */
};


extern struct mutex ist30xx_mutex;

void ist30xx_enable_irq(struct ist30xx_data *data);
void ist30xx_disable_irq(struct ist30xx_data *data);

void ist30xx_start(struct ist30xx_data *data);
int ist30xx_get_ver_info(struct ist30xx_data *data);
int ist30xx_init_touch_driver(struct ist30xx_data *data);

int ist30xx_get_position(struct i2c_client *client, u32 *buf, u16 len);

int ist30xx_read_cmd(struct i2c_client *client, u32 cmd, u32 *buf);
int ist30xx_write_cmd(struct i2c_client *client, u32 cmd, u32 val);

int ist30xx_cmd_run_device(struct i2c_client *client);
int ist30xx_cmd_start_scan(struct i2c_client *client);
int ist30xx_cmd_calibrate(struct i2c_client *client);
int ist30xx_cmd_check_calib(struct i2c_client *client);
int ist30xx_cmd_update(struct i2c_client *client, int cmd);
int ist30xx_cmd_reg(struct i2c_client *client, int cmd);

int ist30xx_power_on(void);
int ist30xx_power_off(void);
int ist30xx_reset(void);
void ts_power_enable(int en);

int ist30xx_internal_suspend(struct ist30xx_data *data);
int ist30xx_internal_resume(struct ist30xx_data *data);

int ist30xx_factory_sysfs(void);
int __devinit ist30xx_init_system(void);

#endif  // __IST30XX_H__
