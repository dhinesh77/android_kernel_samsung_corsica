/*
 * Copyright (C) 2011 AuthenTec, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __AUTH_FP_DEV_H
#define __AUTH_FP_DEV_H

#define AUTHFP_NAME 	        "aeswipe"
#define AUTHFP_VERSION	        3

/*DON'T CHANGE*/
/*Protocol IDs supported by the driver*/
#define FP_PROTOCOL_UNKNOWN     0            /* Unknown protocol.*/
#define FP_PROTOCOL_1750        1            /* 1750 sensor.  */
#define FP_PROTOCOL_0850        2            /*  850 sensor.  */

/*DON'T CHANGE*/
/*Interface Types */
#define TR_IFACE_UNDEF 		0x00000000   /* Undefined   */
#define TR_IFACE_SPI_S 		0x00000001   /* SPI Slave   */
#define TR_IFACE_SPI_M 		0x00000002   /* SPI Master  */
#define TR_IFACE_MCBSP 		0x00000004   /* McBSP       */
#define TR_IFACE_I2C   		0x00000008   /* I2C         */
#define TR_IFACE_USB   		0x00000010   /* USB         */
#define TR_IFACE_PARALLEL   	0x00000020   /* Parallel    */

struct authfp_gpio_power_data {
	u32 power_gpio;
	u32 active_polarity;
	u32 delay_after_power_on;            /*in ms*/
	u32 delay_after_power_off;           /*in ms*/
};

struct authfp_platform_data {
	u32 sensor_index;       /*starting from 0*/
	s32 protocol_id;           /*Protocol IDs supported by the driver*/
	s32 interrupt_gpio;
	s32 interrupt_polarity;
	void *power_data;
	u32 max_frame_size;
	u32 interface_type;     /*Interface Types */
	u32 interface_speed;
	bool support_power_off;
};

#endif

