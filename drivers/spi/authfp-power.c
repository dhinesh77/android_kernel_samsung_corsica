/*
 * Driver for AuthenTec fingerprint sensor
 *
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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <linux/authfp_defs.h>
#include <linux/authfp_dev.h>
#include <linux/authfp.h>


int authfp_power_init(void **context,
	struct authfp_platform_data *platform_data)
{
	struct authfp_gpio_power_data *data = platform_data->power_data;
	int status = 0;
	u8 label[32] = {0};

	*context = 0;

	if (!data || !data->power_gpio)
		return 0;

	sprintf(label, "fp_power_%d", platform_data->sensor_index);
	status = gpio_request(data->power_gpio, label);
	if (status < 0)
		goto exit;

exit:
	return status;

}

int authfp_power_uninit(void *context,
	struct authfp_platform_data *platform_data)
{
	struct authfp_gpio_power_data *data = platform_data->power_data;

	if (!data || !data->power_gpio)
		return 0;

	gpio_free(data->power_gpio);
	return 0;
}

int authfp_power_on(void *context, struct authfp_platform_data *platform_data)
{
	struct authfp_gpio_power_data *data = platform_data->power_data;
	int status;

	if (!data || !data->power_gpio)
		return 0;

	status = gpio_direction_output(data->power_gpio, data->active_polarity);
	if (!status)
		msleep(data->delay_after_power_on);

        printk(KERN_INFO "[AUTHFP] %s (%d)\n",__FUNCTION__, data->power_gpio);
	return status;
}

int authfp_power_off(void *context, struct authfp_platform_data *platform_data)
{
	struct authfp_gpio_power_data *data = platform_data->power_data;
	int status;

	if (!data || !data->power_gpio)
		return 0;

	status = gpio_direction_output(data->power_gpio,
						!data->active_polarity);
	if (!status)
		msleep(data->delay_after_power_off);

        printk(KERN_INFO "[AUTHFP] %s (%d)\n",__FUNCTION__, data->power_gpio);
	return status;
}


MODULE_DESCRIPTION("AuthenTec fingerprint sensor power driver");
MODULE_AUTHOR("AuthenTec");
MODULE_LICENSE("GPL");
