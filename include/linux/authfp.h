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

#ifndef __AUTH_FP_H
#define __AUTH_FP_H


struct authfp_bus_ops {
	int (*bus_read)(void *device, u8 *buf, size_t len);
	int (*bus_write)(void *device, const u8 *buf, size_t len);
	void (*bus_enable_write_buffering)(void *device);
	void (*bus_disable_write_buffering)(void *device, bool discard);
};

struct authfp_data_bus {
	const struct authfp_bus_ops *bops;
	void *device;
};

struct authfp_platform_data;


int authfp_probe(struct device *dev,
	const struct authfp_data_bus *data_bus,
	size_t extension_size);
int authfp_remove(struct device *dev);


int authfp_suspend(struct device *dev);
int authfp_resume(struct device *dev);

void *authfp_get_extension(struct device *dev);

int authfp_power_init(void **context,
	struct authfp_platform_data *platform_data);

int authfp_power_uninit(void *context,
	struct authfp_platform_data *platform_data);

int authfp_power_on(void *context,
	struct authfp_platform_data *platform_data);

int authfp_power_off(void *context,
	struct authfp_platform_data *platform_data);


#endif
