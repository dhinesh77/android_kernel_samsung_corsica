/*
 * s2200_ts.h
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef _LINUX_S2200_TOUCH_H
#define _LINUX_S2200_TOUCH_H

struct s2200_ts_platform_data {
	int	max_x;
	int	max_y;

	bool	invert_x;
	bool	invert_y;

	int	gpio_sda;
	int	gpio_scl;
	int	gpio_int;
	int	gpio_vdd_en;
	
	int	(*pins_to_gpio)(bool to_gpio);
};

#endif /* _LINUX_S2200_TOUCH_H */