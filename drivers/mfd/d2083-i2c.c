/*
 * d2083-i2c.c: I2C (Serial Communication) driver for D2083
 *   
 * Copyright(c) 2012 Dialog Semiconductor Ltd.
 *  
 * Author: Dialog Semiconductor Ltd. D. Chen, D. Patel
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/d2083/core.h>
#include <linux/d2083/d2083_reg.h> 
#include <linux/slab.h>
#include <linux/delay.h>
#include <mach/pwr_mgr.h>
#include <plat/pwr_mgr.h>

#define CONFIG_D2083_USE_SMBUS_API

#define PWRMGR_I2C_RDWR_MAX_TRIES	(10)
#define PWRMGR_I2C_RETRY_DELAY_US	(10)

enum {
	I2C_TRANS_NONE,
	I2C_TRANS_READ,
	I2C_TRANS_WRITE,
};

int last_i2c_trans = I2C_TRANS_NONE;

/**
 *function must be called with i2c_mutex locked
 */
static int i2c_try_read_write(struct d2083 * const d2083, bool check_fifo,
			int trans_type, u8 reg_addr, u8 slave_id, u8 *value)
{
	int err = 0;
	int tries = PWRMGR_I2C_RDWR_MAX_TRIES;

	pr_debug("%s: trans %d addr %x, slave %x\n", __func__,
		 trans_type, reg_addr, slave_id);
	
	switch (trans_type) {
	case I2C_TRANS_READ:
		while (tries--) {
			/**
			 * Read the FIFORDBLOCK Bit of PMU before
			 *initiating an read transaction if the last
			 *trasaction was i2c write. This bit is set by
			 *the
			 * PMU when its busy finishing previous write
			 *operation (if this bit is ignored there is a
			 *chance of reading a stale data of the
			 *register !!)
			 */

				err = pwr_mgr_pmu_reg_read(reg_addr,
							   slave_id, value);
				if (err == 0) 
				{
					last_i2c_trans = I2C_TRANS_READ;
					break;
				}

			udelay(PWRMGR_I2C_RETRY_DELAY_US);
		}
		if (tries <= 0) {
			dlg_err("I2C SW SEQ Max Tries\n");
			err = -EAGAIN;
			break;
		}
		break;
	case I2C_TRANS_WRITE:
		while (tries--) 
		{

				err = pwr_mgr_pmu_reg_write(reg_addr, slave_id,
							    *value);
				if (err == 0) 
				{
					last_i2c_trans = I2C_TRANS_WRITE;
					break;
				}
				udelay(PWRMGR_I2C_RETRY_DELAY_US);
		}
		if (tries <= 0) {
			err = -EAGAIN;
			dlg_err("I2C SW SEQ Write MAX Tries\n");
			break;
		}

		break;
	default:
		err = -EINVAL;
		break;
	}
	
	return err;
}

static int d2083_i2c_read_device(struct d2083 * const d2083, char const reg,
					int const bytes, void * const dest)
{
	int err = 0,i;
	u8 temp;
	
	mutex_lock(&d2083->i2c_mutex);

	if(bytes > 1)
	{
		for (i = 0; i < bytes; i++) 
		{
			err = i2c_try_read_write(d2083, true, I2C_TRANS_READ,
							 reg + i,
							 d2083->i2c_client->addr, &temp);

			if (err < 0)
				break;
			
			*((u8 *)dest+i) = temp;
		}
	}
	else
	{
		err = i2c_try_read_write(d2083, true, I2C_TRANS_READ, reg,
					 d2083->i2c_client->addr, &temp);

		if (err < 0)
			goto out_unlock;

		*((u8 *)dest) = temp;
	}
      out_unlock:
	mutex_unlock(&d2083->i2c_mutex);
	return err;
}

static int d2083_i2c_write_device(struct d2083 * const d2083, char const reg,
				   int const bytes, const u8 *src )
{
	int err = 0,i;
	u8 temp;

	
	mutex_lock(&d2083->i2c_mutex);

	if(bytes > 1)
	{
		for (i = 0; i < bytes; i++) 
		{
			temp = *(src +i);

			err = i2c_try_read_write(d2083, true, I2C_TRANS_WRITE,
							 reg + i,
							 d2083->i2c_client->addr, &temp);	
			if (err < 0)
				break;
		}
	}
	else
	{
		temp=*src;
		
		err = i2c_try_read_write(d2083, true, I2C_TRANS_WRITE,
									reg,
						 d2083->i2c_client->addr, &temp);
	}

	mutex_unlock(&d2083->i2c_mutex);
	
	return err;
}

#if 0	// interface for i2c-bsc
static int d2083_i2c_read_device_bsc(struct d2083 * const d2083, char const reg,
					int const bytes, void * const dest)
{
	int ret;

#ifdef CONFIG_D2083_USE_SMBUS_API
	if (bytes > 1)
		ret = i2c_smbus_read_i2c_block_data(d2083->i2c_client, reg, bytes, dest);
	else {
		ret = i2c_smbus_read_byte_data(d2083->i2c_client, reg);
		if (ret < 0)
			return ret;
		*(unsigned char *)dest = (unsigned char)ret;
	}
	return 0;
#else
	ret = i2c_master_send(d2083->i2c_client, &reg, 1);
	if (ret < 0) {
        dlg_err("Err in i2c_master_send(0x%x)\n", reg);
		return ret;
    }
	ret = i2c_master_recv(d2083->i2c_client, dest, bytes);
	if (ret < 0) {
        dlg_err("Err in i2c_master_recv(0x%x)\n", ret);
		return ret;
    }
	if (ret != bytes)
		return -EIO;
#endif /* CONFIG_D2083_USE_SMBUS_API */
	return 0;
}


/*
 *
 */
static int d2083_i2c_write_device_bsc(struct d2083 * const d2083, char const reg,
				   int const bytes, const u8 *src /*void * const src*/)
{
	int ret = 0;

	// Redundant. It already checked in d2083_reg_read & write function
	//if ((reg + bytes) > D2083_MAX_REGISTER_CNT) {
	//    printk(KERN_ERR "Bad input to d2083_i2c_write_device(0x%x, %d)\n", reg, bytes);
	//    return -EINVAL;
	//}

#ifdef CONFIG_D2083_USE_SMBUS_API
	if(bytes > 1)
		ret = i2c_smbus_write_i2c_block_data(d2083->i2c_client, reg, bytes, src);
	else
		ret = i2c_smbus_write_byte_data(d2083->i2c_client, reg, *src);

	return ret;
#else
	u8 msg[bytes + 1];

	msg[0] = reg;
	memcpy(&msg[1], src, bytes);

	ret = i2c_master_send(d2083->i2c_client, msg, bytes + 1);
	if (ret < 0)
		return ret;
	if (ret != bytes + 1)
		return -EIO;

	return 0;
#endif /* CONFIG_D2083_USE_SMBUS_API */
}
#endif

static int d2083_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct d2083 *d2083;
	int ret = 0;

	d2083 = kzalloc(sizeof(struct d2083), GFP_KERNEL);
	if (d2083 == NULL) {
		kfree(i2c);
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, d2083);
	d2083->dev = &i2c->dev;
	d2083->i2c_client = i2c;
	
	mutex_init(&d2083->i2c_mutex);

	rhea_pwr_mgr_init_sequencer();
	
	d2083->read_dev = d2083_i2c_read_device;
	d2083->write_dev = d2083_i2c_write_device;
	//d2083->read_dev = d2083_i2c_read_device_bsc;
	//d2083->write_dev = d2083_i2c_write_device_bsc;

	ret = d2083_device_init(d2083, i2c->irq, i2c->dev.platform_data);
	if (ret < 0)
		goto err;


	return ret;

err:
	kfree(d2083);
	return ret;
}

static int d2083_i2c_remove(struct i2c_client *i2c)
{
	struct d2083 *d2083 = i2c_get_clientdata(i2c);

	d2083_device_exit(d2083);
	kfree(d2083);
	return 0;
}


static const struct i2c_device_id d2083_i2c_id[] = {
	{ D2083_I2C, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, d2083_i2c_id);


static struct i2c_driver d2083_i2c_driver = {
	.driver = {
		   .name = D2083_I2C,
		   .owner = THIS_MODULE,
	},
	.probe = d2083_i2c_probe,
	.remove = d2083_i2c_remove,
	.id_table = d2083_i2c_id,
};

static int __init d2083_i2c_init(void)
{
	return i2c_add_driver(&d2083_i2c_driver);
}

/* Initialised very early during bootup (in parallel with Subsystem init) */
subsys_initcall(d2083_i2c_init);
//module_init(d2083_i2c_init);

static void __exit d2083_i2c_exit(void)
{
	i2c_del_driver(&d2083_i2c_driver);
}
module_exit(d2083_i2c_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd < william.seo@diasemi.com >");
MODULE_DESCRIPTION("I2C MFD driver for Dialog D2083 PMIC plus Audio");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" D2083_I2C);
