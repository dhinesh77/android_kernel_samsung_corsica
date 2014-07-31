/*****************************************************************************
*  Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/pinmux.h>

#include "tpa2026-i2c.h"

//#include <linux/mfd/tpa2026/core.h>

static bool Pinmux_check=false;

static void tpa2026_check_pinmux(void);

struct tpa2026_data {
    struct i2c_client *i2c_client;
};

static struct i2c_client *tpa2026_client = NULL;
// static int tpa2026_i2c_write_device(struct tpa2026 *tpa2026, char reg, int bytes, void *value)


struct tpa2026_i2c_addr_data{
	u8 addr;
	u8 data;
};

static int TPA_write_reg(u8 reg, u8 data)
{
	int ret = 0;
	u8 buf[2];
	struct i2c_msg msg[1];

	buf[0] = reg;
	buf[1] = data;

	msg[0].addr = 0x58;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	printk("[TPA_write_reg] reg[0x%2x] data[0x%2x]\n",buf[0],buf[1]);

	ret = i2c_transfer(tpa2026_client->adapter, msg, 1);
	if (ret != 1) {
		printk("\n [TPA_write_reg] i2c Write Failed (ret=%d) \n", ret);
		return -1;
	}

	return ret;
}
  
static int TPA_read_reg(u8 reg, u8 *data)
{
	int ret = 0;
	u8 buf[1];

	struct i2c_msg msg[2];

	buf[0] = reg;

	msg[0].addr = 0x58;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = 0x58;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;

	printk("[TPA_read_reg] TPA_read_reg reg[0x%2x]\n", buf[0]);
	
	tpa2026_check_pinmux();

	ret = i2c_transfer(tpa2026_client->adapter, msg, 2);
	if (ret != 2) {
		printk("\n [TPA_read_reg] i2c Read Failed (ret=%d) \n", ret);
		return -1;
	}
	*data = buf[0];

	printk("[TPA_read_reg] [0x%2x], [0x%2x], [0x%2x] i2c Read success\n",buf[0], msg[0].buf, msg[1].buf);

	return 0;
}



static int tpa2026_i2c_write(char *buf, int len)
{
	uint8_t i;
	uint8_t ret;
	
	
	struct i2c_msg msg[] = {
		{
			.addr	= tpa2026_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};

	if (Pinmux_check == false)
	{
		tpa2026_check_pinmux();
		Pinmux_check=true;
	}

		ret = i2c_transfer(tpa2026_client->adapter, msg, 1);

//        printk(KERN_INFO "[%s] write ret = %d \n",__FUNCTION__, ret);
	if (ret < 0)
		{
        printk(KERN_INFO "[%s] write error = %d \n",__FUNCTION__, ret);
		return -1;
	}

	return 0;
}


static int tpa2026_i2c_write_device( u8 reg, u8 value)
{
    unsigned char data[2] = {0};
        
    data[0] = reg;
    data[1] = value;

    if(tpa2026_i2c_write(data, 2) !=0)
    {
        printk(KERN_INFO "[%s] Error. tpa2026_i2c_write_device\n",__FUNCTION__);
		return -1;
    }
    
    return 1;
}

#define CONFIG_TPA2026_DBG
#ifdef CONFIG_TPA2026_DBG
//TPA2026 seemed not to support reading register. 
//Anyway, it's better to have reading function. 
static unsigned int sysfs_reg,sysfs_val;

//usage: cat /sys/bus/i2c/devices/i2c-x/regwrite
static ssize_t show_regwrite(struct device *dev, 
				struct device_attribute *attr,
                                const char *buf)
{
	unsigned int reg, val;

	reg = sysfs_reg; 
	val = sysfs_val;
	
	printk("TPA2026 register=0x%x, value=0x%x\n", reg, val);

	return sprintf(buf,"register=0x%x,value=0x%x\n",reg,val);
}

//usage: echo reg val > /sys/bus/i2c/devices/i2c-x/regwrite
//example: # echo 0 0xff > /sys/bus/i2c/devices/i2c-x/regwrite
static ssize_t store_regwrite(struct device *dev, struct device_attribute *attr,
                                const char *buf, size_t count)
{
	unsigned int reg, val, ret;
	sscanf(buf, "%x %x ", &reg, &val);
	sysfs_reg = reg; 
	sysfs_val = val; 
	printk("TPA2026 register=0x%X, value=0x%X\n", reg, val);

	//put the i2c writing code here. 
	//for debugging, first comment out, 
	ret = tpa2026_i2c_write_device(reg, val);
	if (ret < 0)
		printk("TPA2026 ret=%d\n", ret);

	return count;
}

static DEVICE_ATTR(regwrite, 0644, show_regwrite, store_regwrite);
//If adb-shell is open in user space, you may need to change mode to 0666. 
//
static struct attribute *tpa2026_attrs[] = {
        &dev_attr_regwrite.attr,
        NULL
};

static const struct attribute_group tpa2026_attr_group = {
        .attrs = tpa2026_attrs,
};
#endif

static void tpa2026_check_pinmux(void)
{
	struct pin_config tpa_i2c_gpio_119,tpa_i2c_gpio_121;	

	tpa_i2c_gpio_119.name = PN_MDMGPIO07;	
	if(pinmux_get_pin_config(&tpa_i2c_gpio_119)){
		printk("%s, Fail to get pin configuration\n",__func__);
	}
	else{
		printk("%s, PinMux(GPIO119):0x%x\n",__func__,tpa_i2c_gpio_119.reg.val);
	}
	
	if(tpa_i2c_gpio_119.reg.val != 0x403){
		tpa_i2c_gpio_119.func = PF_GPIO119;
		tpa_i2c_gpio_119.reg.val = 0x403;
		pinmux_set_pin_config(&tpa_i2c_gpio_119);
		pinmux_get_pin_config(&tpa_i2c_gpio_119);
		printk("%s,after PinMux(GPIO119):0x%x\n",__func__,tpa_i2c_gpio_119.reg.val);
	}

	
	tpa_i2c_gpio_121.name = PN_ICUSBDP;	
	if(pinmux_get_pin_config(&tpa_i2c_gpio_121)){
		printk("%s, Fail to get pin configuration\n",__func__);
	}
	else{
		printk("%s, PinMux(GPIO121):0x%x\n",__func__,tpa_i2c_gpio_121.reg.val);
	}
	
	if(tpa_i2c_gpio_121.reg.val != 0x403){
		tpa_i2c_gpio_121.func = PF_GPIO121;
		tpa_i2c_gpio_121.reg.val = 0x403;
		pinmux_set_pin_config(&tpa_i2c_gpio_121);
		pinmux_get_pin_config(&tpa_i2c_gpio_121);
		printk("%s,after PinMux(GPIO121):0x%x\n",__func__,tpa_i2c_gpio_121.reg.val);
	}

}


//struct i2c_client *client
static int tpa2026_i2c_probe(struct i2c_client *i2c,  const struct i2c_device_id *id)
{
   // struct tpa2026_data *tpa2026;
    int ret = 0;

	struct pin_config tpa_i2c_gpio_119,tpa_i2c_gpio_121;

    printk(KERN_INFO "[TPA2026] [%s --1] name:%s, adapter = %x, addr=%x, flag=%x\n",__FUNCTION__,i2c->name, i2c->adapter, i2c->addr, i2c->flags);  

    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
    	printk("%s: i2c functionality check failed!\n", __func__);
    	ret = -ENODEV;
    	goto exit;
    }

    printk("REG : tpa2026_i2c_probe called. slave addr=0x%x\n", i2c->addr);


#if 0
	tpa_i2c_gpio_119.name = PN_MDMGPIO07;
	
	if(pinmux_get_pin_config(&tpa_i2c_gpio_119)){
		printk("%s, Fail to get pin configuration\n",__func__);
	}
	else{
		printk("%s, PinMux(GPIO119):0x%\n",__func__,tpa_i2c_gpio_119.reg.val);
	}
	tpa_i2c_gpio_119.func = PF_GPIO119;
	tpa_i2c_gpio_119.reg.val = 0x403;
	pinmux_set_pin_config(&tpa_i2c_gpio_119);
	pinmux_get_pin_config(&tpa_i2c_gpio_119);
	printk("%s,after PinMux(GPIO119):0x%x\n",__func__,tpa_i2c_gpio_119.reg.val);
	
	tpa_i2c_gpio_121.name = PN_ICUSBDP;	
	if(pinmux_get_pin_config(&tpa_i2c_gpio_121)){
		printk("%s, Fail to get pin configuration\n",__func__);
	}
	else{
		printk("%s, PinMux(GPIO121):0x%x\n",__func__,tpa_i2c_gpio_121.reg.val);
	}

	tpa_i2c_gpio_121.func = PF_GPIO121;
	tpa_i2c_gpio_121.reg.val = 0x403;
	pinmux_set_pin_config(&tpa_i2c_gpio_121);
	pinmux_get_pin_config(&tpa_i2c_gpio_121);
	printk("%s,after PinMux(GPIO121):0x%x\n",__func__,tpa_i2c_gpio_121.reg.val);
#endif	


   // tpa2026->i2c_client = i2c;
  //  i2c_set_clientdata(i2c, tpa2026);   
    tpa2026_client = i2c;    

    printk(KERN_INFO "[TPA2026] [%s --2] adapter = %x, addr=%x, flag=%x\n",__FUNCTION__, tpa2026_client->adapter, tpa2026_client->addr, tpa2026_client->flags);  

#ifdef CONFIG_TPA2026_DBG
    ret = sysfs_create_group(&tpa2026_client->dev.kobj, &tpa2026_attr_group);
#endif

	printk(KERN_INFO "[TPA2026] sysfs_create_group=%d \n",__FUNCTION__, ret);



exit :
    return ret;
}

static int tpa2026_i2c_remove(struct i2c_client *i2c)
{
	struct tpa2026 *tpa2026 = i2c_get_clientdata(i2c);

#ifdef CONFIG_TPA2026_DBG
        sysfs_remove_group(&tpa2026_client->dev.kobj,&tpa2026_attr_group);
#endif
       printk(KERN_INFO "[TPA2026] [%s] \n",__FUNCTION__);  
       
	kfree(tpa2026);

	return 0;
}

int tpa2026_amp_shutdown(void)
{          
    printk(KERN_INFO "[TPA2026] [%s] Amp shutDown\n",__FUNCTION__);    

    tpa2026_i2c_write_device(0x01, 0x22);

    return 1;
}

EXPORT_SYMBOL(tpa2026_amp_shutdown);



int tpa2026_amp_On(void)
{         
    printk(KERN_INFO "[TPA2026] [%s] Amp On\n",__FUNCTION__);	

    tpa2026_i2c_write_device(0x01, 0xC3);
    tpa2026_i2c_write_device(0x05, 0x6);

    return 1;
}

EXPORT_SYMBOL(tpa2026_amp_On);


int tpa2026_default_set(void)
{         
    printk(KERN_INFO "[TPA2026] [%s] default_set\n",__FUNCTION__);	

    tpa2026_i2c_write_device(0x01, 0xC2);
    tpa2026_i2c_write_device(0x02, 0x0);
    tpa2026_i2c_write_device(0x03, 0x0);
    tpa2026_i2c_write_device(0x04, 0x0);
    tpa2026_i2c_write_device(0x05, 0x12);
    tpa2026_i2c_write_device(0x06, 0x80);
    tpa2026_i2c_write_device(0x07, 0x00);
	
    return 1;
}

EXPORT_SYMBOL(tpa2026_default_set);




static const struct i2c_device_id tpa2026_i2c_id[] = {
       { "tpa2026", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, tpa2026_i2c_id);

static struct i2c_driver tpa2026_i2c_driver = {
	.driver = {
		.name = "tpa2026",
		.owner = THIS_MODULE,
	},
	.probe = tpa2026_i2c_probe,
	.remove = tpa2026_i2c_remove,
	.id_table = tpa2026_i2c_id,
};



static int __init tpa2026_i2c_init(void)
{
       printk(KERN_INFO "[TPA2026] [%s] \n",__FUNCTION__);  

	
	return i2c_add_driver(&tpa2026_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(tpa2026_i2c_init);

static void __exit tpa2026_i2c_exit(void)
{
	i2c_del_driver(&tpa2026_i2c_driver);
}
module_exit(tpa2026_i2c_exit);

MODULE_DESCRIPTION("tpa2026 amp driver");
MODULE_AUTHOR("SAMSUNG");
MODULE_LICENSE("GPL");
