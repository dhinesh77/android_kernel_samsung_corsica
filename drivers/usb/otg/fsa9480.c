/*
 * For TI 6111 micro usb IC & FSA880
 *
 * Copyright (C) 2009 Samsung Electronics
 * Wonguk Jeong <wonguk.jeong@samsung.com>
 * Minkyu Kang <mk7.kang@samsung.com>
 *
 * Modified by Sumeet Pawnikar <sumeet.p@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/fsa9480.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#if 0	// TODO: 
#include <plat/microusbic.h>
#include <plat/mfp.h>

#include <plat/vbus.h>

#include <linux/mfd/88pm860x.h>
#endif

#include <linux/wakelock.h>

#if defined(CONFIG_SPA)
#include <mach/spa.h>
static void (*spa_external_event)(int, int) = NULL;
#elif defined(CONFIG_BATTERY_D2083)
#include <linux/delay.h>
#include <mach/pwr_mgr.h>
#include <plat/pwr_mgr.h>
#include <linux/d2083/core.h>
#include <linux/d2083/d2083_battery.h>
static void (*spa_external_event)(int, int) = NULL;
#endif

#include <linux/delay.h>

#ifdef CONFIG_KONA_PI_MGR
#include <plat/pi_mgr.h>
static struct pi_mgr_qos_node qos_node;
#endif


/*
#include "mv/mvUsbDevApi.h"
#include "mv/mvUsbCh9.h"
#include <mach/hardware.h>
#include <plat/vbus.h>
#include <plat/pxausb_comp.h>
#include <plat/pxa3xx_otg.h>
#include <plat/pxa_u2o.h>
#include "mvUsb.h"
*/


/* FSA9480 I2C registers */
#define FSA9480_REG_DEVID              0x01
#define FSA9480_REG_CTRL               0x02
#define FSA9480_REG_INT1               0x03
#define FSA9480_REG_INT2               0x04
#define FSA9480_REG_INT1_MASK          0x05
#define FSA9480_REG_INT2_MASK          0x06
#define FSA9480_REG_ADC                        0x07
#define FSA9480_REG_TIMING1            0x08
#define FSA9480_REG_TIMING2            0x09
#define FSA9480_REG_DEV_T1             0x0a
#define FSA9480_REG_DEV_T2             0x0b
#define FSA9480_REG_BTN1               0x0c
#define FSA9480_REG_BTN2               0x0d
#define FSA9480_REG_CK                 0x0e
#define FSA9480_REG_CK_INT1            0x0f
#define FSA9480_REG_CK_INT2            0x10
#define FSA9480_REG_CK_INTMASK1                0x11
#define FSA9480_REG_CK_INTMASK2                0x12
#define FSA9480_REG_MANSW1             0x13
#define FSA9480_REG_MANSW2             0x14

// Luke
#define FSA9480_REG_RESET             0x1B
#define FSA9480_REG_CHG_CTRL1             0x20
#define FSA9480_REG_CHG_CTRL2             0x21
#define FSA9480_REG_CHG_CTRL3             0x22
#define FSA9480_REG_CHG_CTRL4             0x23
#define FSA9480_REG_CHG_INT             0x24
#define FSA9480_REG_CHG_INT_MASK             0x25
#define FSA9480_REG_CHG_STATUS             0x26

/* MANSW1 */
#define VAUDIO                 0x90
#define UART                   0x6c
#define AUDIO                  0x48
#define DHOST                  0x24
#define AUTO                   0x0

/*FSA9485 MANSW1*/
#define VAUDIO_9485            0x93
#define AUDIO_9485             0x4B
#define DHOST_9485             0x27

/* MANSW2 */
#define MANSW2_JIG		(1 << 2)

/* Control */
#define SWITCH_OPEN            (1 << 4)
#define RAW_DATA               (1 << 3)
#define MANUAL_SWITCH          (1 << 2)
#define WAIT                   (1 << 1)
#define INT_MASK               (1 << 0)
#define CTRL_MASK              (SWITCH_OPEN | RAW_DATA | MANUAL_SWITCH | \
                                       WAIT | INT_MASK)
/* Device Type 1*/
#define DEV_USB_OTG            (1 << 7)
#define DEV_DEDICATED_CHG      (1 << 6)
#define DEV_USB_CHG            (1 << 5)
#define DEV_CAR_KIT            (1 << 4)
#define DEV_UART               (1 << 3)
#define DEV_USB                        (1 << 2)
#define DEV_VBUS            (1 << 1)
#define DEV_MHL            (1 << 0)

#define FSA9480_DEV_T1_HOST_MASK               (DEV_USB_OTG)
#define FSA9480_DEV_T1_USB_MASK                (DEV_USB | DEV_USB_CHG)
#define FSA9480_DEV_T1_UART_MASK       (DEV_UART)
#define FSA9480_DEV_T1_CHARGER_MASK    (DEV_DEDICATED_CHG |DEV_CAR_KIT)

/* Device Type 2*/
#define DEV_AV                 (1 << 6)
#define DEV_TTY                        (1 << 5)
#define DEV_PPD                        (1 << 4)
#define DEV_JIG_UART_OFF       (1 << 3)
#define DEV_JIG_UART_ON                (1 << 2)
#define DEV_JIG_USB_OFF                (1 << 1)
#define DEV_JIG_USB_ON         (1 << 0)

#define FSA9480_DEV_T2_UART_MASK       (DEV_JIG_UART_OFF | DEV_JIG_UART_ON)

#define FSA9480_DEV_T2_JIG_MASK                (DEV_JIG_USB_OFF | DEV_JIG_USB_ON | \
                                       DEV_JIG_UART_OFF | DEV_JIG_UART_ON)

#define DEV_MHL                 (DEV_AV)
#define FSA9480_DEV_T2_MHL_MASK         (DEV_MHL)


/* Charger Control 1 Luke*/
#define CH_DIS                 (1 << 7)
#define FCMEN			(1 <<3)


/* Charger Status */
#define CH_FAULT	(1<<5)
#define CH_DONE                 (1 << 4)
#define CH_CV		(1<<3)
#define CH_FC		(1<<2)
#define CH_PC		(1<<1)
#define CH_IDLE		(1<<0)


struct fsa9480_usbsw {
       struct i2c_client               *client;
       struct fsa9480_platform_data    *pdata;
       struct work_struct              work;
       int                             dev1;
       int                             dev2;
       int                             mansw;
	   u8                              id;
};

static struct fsa9480_usbsw *chip;

/*Totoro*/
struct pm860x_vbus_info {
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	struct resource		*res;
	struct work_struct      wq;
	unsigned int		base;
	int			irq_status;
	int			irq_id;
	int			idpin;
	int			supply;
	int			work_data;
	int			work_mask;
};

//static struct wake_lock JIGConnect_idle_wake;
static struct wake_lock JIGConnect_suspend_wake;
static struct wake_lock mUSB_suspend_wake;

#if 0	// TODO: 
//static DEFINE_SPINLOCK(lock);
#define VBUS_DETECT /*TotoroTD*/
#endif

#ifdef CONFIG_VIDEO_MHL_V1
/*for MHL cable insertion*/
static int isMHLconnected=0;
#endif
static int isDeskdockconnected=0;

static int isProbe=0;
static int ovp_status=0;
enum {
	muicTypeTI6111 = 1,
	muicTypeFSA880 = 2,
	muicTypeFSA = 3,
};
static int muic_type=0;
//static int isManual=0;

// Static function prototype
static void TSU8111_Charger_Enable(void);
static void TSU8111_Charger_Disable(void);
static void TSU8111_Charger_BackCHG_Enable(void);
static void fsa9480_detect_dev(struct fsa9480_usbsw *usbsw, u8 intr);

static int fsa9480_write_reg(struct i2c_client *client,        u8 reg, u8 data)
{
       int ret = 0;
       u8 buf[2];
       struct i2c_msg msg[1];

       buf[0] = reg;
       buf[1] = data;

       msg[0].addr = client->addr;
       msg[0].flags = 0;
       msg[0].len = 2;
       msg[0].buf = buf;

       ret = i2c_transfer(client->adapter, msg, 1);
       if (ret != 1) {
               printk("\n [FSA9480] i2c Write Failed (ret=%d) \n", ret);
               return -1;
       }

       return ret;
}

static int fsa9480_read_reg(struct i2c_client *client, u8 reg, u8 *data)
{
       int ret = 0;
       u8 buf[1];
       struct i2c_msg msg[2];

       buf[0] = reg;

        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = buf;

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 1;
        msg[1].buf = buf;
		
       ret = i2c_transfer(client->adapter, msg, 2);
       if (ret != 2) {
               printk("\n [FSA9480] i2c Read Failed (ret=%d) \n", ret);
               return -1;
       }
       *data = buf[0];

       return 0;
}


int fsa9480_read_charger_status(u8 *val)
{
	u8 val1;
	int ret = 0;
	struct i2c_client *client = chip->client;

	ret = fsa9480_read_reg(client, FSA9480_REG_CHG_STATUS, &val1);
	if(ret < 0)
		ret = -EIO;
	else {
		*val = val1;
	}

	return ret;
}
EXPORT_SYMBOL(fsa9480_read_charger_status);


int fsa9480_read_charge_current(u8 *val)
{
	u8 val1;
	int ret = 0;
	struct i2c_client *client = chip->client;

	ret = fsa9480_read_reg(client, FSA9480_REG_CHG_CTRL3, &val1);
	if(ret < 0)
		ret = -EIO;
	else {
		*val = val1;
	}

	return ret;
}
EXPORT_SYMBOL(fsa9480_read_charge_current);


int fsa9480_is_back_chg()
{
	u8 val1;
	int ret = 0;

	struct i2c_client *client = chip->client;

	ret = fsa9480_read_reg(client, FSA9480_REG_CHG_CTRL2, &val1);
	if(ret < 0)
		ret = -EIO;

	if( val1==0x19)	// 0x19 // set EOC current 60mA, Full 4.18V
		return 1;
	else
		return 0;
	
}
EXPORT_SYMBOL(fsa9480_is_back_chg);

int fsa9480_reset_ic(void)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = chip->client;

    /*
	Disable Interrupt
	Write 0x01 to 0x1B Register <= Reset IC
	Write 0x40 to 0x05 Register <= Mask interrupt1
	Write 0x20 to 0x06 Register <= Mask interrupt2
	Write 0x2F to 0x25 Register <= Mask interrupt charger
	Write 0x1E to 0x02 Register <= Initialize control register
	Enable Interrupt 
	*/
	printk(KERN_WARNING "[FSA9480] %s. TSU8111 will be reset\n", __func__);
	fsa9480_detect_dev(usbsw, 2);

	disable_irq(client->irq);

	fsa9480_write_reg(client, 0x1B, 0x01);
	msleep(1);

	fsa9480_write_reg(client, FSA9480_REG_INT1_MASK, 0x40);
	fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x20);	
	fsa9480_write_reg(client, FSA9480_REG_CHG_INT_MASK, 0xc0);
	fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x1E);	

	enable_irq(client->irq);

	printk(KERN_WARNING "[FSA9480] %s. TSU8111 reset...Done\n", __func__);
	
	return 0;
	
}
EXPORT_SYMBOL(fsa9480_reset_ic);

static void fsa9480_read_adc_value(void)
{
	u8 adc=0;
    struct fsa9480_usbsw *usbsw = chip;
    struct i2c_client *client = usbsw->client;
	
	fsa9480_read_reg(client, FSA9480_REG_ADC, &adc);
	printk("[FSA9480] %s: adc is 0x%x\n",__func__,adc);
}

static void fsa9480_id_open(void)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	pr_alert("fsa9480 id_open\n");
	fsa9480_write_reg(client, 0x1B, 1);
}

void fsa9480_set_switch(const char *buf)
{
       struct fsa9480_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
       u8 value = 0;
       unsigned int path = 0;

       fsa9480_read_reg(client, FSA9480_REG_CTRL, &value);

       if (!strncmp(buf, "VAUDIO", 6)) {
	   	       if(usbsw->id == 0)
			   	path = VAUDIO_9485;
			   else
			   	path = VAUDIO;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "UART", 4)) {
               path = UART;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "AUDIO", 5)) {
               if(usbsw->id == 0)
			   	path = AUDIO_9485;
			   else
                path = AUDIO;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "DHOST", 5)) {
               path = DHOST;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "AUTO", 4)) {
               path = AUTO;
               value |= MANUAL_SWITCH;
       } else {
               printk(KERN_ERR "Wrong command\n");
               return;
       }

       usbsw->mansw = path;
       fsa9480_write_reg(client, FSA9480_REG_MANSW1, path);
       fsa9480_write_reg(client, FSA9480_REG_CTRL, value);
}
EXPORT_SYMBOL_GPL(fsa9480_set_switch);

ssize_t fsa9480_get_switch(char *buf)
{
struct fsa9480_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
       u8 value;

       fsa9480_read_reg(client, FSA9480_REG_MANSW1, &value);

       if (value == VAUDIO)
               return sprintf(buf, "VAUDIO\n");
       else if (value == UART)
               return sprintf(buf, "UART\n");
       else if (value == AUDIO)
               return sprintf(buf, "AUDIO\n");
       else if (value == DHOST)
               return sprintf(buf, "DHOST\n");
       else if (value == AUTO)
               return sprintf(buf, "AUTO\n");
       else
               return sprintf(buf, "%x", value);
}
EXPORT_SYMBOL_GPL(fsa9480_get_switch);


#ifdef CONFIG_VIDEO_MHL_V1
static void DisableFSA9480Interrupts(void)
{
       struct fsa9480_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
        printk ("DisableFSA9480Interrupts-2\n");

     fsa9480_write_reg(client, FSA9480_REG_INT1_MASK, 0xFF);
     fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x1F);
	 
} // DisableFSA9480Interrupts()

static void EnableFSA9480Interrupts(void)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;
	u8 intr, intr2;

	printk ("EnableFSA9480Interrupts\n");

     /*clear interrupts*/
     fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
     fsa9480_read_reg(client, FSA9480_REG_INT2, &intr2);
	 
     fsa9480_write_reg(client, FSA9480_REG_INT1_MASK, 0x00);
     fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x00);

} //EnableFSA9480Interrupts()

void FSA9480_EnableIntrruptByMHL(bool _bDo)
{
	struct fsa9480_platform_data *pdata = chip->pdata;
	struct i2c_client *client = chip->client;
	char buf[16];

	if(true == _bDo)
	{
		fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1E);
		EnableFSA9480Interrupts();
	}
	else
	{
		DisableFSA9480Interrupts();
	}

	fsa9480_get_switch(buf);
	printk("[%s] fsa switch status = %s\n",__func__, buf);
}

/*MHL call this function to change VAUDIO path*/
void FSA9480_CheckAndHookAudioDock(void)
{
   struct fsa9480_platform_data *pdata = chip->pdata;
   struct i2c_client *client = chip->client;

   printk("[FSA9480] %s: FSA9485 VAUDIO\n",__func__);
   
   isMHLconnected = 0;
   isDeskdockconnected = 1;
   
   if (pdata->mhl_cb)
   	       pdata->mhl_cb(FSA9480_DETACHED);

   EnableFSA9480Interrupts();

   if(chip->id == 0)
	chip->mansw = VAUDIO_9485;
   else
	chip->mansw = VAUDIO;

   /*make ID change report*/
   fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x16);
   
   if(pdata->deskdock_cb)
           pdata->deskdock_cb(FSA9480_ATTACHED);   

}
EXPORT_SYBMOL_GPL(FSA9480_CheckAndHookAudioDock);
#endif


static ssize_t fsa9480_show_status(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
       struct fsa9480_usbsw *usbsw = dev_get_drvdata(dev);
       struct i2c_client *client = usbsw->client;
       u8 devid, ctrl, adc, dev1, dev2, intr;
       u8 intmask1, intmask2, time1, time2, mansw1;

       fsa9480_read_reg(client, FSA9480_REG_DEVID, &devid);
       fsa9480_read_reg(client, FSA9480_REG_CTRL, &ctrl);
       fsa9480_read_reg(client, FSA9480_REG_ADC, &adc);
       fsa9480_read_reg(client, FSA9480_REG_INT1_MASK, &intmask1);
       fsa9480_read_reg(client, FSA9480_REG_INT2_MASK, &intmask2);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &dev1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &dev2);
       fsa9480_read_reg(client, FSA9480_REG_TIMING1, &time1);
       fsa9480_read_reg(client, FSA9480_REG_TIMING2, &time2);
       fsa9480_read_reg(client, FSA9480_REG_MANSW1, &mansw1);

       fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
       intr &= 0xffff;

       return sprintf(buf, "Device ID(%02x), CTRL(%02x)\n"
                       "ADC(%02x), DEV_T1(%02x), DEV_T2(%02x)\n"
                       "INT(%04x), INTMASK(%02x, %02x)\n"
                       "TIMING(%02x, %02x), MANSW1(%02x)\n",
                       devid, ctrl, adc, dev1, dev2, intr,
                       intmask1, intmask2, time1, time2, mansw1);
}

static ssize_t fsa9480_show_manualsw(struct device *dev,
               struct device_attribute *attr, char *buf)
{
       return fsa9480_get_switch(buf);

}

static ssize_t fsa9480_set_manualsw(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
       fsa9480_set_switch(buf);
       return count;
}


void fsa9480_force_sleep(void)
{
    struct fsa9480_usbsw *usbsw = chip;
    struct i2c_client *client = usbsw->client;
    u8 value = 0;

    //wake_unlock(&JIGConnect_idle_wake);
    wake_unlock(&JIGConnect_suspend_wake);
#ifdef CONFIG_KONA_PI_MGR
    pi_mgr_qos_request_update(&qos_node, PI_MGR_QOS_DEFAULT_VALUE);
#endif
    if(muic_type==muicTypeFSA880)
    {
        fsa9480_read_reg(client, FSA9480_REG_MANSW2, &value);
        fsa9480_write_reg(client, FSA9480_REG_MANSW2, 0x00);

        fsa9480_read_reg(client, FSA9480_REG_CTRL, &value);
        fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x00);
    }
    else
    {
        fsa9480_read_reg(client, FSA9480_REG_CTRL, &value);
        value &= ~MANUAL_SWITCH;
        fsa9480_write_reg(client, FSA9480_REG_CTRL, value);

        fsa9480_read_reg(client, FSA9480_REG_MANSW2, &value);
        value &= ~MANSW2_JIG;
        fsa9480_write_reg(client, FSA9480_REG_MANSW2, value);
    }
}



static ssize_t fsa9480_set_syssleep(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;
	u8 value = 0;

	if (!strncmp(buf, "1", 1))
	{
		//wake_unlock(&JIGConnect_idle_wake);
		wake_unlock(&JIGConnect_suspend_wake);
#ifdef CONFIG_KONA_PI_MGR
		pi_mgr_qos_request_update(&qos_node, PI_MGR_QOS_DEFAULT_VALUE);
#endif
		if(muic_type==muicTypeFSA880)
		{
			fsa9480_read_reg(client, FSA9480_REG_MANSW2, &value);
			fsa9480_write_reg(client, FSA9480_REG_MANSW2, 0x00);

			fsa9480_read_reg(client, FSA9480_REG_CTRL, &value);
			fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x00);
		}
		else
		{
			fsa9480_read_reg(client, FSA9480_REG_CTRL, &value);
			value &= ~MANUAL_SWITCH;
			fsa9480_write_reg(client, FSA9480_REG_CTRL, value);
		
			fsa9480_read_reg(client, FSA9480_REG_MANSW2, &value);
			value &= ~MANSW2_JIG;
			fsa9480_write_reg(client, FSA9480_REG_MANSW2, value);
		}
		//isManual=1;
	}
	else
	{
		fsa9480_read_reg(client, FSA9480_REG_CTRL, &value);
		value |= MANUAL_SWITCH;
		fsa9480_write_reg(client, FSA9480_REG_CTRL, value);
	}
	return count;
}
					

static DEVICE_ATTR(status, S_IRUGO, fsa9480_show_status, NULL);
static DEVICE_ATTR(switch, S_IRUGO | S_IWGRP,
               fsa9480_show_manualsw, fsa9480_set_manualsw);
static DEVICE_ATTR(syssleep, S_IWUSR, NULL, fsa9480_set_syssleep);

static struct attribute *fsa9480_attributes[] = {
       &dev_attr_status.attr,
       &dev_attr_switch.attr,
       &dev_attr_syssleep.attr,
       NULL
};

static const struct attribute_group fsa9480_group = {
       .attrs = fsa9480_attributes,
};

static irqreturn_t fsa9480_irq_handler(int irq, void *data)
{
	struct fsa9480_usbsw *usbsw = data;
  
	schedule_work(&usbsw->work);
	return IRQ_HANDLED;
}

/* SW RESET for TI USB:To fix no USB recog problem after jig attach&detach*/
static void TI_SWreset(struct fsa9480_usbsw *usbsw)
{
#if 0	// TODO: 
	struct i2c_client *client = usbsw->client;

	printk("[FSA9480] TI_SWreset ...Start\n");
	disable_irq(client->irq); 

	/*Hold SCL&SDA Low more than 30ms*/
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO64),0);
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO66),0);
	msleep(31);
	/*Make SCL&SDA High again*/
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO64),1);
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO66),1);
	/*Should I make this input setting? Not Sure*/
	//gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO64));
	//gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO66));

	/*Write SOME Init register value again*/
	fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x20);
	fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x1E);

	enable_irq(client->irq); 
#endif	// TODO: 
	printk("[FSA9480] TI_SWreset ...Done\n");
}
 
 //TI Switch Test :(USB)->Uart->USB
  static void TI_Switch(void)
  {
	 struct i2c_client *client = chip->client;
 
	 printk("[FSA9480] TI_Switch ...Start\n");
	 disable_irq(client->irq); 
 
	 if(chip->dev1 &  DEV_USB)
		 printk("[FSA9480] Waring! previous status is NOT USB attached!! \n");
 
	 //Set Manual switching 
	 fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x1B);
	 // Manual to UART
	 fsa9480_write_reg(client, FSA9480_REG_MANSW1, 0x6C); // 011 011 00
 
	 msleep(1000);
	 //Manual to USB
	 fsa9480_write_reg(client, FSA9480_REG_MANSW1, 0x24); // 001 001 00
	 // Set Auto switching by interrupt
	 fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x1E);
 
	 enable_irq(client->irq); 
	 printk("[FSA9480] TI_Switch ...Done\n");
 }
  

void send_usb_attach_event();
void send_usb_detach_event();
 //extern void mv_usb_connect_change(int status); // khMa
static void fsa9480_detect_dev(struct fsa9480_usbsw *usbsw, u8 intr)
{
       u8 val1, val2;// , ctrl,temp;
       //struct fsa9480_platform_data *pdata = usbsw->pdata;
       struct i2c_client *client = usbsw->client;
	   printk("[FSA9480] fsa9480_detect_dev !!!!!\n");

#if 0 // Not for TI
       /*reset except CP USB and AV dock*/
	   if ((usbsw->mansw != AUDIO) && (usbsw->mansw != AUDIO_9485)
	   	   && (usbsw->mansw != VAUDIO) && (usbsw->mansw != VAUDIO_9485))
       	{
		  /*reset UIC when mansw is not set*/
		  printk("[FSA9480] %s: reset UIC mansw is 0x%x\n",__func__,usbsw->mansw);
          fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x1E);
		  usbsw->mansw = AUTO;
       	} 
#endif

       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);
       printk("intr: 0x%x, dev1: 0x%x, dev2: 0x%x\n",intr, val1, val2);

#if 0//Not for TI	   
       fsa9480_read_reg(client, FSA9480_REG_CTRL, &ctrl);

       dev_info(&client->dev, "intr: 0x%x, dev1: 0x%x, dev2: 0x%x, ctrl: 0x%x\n",
                       intr, val1, val2,ctrl);
#endif

	if((intr==0x01) &&(val1==0x00) && (val2==0x00) && (isProbe == 0))
	{
		printk("[FSA9480] (intr==0x01) &&(val1==0x00) && (val2==0x00) !!!!!\n");
		fsa9480_read_adc_value();
		if(muic_type==muicTypeTI6111)
		{
			msleep(50);

			fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
			fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);

			//TI_SWreset(usbsw);
			//return;
		}
	}
	else if(intr==0x03)
	{
		printk("[FSA9480] error read INT register !!!!!\n");
		intr = (1 << 1); // change to DETACH
	}

	/* Attached */
	if (intr & (1 << 0)) 
	{
		if (val1 & FSA9480_DEV_T1_USB_MASK ) {
			printk("[FSA9480] FSA9480_USB ATTACHED*****\n");

#if defined (VBUS_DETECT) /* Enable clock to AP USB block */
			pxa_vbus_handler(VBUS_HIGH);
#endif			
#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_USB_ATTACHED);
			}
#elif defined(CONFIG_BATTERY_D2083)
			if(spa_external_event)
			{
				spa_external_event(D2083_CATEGORY_DEVICE, D2083_EVENT_USB_ATTACHED);
			}
#endif
	                send_usb_attach_event();
		}

		if (val1 & FSA9480_DEV_T1_UART_MASK || val2 & FSA9480_DEV_T2_UART_MASK) {
			//if (pdata->uart_cb)
			//        pdata->uart_cb(FSA9480_ATTACHED);				   
		}

		if (val1 & FSA9480_DEV_T1_CHARGER_MASK || val1==DEV_VBUS || val2 & DEV_JIG_USB_OFF) {
			printk("[FSA9480] Charger ATTACHED*****\n");
#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_TA_ATTACHED);
			}
#elif defined(CONFIG_BATTERY_D2083)
			if(spa_external_event)
			{
				spa_external_event(D2083_CATEGORY_DEVICE, D2083_EVENT_TA_ATTACHED);
			}
#endif
		}

		if (val2 & FSA9480_DEV_T2_JIG_MASK) {
			printk("[FSA9480] JIG ATTACHED*****\n"); 			   
			//wake_lock(&JIGConnect_idle_wake);
			wake_lock(&JIGConnect_suspend_wake);
#ifdef CONFIG_KONA_PI_MGR
			pi_mgr_qos_request_update(&qos_node, 0);
#endif

#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_JIG_ATTACHED);
			}
#elif defined(CONFIG_BATTERY_D2083)
			if(spa_external_event)
			{
				spa_external_event(D2083_CATEGORY_DEVICE, D2083_EVENT_JIG_ATTACHED);
			}
#endif
		}
	} 
	else if (intr & (1 << 1)) 
	{    /* DETACH */
		if(usbsw->dev1 & FSA9480_DEV_T1_USB_MASK) {
				printk("[FSA9480] FSA9480_USB Detached*****\n");
#if defined (VBUS_DETECT) /* Disable clock to AP USB block */
			pxa_vbus_handler(VBUS_LOW);
#endif			
#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_USB_DETACHED);
			}
#elif defined(CONFIG_BATTERY_D2083)
			if(spa_external_event)
			{
				spa_external_event(D2083_CATEGORY_DEVICE, D2083_EVENT_USB_DETACHED);
			}
#endif
			send_usb_detach_event();
		}

		if (usbsw->dev1 & FSA9480_DEV_T1_UART_MASK ||
                       usbsw->dev2 & FSA9480_DEV_T2_UART_MASK) {
			//if (pdata->uart_cb)
			//     pdata->uart_cb(FSA9480_DETACHED);
		}

		if (usbsw->dev1 & FSA9480_DEV_T1_CHARGER_MASK || usbsw->dev1==DEV_VBUS || usbsw->dev2 & DEV_JIG_USB_OFF) {
			printk("[FSA9480] Charger Detached*****\n");
#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_TA_DETACHED);
			}
#elif defined(CONFIG_BATTERY_D2083)
			if(spa_external_event)
			{
				spa_external_event(D2083_CATEGORY_DEVICE, D2083_EVENT_TA_DETACHED);
			}
#endif
		}

		if (usbsw->dev2 & FSA9480_DEV_T2_JIG_MASK) {      
			printk("[FSA9480] JIG Detached*****\n");			   	
			//wake_unlock(&JIGConnect_idle_wake);
			wake_unlock(&JIGConnect_suspend_wake);
#ifdef CONFIG_KONA_PI_MGR
			pi_mgr_qos_request_update(&qos_node, PI_MGR_QOS_DEFAULT_VALUE);
#endif

#if defined(CONFIG_SPA)
			if(spa_external_event)
			{
				spa_external_event(SPA_CATEGORY_DEVICE, SPA_DEVICE_EVENT_JIG_DETACHED);
			}
#elif defined(CONFIG_BATTERY_D2083)
			if(spa_external_event)
			{
				spa_external_event(D2083_CATEGORY_DEVICE, D2083_EVENT_JIG_DETACHED);
			}
#endif
			if(muic_type==muicTypeTI6111)
			{
				/*SW RESET for TI USB:To fix no USB recog problem after jig attach&detach*/
				//TI_SWreset(usbsw);
			}
		}
	}

	if( intr ){
		usbsw->dev1 = val1;
		usbsw->dev2 = val2;

		chip->dev1 = val1;
		chip->dev2 = val2;
	}
#if 0 // Not for TI
	fsa9480_read_reg(client, FSA9480_REG_CTRL, &ctrl);
	ctrl &= ~INT_MASK;
	fsa9480_write_reg(client, FSA9480_REG_CTRL, ctrl);

	fsa9480_read_reg(client, FSA9480_REG_MANSW1, &temp); //khMa	   
#endif
}

#if 0	// TODO: 
int get_real_usbic_state(void)
{
       struct fsa9480_usbsw *usbsw = chip;
       int ret = MICROUSBIC_NO_DEVICE ;
       u8 val1 = 0;
       u8 val2 = 0;

       /* read real usb ic state
       val1 = chip->dev1;
       val2 = chip->dev2;
       */
       struct i2c_client *client = usbsw->client;
       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);

       if (val1 & FSA9480_DEV_T1_USB_MASK)
               ret = MICROUSBIC_USB_CABLE;
       else if (val1 & FSA9480_DEV_T1_CHARGER_MASK)
               ret = MICROUSBIC_USB_CHARGER;
       else if (val1 & FSA9480_DEV_T1_UART_MASK)
               ret = MICROUSBIC_USB_CHARGER;
	   else if (val1 & FSA9480_DEV_T1_HOST_MASK)
		   	   ret = MICROUSBIC_HOST;

       if (ret ==  MICROUSBIC_NO_DEVICE) {
               if (val2 & DEV_JIG_USB_ON)
				   ret = MICROUSBIC_JIG_USB_ON;
			   else if (val2 & FSA9480_DEV_T2_MHL_MASK)
				   ret = MICROUSBIC_MHL_CHARGER;
       }

       return ret;
}
#endif

static void tsu8111_check_ovp()
{
	 struct i2c_client *client = chip->client;
	u8 val1,chg_status;
	
	msleep(300);	
       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
   	fsa9480_read_reg(client, FSA9480_REG_CHG_STATUS, &chg_status);
	
  	printk("check ovp: dev1: 0x%x, chg_status: 0x%x\n",val1, chg_status);		
		
	if( (val1 & DEV_VBUS) && (chg_status & (CH_FAULT|CH_IDLE)) ){
		ovp_status = 1;
#if defined(CONFIG_BATTERY_D2083)
		if(spa_external_event)
			spa_external_event(D2083_CATEGORY_BATTERY, D2083_EVENT_OVP_CHARGE_STOP);
#endif		
		pr_info("%s: OVP Chg Stop\n",__func__);
	}
	else if( ovp_status==1 ){

		if( chg_status & (CH_FC|CH_CV) ){
			ovp_status = 0;
#if defined(CONFIG_BATTERY_D2083)
			if(spa_external_event)
				spa_external_event(D2083_CATEGORY_BATTERY, D2083_EVENT_OVP_CHARGE_RESTART);
#endif		
			pr_info("%s: OVP Chg Restart\n",__func__);
		}
	}
}

static void fsa9480_work_cb(struct work_struct *work)
{
	u8 intr, intr2, intr3;
	struct fsa9480_usbsw *usbsw = container_of(work, struct fsa9480_usbsw, work);
	struct i2c_client *client = usbsw->client;

	wake_lock_timeout(&mUSB_suspend_wake,1*HZ);
	
       /* clear interrupt */
	if(muic_type==muicTypeTI6111)
	{
		msleep(200);
	
		fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
		fsa9480_read_reg(client, FSA9480_REG_INT2, &intr2);	
		fsa9480_read_reg(client, FSA9480_REG_CHG_INT, &intr3);
		printk("[FSA9480] %s: intr=0x%x, intr2 = 0x%X, chg_intr=0x%x \n",__func__,intr,intr2, intr3);


	}
	else
	{
		fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
		fsa9480_read_reg(client, FSA9480_REG_INT2, &intr2);	
		printk("[FSA9480] %s: intr=0x%x, intr2=0x%x \n",__func__,intr,intr2);
	}

	if(intr3 & CH_DONE) //EOC disable charger // Luke 
	{
		//msleep(500);	 // Test only

		//fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
		//if(val1&=DEV_VBUS) // Check if VBUS is valid //Luke
		//{
#if defined(CONFIG_BATTERY_D2083)
			if(spa_external_event)
			{
				pr_info("%s. Send D2083_EVENT_CHARGE_FULL event\n", __func__);
				spa_external_event(D2083_CATEGORY_BATTERY, D2083_EVENT_CHARGE_FULL);
			}
		//}
#endif
	}

	intr &= 0xffff;

	/* device detection */
	fsa9480_detect_dev(usbsw, intr);

	if((intr== 0x00) && (intr3 == 0))
	{	
		printk("[FSA9480] (intr== 0x00) in work_cb !!!!!\n");
		fsa9480_read_adc_value();
		#if 0	// TODO: 
		if(muic_type==muicTypeTI6111)
			TI_SWreset(usbsw);
		#endif
		
		return;
	}

	if(!(intr3 & CH_DONE))
		tsu8111_check_ovp();
	
	if( intr==0x03) // INT error case
		fsa9480_reset_ic();
	
}

static int fsa9480_irq_init(struct fsa9480_usbsw *usbsw)
{
	//struct fsa9480_platform_data *pdata = usbsw->pdata;
	struct i2c_client *client = usbsw->client;
	int ret, irq = -1;

	INIT_WORK(&usbsw->work, fsa9480_work_cb);

	printk(KERN_ERR "[FSA9480] fsa9480_irq_init\n");
	ret = gpio_request(irq_to_gpio(client->irq), "fsa9480 irq");
	if(ret)
	{
		dev_err(&client->dev,"fsa9480: Unable to get gpio %d\n", client->irq);
		goto gpio_out;
	}
	gpio_direction_input(irq_to_gpio(client->irq));

	ret = request_irq(client->irq, fsa9480_irq_handler,
	              IRQF_NO_SUSPEND|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_LOW*/,
					  ///*IRQF_NO_SUSPEND|*/ IRQF_TRIGGER_FALLING /*|IRQF_TRIGGER_LOW*/,
	               "fsa9480 micro USB", usbsw); /*2. Low level detection*/
	if (ret) {
		dev_err(&client->dev,
				   "fsa9480: Unable to get IRQ %d\n", irq);
		goto out;
	}

	return 0;
gpio_out:
	gpio_free(client->irq);
out:
       return ret;
}



static int __devinit fsa9480_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
	//struct regulator *regulator;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct fsa9480_platform_data *pdata = client->dev.platform_data;
	struct fsa9480_usbsw *usbsw;
	struct pm860x_vbus_info *vbus;
#if 0	// TODO: 
	struct pxa_vbus_info info;
#endif
	unsigned int data;
	int ret = 0;
	u8 devID;
	
	u8 intr, intr2, intr_chg;
	u8 mansw1;
	unsigned int ctrl = CTRL_MASK;

	printk("[FSA9480] PROBE ......\n");
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;	   
	isProbe = 1;
	   
	//add for AT Command 
	//wake_lock_init(&JIGConnect_idle_wake, WAKE_LOCK_IDLE, "jig_connect_idle_wake");
	wake_lock_init(&JIGConnect_suspend_wake, WAKE_LOCK_SUSPEND, "jig_connect_suspend_wake");
	wake_lock_init(&mUSB_suspend_wake, WAKE_LOCK_SUSPEND, "mUSB_detect");

#ifdef CONFIG_KONA_PI_MGR
	ret=pi_mgr_qos_add_request(&qos_node, "fsa9480", PI_MGR_PI_ID_ARM_SUB_SYSTEM,
			       PI_MGR_QOS_DEFAULT_VALUE);
	if (ret)
	{
		pr_err("%s: failed to add fsa9480 to qos\n",__func__);
		return -1;
	}
#endif

	usbsw = kzalloc(sizeof(struct fsa9480_usbsw), GFP_KERNEL);
	if (!usbsw) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	usbsw->client = client;
	usbsw->pdata = client->dev.platform_data;

	chip = usbsw;

	i2c_set_clientdata(client, usbsw);

#if defined (VBUS_DETECT) /* Init vbus for pxa(MARVELL) */
//	chip->pdata = pm860x_pdata->vbus;
	vbus = kzalloc(sizeof(struct pm860x_vbus_info), GFP_KERNEL);
	if (!vbus) {
		ret = -ENOMEM;
		goto out_mem;
	}
	dev_set_drvdata(&client->dev, vbus);
	
	vbus->res = kzalloc(sizeof(struct resource), GFP_KERNEL);
	if (!vbus->res) {
		ret = -ENOMEM;
		goto out_mem2;
	}
	vbus->res->start = pdata->vbus->reg_base;
	vbus->res->end = pdata ->vbus->reg_end;

	memset(&info,0,sizeof(struct pxa_vbus_info));
	info.dev = &client->dev;
	info.res = vbus->res;

	pxa_vbus_init(&info);

	data = VBUS_A_VALID | VBUS_A_SESSION_VALID | VBUS_B_SESSION_VALID | VBUS_B_SESSION_END | VBUS_ID;
	pxa_unmask_vbus(data);
#endif

#if defined(CONFIG_SPA)
	spa_external_event = spa_get_external_event_handler();
#elif defined(CONFIG_BATTERY_D2083)
	ret = d2083_register_enable_charge(TSU8111_Charger_Enable);
	if(ret < 0) {
		pr_err("%s. fail to register enable charge function\n", __func__);
		goto out_charger_enable;
	}
	ret = d2083_register_enable_back_charge(TSU8111_Charger_BackCHG_Enable);
	if(ret < 0) {
		pr_err("%s. fail to register enable charge function\n", __func__);
		goto out_charger_enable;
	}	
	ret = d2083_register_disable_charge(TSU8111_Charger_Disable);
	if(ret < 0) {
		pr_err("%s. fail to register disable charge function\n", __func__);
		goto out_charger_disable;
	}
	spa_external_event = d2083_get_external_event_handler();
#endif

	/* clear interrupt */
	fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);

	fsa9480_read_reg(client, FSA9480_REG_DEVID, &devID);
	if(devID==0x0a || devID==0x5A)
		muic_type=muicTypeTI6111;
	else if(devID==0x00)
		muic_type=muicTypeFSA880;
	else
		muic_type=muicTypeFSA;

	if(muic_type==muicTypeFSA880)
	{
		intr &= 0xffff;
		/* set control register */
		fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x04);
	}
	else if(muic_type==muicTypeTI6111)
	{
		intr &= 0xffff;

		/* clear interrupt */
		fsa9480_read_reg(client, FSA9480_REG_INT2, &intr2);
		fsa9480_read_reg(client, FSA9480_REG_CHG_INT, &intr_chg);

		/* unmask interrupt (attach/detach only) */
		ret = fsa9480_write_reg(client, FSA9480_REG_INT1_MASK, 0x00);
		if (ret < 0)
			return ret;

#if 0/*FSA9480's Interrupt Mask init setting*/
	   //ret = fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x00);
#endif
		/*TI USB : not to get Connect Interrupt : no more double interrupt*/
		ret = fsa9480_write_reg(client, FSA9480_REG_INT1_MASK, 0x40);
		if (ret < 0)
			return ret;

		ret = fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x20);
		if (ret < 0)
			return ret;

		ret = fsa9480_write_reg(client, FSA9480_REG_CHG_INT_MASK, 0xc0);
		if (ret < 0)
			return ret;


	   fsa9480_read_reg(client, FSA9480_REG_MANSW1, &mansw1);
	   usbsw->mansw = mansw1;

		ctrl &= ~INT_MASK;			  /* Unmask Interrupt */
		if (usbsw->mansw)
			ctrl &= ~MANUAL_SWITCH; /* Manual Switching Mode */
	   
		fsa9480_write_reg(client, FSA9480_REG_CTRL, ctrl);
	}
	else
	   printk("[FSA9480] Error!!!! No Type. Check dev ID(0x01 addr) ......\n");

   ret = fsa9480_irq_init(usbsw);
   if (ret)
           goto fsa9480_probe_fail;

   ret = sysfs_create_group(&client->dev.kobj, &fsa9480_group);
   if (ret) {
           dev_err(&client->dev,
                           "[FSA9480] Creating fsa9480 attribute group failed");
           goto fsa9480_probe_fail2;
   }

	/* device detection */
	fsa9480_detect_dev(usbsw, 1);
	isProbe = 0;
	tsu8111_check_ovp();

	/*reset UIC*/
	if(muic_type==muicTypeFSA880)
		fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x04);
	else
	{
		fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x1E);
		/*set timing1 to 100ms*/
//		fsa9480_write_reg(client, FSA9480_REG_TIMING1, 0x1);
	}	   

#ifdef CONFIG_KONA_PI_MGR
	//ret=pi_mgr_qos_request_update(&qos_node, 0);
	//if (ret)
	//{
	//	pr_err("%s: failed to request update qos_node\n",__func__);
	//}
#endif


       printk("[FSA9480] PROBE Done.\n");
       return 0;

out_mem:
	return ret;
#if defined(CONFIG_BATTERY_D2083)
out_charger_enable:
out_charger_disable:
#endif /* CONFIG_BATTERY_D2083 */
out_mem2:
	kfree(vbus);
fsa9480_probe_fail2:
   if (client->irq)
       free_irq(client->irq, NULL);
fsa9480_probe_fail:
	i2c_set_clientdata(client, NULL);
	kfree(usbsw);
	return ret;
}

static int __devexit fsa9480_remove(struct i2c_client *client)
{
       struct fsa9480_usbsw *usbsw = i2c_get_clientdata(client);
       if (client->irq)
               free_irq(client->irq, NULL);
       i2c_set_clientdata(client, NULL);

       sysfs_remove_group(&client->dev.kobj, &fsa9480_group);
       kfree(usbsw);
       return 0;
}

static int fsa9480_suspend(struct i2c_client *client)
{
       return 0;
}


#ifdef CONFIG_PM
static int fsa9480_resume(struct i2c_client *client)
{

       return 0;
}
#else
#define fsa9480_resume         NULL

#endif

static void TSU8111_Charger_BackCHG_Enable(void)
{
	u8 val1,dev1;
	int ret = 0;
	struct i2c_client *client = chip->client;
	pr_info("%s\n",__func__);

	fsa9480_write_reg(client, FSA9480_REG_CHG_CTRL2, 0x19); // set EOC current 60mA, Full 4.18V

	fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &dev1);

	if( dev1 & FSA9480_DEV_T1_CHARGER_MASK)
		fsa9480_write_reg(client, FSA9480_REG_CHG_CTRL3, 0xD9); // 650mA
	else
		fsa9480_write_reg(client, FSA9480_REG_CHG_CTRL3, 0xD5); // 450mA
	
	fsa9480_read_reg(client, FSA9480_REG_CHG_CTRL1, &val1);
	val1 &= ~CH_DIS;
	val1 |= FCMEN;
	pr_info("%s. Register(%d) = (0x%X)\n", __func__, FSA9480_REG_CHG_CTRL1, val1);
	ret = fsa9480_write_reg(client, FSA9480_REG_CHG_CTRL1, val1);
	if (ret < 0) {
		printk("[FSA9480] I2C write fail\n");
	}
		
	return ;
}

static void TSU8111_Charger_Enable(void)
{
	u8 val1,dev1;
	int ret = 0;
	struct i2c_client *client = chip->client;
	pr_info("%s\n",__func__);

	fsa9480_write_reg(client, FSA9480_REG_CHG_CTRL2, 0x89); // set EOC current 130mA, Full 4.18V

	fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &dev1);

	if( (dev1 & (FSA9480_DEV_T1_CHARGER_MASK|DEV_USB_CHG)) || (dev1==DEV_VBUS))
		fsa9480_write_reg(client, FSA9480_REG_CHG_CTRL3, 0xD9); // 650mA
	else
		fsa9480_write_reg(client, FSA9480_REG_CHG_CTRL3, 0xD5); // 450mA
	
	fsa9480_read_reg(client, FSA9480_REG_CHG_CTRL1, &val1);
	val1 &= ~CH_DIS;
	val1 |= FCMEN;
	
	// Set Fast Charge Timer 6 Hour
	val1 &= ~0x03; val1 |= 0x01;
	
	pr_info("%s. Register(%d) = (0x%X)\n", __func__, FSA9480_REG_CHG_CTRL1, val1);
	ret = fsa9480_write_reg(client, FSA9480_REG_CHG_CTRL1, val1);
	if (ret < 0) {
		printk("[FSA9480] I2C write fail\n");
	}
		
	return ;
}

static void TSU8111_Charger_Disable(void)
{
	u8 val1;
	int ret = 0;
	struct i2c_client *client = chip->client;
	pr_info("%s\n",__func__);

	fsa9480_read_reg(client, FSA9480_REG_CHG_CTRL1, &val1);
	val1 |= CH_DIS;
	pr_info("%s. Register(%d) = (0x%X)\n", __func__, FSA9480_REG_CHG_CTRL1, val1);
	ret = fsa9480_write_reg(client, FSA9480_REG_CHG_CTRL1, val1);
	if (ret < 0) {
		printk("[FSA9480] I2C write fail\n");
	}

	return ;
}


static const struct i2c_device_id fsa9480_id[] = {
       {"fsa9480", 0},
       {}
};
MODULE_DEVICE_TABLE(i2c, fsa9480_id);

static struct i2c_driver fsa9480_i2c_driver = {
       .driver = {
               .name = "fsa9480",
       },
       .probe = fsa9480_probe,
       .remove = __devexit_p(fsa9480_remove),
       .suspend=fsa9480_suspend,
       .resume = fsa9480_resume,
       .id_table = fsa9480_id,
};

static int __init fsa9480_init(void)
{
       return i2c_add_driver(&fsa9480_i2c_driver);
}

module_init(fsa9480_init);

#ifdef CONFIG_CHARGER_DETECT_BOOT
charger_module_init(fsa9480_init);
#endif

static void __exit fsa9480_exit(void)
{
       i2c_del_driver(&fsa9480_i2c_driver);
}

module_exit(fsa9480_exit);

MODULE_AUTHOR("Wonguk.Jeong <wonguk.jeong@samsung.com>");
MODULE_DESCRIPTION("FSA9480 USB Switch driver");
MODULE_LICENSE("GPL");



