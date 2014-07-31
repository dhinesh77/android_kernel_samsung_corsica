/*
 * d2083-core.c  --  Device access for Dialog D2083
 *   
 * Copyright(c) 2012 Dialog Semiconductor Ltd.
 *  
 * Author: Dialog Semiconductor Ltd. D. Chen
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/bug.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/ioport.h>

#include <linux/smp.h>

#include <linux/d2083/d2083_version.h>
#include <linux/d2083/d2083_reg.h> 
#include <linux/d2083/core.h>
#include <linux/d2083/pmic.h>
#include <linux/d2083/rtc.h>
#include <mach/gpio.h>

#include <linux/proc_fs.h>
#include <linux/kthread.h>

#if defined(CONFIG_MACH_RHEA_SS_IVORY) || defined(CONFIG_MACH_RHEA_SS_NEVIS) || defined(CONFIG_MACH_RHEA_SS_NEVIS_REV00) || defined(CONFIG_MACH_RHEA_SS_NEVISP) || defined(CONFIG_MACH_RHEA_SS_CORSICA)|| defined(CONFIG_MACH_RHEA_SS_NEVISDS)
#include <linux/broadcom/pmu_chip.h>
#include <asm/uaccess.h>
#endif /* CONFIG_MACH_RHEA_SS_IVORY */

#define D2083_REG_DEBUG

#ifdef D2083_REG_DEBUG
#define D2083_MAX_HISTORY           100
#define D2083_HISTORY_READ_OP       0
#define D2083_HISTORY_WRITE_OP      1

struct d2083_reg_history{
	u8 mode;
	u8 regnum; 
	u8 value;
	long long time;
};


static u8 gD2083RegCache[D2083_MAX_REGISTER_CNT];
static struct d2083_reg_history gD2083RegHistory[D2083_MAX_HISTORY];
static u8 gD2083CurHistory=0;

#define d2083_write_reg_cache(reg,data)   gD2083RegCache[reg]=data;
#endif

/*
 *   Static global variable
 */
static struct d2083 *d2083_dev_info;

/*
 * D2083 Device IO
 */
static DEFINE_MUTEX(io_mutex);

#ifdef D2083_REG_DEBUG
void d2083_write_reg_history(u8 opmode,u8 reg,u8 data) { 
//	int cpu = smp_processor_id(); 

	if(gD2083CurHistory==D2083_MAX_HISTORY) 
		gD2083CurHistory=0;
		
	gD2083RegHistory[gD2083CurHistory].mode=opmode; 
	gD2083RegHistory[gD2083CurHistory].regnum=reg; 
	gD2083RegHistory[gD2083CurHistory].value=data;         
//	gD2083RegHistory[gD2083CurHistory].time= cpu_clock(cpu)/1000; 
	gD2083CurHistory++; 
}
#endif

/*
 *
 */
static int d2083_read(struct d2083 *d2083, u8 reg, int num_regs, u8 * dest)
{
	int bytes = num_regs;

	if (d2083->read_dev == NULL)
		return -ENODEV;

	if ((reg + num_regs - 1) > D2083_MAX_REGISTER_CNT) {
		dev_err(d2083->dev, "invalid reg %x\n", reg + num_regs - 1);
		return -EINVAL;
	}

	/* Actually read it out */
	return d2083->read_dev(d2083, reg, bytes, (char *)dest);
}

/*
 *
 */
static int d2083_write(struct d2083 *d2083, u8 reg, int num_regs, u8 * src)
{
	int bytes = num_regs;

	if (d2083->write_dev == NULL)
		return -ENODEV;

	if ((reg + num_regs - 1) > D2083_MAX_REGISTER_CNT) {
		dev_err(d2083->dev, "invalid reg %x\n",
			reg + num_regs - 1);
		return -EINVAL;
	}
	/* Actually write it out */
	return d2083->write_dev(d2083, reg, bytes, (char *)src);
}

#if 0	// interface for i2c-bsc
static int d2083_read_bsc(struct d2083 *d2083, u8 reg, int num_regs, u8 * dest)
{
	int bytes = num_regs;

	if (d2083->read_dev == NULL)
		return -ENODEV;

	if ((reg + num_regs - 1) > D2083_MAX_REGISTER_CNT) {
		dev_err(d2083->dev, "invalid reg %x\n", reg + num_regs - 1);
		return -EINVAL;
	}

	/* Actually read it out */
	return d2083->read_dev_bsc(d2083, reg, bytes, (char *)dest);
}

/*
 *
 */
static int d2083_write_bsc(struct d2083 *d2083, u8 reg, int num_regs, u8 * src)
{
	int bytes = num_regs;

	if (d2083->write_dev == NULL)
		return -ENODEV;

	if ((reg + num_regs - 1) > D2083_MAX_REGISTER_CNT) {
		dev_err(d2083->dev, "invalid reg %x\n",
			reg + num_regs - 1);
		return -EINVAL;
	}
	/* Actually write it out */
	return d2083->write_dev_bsc(d2083, reg, bytes, (char *)src);
}
#endif

/*
 * d2083_clear_bits - 
 * @ d2083 :
 * @ reg :
 * @ mask :
 *
 */
int d2083_clear_bits(struct d2083 * const d2083, u8 const reg, u8 const mask)
{
	u8 data;
	int err;

	mutex_lock(&io_mutex);
	err = d2083_read(d2083, reg, 1, &data);
	if (err != 0) {
		dev_err(d2083->dev, "read from reg R%d failed\n", reg);
		goto out;
	}
#ifdef D2083_REG_DEBUG   
       else
            d2083_write_reg_history(D2083_HISTORY_READ_OP,reg,data);
#endif
	data &= ~mask;
	err = d2083_write(d2083, reg, 1, &data);
	if (err != 0)
		dlg_err("write to reg R%d failed\n", reg);
#ifdef D2083_REG_DEBUG    
	else  {   
            d2083_write_reg_history(D2083_HISTORY_WRITE_OP,reg,data);
	     d2083_write_reg_cache(reg,data);
       }
#endif    
out:
	mutex_unlock(&io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d2083_clear_bits);

/*
 * d2083_set_var_bits - 
 * @ d2083 :
 * @ reg :
 * @ mask :
 * @ val :
 */
int d2083_set_var_bits(struct d2083 * const d2083, u8 const reg, u8 const mask,  u8 const val)
{
	u8 data;
	int err;

	mutex_lock(&io_mutex);
	err = d2083_read(d2083, reg, 1, &data);
	if (err != 0) {
		dev_err(d2083->dev, "read from reg R%d failed\n", reg);
		goto out;
	}
#ifdef D2083_REG_DEBUG    
      else {
            d2083_write_reg_history(D2083_HISTORY_READ_OP,reg,data);
        }
#endif      
	data &= ~mask;
	data |= val;
	
	err = d2083_write(d2083, reg, 1, &data);
	if (err != 0)
		dlg_err("write to reg R%d failed\n", reg);
#ifdef D2083_REG_DEBUG    
	else  {   
            d2083_write_reg_history(D2083_HISTORY_WRITE_OP,reg,data);
	     d2083_write_reg_cache(reg,data);
       }
#endif    
out:
	mutex_unlock(&io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d2083_set_var_bits);


/*
 * d2083_set_bits - 
 * @ d2083 :
 * @ reg :
 * @ mask :
 *
 */
int d2083_set_bits(struct d2083 * const d2083, u8 const reg, u8 const mask)
{
	u8 data;
	int err;

	mutex_lock(&io_mutex);
	err = d2083_read(d2083, reg, 1, &data);
	if (err != 0) {
		dev_err(d2083->dev, "read from reg R%d failed\n", reg);
		goto out;
	}
#ifdef D2083_REG_DEBUG    
      else {
            d2083_write_reg_history(D2083_HISTORY_READ_OP,reg,data);
        }
#endif      
	data |= mask;
	err = d2083_write(d2083, reg, 1, &data);
	if (err != 0)
		dlg_err("write to reg R%d failed\n", reg);
#ifdef D2083_REG_DEBUG    
	else  {   
            d2083_write_reg_history(D2083_HISTORY_WRITE_OP,reg,data);
	     d2083_write_reg_cache(reg,data);
       }
#endif    
out:
	mutex_unlock(&io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d2083_set_bits);


/*
 * d2083_reg_read - 
 * @ d2083 :
 * @ reg :
 * @ *dest :
 *
 */
int d2083_reg_read(struct d2083 * const d2083, u8 const reg, u8 *dest)
{
	u8 data;
	int err;

	mutex_lock(&io_mutex);
	err = d2083_read(d2083, reg, 1, &data);
	if (err != 0)
		dlg_err("read from reg R%d failed\n", reg);
#ifdef D2083_REG_DEBUG    
       else
            d2083_write_reg_history(D2083_HISTORY_READ_OP,reg,data);
#endif       
	*dest = data;
	mutex_unlock(&io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d2083_reg_read);


/*
 * d2083_reg_write - 
 * @ d2083 :
 * @ reg :
 * @ val :
 *
 */
int d2083_reg_write(struct d2083 * const d2083, u8 const reg, u8 const val)
{
	int ret;
	u8 data = val;


	mutex_lock(&io_mutex);
	ret = d2083_write(d2083, reg, 1, &data);
	if (ret != 0)
		dlg_err("write to reg R%d failed\n", reg);
#ifdef D2083_REG_DEBUG    
	else  {   
    	 d2083_write_reg_history(D2083_HISTORY_WRITE_OP,reg,data);
		 d2083_write_reg_cache(reg,data);
       }
#endif    
	mutex_unlock(&io_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(d2083_reg_write);


/*
 * d2083_block_read - 
 * @ d2083 :
 * @ start_reg :
 * @ regs :
 * @ *dest :
 *
 */
int d2083_block_read(struct d2083 * const d2083, u8 const start_reg, u8 const regs,
		      u8 * const dest)
{
	int err = 0;
#ifdef D2083_REG_DEBUG   
       int i;
#endif	   

	mutex_lock(&io_mutex);
	err = d2083_read(d2083, start_reg, regs, dest);
	if (err != 0)
		dlg_err("block read starting from R%d failed\n", start_reg);
#ifdef D2083_REG_DEBUG    
       else {
            for(i=0; i<regs; i++)
                 d2083_write_reg_history(D2083_HISTORY_WRITE_OP,start_reg+i,*(dest+i));
        }
#endif       
	mutex_unlock(&io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d2083_block_read);


/*
 * d2083_block_write - 
 * @ d2083 :
 * @ start_reg :
 * @ regs :
 * @ *src :
 *
 */
int d2083_block_write(struct d2083 * const d2083, u8 const start_reg, u8 const regs,
		       u8 * const src)
{
	int ret = 0;
#ifdef D2083_REG_DEBUG   
	int i;
#endif

	mutex_lock(&io_mutex);
	ret = d2083_write(d2083, start_reg, regs, src);
	if (ret != 0)
		dlg_err("block write starting at R%d failed\n", start_reg);
#ifdef D2083_REG_DEBUG    
	else {
	      for(i=0; i<regs; i++)
                 d2083_write_reg_cache(start_reg+i,*(src+i));
	}
#endif    
	mutex_unlock(&io_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(d2083_block_write);



/*
 * Register a client device.  This is non-fatal since there is no need to
 * fail the entire device init due to a single platform device failing.
 */
static void d2083_client_dev_register(struct d2083 *d2083,
				       const char *name,
				       struct platform_device **pdev)
{
	int ret;

	*pdev = platform_device_alloc(name, -1);
	if (*pdev == NULL) {
		dev_err(d2083->dev, "Failed to allocate %s\n", name);
		return;
	}

	(*pdev)->dev.parent = d2083->dev;
	platform_set_drvdata(*pdev, d2083);
	ret = platform_device_add(*pdev);
	if (ret != 0) {
		dev_err(d2083->dev, "Failed to register %s: %d\n", name, ret);
		platform_device_put(*pdev);
		*pdev = NULL;
	}
}

/*
static void d2083_worker_init(unsigned int irq)
{

}
*/

/*
 *
 */
static irqreturn_t d2083_system_event_handler(int irq, void *data)
{
	//todo DLG export the event??
	//struct d2083 *d2083 = data;
	return IRQ_HANDLED;	
}

/*
 *
 */
static void d2083_system_event_init(struct d2083 *d2083)
{
	d2083_register_irq(d2083, D2083_IRQ_EVDD_MON, d2083_system_event_handler, 
                            0, "VDD MON", d2083);
}

/*****************************************/
/* 	Debug using proc entry           */
/*****************************************/
#if defined(CONFIG_MACH_RHEA_SS_IVORY) || defined(CONFIG_MACH_RHEA_SS_NEVIS) || defined(CONFIG_MACH_RHEA_SS_NEVIS_REV00) || defined(CONFIG_MACH_RHEA_SS_NEVISP) || defined(CONFIG_MACH_RHEA_SS_CORSICA)|| defined(CONFIG_MACH_RHEA_SS_NEVISDS)
static int d2083_ioctl_open(struct inode *inode, struct file *file)
{
	dlg_info("%s\n", __func__);
	file->private_data = PDE(inode)->data;
	return 0;
}

int d2083_ioctl_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*
 *
 */
static long d2083_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct d2083 *d2083 =  file->private_data;
	pmu_reg reg;
	int ret = 0;
	u8 reg_val, event_reg[4];

	if (!d2083)
		return -ENOTTY;
		
	// _kg TODO: Checking if d2083_reg_write() and d2083_reg_read()
	// return with success. 
	switch (cmd) {
    	case BCM_PMU_IOCTL_ENABLE_INTS:
		ret = d2083_block_read(d2083, D2083_EVENTA_REG, 4, event_reg);
		dlg_info("int register 0x%X = 0x%X\n", D2083_EVENTA_REG, event_reg[0]);
		dlg_info("int register 0x%X = 0x%X\n", D2083_EVENTB_REG, event_reg[1]);
		dlg_info("int register 0x%X = 0x%X\n", D2083_EVENTC_REG, event_reg[2]);
		dlg_info("int register 0x%X = 0x%X\n", D2083_EVENTD_REG, event_reg[3]);

		/* Clear all latched interrupts if any */
		d2083_reg_write(d2083, D2083_EVENTA_REG, 0xFF);
		d2083_reg_write(d2083, D2083_EVENTB_REG, 0xFF);
		d2083_reg_write(d2083, D2083_EVENTC_REG, 0xFF);
		d2083_reg_write(d2083, D2083_EVENTD_REG, 0xFF);

		enable_irq(d2083->chip_irq);
		break;

    	case BCM_PMU_IOCTL_DISABLE_INTS:
		disable_irq_nosync(d2083->chip_irq);
		break;

    	case BCM_PMU_IOCTL_READ_REG:
		if (copy_from_user(&reg, (pmu_reg *)arg, sizeof(pmu_reg)) != 0)
			return -EFAULT;
		// DLG eric. 03/Nov/2011. Change prototype
		//reg.val = d2083_reg_read(d2083, reg.reg);
		// TODO: Check parameter. &reg.val
		ret = d2083_reg_read(d2083, reg.reg, &reg_val);
		reg.val = (unsigned short)reg_val;
		if (copy_to_user((pmu_reg *)arg, &reg, sizeof(pmu_reg)) != 0)
			return -EFAULT;
		break;

	case BCM_PMU_IOCTL_WRITE_REG:
		if (copy_from_user(&reg, (pmu_reg *)arg, sizeof(pmu_reg)) != 0)
			return -EFAULT;
		d2083_reg_write(d2083, reg.reg, (u8)reg.val);
		break;

#if 0
	case BCM_PMU_IOCTL_SET_VOLTAGE:
	case BCM_PMU_IOCTL_GET_VOLTAGE:
	case BCM_PMU_IOCTL_GET_REGULATOR_STATE:
	case BCM_PMU_IOCTL_SET_REGULATOR_STATE:
	case BCM_PMU_IOCTL_ACTIVATESIM:
	case BCM_PMU_IOCTL_DEACTIVATESIM:
		ret = d2083_ioctl_regulator(d2083, cmd, arg);
		break;
#endif
	case BCM_PMU_IOCTL_POWERONOFF:
		//    d2083_set_bits(d2083,D2083_POWERCONT_REG,D2083_POWERCONT_RTCAUTOEN);
		d2083_shutdown(d2083);
		break;

	default:
		dlg_err("%s: unsupported cmd\n", __func__);
		ret = -ENOTTY;
	}

	return ret;
}

#define MAX_USER_INPUT_LEN      100
#define MAX_REGS_READ_WRITE     10

enum pmu_debug_ops {
	PMUDBG_READ_REG = 0UL,
	PMUDBG_WRITE_REG,
};

struct pmu_debug {
	int read_write;
	int len;
	int addr;
	u8 val[MAX_REGS_READ_WRITE];
};

/*
 *
 */
static void d2083_dbg_usage(void)
{
	printk(KERN_INFO "Usage:\n");
	printk(KERN_INFO "Read a register: echo 0x0800 > /proc/pmu0\n");
	printk(KERN_INFO
		"Read multiple regs: echo 0x0800 -c 10 > /proc/pmu0\n");
	printk(KERN_INFO
		"Write multiple regs: echo 0x0800 0xFF 0xFF > /proc/pmu0\n");
	printk(KERN_INFO
		"Write single reg: echo 0x0800 0xFF > /proc/pmu0\n");
	printk(KERN_INFO "Max number of regs in single write is :%d\n",
		MAX_REGS_READ_WRITE);
	printk(KERN_INFO "Register address is encoded as follows:\n");
	printk(KERN_INFO "0xSSRR, SS: i2c slave addr, RR: register addr\n");
}


/*
 *
 */
static int d2083_dbg_parse_args(char *cmd, struct pmu_debug *dbg)
{
	char *tok;                 /* used to separate tokens             */
	const char ct[] = " \t";   /* space or tab delimits the tokens    */
	bool count_flag = false;   /* whether -c option is present or not */
	int tok_count = 0;         /* total number of tokens parsed       */
	int i = 0;

	dbg->len        = 0;

	/* parse the input string */
	while ((tok = strsep(&cmd, ct)) != NULL) {
		dlg_info("token: %s\n", tok);

		/* first token is always address */
		if (tok_count == 0) {
			sscanf(tok, "%x", &dbg->addr);
		} else if (strnicmp(tok, "-c", 2) == 0) {
			/* the next token will be number of regs to read */
			tok = strsep(&cmd, ct);
			if (tok == NULL)
				return -EINVAL;

			tok_count++;
			sscanf(tok, "%d", &dbg->len);
			count_flag = true;
			break;
		} else {
			int val;

			/* this is a value to be written to the pmu register */
			sscanf(tok, "%x", &val);
			if (i < MAX_REGS_READ_WRITE) {
				dbg->val[i] = val;
				i++;
			}
		}

		tok_count++;
	}

	/* decide whether it is a read or write operation based on the
	 * value of tok_count and count_flag.
	 * tok_count = 0: no inputs, invalid case.
	 * tok_count = 1: only reg address is given, so do a read.
	 * tok_count > 1, count_flag = false: reg address and atleast one
	 *     value is present, so do a write operation.
	 * tok_count > 1, count_flag = true: to a multiple reg read operation.
	 */
	switch (tok_count) {
	case 0:
		return -EINVAL;
	case 1:
		dbg->read_write = PMUDBG_READ_REG;
		dbg->len = 1;
		break;
	default:
		if (count_flag == true) {
			dbg->read_write = PMUDBG_READ_REG;
		} else {
			dbg->read_write = PMUDBG_WRITE_REG;
			dbg->len = i;
		}
	}

	return 0;
}

/*
 *
 */
static ssize_t d2083_ioctl_write(struct file *file, const char __user *buffer,
	size_t len, loff_t *offset)
{
	struct d2083 *d2083 = file->private_data;
	struct pmu_debug dbg;
	char cmd[MAX_USER_INPUT_LEN];
	int ret, i;

	dlg_info("%s\n", __func__);

	if (!d2083) {
		dlg_err("%s: driver not initialized\n", __func__);
		return -EINVAL;
	}

	if (len > MAX_USER_INPUT_LEN)
		len = MAX_USER_INPUT_LEN;

	if (copy_from_user(cmd, buffer, len)) {
		dlg_err("%s: copy_from_user failed\n", __func__);
		return -EFAULT;
	}

	/* chop of '\n' introduced by echo at the end of the input */
	if (cmd[len - 1] == '\n')
		cmd[len - 1] = '\0';

	if (d2083_dbg_parse_args(cmd, &dbg) < 0) {
		d2083_dbg_usage();
		return -EINVAL;
	}

	dlg_info("operation: %s\n", (dbg.read_write == PMUDBG_READ_REG) ?
		"read" : "write");
	dlg_info("address  : 0x%x\n", dbg.addr);
	dlg_info("length   : %d\n", dbg.len);

	if (dbg.read_write == PMUDBG_READ_REG) {
		ret = d2083_read(d2083, dbg.addr, dbg.len, dbg.val);
		if (ret < 0) {
			dlg_err("%s: pmu reg read failed\n", __func__);
			return -EFAULT;
		}

		for (i = 0; i < dbg.len; i++, dbg.addr++)
			dlg_info("[%x] = 0x%02x\n", dbg.addr,
				dbg.val[i]);
	} else {
		ret = d2083_write(d2083, dbg.addr, dbg.len, dbg.val);
		if (ret < 0) {
			dlg_err("%s: pmu reg write failed\n", __func__);
			return -EFAULT;
		}
	}

	*offset += len;

	return len;
}

static const struct file_operations d2083_pmu_ops = {
	.open = d2083_ioctl_open,
	.unlocked_ioctl = d2083_ioctl,
	.write = d2083_ioctl_write,
	.release = d2083_ioctl_release,
	.owner = THIS_MODULE,
};

void d2083_debug_proc_init(struct d2083 *d2083)
{
	struct proc_dir_entry *entry;

	disable_irq(d2083->chip_irq);
	entry = proc_create_data("pmu0", S_IRWXUGO, NULL, &d2083_pmu_ops, d2083);
	enable_irq(d2083->chip_irq);
	dlg_crit("\nD2083-core.c: proc_create_data() = %p; name=\"%s\"\n", entry, (entry?entry->name:""));
}

void d2083_debug_proc_exit(void)
{
	//disable_irq(client->irq);
	remove_proc_entry("pmu0", NULL);
	//enable_irq(client->irq);
}
#endif /* CONFIG_MACH_RHEA_SS_IVORY */

struct d2083 *d2083_regl_info=NULL;
EXPORT_SYMBOL(d2083_regl_info);


int d2083_device_init(struct d2083 *d2083, int irq,
		       struct d2083_platform_data *pdata)
{
	u8 res_msb, res_lsb;
	u16 read_adc;
	int ret = 0;//, tmp;
	//struct regulator *regulator;
#ifdef D2083_REG_DEBUG 
	int i;
	u8 data;
#endif

	if(d2083 != NULL)
		d2083_regl_info = d2083;
	else
		goto err;

	dlg_info("D2083 Driver version : %s\n", D2083_VERSION);

	d2083->pmic.max_dcdc = 25; //
	d2083->pdata = pdata;

#if defined(CONFIG_KONA_PMU_BSC_HS_MODE) || defined(CONFIG_KONA_PMU_BSC_HS_1625KHZ)
    d2083_set_bits(d2083, D2083_CONTROLB_REG, D2083_CONTROLB_I2C_SPEED);

	/* Page write for I2C we donot support repeated write and I2C speed set to 1.7MHz */
	d2083_clear_bits(d2083, D2083_CONTROLB_REG, D2083_CONTROLB_WRITEMODE);
#else
	/* Page write for I2C we donot support repeated write and I2C speed set to 400KHz */
	d2083_clear_bits(d2083, D2083_CONTROLB_REG, D2083_CONTROLB_WRITEMODE | D2083_CONTROLB_I2C_SPEED);
#endif

	d2083_clear_bits(d2083,D2083_POWERCONT_REG,(0x1 << 2)); // Clear PULSED_EN bit

	msleep(1);
	d2083_reg_write(d2083, D2083_ADC_CONT_REG, (D2083_ADCCONT_ADC_AUTO_EN 
												| D2083_ADCCONT_ADC_MODE 
												| D2083_ADCCONT_AUTO_VBAT_EN));
	msleep(1);
	d2083_reg_read(d2083, D2083_VBAT_RES_REG, &res_msb);
	d2083_reg_read(d2083, D2083_ADC_RES_AUTO1_REG, &res_lsb);
	read_adc = (((res_msb & 0xFF) << 4) | (res_lsb & 0x0F));
	d2083->vbat_init_adc[0] = read_adc;
	pr_info(">>>>>>>>>>>> [L%04d] %s. READ VBAT ADC is %d\n", __LINE__, __func__, d2083->vbat_init_adc[0]);
	d2083_reg_write(d2083, D2083_ADC_CONT_REG, 0x0);

	d2083_reg_write(d2083, D2083_BUCKA_REG,0x9A);
	d2083_reg_write(d2083, D2083_BUCKB_REG,0x9A);	// BUCK3	: forced PWM
	

#if 1	// 20120221 LDO13 issue
    d2083_reg_write(d2083, D2083_PDDIS_REG,0x0);
    d2083_reg_write(d2083, D2083_PULLDOWN_REG_D,0x0);
    // audio
    d2083_reg_write(d2083, D2083_PREAMP_A_CTRL1_REG,0x34);
    d2083_reg_write(d2083, D2083_PREAMP_A_CTRL2_REG,0x0);
    d2083_reg_write(d2083, D2083_SP_CTRL_REG,0xCC);
    
    // LDO 
	d2083_set_var_bits(d2083, D2083_LDO1_REG, D2083_LDO_VOL_MASK, 0x00); //LDO 1 1.2V	// spare
	d2083_set_var_bits(d2083, D2083_LDO3_REG, D2083_LDO_VOL_MASK, 0x24); //LDO 3 3.0V	// VDD_SENSOR_3.0V
	d2083_set_var_bits(d2083, D2083_LDO6_REG, D2083_LDO_VOL_MASK, 0x20); //LDO 6 2.8V	// VCAM_A_2.8V
	d2083_set_var_bits(d2083, D2083_LDO7_REG, D2083_LDO_VOL_MASK, 0x2A); //LDO 7 3.3V	// VDD_VIB_3.3V
	d2083_set_var_bits(d2083, D2083_LDO8_REG, D2083_LDO_VOL_MASK, 0x24); //LDO 8 3.0V	// VLCD_3.0V
	d2083_set_var_bits(d2083, D2083_LDO9_REG, D2083_LDO_VOL_MASK, 0x24); //LDO 9 3.0V	// VDD_SDXC

	d2083_set_var_bits(d2083, D2083_LDO11_REG, D2083_LDO_VOL_MASK, 0x24); //LDO 11 3.0V	// VSIM1_3.0V
	d2083_set_var_bits(d2083, D2083_LDO13_REG, D2083_LDO_VOL_MASK, 0x24); //LDO 13 3.0V	// VDD_SDIO_3.0V
	d2083_set_var_bits(d2083, D2083_LDO14_REG, D2083_LDO_VOL_MASK, 0x0C); //LDO 14 1.8V	// VTOUCH_1.8V
	d2083_set_var_bits(d2083, D2083_LDO15_REG, D2083_LDO_VOL_MASK, 0x2A); //LDO 15 3.3V	// VTOUCH_3.3V
	d2083_set_var_bits(d2083, D2083_LDO16_REG, D2083_LDO_VOL_MASK, 0x0C); //LDO 16 1.8V	// VCAMC_IO_1.8V
	d2083_set_var_bits(d2083, D2083_LDO17_REG, D2083_LDO_VOL_MASK, 0x20); //LDO 17 2.8V	// VCAM_AF_2.8V
	// BUCK
	d2083_set_var_bits(d2083, D2083_BUCK4_REG, D2083_BUCK4_VOL_MASK, 0x54); //BUCK 4 3.3V	// VDD_3G_PAM_3.3V
#endif

    d2083_reg_write(d2083,D2083_BBATCONT_REG,0x1F);
    d2083_set_bits(d2083,D2083_SUPPLY_REG,D2083_SUPPLY_BBCHGEN);

	/* DLG todo */
	if (pdata && pdata->irq_init) {
		dlg_crit("\nD2083-core.c: IRQ PIN Configuration \n");
		ret = pdata->irq_init(d2083);
		if (ret != 0) {
			dev_err(d2083->dev, "Platform init() failed: %d\n", ret);
			goto err_irq;
		}
	}

	d2083_dev_info = d2083;
	pm_power_off = d2083_system_poweroff;

	ret = d2083_irq_init(d2083, irq, pdata);
	if (ret < 0)
		goto err;
    
	//DLG todo d2083_worker_init(irq); //new for Samsung
    
	if (pdata && pdata->init) {
		ret = pdata->init(d2083);
		if (ret != 0) {
			dev_err(d2083->dev, "Platform init() failed: %d\n", ret);
			goto err_irq;
		}
	}
	
	///////////////////////////////////
	d2083_reg_write(d2083, D2083_ADC_CONT_REG, (D2083_ADCCONT_ADC_AUTO_EN 
												| D2083_ADCCONT_ADC_MODE 
												| D2083_ADCCONT_AUTO_VBAT_EN));
	msleep(1);
	d2083_reg_read(d2083, D2083_VBAT_RES_REG, &res_msb);
	d2083_reg_read(d2083, D2083_ADC_RES_AUTO1_REG, &res_lsb);
	read_adc = (((res_msb & 0xFF) << 4) | (res_lsb & 0x0F));
	d2083->vbat_init_adc[1] = read_adc;
	pr_info(">>>>>>>>>>>> [L%04d] %s. READ VBAT ADC is %d\n", __LINE__, __func__, d2083->vbat_init_adc[1]);
	d2083_reg_write(d2083, D2083_ADC_CONT_REG, 0x0);
	///////////////////////////////////

	// Regulator Specific Init
	ret = d2083_platform_regulator_init(d2083);
	if (ret != 0) {
		dev_err(d2083->dev, "Platform Regulator init() failed: %d\n", ret);
		goto err_irq;
	}

	///////////////////////////////////
	d2083_reg_write(d2083, D2083_ADC_CONT_REG, (D2083_ADCCONT_ADC_AUTO_EN 
												| D2083_ADCCONT_ADC_MODE 
												| D2083_ADCCONT_AUTO_VBAT_EN));
	msleep(1);
	d2083_reg_read(d2083, D2083_VBAT_RES_REG, &res_msb);
	d2083_reg_read(d2083, D2083_ADC_RES_AUTO1_REG, &res_lsb);
	read_adc = (((res_msb & 0xFF) << 4) | (res_lsb & 0x0F));
	d2083->vbat_init_adc[2] = read_adc;
	pr_info(">>>>>>>>>>>> [L%04d] %s. READ VBAT ADC is %d\n", __LINE__, __func__, d2083->vbat_init_adc[2]);
	d2083_reg_write(d2083, D2083_ADC_CONT_REG, 0x0);
	///////////////////////////////////

	d2083_client_dev_register(d2083, "d2083-battery", &(d2083->batt.pdev));
	d2083_client_dev_register(d2083, "d2083-rtc", &(d2083->rtc.pdev));
	d2083_client_dev_register(d2083, "d2083-onkey", &(d2083->onkey.pdev));
	d2083_client_dev_register(d2083, "d2083-audio", &(d2083->audio.pdev));

	d2083_system_event_init(d2083);
#if defined(CONFIG_MACH_RHEA_SS_IVORY) || defined(CONFIG_MACH_RHEA_SS_NEVIS) || defined(CONFIG_MACH_RHEA_SS_NEVIS_REV00) || defined(CONFIG_MACH_RHEA_SS_NEVISP) || defined(CONFIG_MACH_RHEA_SS_CORSICA)|| defined(CONFIG_MACH_RHEA_SS_NEVISDS)
	d2083_debug_proc_init(d2083);
#endif /* CONFIG_MACH_RHEA_SS_IVORY */

#ifdef D2083_REG_DEBUG    
	  for(i=0; i<D2083_MAX_REGISTER_CNT; i++)
	  {
	  	d2083_reg_read(d2083, i, &data);
		d2083_write_reg_cache(i,data);
	  }
#endif   

    // temporary code
	if (pdata->pmu_event_cb)
		pdata->pmu_event_cb(0, 0);		//PMU_EVENT_INIT_PLATFORM

	// set MCTRL_EN enabled
	set_MCTL_enabled();

	return 0;

err_irq:
	d2083_irq_exit(d2083);
	d2083_dev_info = NULL;
	pm_power_off = NULL;
err:
	dlg_crit("\n\nD2083-core.c: device init failed ! \n\n");
	return ret;
}
EXPORT_SYMBOL_GPL(d2083_device_init);

void d2083_device_exit(struct d2083 *d2083)
{
#if defined(CONFIG_MACH_RHEA_SS_IVORY) || defined(CONFIG_MACH_RHEA_SS_NEVIS) || defined(CONFIG_MACH_RHEA_SS_NEVIS_REV00) || defined(CONFIG_MACH_RHEA_SS_NEVISP) || defined(CONFIG_MACH_RHEA_SS_CORSICA)|| defined(CONFIG_MACH_RHEA_SS_NEVISDS)
	d2083_debug_proc_exit();
#endif /* CONFIG_MACH_RHEA_SS_IVORY */
	d2083_dev_info = NULL;

	platform_device_unregister(d2083->rtc.pdev);
	platform_device_unregister(d2083->onkey.pdev);
	platform_device_unregister(d2083->audio.pdev);
	platform_device_unregister(d2083->batt.pdev);

	d2083_free_irq(d2083, D2083_IRQ_EVDD_MON);
	d2083_irq_exit(d2083);
}
EXPORT_SYMBOL_GPL(d2083_device_exit);

extern int d2083_get_last_vbat_adc(void);
extern int d2083_get_last_capacity(void);

int d2083_shutdown(struct d2083 *d2083)
{
	u8 dst;
	u32 capacity, last_adc;

	dlg_info("%s\n", __func__);

	if (d2083->read_dev == NULL)
		return -ENODEV;

	d2083_reg_write(d2083, D2083_BUCK1_REG, 0x63);

	capacity = d2083_get_last_capacity();
	d2083_reg_write(d2083, D2083_GPID3_REG, (0xFF & capacity));
	d2083_reg_write(d2083, D2083_GPID4_REG, (0x0F & (capacity>>8)));

	last_adc = d2083_get_last_vbat_adc();
	d2083_reg_write(d2083, D2083_GPID5_REG, (0xFF&last_adc)); //8 LSB
	d2083_reg_write(d2083, D2083_GPID6_REG, (0xF&(last_adc>>8))); // 4 MSB
	
	d2083_clear_bits(d2083,D2083_CONTROLB_REG,D2083_CONTROLB_OTPREADEN); //otp reload disable
	d2083_clear_bits(d2083,D2083_POWERCONT_REG,D2083_POWERCONT_MCTRLEN); //mctl disable

	d2083_reg_write(d2083, D2083_BUCK1_REG, 0x54); //buck1 1.0 V 0x14

	// 20120221 Go deepsleep : Manual LDO off
	d2083_reg_write(d2083, D2083_IRQMASKB_REG, 0x0);	//onkey mask clear


	// buck4, ldo1, ldo3, ldo6, 7, 8, 9, 11, 13, 14, 15, 16, 17
	d2083_clear_bits(d2083, D2083_LDO1_REG, (1<<6)); //LDO 1 disable
	d2083_clear_bits(d2083, D2083_LDO3_REG, (1<<6)); //LDO 3 disable
	d2083_clear_bits(d2083, D2083_LDO6_REG, (1<<6)); //LDO 6 disable
	d2083_clear_bits(d2083, D2083_LDO7_REG, (1<<6)); //LDO 7 disable
	d2083_clear_bits(d2083, D2083_LDO8_REG, (1<<6)); //LDO 8 disable
	d2083_clear_bits(d2083, D2083_LDO9_REG, (1<<6)); //LDO 9 disable

	d2083_clear_bits(d2083, D2083_LDO11_REG, (1<<6)); //LDO 11 disable
	d2083_clear_bits(d2083, D2083_LDO13_REG, (1<<6)); //LDO 13 disable
	d2083_clear_bits(d2083, D2083_LDO14_REG, (1<<6)); //LDO 14 disable
	d2083_clear_bits(d2083, D2083_LDO15_REG, (1<<6)); //LDO 15 disable
	d2083_clear_bits(d2083, D2083_LDO16_REG, (1<<6)); //LDO 16 disable
	d2083_clear_bits(d2083, D2083_LDO17_REG, (1<<6)); //LDO 17 disable
	d2083_clear_bits(d2083, D2083_LDO_AUD_REG, (1<<6)); //LDO_AUD disable

	d2083_reg_write(d2083, D2083_BUCK4_REG, 0x0);	//BUCK 4
	d2083_reg_write(d2083, D2083_SUPPLY_REG, 0x10);
	d2083_reg_write(d2083, D2083_POWERCONT_REG, 0x0E);
	d2083_reg_write(d2083, D2083_PDDIS_REG, 0xCF);

	d2083_reg_read(d2083, D2083_CONTROLB_REG, &dst);
	dst |= D2083_CONTROLB_DEEPSLEEP;
	d2083_reg_write(d2083, D2083_CONTROLB_REG, dst);

	return 0;
}
EXPORT_SYMBOL(d2083_shutdown);


/* D2083 poweroff function */
void d2083_system_poweroff(void)
{
	dlg_info("%s\n", __func__);
	
	if(d2083_dev_info) {
		d2083_shutdown(d2083_dev_info);
	}
}
EXPORT_SYMBOL(d2083_system_poweroff);

MODULE_AUTHOR("Dialog Semiconductor Ltd < william.seo@diasemi.com >");
MODULE_DESCRIPTION("D2083 PMIC Core");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" D2083_I2C);
