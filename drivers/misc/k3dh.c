/*
 *  STMicroelectronics k3dh acceleration sensor driver
 *
 *  Copyright (C) 2010 Samsung Electronics Co.Ltd
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>

#include <linux/k3dh.h>
#include <linux/k3dh_dev.h> 
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#define K3DH_DEBUG 0

#if K3DH_DEBUG
#define ACCDBG(fmt, args...) printk(KERN_INFO fmt, ## args)
#else
#define ACCDBG(fmt, args...)
#endif

/* The default settings when sensor is on is for all 3 axis to be enabled
 * and output data rate set to 400Hz.  Output is via a ioctl read call.
 */
#define DEFAULT_POWER_ON_SETTING (ODR400 | ENABLE_ALL_AXES)
#define DEFAULT_POWER_ON_SETTING_1344HZ (ODR1344 | ENABLE_ALL_AXES)
#define ACC_DEV_NAME "accelerometer"
#define ACC_DEV_MAJOR 241
#define K3DH_RETRY_COUNT	3

#define CALIBRATION_FILE_PATH	"/efs/calibration_data"
#define CALIBRATION_DATA_AMOUNT	20

#define ACC_ENABLED 1

#if defined(CONFIG_SENSORS_ACC_FILTER)
 // coefficients for filter
#define ORDER   2
#define COEFFA0                      ( 100000)
#define COEFFA1                      (-153840)
#define COEFFA2                      ( 62650) 

#define COEFFB0                      (   2200)
#define COEFFB1                      (   4410)
#define COEFFB2                      (   2200)

#define IN_COEFF_NUM	(ORDER + 1)
#define OUT_COEFF_NUM	(ORDER + 1)
#define INBUFSIZE 		(2 * (IN_COEFF_NUM))
#define OUTBUFSIZE		(2 * (OUT_COEFF_NUM))
#endif

static const struct odr_delay {
	u8 odr; /* odr reg setting */
	s64 delay_ns; /* odr in ns */
} odr_delay_table[] = {
	{ ODR1344,     744047LL }, /* 1344Hz */
	{  ODR400,    2500000LL }, /*  400Hz */
	{  ODR200,    5000000LL }, /*  200Hz */
	{  ODR100,   10000000LL }, /*  100Hz */
	{   ODR50,   20000000LL }, /*   50Hz */
	{   ODR25,   40000000LL }, /*   25Hz */
	{   ODR10,  100000000LL }, /*   10Hz */
	{    ODR1, 1000000000LL }, /*    1Hz */
};

/* K3DH acceleration data */
struct k3dh_acc {
	s16 x;
	s16 y;
	s16 z;
};

struct k3dh_data {
	struct i2c_client *client;
	struct miscdevice k3dh_device;
	struct input_dev *acc_input_dev;    
	struct mutex read_lock;
	struct mutex write_lock;
    	struct mutex power_lock;
	struct k3dh_acc cal_data;
	u8 state;    
	u8 ctrl_reg1_shadow;
	atomic_t opened; /* opened implies enabled */
        s8 orientation[9];

#if defined(CONFIG_SENSORS_ACC_FILTER)
	struct work_struct read_work; 
	struct workqueue_struct *read_workqueue;
	struct hrtimer timer;
	ktime_t device_polling_delay;
    	unsigned int hrtimer_delay_usec;
        
	int xyz[3];
	bool reset_filter;
	bool enabled_filter;
#endif    
};

#if defined(CONFIG_SENSORS_CORE)
extern struct class *sensors_class;
#endif
static struct k3dh_data * g_k3dh;
static struct k3dh_acc g_acc;
static struct i2c_client *k3dh_client = NULL;
static int acc_mode_cnt = 0;
static struct regulator *VSENSOR_3_0_V;  //HV8

static int k3dh_acc_i2c_read(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msgs[] = {
		{
			.addr	= k3dh_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= k3dh_client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	for (i = 0; i < K3DH_RETRY_COUNT; i++) {
		if (i2c_transfer(k3dh_client->adapter, msgs, 2) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= K3DH_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, K3DH_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int k3dh_acc_i2c_write(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msg[] = {
		{
			.addr	= k3dh_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};
	
	for (i = 0; i < K3DH_RETRY_COUNT; i++) {
		if (i2c_transfer(k3dh_client->adapter, msg, 1) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= K3DH_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, K3DH_RETRY_COUNT);
		return -EIO;
	}
	return 0;
}


#if defined(CONFIG_SENSORS_ACC_FILTER)
static void k3dh_acc_filter(struct k3dh_acc *acc)
{
        int axis, i;// j, modout;
        s32 mean_avg = 0;
        s32 auto_regr = 0;
        s32 temp = 0;
        int posin, posout;

        static const s32 a0 = COEFFA0;
        static const s32 a1 = COEFFA1;
        static const s32 a2 = COEFFA2;

        static const s32 b0 = COEFFB0;
        static const s32 b1 = COEFFB1;
        static const s32 b2 = COEFFB2;

        static s32 in[3][INBUFSIZE];
        static s32 out[3][OUTBUFSIZE];
        static int currin = INBUFSIZE - 1;
        static int currout = OUTBUFSIZE - 1;

        g_k3dh->xyz[0] = acc->x;
        g_k3dh->xyz[1] = acc->y;
        g_k3dh->xyz[2] = acc->z;        

	if (g_k3dh->reset_filter)
	{
		for(axis = 0; axis < 3; axis++)
		{
			for(i = 0; i < INBUFSIZE; i++)
			{
				in[axis][i] = g_k3dh->xyz[axis];
			}
			for(i = 0; i < OUTBUFSIZE; i++)
			{
				out[axis][i] = g_k3dh->xyz[axis];
			}

			currin = INBUFSIZE - 1;
			currout = OUTBUFSIZE - 1;
		}
		g_k3dh->reset_filter = false;
	}

	for (axis = 2; axis >= 0; axis--)
	{
		posin = currin;
		posout = currout ;

		in[axis][currin] = g_k3dh->xyz[axis];

		//MA section
		mean_avg += b0 * in[axis][posin];
		posin--;
		if (posin < 0) posin = INBUFSIZE - 1;
		mean_avg += b1 * in[axis][posin];
		posin--;
		if (posin < 0) posin = INBUFSIZE - 1;
		mean_avg += b2 * in[axis][posin];

		posout--;
		if (posout < 0) posout = OUTBUFSIZE - 1;
		auto_regr += - a1 * out[axis][posout];
		posout--;
		if (posout < 0) posout = OUTBUFSIZE - 1;
		auto_regr += - a2 * out[axis][posout];

		out[axis][currout] = mean_avg + auto_regr; //MA sec + AR section
		temp = out[axis][currout] / a0;

		out[axis][currout] = temp;
		g_k3dh->xyz[axis] = out[axis][currout];

		mean_avg = 0;
		auto_regr = 0;
	}

        acc->x = g_k3dh->xyz[0];
        acc->y = g_k3dh->xyz[1];
        acc->z = g_k3dh->xyz[2];       

	currin++;
	if (currin == INBUFSIZE)
		currin = 0;
	currout++;
	if (currout == OUTBUFSIZE)
		currout = 0;

}
#endif

/* Read X,Y and Z-axis acceleration raw data */
static int k3dh_read_accel_raw_xyz(struct k3dh_acc *acc)
{
	int err;
	unsigned char acc_data[6] = {0};
	s16 temp;
    
	acc_data[0] = OUT_X_L | AC; /* read from OUT_X_L to OUT_Z_H by auto-inc */
	err = k3dh_acc_i2c_read(acc_data, 6);
	if (err < 0)
	{
		pr_err("k3dh_read_accel_raw_xyz() failed\n");        
		return err;    
	}

	acc->x = (acc_data[1] << 8) | acc_data[0];
	acc->y = (acc_data[3] << 8) | acc_data[2];
	acc->z = (acc_data[5] << 8) | acc_data[4];

#if defined(CONFIG_SENSORS_ACC_12BIT)
	acc->x = acc->x >> 4;
	acc->y = acc->y >> 4;
	acc->z = acc->z >> 4;
#else
	acc->x = acc->x >> 8;
	acc->y = acc->y >> 8;
	acc->z = acc->z >> 8;
#endif

#if defined(CONFIG_SENSORS_ACC_FILTER)
	if (g_k3dh->enabled_filter){
		k3dh_acc_filter(acc);
	}
#endif

	if (g_k3dh->orientation[0]) {
		acc->x *= g_k3dh->orientation[0];
		acc->y *= g_k3dh->orientation[4];
	} else {
		temp = acc->x*g_k3dh->orientation[1];
		acc->x = acc->y*g_k3dh->orientation[3];
		acc->y = temp;
	}
	acc->z *= g_k3dh->orientation[8];

	return 0;
}

static int k3dh_read_accel_xyz(struct k3dh_acc *acc)
{
	int err = 0;

	err = k3dh_read_accel_raw_xyz(acc);
	if (err < 0) {
		pr_err("k3dh_read_accel_xyz() failed\n");
		return err;
	}

	acc->x -= g_k3dh->cal_data.x;
	acc->y -= g_k3dh->cal_data.y;
	acc->z -= g_k3dh->cal_data.z;

        g_acc.x = acc->x;
        g_acc.y = acc->y;
        g_acc.z = acc->z;

	return err;
}

#if defined(CONFIG_SENSORS_HSCDTD006A) || defined(CONFIG_SENSORS_HSCDTD008A) 
static atomic_t flgEna;
static atomic_t delay;

int accsns_get_acceleration_data(int *xyz)
{    
	struct k3dh_acc acc;
	int err = -1;

        err = k3dh_read_accel_xyz(&acc);

#if defined(CONFIG_SENSORS_ACC_12BIT)
        xyz[0] = (int)(acc.x);
        xyz[1] = (int)(acc.y);
        xyz[2] = (int)(acc.z);               
#else
        xyz[0] = ((int)(acc.x)*4);   // raw * 4
        xyz[1] = ((int)(acc.y)*4);
        xyz[2] = ((int)(acc.z)*4);
#endif        

	ACCDBG("[K3DH] Acc_I2C, x:%d, y:%d, z:%d\n", xyz[0], xyz[1], xyz[2]);

	return err;
}

void accsns_activate(int flgatm, int flg, int dtime)
{
    unsigned char acc_data[2] = {0};
        
    printk(KERN_INFO "[K3DH] accsns_activate : flgatm=%d, flg=%d, dtime=%d\n", flgatm, flg, dtime);

    if (flg != 0) flg = 1;

    //Power modes
    if (flg == 0) //sleep
    {
            acc_data[0] = CTRL_REG1;
            acc_data[1] = PM_OFF;
            if(k3dh_acc_i2c_write(acc_data, 2) !=0)
                printk(KERN_ERR "[%s] Change to Suspend Mode is failed\n",__FUNCTION__);  
            
            g_k3dh->state = 0;
            
#if defined(CONFIG_SENSORS_ACC_FILTER)
            hrtimer_cancel(&g_k3dh->timer);
            cancel_work_sync(&g_k3dh->read_work);
#endif
            acc_mode_cnt=0;
    }
    else 
    {
            acc_data[0] = CTRL_REG1;
#if defined(CONFIG_SENSORS_ACC_FILTER)
            acc_data[1] = DEFAULT_POWER_ON_SETTING_1344HZ;
#else
            acc_data[1] = DEFAULT_POWER_ON_SETTING;
#endif
            if(k3dh_acc_i2c_write(acc_data, 2) !=0)
                printk(KERN_ERR "[%s] Change to Normal Mode(CTRL_REG1) is failed\n",__FUNCTION__);  

#if defined(CONFIG_SENSORS_ACC_12BIT)
            acc_data[0] = CTRL_REG4;
            acc_data[1] = CTRL_REG4_HR;
            if(k3dh_acc_i2c_write(acc_data, 2) !=0)
                printk(KERN_ERR "[%s] Change to Normal Mode(CTRL_REG4) is failed\n",__FUNCTION__);  
#endif

            g_k3dh->state |= ACC_ENABLED;

#if defined(CONFIG_SENSORS_ACC_FILTER)
            g_k3dh->reset_filter = true;
            hrtimer_start(&g_k3dh->timer, g_k3dh->device_polling_delay, HRTIMER_MODE_REL);
#endif

            if(!acc_mode_cnt)
            {
                printk(KERN_INFO "[K3DH] accsns_activate : (%d,%d,%d)\n", g_k3dh->cal_data.x, g_k3dh->cal_data.y, g_k3dh->cal_data.z);
                acc_mode_cnt++;
            }
    }

    mdelay(2);
    
    if (flgatm) {
        atomic_set(&flgEna, flg);
        atomic_set(&delay, dtime);
    }
}
EXPORT_SYMBOL(accsns_get_acceleration_data);
EXPORT_SYMBOL(accsns_activate);
#endif


#if defined(CONFIG_SENSORS_ACC_FILTER)
static void k3dh_acc_read_work_func(struct work_struct *work)
{
	struct k3dh_acc acc;
	int err = -1;
    	mutex_lock(&g_k3dh->read_lock);
        err = k3dh_read_accel_xyz(&acc);
	hrtimer_start(&g_k3dh->timer, g_k3dh->device_polling_delay, HRTIMER_MODE_REL);
	mutex_unlock(&g_k3dh->read_lock);
}

static enum hrtimer_restart k3dh_acc_timer_func(struct hrtimer *timer)
{
	queue_work(g_k3dh->read_workqueue, &g_k3dh->read_work);
	return HRTIMER_NORESTART;
}
#endif

static int k3dh_open_calibration(void)
{
	struct file *cal_filp = NULL;
	int err = 0, ret = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0660);
	if (IS_ERR(cal_filp)) {
            printk(KERN_INFO "[K3DH] %s: no calibration file\n", __func__);        
            err = PTR_ERR(cal_filp);
            if (err != -ENOENT)
                pr_err("%s: Can't open calibration file= %d\n", __func__, err);
            set_fs(old_fs);
            return err;
	}

        if (cal_filp && cal_filp->f_op && cal_filp->f_op->read){

            ret = cal_filp->f_op->read(cal_filp, (char *)(&g_k3dh->cal_data), 3 * sizeof(s16), &cal_filp->f_pos);
            if (ret != 3 * sizeof(s16)) {
                pr_err("%s: Can't read the cal data from file= %d\n", __func__, ret);
                err = -EIO;
            }

            printk(KERN_INFO "%s: (%d,%d,%d)\n", __func__, g_k3dh->cal_data.x, g_k3dh->cal_data.y, g_k3dh->cal_data.z);

        }

        if(cal_filp) 
            filp_close(cal_filp, NULL);
	set_fs(old_fs);    
     
	return err;
}


static int k3dh_do_calibration(void)
{
	struct k3dh_acc data = { 0, };
	int sum[3] = { 0, };
	int err = 0;
	int i;

	for (i = 0; i < CALIBRATION_DATA_AMOUNT; i++) {

            err = k3dh_read_accel_raw_xyz(&data);
            if (err < 0) {
                pr_err("%s: k3dh_read_accel_raw_xyz() failed in the %dth loop\n", __func__, i);
                return err;
            }

            sum[0] += data.x;
            sum[1] += data.y;
            sum[2] += data.z;

            ACCDBG("[K3DH] calibration sum data (%d,%d,%d)\n", sum[0], sum[1], sum[2]);        
	}

	g_k3dh->cal_data.x = sum[0] / CALIBRATION_DATA_AMOUNT;      //K3DH(12bit) 0+-154, K3DM(8bit) 0+-12
	g_k3dh->cal_data.y = sum[1] / CALIBRATION_DATA_AMOUNT;      //K3DH(12bit) 0+-154, K3DM(8bit) 0+-12

        if(sum[2] >= 0) {    
#if defined(CONFIG_SENSORS_ACC_12BIT)	
            g_k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) - 1024;       //K3DH(12bit) 1024 +-226, K3DM(8bit) 64+-16
#else	
            g_k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) - 64;       //K3DH(12bit) 1024 +-226, K3DM(8bit) 64+-16	
#endif       
        } else {
#if defined(CONFIG_SENSORS_ACC_12BIT)	
            g_k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) + 1024;       //K3DH(12bit) 1024 +-226, K3DM(8bit) 64+-16
#else	
            g_k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) + 64;       //K3DH(12bit) 1024 +-226, K3DM(8bit) 64+-16	
#endif
        }

	printk(KERN_INFO "%s: cal data (%d,%d,%d)\n", __func__,	g_k3dh->cal_data.x, g_k3dh->cal_data.y, g_k3dh->cal_data.z);
        
	return err;
}


static int k3dh_do_calibration_fs(int enable)
{
	struct k3dh_acc data = { 0, };
	struct file *cal_filp = NULL;
	int sum[3] = { 0, };
	int err = 0, ret = 0;
	int i;
	mm_segment_t old_fs;

	for (i = 0; i < CALIBRATION_DATA_AMOUNT; i++) {

            err = k3dh_read_accel_raw_xyz(&data);
            if (err < 0) {
                pr_err("%s: k3dh_read_accel_raw_xyz() failed in the %dth loop\n", __func__, i);
                return err;
            }

            sum[0] += data.x;
            sum[1] += data.y;
            sum[2] += data.z;

            ACCDBG("[K3DH] calibration sum data (%d,%d,%d)\n", sum[0], sum[1], sum[2]);        
	}

	if (enable) {
        	g_k3dh->cal_data.x = sum[0] / CALIBRATION_DATA_AMOUNT;      //K3DH(12bit) 0+-154, K3DM(8bit) 0+-12
        	g_k3dh->cal_data.y = sum[1] / CALIBRATION_DATA_AMOUNT;      //K3DH(12bit) 0+-154, K3DM(8bit) 0+-12

                if(sum[2] >= 0) {
#if defined(CONFIG_SENSORS_ACC_12BIT)
                    g_k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) - 1024;       //K3DH(12bit) 1024 +-226, K3DM(8bit) 64+-16
#else	
                    g_k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) - 64;       //K3DH(12bit) 1024 +-226, K3DM(8bit) 64+-16	
#endif
	} else {
#if defined(CONFIG_SENSORS_ACC_12BIT)	
                    g_k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) + 1024;       //K3DH(12bit) 1024 +-226, K3DM(8bit) 64+-16
#else	
                    g_k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) + 64;       //K3DH(12bit) 1024 +-226, K3DM(8bit) 64+-16	
#endif
                }

	} else {
		g_k3dh->cal_data.x = 0;
		g_k3dh->cal_data.y = 0;
		g_k3dh->cal_data.z = 0;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_CREAT |O_TRUNC | O_WRONLY | O_SYNC, 0660);
	if (IS_ERR(cal_filp)) {
            err = PTR_ERR(cal_filp);        
            pr_err("%s: Can't open calibration file= %d\n", __func__, err);        
            set_fs(old_fs);
            return err;
	}

        if (cal_filp && cal_filp->f_op && cal_filp->f_op->write){
            ret = cal_filp->f_op->write(cal_filp,(char *)(&g_k3dh->cal_data), 3 * sizeof(s16), &cal_filp->f_pos);
            if (ret != 3 * sizeof(s16)) {
                pr_err("%s: Can't write the cal data to file = %d\n", __func__, ret);        
                err = -EIO;
            }
        }
        else{
                pr_err("%s: (cal_filp && cal_filp->f_op && cal_filp->f_op->write)= 0\n", __func__);        
                err = -EIO;                
        }

        if(cal_filp) 
            filp_close(cal_filp, NULL);
	set_fs(old_fs);
       
	return err;
}


/*  open command for K3DH device file  */
static int k3dh_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);	
}

/*  release command for K3DH device file */
static int k3dh_close(struct inode *inode, struct file *file)
{
	atomic_sub(1, &g_k3dh->opened);
	return 0;
}

static s64 k3dh_get_delay(struct k3dh_data *k3dh)
{
	int i;
	u8 odr;
	s64 delay = -1;

	odr = k3dh->ctrl_reg1_shadow & ODR_MASK;
	for (i = 0; i < ARRAY_SIZE(odr_delay_table); i++) {
		if (odr == odr_delay_table[i].odr) {
			delay = odr_delay_table[i].delay_ns;
			break;
		}
	}
	return delay;
}

static int k3dh_set_delay(struct k3dh_data *k3dh, s64 delay_ns)
{
	int odr_value = ODR1;
	int res = 0;
	int i;
	/* round to the nearest delay that is less than
	 * the requested value (next highest freq)
	 */
	ACCDBG(" passed %lldns\n", delay_ns);
	for (i = 0; i < ARRAY_SIZE(odr_delay_table); i++) {
		if (delay_ns < odr_delay_table[i].delay_ns)
			break;
	}
	if (i > 0)
		i--;
	ACCDBG("matched rate %lldns, odr = 0x%x\n",
			odr_delay_table[i].delay_ns,
			odr_delay_table[i].odr);
	odr_value = odr_delay_table[i].odr;
	delay_ns = odr_delay_table[i].delay_ns;
	mutex_lock(&k3dh->write_lock);
	ACCDBG("old = %lldns, new = %lldns\n",
		     k3dh_get_delay(k3dh), delay_ns);
	if (odr_value != (k3dh->ctrl_reg1_shadow & ODR_MASK)) {
		u8 ctrl = (k3dh->ctrl_reg1_shadow & ~ODR_MASK);
		ctrl |= odr_value;
		k3dh->ctrl_reg1_shadow = ctrl;
		res = i2c_smbus_write_byte_data(k3dh->client, CTRL_REG1, ctrl);
		ACCDBG("writing odr value 0x%x\n", odr_value);
	}
	mutex_unlock(&k3dh->write_lock);
	return res;
}

/*  ioctl command for K3DH device file */
static long k3dh_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct k3dh_data *k3dh = file->private_data;
	struct k3dh_acc data = { 0, };    
	s64 delay_ns;

	/* cmd mapping */
	switch (cmd) {
	case K3DH_IOCTL_SET_DELAY:
		printk(KERN_INFO "[K3DH] K3DH_IOCTL_SET_DELAY\n");
		if (copy_from_user(&delay_ns, (void __user *)arg, sizeof(delay_ns)))
			return -EFAULT;
		err = k3dh_set_delay(k3dh, delay_ns);
		break;
	case K3DH_IOCTL_GET_DELAY:
		printk(KERN_INFO "[K3DH] K3DH_IOCTL_GET_DELAY\n");        
		delay_ns = k3dh_get_delay(k3dh);
		if (put_user(delay_ns, (s64 __user *)arg))
			return -EFAULT;
		break;

	case K3DH_IOCTL_SET_CALIBRATION:
		printk(KERN_INFO "[K3DH] K3DH_IOCTL_SET_CALIBRATION\n");                
		if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
			return -EFAULT;
                err = k3dh_open_calibration();
		if (err < 0 && err != -ENOENT)
			printk(KERN_INFO "[K3DH] k3dh_open_calibration() failed\n");
		break;
	case K3DH_IOCTL_GET_CALIBRATION:
		printk(KERN_INFO "[K3DH] K3DH_IOCTL_GET_CALIBRATION\n");      
            	data.x = g_k3dh->cal_data.x;
            	data.y = g_k3dh->cal_data.y;
            	data.z = g_k3dh->cal_data.z;
		if (copy_to_user((void __user *)arg, &data, sizeof(data)))
			return -EFAULT;
		break;
	case K3DH_IOCTL_DO_CALIBRATION:
		printk(KERN_INFO "[K3DH] K3DH_IOCTL_DO_CALIBRATION\n");      
                k3dh_do_calibration();
            	data.x = g_k3dh->cal_data.x;
            	data.y = g_k3dh->cal_data.y;
            	data.z = g_k3dh->cal_data.z;
		if (copy_to_user((void __user *)arg, &data, sizeof(data)))
			return -EFAULT;
		break;       
	case K3DH_IOCTL_READ_ACCEL_XYZ:
		printk(KERN_INFO "[K3DH] K3DH_IOCTL_READ_ACCEL_XYZ\n");                
		err = k3dh_read_accel_xyz(&data);
		if (err)
			break;
		if (copy_to_user((void __user *)arg, &data, sizeof(data)))
			return -EFAULT;
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

static int k3dh_suspend(struct device *dev)
{
        unsigned char acc_data[2] = {0};

        acc_data[0] = CTRL_REG1;
        acc_data[1] = PM_OFF;
        if(k3dh_acc_i2c_write(acc_data, 2) !=0)
            printk(KERN_ERR "[%s] Change to Suspend Mode is failed\n",__FUNCTION__);  

	printk(KERN_INFO "[K3DH] [%s] K3DH !!suspend mode!!\n",__FUNCTION__);
	return 0;
}

static int k3dh_resume(struct device *dev)
{
        unsigned char acc_data[2] = {0};

        acc_data[0] = CTRL_REG1;
        acc_data[1] = DEFAULT_POWER_ON_SETTING;
        if(k3dh_acc_i2c_write(acc_data, 2) !=0)
            printk(KERN_ERR "[%s] Change to Normal Mode is failed\n",__FUNCTION__);  

	printk(KERN_INFO "[K3DH] [%s] K3DH !!resume mode!!\n",__FUNCTION__);
	return 0;
}

static const struct file_operations k3dh_fops = {
	.owner = THIS_MODULE,
	.open = k3dh_open,
	.release = k3dh_close,
	.unlocked_ioctl = k3dh_ioctl,
};

static ssize_t k3dh_fs_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct k3dh_acc acc = { 0, };

	if (g_k3dh->state & ACC_ENABLED)     
	{
            acc.x = g_acc.x;
            acc.y = g_acc.y;
            acc.z = g_acc.z;
	}
        else
        {
	    k3dh_read_accel_xyz(&acc);
        }  

	count = sprintf(buf,"%d,%d,%d\n", acc.x, acc.y, acc.z );

	return count;
}

static ssize_t accel_calibration_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int count = 0;
        int isCalibration = 0;

	printk(KERN_INFO "[K3DH] cal_data : %d %d %d\n", g_k3dh->cal_data.x, g_k3dh->cal_data.y, g_k3dh->cal_data.z);

        if(g_k3dh->cal_data.x == 0 && g_k3dh->cal_data.y == 0 && g_k3dh->cal_data.z == 0)
            isCalibration = 0;
        else
            isCalibration = 1;
        
	count= sprintf(buf, "%d\n", isCalibration);
	return count;
}

static ssize_t accel_calibration_store(struct device *dev,  struct device_attribute *attr,  const char *buf, size_t size)
{
	int err;
	int enable = 0;

	err = kstrtoint(buf, 10, &enable);

	if (err) {
		pr_err("ERROR: %s got bad char\n", __func__);
		return -EINVAL;
	}

	err = k3dh_do_calibration_fs(enable);
	if (err < 0) {
		pr_err("%s: k3dh_do_calibration_fs() failed\n", __func__);
		return err;
	}

	return size;
}


static DEVICE_ATTR(calibration, S_IRUGO|S_IWUSR|S_IWGRP, accel_calibration_show, accel_calibration_store);
static DEVICE_ATTR(raw_data, S_IRUGO, k3dh_fs_read, NULL);


/////////////////////////////////////////////////////////////////////////////////////

static ssize_t poll_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}


static ssize_t poll_delay_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	return size;
}

static ssize_t acc_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (g_k3dh->state & ACC_ENABLED) ? 1 : 0);
}


static ssize_t acc_enable_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	bool new_value;
    
	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mutex_lock(&g_k3dh->power_lock);

	ACCDBG("[BMA222] new_value = %d, old state = %d\n", new_value, (g_k3dh->state & ACC_ENABLED) ? 1 : 0);

	if (new_value && !(g_k3dh->state & ACC_ENABLED)) {
		g_k3dh->state |= ACC_ENABLED;
#if defined(CONFIG_SENSORS_ACC_FILTER)
                g_k3dh->reset_filter = true;
#endif
	} else if (!new_value && (g_k3dh->state & ACC_ENABLED)) {
		g_k3dh->state = 0;
	}
	mutex_unlock(&g_k3dh->power_lock);
	return size;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,   poll_delay_show, poll_delay_store);

static struct device_attribute dev_attr_acc_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       acc_enable_show, acc_enable_store);

static struct attribute *acc_sysfs_attrs[] = {
	&dev_attr_acc_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group acc_attribute_group = {
	.attrs = acc_sysfs_attrs,
};
///////////////////////////////////////////////////////////////////////////////////

void k3dh_shutdown(struct i2c_client *client)
{
        unsigned char acc_data[2] = {0};

        acc_data[0] = CTRL_REG1;
        acc_data[1] = PM_OFF;
        if(k3dh_acc_i2c_write(acc_data, 2) !=0)
            pr_err("%s: pm_off failed\n", __func__);
}

static int k3dh_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct k3dh_data *k3dh;
	struct input_dev *input_dev;    
    	struct k3dh_platform_data *platform_data;
	int err, tempvalue;
    	int ii = 0;

	printk(KERN_INFO "[K3DH] %s\n",__FUNCTION__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c functionality check failed!\n", __func__);
		err = -ENODEV;
		goto exit;
	}

	k3dh = kzalloc(sizeof(struct k3dh_data), GFP_KERNEL);
	if (k3dh == NULL) {
		dev_err(&client->dev, "failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto exit;
	}

	printk(KERN_INFO "[K3DH] [%s] slave addr = %x\n", __func__, client->addr);

        platform_data = client->dev.platform_data;
	/* read chip id */
	tempvalue = WHO_AM_I;
	err = i2c_master_send(client, (char*)&tempvalue, 1);
	if(err < 0)
	{
		printk(KERN_ERR "k3dh_probe : i2c_master_send [%d]\n", err);			
	}
    
	err = i2c_master_recv(client, (char*)&tempvalue, 1);
	if(err < 0)
	{
		printk(KERN_ERR "k3dh_probe : i2c_master_recv [%d]\n", err);			
	}       

	if((tempvalue&0x00FF) == 0x0033)  // changed for K3DM.
		printk(KERN_INFO "KR3DM I2C driver registered 0x%x!\n", tempvalue);
	else
		printk(KERN_ERR "KR3DM I2C driver not registered 0x%x!\n", tempvalue);

	k3dh_client = client;
	k3dh->client = k3dh_client;
	i2c_set_clientdata(client, k3dh);

	/* sensor HAL expects to find /dev/accelerometer */
	k3dh->k3dh_device.minor = MISC_DYNAMIC_MINOR;
	k3dh->k3dh_device.name = "k3dh";
	k3dh->k3dh_device.fops = &k3dh_fops;

        g_k3dh = k3dh;
        
#if defined(CONFIG_SENSORS_ACC_FILTER)
	hrtimer_init(&g_k3dh->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_k3dh->hrtimer_delay_usec = 10526; //Fs=95Hz
	g_k3dh->device_polling_delay = ns_to_ktime( (g_k3dh->hrtimer_delay_usec)  * USEC_PER_MSEC);
	g_k3dh->timer.function = k3dh_acc_timer_func;

	INIT_WORK(&g_k3dh->read_work, k3dh_acc_read_work_func);
        g_k3dh->read_workqueue = create_workqueue("k3dh_acc_read_wq");
	if (!g_k3dh->read_workqueue) {
		err = -ENOMEM;
		printk(KERN_ERR "cannot create workqueue read_wq: %d\n", err);
    		goto exit;
	}    
	/* Accelerometer Software Filter */
	g_k3dh->reset_filter = true;
	g_k3dh->enabled_filter = true;
    	printk(KERN_INFO "[K3DH] Enabled Accelerometer Software Filtering!\n");
#endif
        
    	mutex_init(&g_k3dh->read_lock);
	mutex_init(&g_k3dh->write_lock);
	mutex_init(&g_k3dh->power_lock);
	atomic_set(&g_k3dh->opened, 0);  

	err = misc_register(&k3dh->k3dh_device);
	if (err) {
		pr_err("%s: misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* allocate lightsensor-level input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		printk(KERN_ERR "%s: could not allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_input_allocate_device_light;        
	}
	input_set_drvdata(input_dev, g_k3dh);
	input_dev->name = "accelerometer_sensor";

	set_bit(EV_REL, input_dev->evbit);	
	/* 32768 == 1g, range -4g ~ +4g */
	/* acceleration x-axis */
	input_set_capability(input_dev, EV_REL, REL_X);
	input_set_abs_params(input_dev, REL_X, -256, 256, 0, 0);
	/* acceleration y-axis */
	input_set_capability(input_dev, EV_REL, REL_Y);
	input_set_abs_params(input_dev, REL_Y, -256, 256, 0, 0);
	/* acceleration z-axis */
	input_set_capability(input_dev, EV_REL, REL_Z);
	input_set_abs_params(input_dev, REL_Z, -256, 256, 0, 0);

	printk(KERN_INFO "[K3DH] registering sensor-level input device\n");

	err = input_register_device(input_dev);
	if (err < 0) {
		printk(KERN_ERR "%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_light;
	}
	g_k3dh->acc_input_dev = input_dev;

	err = sysfs_create_group(&input_dev->dev.kobj,&acc_attribute_group);
	if (err) {
		printk(KERN_ERR "Creating bh1721 attribute group failed");
		goto error_device;
	}

	/* initialized sensor orientation */
        for (ii = 0; ii < 9; ii++){
        	g_k3dh->orientation[ii] = platform_data->orientation[ii];
        }

	/* initialized sensor cal data */
        g_k3dh->cal_data.x=0;
        g_k3dh->cal_data.y=0;
        g_k3dh->cal_data.z=0;        
        
	return 0;

error_device:
	input_unregister_device(input_dev);        
err_input_register_device_light:
        input_free_device(input_dev);
err_input_allocate_device_light:	
	misc_deregister(&k3dh->k3dh_device);    
err_misc_register:
	mutex_destroy(&k3dh->read_lock);
	mutex_destroy(&k3dh->write_lock);
	mutex_destroy(&k3dh->power_lock);    
#if defined(CONFIG_SENSORS_ACC_FILTER)
        destroy_workqueue(k3dh->read_workqueue);
#endif    
	kfree(k3dh);
exit:
	return err;
}

static int k3dh_remove(struct i2c_client *client)
{
	struct k3dh_data *k3dh = i2c_get_clientdata(client);

	sysfs_remove_group(&k3dh->acc_input_dev->dev.kobj, &acc_attribute_group);
    
	input_unregister_device(k3dh->acc_input_dev);

	misc_deregister(&k3dh->k3dh_device);
	mutex_destroy(&k3dh->read_lock);
	mutex_destroy(&k3dh->write_lock);
	mutex_destroy(&k3dh->power_lock);    
#if defined(CONFIG_SENSORS_ACC_FILTER)
        hrtimer_cancel(&k3dh->timer);
        cancel_work_sync(&k3dh->read_work);
        destroy_workqueue(k3dh->read_workqueue);
#endif
	kfree(k3dh);
	g_k3dh = NULL;

	return 0;
}

static const struct i2c_device_id k3dh_id[] = {
	{ "k3dh", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, k3dh_id);

static const struct dev_pm_ops k3dh_pm_ops = {
	.suspend = k3dh_suspend,
	.resume = k3dh_resume,
};

static struct i2c_driver k3dh_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "k3dh",
#if !defined(CONFIG_SENSORS_HSCDTD006A)  && !defined(CONFIG_SENSORS_HSCDTD008A)
                .pm = &k3dh_pm_ops,
#endif                
	},
	.id_table = k3dh_id,
	.probe = k3dh_probe,
	.shutdown = k3dh_shutdown,
	.remove = k3dh_remove,
};


static int __init k3dh_init(void)
{
	struct device *dev_t;
        int ret=0;
        
    	printk(KERN_INFO "[K3DH] %s\n",__FUNCTION__);

	VSENSOR_3_0_V = regulator_get(NULL,"hv8");
	if(IS_ERR(VSENSOR_3_0_V)){
		printk(KERN_ERR "[K3DH] can not get VSENSOR_3.0V\n");
	}	

        ret = regulator_is_enabled(VSENSOR_3_0_V);
        printk(KERN_INFO "[K3DH] regulator_is_enabled : %d\n", ret);

         ret = regulator_set_voltage(VSENSOR_3_0_V,3000000,3000000);	
        printk(KERN_INFO "[K3DH] regulator_set_voltage : %d\n", ret);

        ret = regulator_enable(VSENSOR_3_0_V);
        printk(KERN_INFO "[K3DH] regulator_enable : %d\n", ret);

        /*After Power Supply is supplied, about 1ms delay is required before issuing read/write commands */
        mdelay(10);            

#if defined(CONFIG_SENSORS_CORE)
	dev_t = device_create( sensors_class, NULL, 0, NULL, "accelerometer_sensor");

	if (device_create_file(dev_t, &dev_attr_raw_data) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_raw_data.attr.name);
	if (device_create_file(dev_t, &dev_attr_calibration) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_calibration.attr.name);

	if (IS_ERR(dev_t)) 
	{
            return PTR_ERR(dev_t);
	}
#endif  
       
	return i2c_add_driver(&k3dh_driver);
}

static void __exit k3dh_exit(void)
{
    	printk(KERN_INFO "[K3DH] %s\n",__FUNCTION__);    
        
	i2c_del_driver(&k3dh_driver);
#if defined(CONFIG_SENSORS_CORE)    
    	device_destroy(sensors_class, 0);
#endif
}

module_init(k3dh_init);
module_exit(k3dh_exit);

MODULE_DESCRIPTION("k3dh accelerometer driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");
