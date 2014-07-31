/*******************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
* 	@file	 drivers/input/keyboard/bcm_keypad.c
*
* Unless you and Broadcom execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*******************************************************************************/
#define DEBUG /* enable the pr_debug calls */
#define tempINTERFACE_OSDAL_KEYPAD

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/memory.h>
#include <linux/io.h>
#include <mach/bcm_keypad.h>
#include <linux/slab.h>
#ifndef _HERA_
#include <mach/rdb/brcm_rdb_sysmap.h>
#include <mach/rdb/brcm_rdb_util.h>
#include <mach/rdb/brcm_rdb_padctrlreg.h>
#endif

#define CONFIG_DEBUG_FS

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#include "plat/chal/chal_types.h"
#include "plat/chal/chal_common.h"
#include "plat/chal/chal_keypad.h"
#include <linux/rtc.h>

/*Debug messages */
#define       BCMKP_DEBUG 
#ifdef BCMKP_DEBUG
#define BCMKP_DBG(format, args...)	pr_info(__FILE__ ":" format, ## args)
#else
#define	BCMKP_DBG(format, args...)
#endif

#define BCM_KEY_REPEAT_PERIOD     100	/*repeat period (msec) */
#define BCM_KEY_REPEAT_DELAY      400	/*First time press delay (msec) */

#define KEYPAD_MAX_ROWS    8
#define KEYPAD_MAX_COLUMNS 8

#define MATRIX_SIZE               2	/* 32 bits */
#define U32_BITS                  32

#define KPDCR_MSK_COLFLT_S       8
#define KPDCR_MSK_ROWFLT_S       12
#define KPDCR_MSK_COLS_S         16
#define KPDCR_MSK_ROWS_S         20

#define KPD_ROWS(x)              ((x - 1) << KPDCR_MSK_ROWS_S)
#define KPD_COLS(x)              ((x - 1) << KPDCR_MSK_COLS_S)
#define KPD_ROWFLT(x)            ((x) << KPDCR_MSK_ROWFLT_S)
#define KPD_COLFLT(x)            ((x) << KPDCR_MSK_COLFLT_S)

#define KPDCR_FLT_1ms            0x0
#define KPDCR_FLT_2ms            0x1
#define KPDCR_FLT_4ms            0x2
#define KPDCR_FLT_8ms            0x3
#define KPDCR_FLT_16ms           0x4
#define KPDCR_FLT_32ms           0x5
#define KPDCR_FLT_64ms           0x6	/* default */
#define KPDCR_FLT_128ms          0x7

#define KPEMR_NO_TRIGGER           0x0
#define KPEMR_RISING_EDGE          0x1
#define KPEMR_FALLING_EDGE         0x2
#define KPEMR_BOTH_EDGE            0x3

/* ---- Constants and Types ---------------------------------------------- */
static void __iomem *bcm_keypad_base_addr;

#ifdef CONFIG_DEBUG_FS
static struct dentry *keypad_root_dir;
#endif

#define REG_KEYPAD_KPCR     0x00
#define REG_KEYPAD_KPIOR    0x04

/* Special clone keypad registers, same bases as GPIO */
/* registers, but custom bit assignments for rows/columns. */
#define REG_KEYPAD_KPEMR0  0x10
#define REG_KEYPAD_KPEMR1  0x14
#define REG_KEYPAD_KPEMR2  0x18
#define REG_KEYPAD_KPEMR3  0x1C
#define REG_KEYPAD_KPSSR0  0x20
#define REG_KEYPAD_KPSSR1  0x24
#define REG_KEYPAD_KPIMR0  0x30
#define REG_KEYPAD_KPIMR1  0x34
#define REG_KEYPAD_KPICR0  0x38
#define REG_KEYPAD_KPICR1  0x3C
#define REG_KEYPAD_KPISR0  0x40
#define REG_KEYPAD_KPISR1  0x44

/* REG_KEYPAD_KPCR bits */
#define REG_KEYPAD_KPCR_ENABLE              0x00000001	/* Enable key pad control */
#define REG_KEYPAD_KPCR_PULL_UP             0x00000002
#define REG_KEYPAD_COLFILTER_EN             0x00000800	/* Enable column filtering */
#define REG_KEYPAD_STATFILTER_EN            0x00008000	/* Enable status filtering */

#define BCM_INTERRUPT_EVENT_FIFO_LENGTH	4
#define FIFO_EMPTY(fifo)				((fifo.head == fifo.tail) && !fifo.fifo_full)
#define FIFO_INCREMENT_HEAD(fifo)	(fifo.head = ((fifo.head+1) & (fifo.length-1)))
#define FIFO_INCREMENT_TAIL(fifo)	(fifo.tail = ((fifo.tail+1) & (fifo.length-1)))

#define FOR_PREVENT_2MULTI_PRESSED

struct bcm_keypad {
	struct input_dev *input_dev;
	struct bcm_keymap *kpmap;
	spinlock_t bcm_kp_spin_Lock;
	unsigned int irq;	/* Device IRQ */
	unsigned int row_num;
	unsigned int col_num;
	unsigned int matrix[MATRIX_SIZE];
	unsigned int oldmatrix[MATRIX_SIZE];
};

typedef struct
{
	unsigned char		head;
	unsigned char		tail;
	unsigned char		length;
	bool				fifo_full;
	CHAL_KEYPAD_REGISTER_SET_t	eventQ[BCM_INTERRUPT_EVENT_FIFO_LENGTH];
} BCM_KEYPAD_INTERRUPT_FIFO_t; // interrupt event Q - register data from an interrupt to be processed later.

int bcm_keypad_check(void);
static void bcm_keypad_tasklet(unsigned long);
/*static void bcm_handle_key_state(struct bcm_keypad *bcm_kb);*/
DECLARE_TASKLET_DISABLED(kp_tasklet, bcm_keypad_tasklet, 0);

int last_key_code = 0;
EXPORT_SYMBOL(last_key_code);
/* sys fs  */
struct class *key_class;
EXPORT_SYMBOL(key_class);
struct device *key_dev;
EXPORT_SYMBOL(key_dev);
 

#ifdef CONFIG_BATTERY_D2083
#include <linux/d2083/d2083_battery.h>
#endif
#ifdef CONFIG_MFD_D2083
extern int d2083_onkey_check(void);
#else
extern unsigned int bcmpmu_get_ponkey_state(void);
#endif
 
#ifdef CONFIG_KEYBOARD_EXPANDER
extern unsigned char GetExpanderKeyStatus(void);
#endif

static atomic_t check_keypad_pressed;

#if defined (FOR_PREVENT_2MULTI_PRESSED)
int check_4key_vk1,check_4key_press1,check_4key_vk2, check_4key_press2=0;
#endif

static ssize_t key_show(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(keyshort, S_IRUGO, key_show, NULL);


static ssize_t key_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t keys_pressed;
    uint32_t onkey_pressed = 0;
	cUInt32 keyreadstatus1;
	cUInt32 keyreadstatus2;
#ifdef CONFIG_KEYBOARD_EXPANDER
	uint8_t expander_key_status;
#endif	
    /*
    KPSSRx : Keypress status
    if scan mode type in KPCR is pull-down : 0-Not pressed, 1-Pressed
    */    
	keyreadstatus1 = chal_keypad_config_read_status1();
    keyreadstatus2 = chal_keypad_config_read_status2();

#ifdef CONFIG_MFD_D2083
	onkey_pressed = d2083_onkey_check();
#else
    onkey_pressed = bcmpmu_get_ponkey_state();
#endif

#ifdef CONFIG_KEYBOARD_EXPANDER
	expander_key_status = GetExpanderKeyStatus();
#endif
    

	printk("[KEYPAD] %s, keyreadstatus1=0x%08x, keyreadstatus1=0x%08x\n", __func__, keyreadstatus1, keyreadstatus2);

	if(keyreadstatus1 || keyreadstatus2 || onkey_pressed
#ifdef CONFIG_KEYBOARD_EXPANDER	
		|| expander_key_status
#endif		
	)
    {
        /* key press */
        keys_pressed = 1;
        
    } 
    else 
    {
        /* key release */
        keys_pressed = 0;                        
    }

     return sprintf(buf, "%d\n", keys_pressed );
}
/* sys fs */


static CHAL_HANDLE keypadHandle = NULL;
static CHAL_KEYPAD_KEY_EVENT_LIST_t keyEventList;
static BCM_KEYPAD_INTERRUPT_FIFO_t intrFifo;
static unsigned int temp_key[2], temp2_key[2]={0x0,0x0};
static bool ignored_key = false;
/* ****************************************************************************** */
/* Function Name: bcm_keypad_interrupt */
/* Description: Interrupt handler, called whenever a keypress/release occur. */
/* ****************************************************************************** */

static irqreturn_t bcm_keypad_interrupt(int irq, void *dev_id)
{
	struct bcm_keypad *bcm_kb = dev_id;
	unsigned long flags;
	CHAL_KEYPAD_REGISTER_SET_t *event;
	int i, j, bit_position, num_of_pressed_bits = 0;
	unsigned int key_status[2]; 

	spin_lock_irqsave(&bcm_kb->bcm_kp_spin_Lock, flags);

	disable_irq_nosync(irq);

	key_status[0] = chal_keypad_config_read_status1();
    key_status[1] = chal_keypad_config_read_status2();

//	printk("[KEYPAD] %s, key_status[0]=0x%08x, key_status[1]=0x%08x\n", __func__, key_status[0], key_status[1]);

	for(i=0;i<2;i++)
	{
		bit_position = 1; 
		for(j=0;j<32;j++) 
		{
			if(key_status[i] & bit_position) num_of_pressed_bits++; 
			bit_position <<= 1; 
		}
	}


	if(num_of_pressed_bits >= 3) 
	{ 	
		ignored_key = true;
		for(i=0;i<2;i++)
		{
			bit_position = 1; 
			for(j=0;j<32;j++) 
	{
				if(temp_key[i] & bit_position)
					if((key_status[i] & bit_position) == 0x0)
	 					temp2_key[i] |= bit_position;

				bit_position <<= 1; 
			}
		}
		printk("[KEYPAD] %s, temp222 : 0x%08x,0x%08x\n", __func__, temp2_key[0],temp2_key[1]);		
		chal_keypad_clear_interrupts(keypadHandle);
		enable_irq(irq);
		spin_unlock_irqrestore(&bcm_kb->bcm_kp_spin_Lock, flags);
	}
	else
	{
		temp_key[0]=key_status[0];
		temp_key[1]=key_status[1];
	if (!intrFifo.fifo_full)
	{
		event = &intrFifo.eventQ[intrFifo.head];
		chal_keypad_retrieve_key_event_registers(keypadHandle, event);
			if(ignored_key)
			{
				// To prevent missing Key release for the previous pressed Key while multi key processing
				event->isr0 |= temp2_key[0];
				event->isr1 |= temp2_key[1];
				event->ssr0 &= ~temp2_key[0];
				event->ssr1 &= ~temp2_key[1];

				printk("[KEYPAD] %s, temp : 0x%08x,0x%08x temp2 : 0x%08x,0x%08x\n", __func__, temp_key[0],temp_key[1],temp2_key[0],temp2_key[1]);
				printk("[KEYPAD] %s, isr=0x%08x,0x%08x, ssr=0x%08x,0x%08x\n", __func__, event->isr0, event->isr1,event->ssr0,event->ssr1);

				ignored_key=false;
				temp2_key[0]=0;
				temp2_key[1]=0;				 
			}

		FIFO_INCREMENT_HEAD(intrFifo);
		if(intrFifo.head == intrFifo.tail)
			intrFifo.fifo_full = TRUE;
	}
	else
		printk("[KEYPAD] %s, intrFifo Full!!!!!\n", __func__);


	chal_keypad_clear_interrupts(keypadHandle);

	tasklet_schedule(&kp_tasklet);
	enable_irq(irq);
	spin_unlock_irqrestore(&bcm_kb->bcm_kp_spin_Lock, flags);
	}

	return IRQ_HANDLED;
}

/* ****************************************************************************** */
/* Function Name: bcm_handle_key */
/* Description: Report key actions to input framework */
/* ****************************************************************************** */
static void bcm_handle_key(struct bcm_keypad *bcm_kb, CHAL_KEYPAD_KEY_ID_t	keyId, 
    CHAL_KEYPAD_ACTION_t    keyAction)
{
	struct bcm_keymap *keymap_p = bcm_kb->kpmap;
    /* KeyId is of the form 0xCR where:  C = column number    R = row number 
            Use it as index into map structure */
    unsigned int vk = keymap_p[keyId].key_code;

    struct timespec ts;
    struct rtc_time tm;
    
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
    /*
    KPSSRx : Keypress status
    if scan mode type in KPCR is pull-down : 0-Not pressed, 1-Pressed
    */    

    if (keyAction == CHAL_KEYPAD_KEY_PRESS)
    {

//        printk("[%02d:%02d:%02d.%03lu][KEY] %s Press vk=%d\n",  tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec, keymap_p[keyId].name, vk);

		printk("[KEY] Press\n");

    
#if defined (FOR_PREVENT_2MULTI_PRESSED)
		if((vk==KEY_HOME)||(vk==KEY_MENU))
		{
			if(((check_4key_vk1==KEY_MENU)&&(check_4key_press1==1)&&(vk==KEY_HOME))||((check_4key_vk1==KEY_HOME)&&(check_4key_press1==1)&&(vk==KEY_MENU)))
			{
			//	printk("[%02d:%02d:%02d.%03lu] check_4key_press1 ignore =%d\n",  tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec, vk);
				return 0;
			}
			else
			{
			//	printk("check_4key_press1 check_4key_vk1 =%d, check_4key_press1=%d, vk=%d\n",  check_4key_vk1, check_4key_press1, vk);
				check_4key_vk1=vk;
				check_4key_press1=1;
			}
		}
		else if((vk==KEY_BACK)||(vk==KEY_SEARCH))
		{
			if(((check_4key_vk2==KEY_BACK)&&(check_4key_press2==1)&&(vk==KEY_SEARCH))||((check_4key_vk2==KEY_SEARCH)&&(check_4key_press2==1)&&(vk==KEY_BACK)))
			{
			//	printk("[%02d:%02d:%02d.%03lu] check_4key_press2 ignore =%d\n",  tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec, vk);
				return 0;
			}
			else
			{
			//	printk("check_4key_press1 check_4key_vk2 =%d, check_4key_press2=%d, vk=%d\n",  check_4key_vk2, check_4key_press2, vk);
				check_4key_vk2=vk;
				check_4key_press2=1;
			}
		}
#endif    
        input_report_key(bcm_kb->input_dev, vk, 1);
	atomic_set(&check_keypad_pressed, 1);        
        input_sync(bcm_kb->input_dev);
	last_key_code=vk;		
    }
    else if (keyAction == CHAL_KEYPAD_KEY_RELEASE)
    {
//        printk("[%02d:%02d:%02d.%03lu][KEY] Release vk=%d\n",  tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec, vk);
		printk("[KEY] Released\n");
    
#if defined (FOR_PREVENT_2MULTI_PRESSED)
		if((vk==KEY_HOME)||(vk==KEY_MENU))
		{
			if(((check_4key_vk1==KEY_MENU)&&(check_4key_press1==1)&&(vk==KEY_HOME))||((check_4key_vk1==KEY_HOME)&&(check_4key_press1==1)&&(vk==KEY_MENU)))
			{
			//	printk("[%02d:%02d:%02d.%03lu] check_4key_press1 ignore =%d\n",	tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec, vk);
				return 0;
			}
			else
			{
		//		printk("check_4key_rlease check_4key_vk1 =%d, check_4key_press1=%d, vk=%d\n",  check_4key_vk1, check_4key_press1, vk);
				check_4key_vk1=vk;
				check_4key_press1=0;
			}
		}
		else if((vk==KEY_BACK)||(vk==KEY_SEARCH))
		{
			if(((check_4key_vk2==KEY_BACK)&&(check_4key_press2==1)&&(vk==KEY_SEARCH))||((check_4key_vk2==KEY_SEARCH)&&(check_4key_press2==1)&&(vk==KEY_BACK)))
			{
		//		printk("[%02d:%02d:%02d.%03lu] check_4key_press2 ignore =%d\n",	tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec, vk);
				return 0;
			}
			else
			{
		//		printk("check_4key_rlease check_4key_vk2 =%d, check_4key_press1=%d, vk=%d\n",  check_4key_vk2, check_4key_press2, vk);
				check_4key_vk2=vk;
				check_4key_press2=0;
			}
		}
#endif    
        input_report_key(bcm_kb->input_dev, vk, 0); 
	atomic_set(&check_keypad_pressed, 0);        
        input_sync(bcm_kb->input_dev);
	last_key_code=0;		
    }
}

int bcm_keypad_check(void)
{
	return atomic_read(&check_keypad_pressed);
}

#ifdef _HERA_

/* ****************************************************************************** */
/* Function Name: bcm_set_keypad_pinmux */
/* Description: configure keypad pin muxing. */
/* ****************************************************************************** */
static void bcm_set_keypad_pinmux(void)
{
    PadCtrlConfig_t padCtrlCfg;
    CHAL_HANDLE handle;

    handle = chal_padctrl_init(HW_IO_PHYS_TO_VIRT(PAD_CTRL_BASE_ADDR));
    
    padCtrlCfg.DWord=0;    
    padCtrlCfg.PadCtrlConfigBitField.mux = 1;
    padCtrlCfg.PadCtrlConfigBitField.hys = 0;
    padCtrlCfg.PadCtrlConfigBitField.pdn = 0;
    padCtrlCfg.PadCtrlConfigBitField.pup = 0;
    padCtrlCfg.PadCtrlConfigBitField.rate = 0;
    padCtrlCfg.PadCtrlConfigBitField.ind = 0;
    padCtrlCfg.PadCtrlConfigBitField.mode = 0;

    // Grant keypad function request
    chal_padctrl_grant_kpd(handle, padCtrlCfg);	
}

#else

/* ****************************************************************************** */
/* Function Name: bcm_set_keypad_pinmux */
/* Description: configure keypad pin muxing. */
/* ****************************************************************************** */
static void bcm_set_keypad_pinmux(void)
{

	//use pmux api when it's available for Rhea
	// currently keypad pinmux is set through BSP
}

#endif

#ifdef CONFIG_DEBUG_FS

static ssize_t bcm_keypad_showkey(struct file *file, char __user * user_buf,
				  size_t count, loff_t * ppos)
{
	uint8_t keys_pressed;
	int onkey_pressed = 0;
	unsigned int keyreadstatus1, keyreadstatus2;
	char buf[256];
	unsigned long len = 0;

	keyreadstatus1 = chal_keypad_config_read_status1();
	keyreadstatus2 = chal_keypad_config_read_status2();

#ifdef CONFIG_MFD_D2083
	onkey_pressed = d2083_onkey_check();
#else
	onkey_pressed = bcmpmu_get_ponkey_state();
#endif

	/*
	   KPSSRx : Keypress status
	   if scan mode type in KPCR is pull-up : 1-Not pressed, 0-Pressed
	   if scan mode type in KPCR is pull-down : 0-Not pressed, 1-Pressed
	 */
	if (keyreadstatus1 || keyreadstatus2 || onkey_pressed)	/* key press */
		keys_pressed =
		    (chal_keypad_get_pullup_status(keypadHandle)) ? 0 : 1;
	else			/* key release */
		keys_pressed =
		    (chal_keypad_get_pullup_status(keypadHandle)) ? 1 : 0;

	len += snprintf(buf + len, sizeof(buf) - len,
			"keyreadstatus1=0x%08x, keyreadstatus2=0x%08x, keys_pressed:%d\n",
			keyreadstatus1, keyreadstatus2, keys_pressed);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static struct file_operations keypad_getkey_fops = {
	.read = bcm_keypad_showkey,
};

static void bcm_keypad_debug_init(void)
{

    printk("[KEYPAD] %s\n", __func__);

	keypad_root_dir = debugfs_create_dir("keypad", NULL);
	if (!keypad_root_dir) {
		pr_err("Failed to initialize debugfs\n");
		return;
	}

	if (!debugfs_create_file("keyshort", S_IRUSR, keypad_root_dir, NULL,
				 &keypad_getkey_fops)) {
		pr_err("Failed to setup keypad debug file\n");
		debugfs_remove(keypad_root_dir);
	}
}

#endif // CONFIG_DEBUG_FS


/* ****************************************************************************** */
/* Function Name: bcm_keypad_probe */
/* Description: Called to perform module initialization when the module is loaded. */
/* ****************************************************************************** */
static int __devinit bcm_keypad_probe(struct platform_device *pdev)
{
	int ret;
	u32 i;
	/*u32 reg_value;*/

	struct bcm_keypad *bcm_kb;
	struct bcm_keypad_platform_info *pdata = pdev->dev.platform_data;

	if (!pdata) {
		pr_err("%s(%s:%u)::Failed to get platform data\n",
		       __FUNCTION__, __FILE__, __LINE__);
		return -ENOMEM;
	}

	struct bcm_keymap *keymap_p = pdata->keymap;
	CHAL_KEYPAD_CONFIG_t hwConfig;
	bcm_keypad_base_addr = pdata->bcm_keypad_base;
	BCMKP_DBG(KERN_NOTICE "bcm_keypad_probe\n");


	bcm_kb = kmalloc(sizeof(*bcm_kb), GFP_KERNEL);
	if (bcm_kb == NULL) {
		pr_err(  "%s(%s:%u)::Failed to allocate keypad structure...\n", __FUNCTION__, __FILE__, __LINE__);
		kfree(bcm_kb);
		return -ENOMEM;
	}
	memset(bcm_kb, 0, sizeof(*bcm_kb));

	bcm_kb->input_dev = input_allocate_device();
	if (bcm_kb->input_dev == NULL) {
		pr_err("%s(%s:%u)::Failed to allocate input device...\n", __FUNCTION__, __FILE__, __LINE__);
		input_free_device(bcm_kb->input_dev);
		kfree(bcm_kb); // added for fixing SM defect.		
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, bcm_kb);
	bcm_kb->kpmap = pdata->keymap;

	/* Setup input device */
	set_bit(EV_KEY, bcm_kb->input_dev->evbit);
	set_bit(EV_REP, bcm_kb->input_dev->evbit);
	bcm_kb->input_dev->rep[REP_PERIOD] = BCM_KEY_REPEAT_PERIOD;	/* repeat period */
	bcm_kb->input_dev->rep[REP_DELAY] = BCM_KEY_REPEAT_DELAY;	/* fisrt press delay */

	bcm_kb->input_dev->name = "bcm_keypad_v2";
	bcm_kb->input_dev->phys = "keypad/input0";
	bcm_kb->input_dev->id.bustype = BUS_HOST;
	bcm_kb->input_dev->id.vendor = 0x0001;
	bcm_kb->input_dev->id.product = 0x0001;
	bcm_kb->input_dev->id.version = 0x0100;

	memset(bcm_kb->matrix, 0, sizeof(bcm_kb->matrix));

	bcm_kb->row_num = pdata->row_num;	/* KPD_ROW_NUM ; */
	bcm_kb->col_num = pdata->col_num;	/* KPD_COL_NUM ; */
	spin_lock_init(&bcm_kb->bcm_kp_spin_Lock);
	tasklet_enable(&kp_tasklet);
	kp_tasklet.data = (unsigned long)bcm_kb;

	bcm_kb->irq = BCM_INT_ID_KEYPAD;

	pr_debug("%s::bcm_keypad_probe\n", __FUNCTION__);

    /* New chal based h/w setup */
    bcm_set_keypad_pinmux();

	intrFifo.head =0;
	intrFifo.tail = 0;
	intrFifo.length = BCM_INTERRUPT_EVENT_FIFO_LENGTH;
	intrFifo.fifo_full = FALSE;

	hwConfig.rows = bcm_kb->row_num;
	hwConfig.columns = bcm_kb->col_num;
	hwConfig.pullUpMode = FALSE;
	// Porbably shouldn't copy these next 2 directly in case one structure changes but not the corresponding one.
	hwConfig.interruptEdge = CHAL_KEYPAD_INTERRUPT_BOTH_EDGES;
	hwConfig.debounceTime = CHAL_KEYPAD_DEBOUNCE_32_ms;

	keypadHandle = chal_keypad_init((cUInt32)bcm_keypad_base_addr);
	// disable all key interrupts
	chal_keypad_disable_interrupts(keypadHandle);
	// clear any old interrupts
	chal_keypad_clear_interrupts(keypadHandle);
	// disable the keypad hardware block
	chal_keypad_set_enable(keypadHandle, FALSE);
	chal_keypad_set_pullup_mode(keypadHandle, hwConfig.pullUpMode);
	chal_keypad_set_column_filter(keypadHandle, TRUE, hwConfig.debounceTime);
	chal_keypad_set_row_width(keypadHandle, hwConfig.rows);
	chal_keypad_set_column_width(keypadHandle, hwConfig.columns);
	chal_keypad_set_status_filter(keypadHandle, TRUE,  hwConfig.debounceTime);
	
#ifdef SWAP_ROW_COL
	chal_keypad_swap_row_and_column(keypadHandle, TRUE);
#endif
	// use rows as output
	chal_keypad_set_row_output_control(keypadHandle, hwConfig.rows);
	// configure the individual key interrupt controls
	chal_keypad_set_interrupt_edge(keypadHandle, hwConfig.interruptEdge);
	// clear any old interrupts
	chal_keypad_clear_interrupts(keypadHandle);
	chal_keypad_set_interrupt_mask(keypadHandle, hwConfig.rows, hwConfig.columns);
	// clear any old interrupts
	chal_keypad_clear_interrupts(keypadHandle);
	// enable the keypad hardware block
	chal_keypad_set_enable(keypadHandle, TRUE);
	
	for (i = 0; i < (KEYPAD_MAX_ROWS * KEYPAD_MAX_COLUMNS); i++) {
		__set_bit(keymap_p->key_code /*& KEY_MAX*/,
			  bcm_kb->input_dev->keybit);
		keymap_p++;
	}

	ret =
	    request_irq(bcm_kb->irq, bcm_keypad_interrupt, IRQF_DISABLED |
			    IRQF_NO_SUSPEND, "BRCM Keypad", bcm_kb);
	if (ret < 0) {
		pr_err("%s(%s:%u)::request_irq failed IRQ %d\n",
		       __FUNCTION__, __FILE__, __LINE__, bcm_kb->irq);
		goto free_irq;
	}
	ret = input_register_device(bcm_kb->input_dev);
	if (ret < 0) {
		pr_err("%s(%s:%u)::Unable to register GPIO-keypad input device\n",
		       __FUNCTION__, __FILE__, __LINE__);
		goto free_dev;
	}

        /* sys fs */
	key_class = class_create(THIS_MODULE, "keyclass");
	if (IS_ERR(key_class))
		pr_err("Failed to create class(key)!\n");

	key_dev = device_create(key_class, NULL, 0, NULL, "keypad");
	if (IS_ERR(key_dev))
		pr_err("Failed to create device(key)!\n");

	if (device_create_file(key_dev, &dev_attr_keyshort) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_keyshort.attr.name); 
	/* sys fs */

#ifdef CONFIG_DEBUG_FS
	bcm_keypad_debug_init();
#endif

	/* Initialization Finished */
	BCMKP_DBG(KERN_DEBUG "BCM keypad initialization completed...\n");
	return ret;

      free_dev:
	input_unregister_device(bcm_kb->input_dev);
	input_free_device(bcm_kb->input_dev);

      free_irq:
	free_irq(bcm_kb->irq, (void *)bcm_kb);
	kfree(bcm_kb); // added for fixing SM defect.	

	return -EINVAL;
}

/* ****************************************************************************** */
/* Function Name: bcm_keypad_remove */
/* Description: Called to perform module cleanup when the module is unloaded. */
/* ****************************************************************************** */
static int __devexit bcm_keypad_remove(struct platform_device *pdev)
{
	struct bcm_keypad *bcm_kb = platform_get_drvdata(pdev);
	BCMKP_DBG(KERN_NOTICE "bcm_keypad_remove\n");

	/* disable keypad interrupt handling */
	tasklet_disable(&kp_tasklet);

	/*disable keypad interrupt handling */
	free_irq(bcm_kb->irq, (void *)bcm_kb);

	/* unregister everything */
	input_unregister_device(bcm_kb->input_dev);
	input_free_device(bcm_kb->input_dev);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(keypad_root_dir);
#endif

	return 0;
}

/* ****************************************************************************** */
/* Function Name: bcm_keypad_tasklet */
/* Description: tasklet for a keypress/release occur. . */
/* ****************************************************************************** */

static void bcm_keypad_tasklet(unsigned long data)
{
	/* Use SSR to see which key be pressed */
	struct bcm_keypad *bcm_kb = (struct bcm_keypad *)data;

	CHAL_KEYPAD_REGISTER_SET_t *event;
	cUInt32 numKeyEvt = 0;
	int i;
	
	while(!FIFO_EMPTY(intrFifo))
	{
		event = &intrFifo.eventQ[intrFifo.tail];
		numKeyEvt = chal_keypad_process_key_event_registers(keypadHandle, event, keyEventList);
		FIFO_INCREMENT_TAIL(intrFifo);
		if(intrFifo.fifo_full)
		{
			intrFifo.fifo_full = FALSE;
		}
		for(i=0; i<numKeyEvt; i++)
		{
			if(keyEventList[i].keyAction != CHAL_KEYPAD_KEY_NO_ACTION)
			{
				bcm_handle_key(bcm_kb, keyEventList[i].keyId, keyEventList[i].keyAction);
			}
		}
	}
}

/****************************************************************************/

struct platform_driver bcm_keypad_device_driver = {
	.probe = bcm_keypad_probe,
	.remove = __devexit_p(bcm_keypad_remove),
	.driver = {
		   .name = "bcm_keypad",
		   }
};

static int __init bcm_keypad_init(void)
{
	pr_info("bcm_keypad_device_driver\n");
	return platform_driver_register(&bcm_keypad_device_driver);
}

static void __exit bcm_keypad_exit(void)
{
	platform_driver_unregister(&bcm_keypad_device_driver);
}

module_init(bcm_keypad_init);
module_exit(bcm_keypad_exit);

MODULE_AUTHOR("Broadcom Corportaion");
MODULE_DESCRIPTION("BCM Keypad Driver");
MODULE_LICENSE("GPL");
