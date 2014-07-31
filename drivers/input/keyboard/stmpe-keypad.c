/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License, version 2
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mfd/stmpe.h>
#include <linux/gpio.h>
#include <linux/delay.h>

/* These are at the same addresses in all STMPE variants */
#define STMPE1801_FIFO_SIZE				10

#define STMPE_ICR_LSB					0x04
#define STMPE_INT_EN_MASK                         0x06

#define STMPE_KPC_ROW					0x30
#define STMPE_KPC_COL_LOW			0x31
#define STMPE_KPC_COL_HIGH			0x32

#define STMPE_KPC_CTRL_LSB			0x33
#define STMPE_KPC_CTRL_MSB			0x34
#define STMPE_KPC_CTRL_HSB			0x35

#define STMPE_KPC_CTRL_CMD			0x36

#define STMPE_KPC_COMBI_KEY_0		0x37
#define STMPE_KPC_COMBI_KEY_1		0x38
#define STMPE_KPC_COMBI_KEY_2		0x39

#define STMPE_KPC_DATA_BYTE0			0x3A
#define STMPE_KPC_DATA_BYTE1			0x3B
#define STMPE_KPC_DATA_BYTE2			0x3C
#define STMPE_KPC_DATA_BYTE3			0x3D
#define STMPE_KPC_DATA_BYTE4			0x3E

//------------------------------------------------------
#define STMPE_KPC_CTRL_LSB_SCAN			(0x1 << 0)
#define STMPE_KPC_CTRL_LSB_DEBOUNCE		(0x7f << 1)
#define STMPE_KPC_CTRL_MSB_SCAN_COUNT	(0xf << 4)

#define STMPE_KPC_ROW_MSB_ROWS			0xff

#define STMPE_KPC_DATA_UP					(0x1 << 7)
#define STMPE_KPC_DATA_ROW				0x07
#define STMPE_KPC_DATA_COL				0x78
#define STMPE_KPC_DATA_NOKEY_MASK		0x78
//-------------------------------------------------------

#define STMPE_KEYPAD_MAX_DEBOUNCE	127
#define STMPE_KEYPAD_MAX_SCAN_COUNT	15

#define STMPE_KEYPAD_MAX_ROWS		8
#define STMPE_KEYPAD_MAX_COLS		10
#define STMPE_KEYPAD_ROW_SHIFT		3
#define STMPE_KEYPAD_KEYMAP_SIZE	\
	(STMPE_KEYPAD_MAX_ROWS * STMPE_KEYPAD_MAX_COLS)

//extern int stmpe_block_read(struct stmpe *stmpe, u8 reg, u8 length, u8 *values);
//extern int stmpe_block_write(struct stmpe *stmpe, u8 reg, u8 length, const u8 *values);
/**
 * struct stmpe_keypad_variant - model-specific attributes
 * @auto_increment: whether the KPC_DATA_BYTE register address
 *		    auto-increments on multiple read
 * @num_data: number of data bytes
 * @num_normal_data: number of normal keys' data bytes
 * @max_cols: maximum number of columns supported
 * @max_rows: maximum number of rows supported
 * @col_gpios: bitmask of gpios which can be used for columns
 * @row_gpios: bitmask of gpios which can be used for rows
 */
struct stmpe_keypad_variant {
	bool		auto_increment;
	int		num_data;
	int		num_normal_data;
	int		max_cols;
	int		max_rows;
	unsigned int	col_gpios;
	unsigned int	row_gpios;
};

static const struct stmpe_keypad_variant stmpe_keypad_variants[] = {
	[STMPE1601] = {
		.auto_increment		= true,
		.num_data		= 5,
		.num_normal_data	= 3,
		.max_cols		= 8,
		.max_rows		= 8,
		.col_gpios		= 0x000ff,	/* GPIO 0 - 7 */
		.row_gpios		= 0x0ff00,	/* GPIO 8 - 15 */
	},
	[STMPE1801] = {
		.auto_increment 	= true,
		.num_data		= 5,
		.num_normal_data	= 3,
		.max_cols		= 10,
		.max_rows		= 8,
	//	.row_gpios		= 0x000f0,	/* GPIO 4 - 7 */
		.row_gpios		= 0x00007,	/* GPIO 0 - 2 */
		.col_gpios		= 0x3ff00,	/* GPIO 8 - 15 */
	},
	[STMPE2401] = {
		.auto_increment		= false,
		.num_data		= 3,
		.num_normal_data	= 2,
		.max_cols		= 8,
		.max_rows		= 12,
		.col_gpios		= 0x0000ff,	/* GPIO 0 - 7*/
		.row_gpios		= 0x1fef00,	/* GPIO 8-14, 16-20 */
	},
	[STMPE2403] = {
		.auto_increment		= true,
		.num_data		= 5,
		.num_normal_data	= 3,
		.max_cols		= 8,
		.max_rows		= 12,
		.col_gpios		= 0x0000ff,	/* GPIO 0 - 7*/
		.row_gpios		= 0x1fef00,	/* GPIO 8-14, 16-20 */
	},
};

struct stmpe_keypad {
	struct stmpe *stmpe;
	struct input_dev *input;
	const struct stmpe_keypad_variant *variant;
	const struct stmpe_keypad_platform_data *plat;

	unsigned int rows;
	unsigned int cols;

	unsigned short keymap[STMPE_KEYPAD_KEYMAP_SIZE];
};

static int stmpe_keypad_read_data(struct stmpe_keypad *keypad, u8 *data)
{
	const struct stmpe_keypad_variant *variant = keypad->variant;
	struct stmpe *stmpe = keypad->stmpe;
	int ret;
	int i;

	if (variant->auto_increment)
		return stmpe_block_read(stmpe, STMPE_KPC_DATA_BYTE0,
					variant->num_data, data);

	for (i = 0; i < variant->num_data; i++) {
		ret = stmpe_reg_read(stmpe, STMPE_KPC_DATA_BYTE0 + i);
		if (ret < 0)
			return ret;

		data[i] = ret;
	}

	return 0;
}



static irqreturn_t stmpe_keypad_irq(int irq, void *dev)
{
	struct stmpe_keypad *keypad = dev;
	struct input_dev *input = keypad->input;

	int i;

	u8 j = 0;
	u8 val[8];	
	u8 regAdd[7];
	int rc;	
	
	printk("---changed: in stmpe_keypad_irq---\n");

	//0x08:	INT_STA_LOW ---: interrupt stqtus register; 
	//IS0: Wake-up interrupt status
	//IS1: Keypad controller interrupt status
	//IS2: Keypad controller FIFO overflow interrupt status
	//IS3: GPIO controller interrupt status
	//IS4: Combination key interrupt status
	regAdd[0]=0x08;   
	rc = stmpe_block_read(keypad->stmpe, regAdd[0], 2, val);

	//0x3A: KPC_DATA_BYTE0 0x3B: KPC_DATA_BYTE1 0x3C: KPC_DATA_BYTE2 0x3D: KPC_DATA_BYTE3 0x3E: KPC_DATA_BYTE4 
	regAdd[0]=0x3A;
	
	//	1801 FIFO has a capacity for ten sets of key data. Each set of key data consists of 5 bytes of information when any of the four dedicated keys is enabled
	for (i = 0; i < STMPE1801_FIFO_SIZE; i++) { // Reading the fifo one by one and processing it.
		rc = stmpe_block_read(keypad->stmpe, regAdd[0], 5, val);
		
		mdelay(50);		
		
		if( (val[0]  != 0xF8) || (val[1]  != 0xF8) || (val[2]  != 0xF8) || (val[3]  != 0xFF) || (val[4]  != 0x0F) ) {      //0xF8 means column =0x1111: No key
			for(j = 0; j < 3; j++) {
				int data = val[j];
				int col = (data & STMPE_KPC_DATA_COL) >> 3;
				int row = data & STMPE_KPC_DATA_ROW;
				int code = MATRIX_SCAN_CODE(row, col, STMPE_KEYPAD_ROW_SHIFT);
				bool up = val[j] & STMPE_KPC_DATA_UP;		//	1: key-up;   0: key-down
				printk("data = %02X, col = %02X, row = %02X, code = %02X, up = %02X \n", data, col, row, code, up);
				printk("Key1 = %02X\n", keypad->keymap[code]);

				if ((data & STMPE_KPC_DATA_NOKEY_MASK) == STMPE_KPC_DATA_NOKEY_MASK)  // column =0x1111: No key
				        continue;

				input_event(input, EV_MSC, MSC_SCAN, code);
				input_report_key(input, keypad->keymap[code], !up);
				printk("Key2 = %02X\n", keypad->keymap[code]);
				input_sync(input);
			}
			printk("KeyData = %02X %02X %02X %02X %02X\n" , val[0], val[1], val[2], val[3], val[4]);
		}
	}

	return IRQ_HANDLED;
}


static int __devinit stmpe_keypad_altfunc_init(struct stmpe_keypad *keypad)
{
	const struct stmpe_keypad_variant *variant = keypad->variant;
	unsigned int col_gpios = variant->col_gpios;
	unsigned int row_gpios = variant->row_gpios;
	struct stmpe *stmpe = keypad->stmpe;
	unsigned int pins = 0;
	int i;

	/*
	 * Figure out which pins need to be set to the keypad alternate
	 * function.
	 *
	 * {cols,rows}_gpios are bitmasks of which pins on the chip can be used
	 * for the keypad.
	 *
	 * keypad->{cols,rows} are a bitmask of which pins (of the ones useable
	 * for the keypad) are used on the board.
	 */

	for (i = 0; i < variant->max_cols; i++) {
		int num = __ffs(col_gpios);

		if (keypad->cols & (1 << i))
			pins |= 1 << num;

		col_gpios &= ~(1 << num);
	}

	for (i = 0; i < variant->max_rows; i++) {
		int num = __ffs(row_gpios);

		if (keypad->rows & (1 << i))
			pins |= 1 << num;

		row_gpios &= ~(1 << num);
	}

	return stmpe_set_altfunc(stmpe, pins, STMPE_BLOCK_KEYPAD);
}

static int __devinit stmpe_keypad_chip_init(struct stmpe_keypad *keypad)
{
	const struct stmpe_keypad_platform_data *plat = keypad->plat;
	struct stmpe *stmpe = keypad->stmpe;
	int ret, iTmp;
	u8 val[8];	
	u8 regAdd[7];

	if (plat->debounce_ms > STMPE_KEYPAD_MAX_DEBOUNCE)
		return -EINVAL;

	if (plat->scan_count > STMPE_KEYPAD_MAX_SCAN_COUNT)
		return -EINVAL;

	regAdd[0]=0x00;

	ret = stmpe_block_read(stmpe, regAdd[0], 3, val);
	mdelay(50);		
	printk("Chip ID = %x %x %x\n" , val[0], val[1], val[2]);		
	
	regAdd[0]=0x02; //SYS_CTRL
	regAdd[1]=0x86;  //1000 0110--softreset and  set the debounce time as 210us
	stmpe_block_write(stmpe, regAdd[0], 1, &regAdd[1]);
	mdelay(20);	

	regAdd[0]=0x30;
	ret = stmpe_block_read(stmpe, regAdd[0], 6, val);
	mdelay(50);		
	printk("key control register Org value  = %x %x %x %x %x %x\n" , val[0], val[1], val[2], val[3], val[4], val[5]);

	printk("keypad->rows=%02X\n", keypad->variant->row_gpios); //row 	
	ret = stmpe_reg_write(stmpe, STMPE_KPC_ROW, keypad->variant->row_gpios);
	if (ret < 0)
		return ret;

	printk("keypad->cols=%04X\n", keypad->variant->col_gpios );
	
	iTmp = (keypad->variant->col_gpios & 0xFF00)>>0x08 ;     			//col Low 8 bits;	
	printk("iTmp=%02X\n", iTmp);	
	ret = stmpe_reg_write(stmpe, STMPE_KPC_COL_LOW, iTmp);
	if (ret < 0)
		return ret;

	iTmp = (keypad->variant->col_gpios &  0xF0000) >> 0x10 ;     //col high 8 bits;
	printk("iTmp=%02X\n", iTmp);
	ret = stmpe_reg_write(stmpe, STMPE_KPC_COL_HIGH, iTmp);
	if (ret < 0)
		return ret;


	iTmp =  plat->scan_count << 0x04;                                                  //scan_count
	printk("iTmp=%02X\n", iTmp);	
	ret = stmpe_reg_write(stmpe, STMPE_KPC_CTRL_LSB,  iTmp);
	if (ret < 0)
		return ret;

	iTmp = ( plat->debounce_ms << 0x01) & 0xFF; 				//debouncing time;
	printk("iTmp=%02X\n", iTmp);	
	ret = stmpe_reg_write(stmpe, STMPE_KPC_CTRL_MSB,  iTmp);
	if (ret < 0)
		return ret;

	iTmp =0x43; 				// KPC_CTRL_HIGH        0100 0011 -- set the combination key mode: And function; Scan frequence: 275 Hz;
	printk("iTmp=%02X\n", iTmp);	
	ret = stmpe_reg_write(stmpe, STMPE_KPC_CTRL_HSB,  iTmp);
	if (ret < 0)
		return ret;
	mdelay(20);

	iTmp =0x16; 				//0001 0110 -- enable combination key interrupt; key fifo overflow interrupt; key interrupt;
	printk("iTmp=%02X\n", iTmp);	
	ret = stmpe_reg_write(stmpe, STMPE_INT_EN_MASK,  iTmp);
	if (ret < 0)
		return ret;
	mdelay(20);

	iTmp =0x01; 				//           0000 0001 -- level interrupt, Low active; allow interruption to host;
	printk("iTmp=%02X\n", iTmp);	
	ret = stmpe_reg_write(stmpe, STMPE_ICR_LSB,  iTmp);
	if (ret < 0)
		return ret;
	mdelay(20);

	iTmp =0x01; 				//              0000 0001-- no stop mode, start to scan;
	printk("iTmp=%02X\n", iTmp);	
	ret = stmpe_reg_write(stmpe, STMPE_KPC_CTRL_CMD,  iTmp);
	if (ret < 0)
		return ret;
	mdelay(20);

	regAdd[0]=0x30;		 //read all the register out to check  whether the values is changed or not?
	ret = stmpe_block_read(stmpe, regAdd[0], 6, val);
	mdelay(50); 	
	printk("key control register Setting value = %x %x %x %x %x %x\n" , val[0], val[1], val[2], val[3], val[4], val[5]);	


	return 0;

}



static int __devinit stmpe_keypad_probe(struct platform_device *pdev)
{
	struct stmpe *stmpe = dev_get_drvdata(pdev->dev.parent);
	struct stmpe_keypad_platform_data *plat;
	struct stmpe_keypad *keypad;
	struct input_dev *input;
	int ret;
	int irq;
	int i;

	printk("---18:50/11/18/2011---in stmpe_keypad_probe() \n");
	
		
	plat = stmpe->pdata->keypad;
	if (!plat)
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	irq = irq -1; 
	
	printk("---irq is = %d\n", irq);

	if (irq < 0)
		return irq;

	keypad = kzalloc(sizeof(struct stmpe_keypad), GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;

	printk("before input_allocate_device \n");

	input = input_allocate_device();
	if (!input) {
		ret = -ENOMEM;
		goto out_freekeypad;
	}

	input->name = "STMPE keypad";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &pdev->dev;

	input_set_capability(input, EV_MSC, MSC_SCAN);

	__set_bit(EV_KEY, input->evbit);
	if (!plat->no_autorepeat)
		__set_bit(EV_REP, input->evbit);

	input->keycode = keypad->keymap;
	input->keycodesize = sizeof(keypad->keymap[0]);
	input->keycodemax = ARRAY_SIZE(keypad->keymap);

	printk("before matrix_keypad_build_keymap \n");

	matrix_keypad_build_keymap(plat->keymap_data, STMPE_KEYPAD_ROW_SHIFT,
				   input->keycode, input->keybit);

	for (i = 0; i < plat->keymap_data->keymap_size; i++) {
		unsigned int key = plat->keymap_data->keymap[i];

		keypad->cols |= 1 << KEY_COL(key);
		keypad->rows |= 1 << KEY_ROW(key);
	}

	keypad->stmpe = stmpe;
	keypad->plat = plat;
	keypad->input = input;
	keypad->variant = &stmpe_keypad_variants[stmpe->partnum];

	printk("before stmpe_keypad_chip_init stmpe->partnum =%d \n", stmpe->partnum);

	ret = stmpe_keypad_chip_init(keypad);
	if (ret < 0)
		goto out_freeinput;

	printk("before input_register_device \n");

	ret = input_register_device(input);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to register input device: %d\n", ret);
		goto out_freeinput;
	}

	printk("before request_threaded_irq \n");

//	ret = request_threaded_irq(irq, NULL, stmpe_keypad_irq, IRQF_ONESHOT,
//				   "stmpe-keypad", keypad);
	ret = request_threaded_irq(irq, NULL, stmpe_keypad_irq, IRQF_TRIGGER_FALLING|IRQF_ONESHOT,  "stmpe-keypad", keypad);	//it also works+++, 11/18/2011 MIchael HU

	if (ret) {
		dev_err(&pdev->dev, "unable to get irq: %d\n", ret);
		goto out_unregisterinput;
	}

	printk("before platform_set_drvdata \n");

	platform_set_drvdata(pdev, keypad);

	printk("after platform_set_drvdata \n");

	return 0;

out_unregisterinput:
	input_unregister_device(input);
	input = NULL;
out_freeinput:
	input_free_device(input);
out_freekeypad:
	kfree(keypad);
	return ret;

}

static int  stmpe_keypad_remove(struct platform_device *pdev)
{

	struct stmpe_keypad *keypad = platform_get_drvdata(pdev);
	struct stmpe *stmpe = keypad->stmpe;
	int irq = platform_get_irq(pdev, 0);
	irq = irq - 1;

	printk(" in the stmpe_keypad_remove irq = %d \n", irq); 

	if ( keypad == NULL )
		printk(" keypad == NULL\n"); 
	
	stmpe_disable(stmpe, STMPE_BLOCK_KEYPAD);
	printk("after stmpe_disable \n");

	free_irq(irq, keypad);
	printk("after free_irq \n");

	input_unregister_device(keypad->input);
	printk("after input_unregister_device \n");
	keypad->input=NULL;

	platform_set_drvdata(pdev, NULL);
	printk("after platform_set_drvdata \n");

	kfree(keypad);

	printk("after kfree(keypad) \n");

	return 0;
}


static struct platform_driver stmpe_keypad_driver = {
	 .driver.name	= "stmpe-keypad",
	 .driver.owner	= THIS_MODULE,
	.probe		= stmpe_keypad_probe,
	.remove		= __devexit_p(stmpe_keypad_remove),
};

static int __init stmpe_keypad_init(void)
{
	return platform_driver_register(&stmpe_keypad_driver);
}
module_init(stmpe_keypad_init);

static void __exit stmpe_keypad_exit(void)
{
	platform_driver_unregister(&stmpe_keypad_driver);
}
module_exit(stmpe_keypad_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("STMPExxxx keypad driver");
MODULE_AUTHOR("Rabin Vincent <rabin.vincent@stericsson.com>");
