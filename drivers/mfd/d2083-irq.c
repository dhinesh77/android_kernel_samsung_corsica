/*
 * d2083-irq.c: IRQ support for Dialog D2083
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
#include <linux/init.h>
#include <linux/bug.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/kthread.h>

#include <asm/mach/irq.h>
#include <asm/gpio.h>

#include <linux/d2083/d2083_reg.h>
#include <linux/d2083/core.h>
#include <linux/d2083/pmic.h>
#include <linux/d2083/rtc.h>


#define D2083_NUM_IRQ_EVT_REGS	    4

#define D2083_INT_OFFSET_1          0
#define D2083_INT_OFFSET_2          1
#define D2083_INT_OFFSET_3          2
#define D2083_INT_OFFSET_4          3

struct d2083_irq_data {
	int reg;
	int mask;
};

static struct d2083_irq_data d2083_irqs[] = {
	/* EVENT Register A start */
	[D2083_IRQ_EVF] = {
		.reg = D2083_INT_OFFSET_1,
		.mask = D2083_IRQMASKA_MVF,
	},
	[D2083_IRQ_ETBAT2] = {
		.reg = D2083_INT_OFFSET_1,
		.mask = D2083_IRQMASKA_MTBAT2,
	},
	[D2083_IRQ_EVDD_LOW] = {
		.reg = D2083_INT_OFFSET_1,
		.mask = D2083_IRQMASKA_MVDDLOW,
	},
	[D2083_IRQ_EVDD_MON] = {
		.reg = D2083_INT_OFFSET_1,
		.mask = D2083_IRQMASKA_MVDDMON,
	},
	[D2083_IRQ_EALARM] = {
		.reg = D2083_INT_OFFSET_1,
		.mask = D2083_IRQMASKA_MALRAM,
	},
	[D2083_IRQ_ESEQRDY] = {
		.reg = D2083_INT_OFFSET_1,
		.mask = D2083_IRQMASKA_MSEQRDY,
	},
	[D2083_IRQ_ETICK] = {
		.reg = D2083_INT_OFFSET_1,
		.mask = D2083_IRQMASKA_MTICK,
	},
	/* EVENT Register B start */
	[D2083_IRQ_ENONKEY_LO] = {
		.reg = D2083_INT_OFFSET_2,
		.mask = D2083_IRQMASKB_MNONKEY_LO,
	},
	[D2083_IRQ_ENONKEY_HI] = {
		.reg = D2083_INT_OFFSET_2,
		.mask = D2083_IRQMASKB_MNONKEY_HI,
	},
	[D2083_IRQ_ENONKEY_HOLDON] = {
		.reg = D2083_INT_OFFSET_2,
		.mask = D2083_IRQMASKB_MNONKEY_HOLDON,
	},
	[D2083_IRQ_ENONKEY_HOLDOFF] = {
		.reg = D2083_INT_OFFSET_2,
		.mask = D2083_IRQMASKB_MNONKEY_HOLDOFF,
	},
	[D2083_IRQ_ETBAT1] = {
		.reg = D2083_INT_OFFSET_2,
		.mask = D2083_IRQMASKB_MTBAT1,
	},
	[D2083_IRQ_EADCEOM] = {
		.reg = D2083_INT_OFFSET_2,
		.mask = D2083_IRQMASKB_MADC_EOM,
	},
	/* EVENT Register C start */
 	[D2083_IRQ_ETA] = {
		.reg = D2083_INT_OFFSET_3,
		.mask = D2083_IRQMASKC_MTA,
	},
	[D2083_IRQ_ENJIGON] = {
		.reg = D2083_INT_OFFSET_3,
		.mask = D2083_IRQMASKC_MNJIGON,
	},
	/* EVENT Register D start */
	[D2083_IRQ_EGPI0] = {
		.reg = D2083_INT_OFFSET_4,
		.mask = D2083_IRQMASKD_MGPI0,
	},
};



static void d2083_irq_call_handler(struct d2083 *d2083, int irq)
{
	mutex_lock(&d2083->irq_mutex);

	if (d2083->irq[irq].handler) {
		d2083->irq[irq].handler(irq, d2083->irq[irq].data);

	} else {
		d2083_mask_irq(d2083, irq);
	}
	mutex_unlock(&d2083->irq_mutex);
}

/*
 * This is a threaded IRQ handler so can access I2C.  Since all
 * interrupts are clear on read the IRQ line will be reasserted and
 * the physical IRQ will be handled again if another interrupt is
 * asserted while we run - in the normal course of events this is a
 * rare occurrence so we save I2C/SPI reads.
 */
void d2083_irq_worker(struct work_struct *work)
{
	struct d2083 *d2083 = container_of(work, struct d2083, irq_work);
	u8 reg_val;
	u8 sub_reg[D2083_NUM_IRQ_EVT_REGS] = {0,};
	int read_done[D2083_NUM_IRQ_EVT_REGS];
	struct d2083_irq_data *data;
	int i;

	memset(&read_done, 0, sizeof(read_done));

	for (i = 0; i < ARRAY_SIZE(d2083_irqs); i++) {
		data = &d2083_irqs[i];

		if (!read_done[data->reg]) {
			d2083_reg_read(d2083, D2083_EVENTA_REG + data->reg, &reg_val);
			sub_reg[data->reg] = reg_val;
			d2083_reg_read(d2083, D2083_IRQMASKA_REG + data->reg, &reg_val);
			sub_reg[data->reg] &= ~reg_val;
			read_done[data->reg] = 1;
		}

    	if (sub_reg[data->reg] & data->mask) {
    		d2083_irq_call_handler(d2083, i);
			/* Now clear EVENT registers */
			d2083_set_bits(d2083, D2083_EVENTA_REG + data->reg, d2083_irqs[i].mask);
		}
	}
	enable_irq(d2083->chip_irq);
	dev_info(d2083->dev, "IRQ Generated [d2083_irq_worker EXIT]\n");
}


static irqreturn_t d2083_irq(int irq, void *data)
{
	struct d2083 *d2083 = data;
	u8 reg_val;
	u8 sub_reg[D2083_NUM_IRQ_EVT_REGS] = {0,};
	int read_done[D2083_NUM_IRQ_EVT_REGS];
	struct d2083_irq_data *pIrq;
	int i;

	memset(&read_done, 0, sizeof(read_done));

	for (i = 0; i < ARRAY_SIZE(d2083_irqs); i++) {
		pIrq = &d2083_irqs[i];

		if (!read_done[pIrq->reg]) {
			d2083_reg_read(d2083, D2083_EVENTA_REG + pIrq->reg, &reg_val);
			sub_reg[pIrq->reg] = reg_val;
			d2083_reg_read(d2083, D2083_IRQMASKA_REG + pIrq->reg, &reg_val);
			sub_reg[pIrq->reg] &= ~reg_val;
			read_done[pIrq->reg] = 1;
		}

		if (sub_reg[pIrq->reg] & pIrq->mask) {
			d2083_irq_call_handler(d2083, i);
			/* Now clear EVENT registers */
			d2083_set_bits(d2083, D2083_EVENTA_REG + pIrq->reg, d2083_irqs[i].mask);
			//dev_info(d2083->dev, "\nIRQ Register [%d] MASK [%d]\n",D2083_EVENTA_REG + data->reg, d2083_irqs[i].mask);
		}
	}
	//enable_irq(d2083->chip_irq);
	
	/* DLG Test Print */
	dev_info(d2083->dev, "IRQ Generated [d2083_irq_worker EXIT]\n");
	return IRQ_HANDLED;
}


int d2083_register_irq(struct d2083 * const d2083, int const irq,
			irq_handler_t handler, unsigned long flags,
			const char * const name, void * const data)
{
	if (irq < 0 || irq >= D2083_NUM_IRQ || !handler)
		return -EINVAL;

	if (d2083->irq[irq].handler)
		return -EBUSY;
	mutex_lock(&d2083->irq_mutex);
	d2083->irq[irq].handler = handler;
	d2083->irq[irq].data = data;
	mutex_unlock(&d2083->irq_mutex);
	/* DLG Test Print */
    dev_info(d2083->dev, "\nIRQ After MUTEX UNLOCK [%s]\n", __func__);

	d2083_unmask_irq(d2083, irq);
	return 0;
}
EXPORT_SYMBOL_GPL(d2083_register_irq);

int d2083_free_irq(struct d2083 *d2083, int irq)
{
	if (irq < 0 || irq >= D2083_NUM_IRQ)
		return -EINVAL;

	d2083_mask_irq(d2083, irq);

	mutex_lock(&d2083->irq_mutex);
	d2083->irq[irq].handler = NULL;
	mutex_unlock(&d2083->irq_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(d2083_free_irq);

int d2083_mask_irq(struct d2083 *d2083, int irq)
{
	return d2083_set_bits(d2083, D2083_IRQMASKA_REG + d2083_irqs[irq].reg,
			       d2083_irqs[irq].mask);
}
EXPORT_SYMBOL_GPL(d2083_mask_irq);

int d2083_unmask_irq(struct d2083 *d2083, int irq)
{
	dev_info(d2083->dev, "\nIRQ[%d] Register [%d] MASK [%d]\n",irq, D2083_IRQMASKA_REG + d2083_irqs[irq].reg, d2083_irqs[irq].mask);
	return d2083_clear_bits(d2083, D2083_IRQMASKA_REG + d2083_irqs[irq].reg,
				 d2083_irqs[irq].mask);
}
EXPORT_SYMBOL_GPL(d2083_unmask_irq);

int d2083_irq_init(struct d2083 *d2083, int irq,
		    struct d2083_platform_data *pdata)
{
	int ret = -EINVAL;
	int reg_data, maskbit;

	if (!irq) {
	    dev_err(d2083->dev, "No IRQ configured \n");
	    return -EINVAL;
	}
	reg_data = 0xFFFFFFFF;
	d2083_block_write(d2083, D2083_EVENTA_REG, 4, (u8 *)&reg_data);
	reg_data = 0;
	d2083_block_write(d2083, D2083_EVENTA_REG, 4, (u8 *)&reg_data);

	/* Clear Mask register starting with Mask A*/
	maskbit = 0xFFFFFFFF;
	d2083_block_write(d2083, D2083_IRQMASKA_REG, 4, (u8 *)&maskbit);

	mutex_init(&d2083->irq_mutex);

	if (irq) {
		ret = request_threaded_irq(irq, NULL, d2083_irq, IRQF_DISABLED|IRQF_TRIGGER_FALLING|IRQF_NO_SUSPEND,
				  "d2083", d2083);
		if (ret != 0) {
			dev_err(d2083->dev, "Failed to request IRQ: %d\n", irq);
			return ret;
		}
 		dev_info(d2083->dev,"\n ######### d2083 isr(%d) configured #########\n", irq);
	} else {
		dev_err(d2083->dev, "No IRQ configured\n");
		return ret;
	}

	d2083->chip_irq = irq;
	return ret;
}

int d2083_irq_exit(struct d2083 *d2083)
{
	free_irq(d2083->chip_irq, d2083);
	return 0;
}
