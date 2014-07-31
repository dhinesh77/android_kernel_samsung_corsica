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
#include <linux/init.h>
#include <linux/bug.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/fs.h>

#include <linux/mfd/bcmpmu.h>

#define BATT_TYPE "SDI_SDI"

#define BCMPMU_PRINT_ERROR (1U << 0)
#define BCMPMU_PRINT_INIT (1U << 1)
#define BCMPMU_PRINT_FLOW (1U << 2)
#define BCMPMU_PRINT_DATA (1U << 3)

#define CONFIG_SEC_BATT_EXT_ATTRS 1

static int debug_mask = BCMPMU_PRINT_ERROR | BCMPMU_PRINT_INIT;
#define pr_batt(debug_level, args...) \
	do { \
		if (debug_mask & BCMPMU_PRINT_##debug_level) { \
			pr_info(args); \
		} \
	} while (0)

struct ss_batt_status {
	int capacity;
	int voltage;
	int temp;
	int temp_adc;
	int present;
	int capacity_lvl;
	int status;
	int health;
	int lp_charging;
	int charging_source;
};

struct bcmpmu_batt {
	struct bcmpmu *bcmpmu;
	struct power_supply batt;
	//struct bcmpmu_batt_state state;
	struct ss_batt_status state;
	wait_queue_head_t wait;
	struct mutex lock;
	char model[30];
	int batt_temp_in_celsius;
};

static void bcmpmu_batt_isr(enum bcmpmu_irq irq, void *data)
{
	struct bcmpmu_batt *pbatt = (struct bcmpmu_batt *)data;
  struct bcmpmu *bcmpmu = pbatt->bcmpmu;

	switch (irq) {
	case PMU_IRQ_BATRM:
		pbatt->state.present = 0;
		break;
	case PMU_IRQ_BATINS:
		pbatt->state.present = 1;
		break;
	case PMU_IRQ_MBOV:
		pbatt->state.health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		break;
	case PMU_IRQ_MBOV_DIS:
		pbatt->state.health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case PMU_IRQ_MBTEMPHIGH:
		pr_debug("bcmpmu_batt_isr: PMU_IRQ_MBTEMPHIGH triggered.\n");
		pbatt->state.health = POWER_SUPPLY_HEALTH_OVERHEAT;
		power_supply_changed(&pbatt->batt);
		break;
	case PMU_IRQ_MBTEMPLOW:
		pr_debug("bcmpmu_batt_isr: PMU_IRQ_MBTEMPLOW triggered \n");
		pbatt->state.health = POWER_SUPPLY_HEALTH_COLD;
		power_supply_changed(&pbatt->batt);
		break;
	case PMU_IRQ_CHGERRDIS:
		pr_debug("bcmpmu_batt_isr: PMU_IRQ_CHGERRDIS triggered \n");
		pbatt->state.health = POWER_SUPPLY_HEALTH_GOOD;
		power_supply_changed(&pbatt->batt);
		break;
	default:
		break;
	}
}

static enum power_supply_property bcmpmu_batt_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_BATT_TEMP_ADC,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int bcmpmu_get_batt_property(struct power_supply *battery,
		enum power_supply_property property,
		union power_supply_propval *propval)
{
	int ret = 0;
	struct bcmpmu_batt *pbatt =
		container_of(battery, struct bcmpmu_batt, batt);

	switch (property) {
	case POWER_SUPPLY_PROP_STATUS:
		propval->intval = pbatt->state.status;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		propval->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		propval->intval = pbatt->state.capacity;
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		propval->intval = pbatt->state.capacity_lvl;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		propval->intval = pbatt->state.voltage * 1000;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		propval->intval = pbatt->state.temp;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		propval->intval = pbatt->state.health;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		propval->intval = pbatt->state.present;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		propval->strval = pbatt->model;
		break;

	case POWER_SUPPLY_PROP_BATT_TEMP_ADC:
		propval->intval = pbatt->state.temp_adc;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int bcmpmu_set_batt_property(struct power_supply *ps,
		enum power_supply_property property,
		const union power_supply_propval *propval)
{
	int ret = 0;
	struct bcmpmu_batt *pbatt = container_of(ps,
		struct bcmpmu_batt, batt);
	switch (property) {
	case POWER_SUPPLY_PROP_STATUS:
		pbatt->state.status = propval->intval;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		pbatt->state.capacity = propval->intval;
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		pbatt->state.capacity_lvl = propval->intval;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		pbatt->state.voltage = propval->intval;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		pbatt->state.temp = propval->intval;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		pbatt->state.present = propval->intval;
		break;

	case POWER_SUPPLY_PROP_BATT_TEMP_ADC:
		pbatt->state.temp_adc = propval->intval;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
#ifdef CONFIG_MFD_BCMPMU_DBG
static ssize_t
dbgmsk_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "debug_mask is %x\n", debug_mask);
}

static ssize_t
dbgmsk_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 0);
	if (val > 0xFF || val == 0)
		return -EINVAL;
	debug_mask = val;
	return count;
}

static DEVICE_ATTR(dbgmsk, 0644, dbgmsk_show, dbgmsk_set);
static struct attribute *bcmpmu_batt_dbg_attrs[] = {
	&dev_attr_dbgmsk.attr,
	NULL
};
static const struct attribute_group bcmpmu_batt_dbg_attr_group = {
	.attrs = bcmpmu_batt_dbg_attrs,
};
#endif

static ssize_t
reset_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct bcmpmu *bcmpmu = dev->platform_data;
	unsigned long val = simple_strtoul(buf, NULL, 0);
	if ((val == 1) && (bcmpmu->em_reset))
		bcmpmu->em_reset(bcmpmu);	
   
	return count;
}

static ssize_t
reset_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct bcmpmu *bcmpmu = dev->platform_data;
	int status = 0;
	if (bcmpmu->em_reset_status)
		status = bcmpmu->em_reset_status(bcmpmu);
	return sprintf(buf, "%d\n", status);
}


static DEVICE_ATTR(reset, 0644, reset_show, reset_store);
static struct attribute *bcmpmu_batt_attrs[] = {
	&dev_attr_reset.attr,
	NULL
};
static const struct attribute_group bcmpmu_batt_attr_group = {
	.attrs = bcmpmu_batt_attrs,
};

#if defined(CONFIG_SEC_BATT_EXT_ATTRS)
enum
{
	SS_BATT_LP_CHARGING,
	SS_BATT_CHARGING_SOURCE,
	SS_BATT_TEMP_AVER,
	SS_BATT_TEMP_ADC_AVER,
	SS_BATT_TYPE,
	SS_BATT_READ_ADJ_SOC,
	SS_BATT_RESET_SOC,
};

static ssize_t ss_batt_ext_attrs_show(struct device *pdev, struct device_attribute *attr, char *buf);
static ssize_t ss_batt_ext_attrs_store(struct device *pdev, struct device_attribute *attr, const char *buf, size_t count);

static struct device_attribute ss_batt_ext_attrs[]=
{
	__ATTR(batt_lp_charging, 0644, ss_batt_ext_attrs_show, ss_batt_ext_attrs_store),
	__ATTR(batt_charging_source, 0644, ss_batt_ext_attrs_show, ss_batt_ext_attrs_store),
	__ATTR(batt_temp_aver, 0644, ss_batt_ext_attrs_show, ss_batt_ext_attrs_store),
	__ATTR(batt_temp_adc_aver, 0644, ss_batt_ext_attrs_show, ss_batt_ext_attrs_store),
	__ATTR(batt_type, 0644, ss_batt_ext_attrs_show, NULL),
	__ATTR(batt_read_adj_soc, 0644, ss_batt_ext_attrs_show , NULL),
	__ATTR(batt_reset_soc, 0664, NULL, ss_batt_ext_attrs_store),
};

static ssize_t ss_batt_ext_attrs_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	ssize_t count=0;
	int lp_charging=0;

	struct bcmpmu *bcmpmu=pdev->parent->platform_data;
	struct bcmpmu_batt *pbatt=(struct bcmpmu_batt *)bcmpmu->battinfo;

	const ptrdiff_t off = attr-ss_batt_ext_attrs;


	struct power_supply *ps;
	union power_supply_propval propval;
	propval.intval=0;
	propval.strval=0;

	ps=power_supply_get_by_name("charger");

	if(ps==0)
	{
		printk("%s: Failed to get power_supply charger\n",__func__);
		return 0;
	}

	switch(off)
	{
		case SS_BATT_LP_CHARGING:
			//lp_charging = (0 != pbatt->state.lp_charging)? 1:0;
			lp_charging = pbatt->state.lp_charging;
			count+=scnprintf(buf+count, PAGE_SIZE-count, "%d\n", lp_charging);
			break;
		case SS_BATT_CHARGING_SOURCE:
			{
				unsigned int charger_type=0;
				unsigned int bc_status=0;
				bc_status=bcmpmu->accy_info_get(bcmpmu, SS_ACCY_GET_BC_STATUS);
				switch(bc_status)
				{
					case PMU_CHRGR_TYPE_DCP:
						charger_type=POWER_SUPPLY_TYPE_MAINS;
					case PMU_CHRGR_TYPE_SDP:
					case PMU_CHRGR_TYPE_CDP:
						charger_type=POWER_SUPPLY_TYPE_USB;
						break;
					default:
						break;
				}
				count+=scnprintf(buf+count, PAGE_SIZE-count, "%d\n", charger_type);
			}
			break;
		case SS_BATT_TEMP_AVER:
			{
				int i=0;
				int temp_aver=0;

				for(i=0; i<5 ; i++)
				{
					temp_aver+=pbatt->state.temp;
					temp_aver/=5;
					msleep(100);
				}

				count+=scnprintf(buf+count, PAGE_SIZE-count, "%d\n", temp_aver);
			}
			break;
		case SS_BATT_TEMP_ADC_AVER:
			{
				int i=0;
				int temp_adc_aver=0;

				for(i=0 ; i < 5 ; i++)
				{
					temp_adc_aver+=pbatt->state.temp_adc;
					temp_adc_aver/=5;
					msleep(100);
				}

				count+=scnprintf(buf+count, PAGE_SIZE-count, "%d\n", temp_adc_aver);
			}
			break;
		case SS_BATT_TYPE:
			count+=scnprintf(buf+count, PAGE_SIZE-count, "%s\n", BATT_TYPE);
			break;
		case SS_BATT_READ_ADJ_SOC:
			count+=scnprintf(buf+count, PAGE_SIZE-count, "%d\n", pbatt->state.capacity);
			break;
		default:
			break;
	}


	return count;
}

static ssize_t ss_batt_ext_attrs_store(struct device *pdev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct bcmpmu *bcmpmu=pdev->parent->platform_data;
	struct bcmpmu_batt *pbatt=(struct bcmpmu_batt *)bcmpmu->battinfo;

	const ptrdiff_t off = attr-ss_batt_ext_attrs;

	struct power_supply *ps;
	union power_supply_propval propval;
	propval.intval=0;
	propval.strval=0;

	ps=power_supply_get_by_name("charger");

	if(ps==0)
	{
		printk("%s: Failed to get power_supply charger\n",__func__);
	return 0;
}

	switch(off)
	{
		case SS_BATT_RESET_SOC:
			{
				count=0;

				unsigned long val = simple_strtoul(buf, NULL, 0);
				if ((val == 1) && (bcmpmu->em_reset))
					bcmpmu->em_reset(bcmpmu);	
			}
			break;
		default:
			break;
	}

	return count;
}

// poweroff charging : check for charging mode.
//DEVICE_ATTR(batt_lp_charging, 0644, ss_batt_ext_attrs_show, ss_batt_ext_attrs_store);
//DEVICE_ATTR(batt_charging_source, 0644, ss_batt_ext_attrs_show, ss_batt_ext_attrs_store);

unsigned int lp_boot_mode;
static int get_boot_mode(char *str)
{
	get_option(&str, &lp_boot_mode);

	return 1;
}
__setup("lpcharge=",get_boot_mode);

#endif

static int __devinit bcmpmu_batt_probe(struct platform_device *pdev)
{
	int ret;

	struct bcmpmu *bcmpmu = pdev->dev.platform_data;
	struct bcmpmu_batt *pbatt;
	struct bcmpmu_platform_data *pdata = bcmpmu->pdata;
	
	printk("bcmpmu_batt: batt_probe called \n") ;

	pbatt = kzalloc(sizeof(struct bcmpmu_batt), GFP_KERNEL);
	if (pbatt == NULL) {
		printk("bcmpmu_batt: failed to alloc mem.\n") ;
		return -ENOMEM;
	}
	init_waitqueue_head(&pbatt->wait);
	mutex_init(&pbatt->lock);
	pbatt->bcmpmu = bcmpmu;
	bcmpmu->battinfo = (void *)pbatt;

	pbatt->state.health = POWER_SUPPLY_HEALTH_GOOD;
	pbatt->batt.properties = bcmpmu_batt_props;
	pbatt->batt.num_properties = ARRAY_SIZE(bcmpmu_batt_props);
	pbatt->batt.get_property = bcmpmu_get_batt_property;
	pbatt->batt.set_property = bcmpmu_set_batt_property;
	pbatt->batt.name = "battery";
	pbatt->batt.type = POWER_SUPPLY_TYPE_BATTERY;
	strlcpy(pbatt->model, pdata->batt_model, sizeof(pbatt->model));
	pbatt->batt_temp_in_celsius = pdata->batt_temp_in_celsius;
	ret = power_supply_register(&pdev->dev, &pbatt->batt);
	if (ret)
		goto err;

	pbatt->batt.dev->platform_data = (void *)(pbatt->bcmpmu);

	//pbatt->state.lp_charging=pbatt->bcmpmu->ss_check_lp_charging(pbatt->bcmpmu);
	pbatt->state.lp_charging=lp_boot_mode;

	#if defined(CONFIG_SEC_BATT_EXT_ATTRS)
	{
		int i=0;
		for(i=0; i < ARRAY_SIZE(ss_batt_ext_attrs) ; i++)
		{
			device_create_file(pbatt->batt.dev, &ss_batt_ext_attrs[i]);
		}
	}
	#endif

	// hot temp. for stop : 60'C, 0x3F
	bcmpmu->write_dev_drct(bcmpmu, 0, 0x13, 0x3F, 0xFF);
	// hot temp. for restore : 40'C, 0x78
	bcmpmu->write_dev_drct(bcmpmu, 0, 0x14, 0x78, 0xFF);
	// cold temp. for stop : -5'C, 0x21B => 0b10, 0x1B
	bcmpmu->write_dev_drct(bcmpmu, 0, 0x16, 0x1B, 0xFF);
	// cold temp. for restore : 0'C, 0x1CF => 0b01, 0xCF
	bcmpmu->write_dev_drct(bcmpmu, 0, 0x15, 0xCF, 0xFF);
	// set high 2bit for cold temp.
	// NTCHT_rise / NTCHT_fall / NTCCT_rise / NTCCT_fall
	//	00			00				01			10      => 0x06
	bcmpmu->write_dev_drct(bcmpmu, 0, 0x1D, 0x6, 0xFF);

	bcmpmu->register_irq(bcmpmu, PMU_IRQ_BATINS, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_BATRM, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_GBAT_PLUG_IN, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_SMPL_INT, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_MBTEMPLOW, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_MBTEMPHIGH, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_CHGERRDIS, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_MBOV, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_MBOV_DIS, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_MBWV_R_10S_WAIT, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_BBLOW, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_LOWBAT, bcmpmu_batt_isr, pbatt);
	bcmpmu->register_irq(bcmpmu, PMU_IRQ_VERYLOWBAT, bcmpmu_batt_isr, pbatt);

	bcmpmu->unmask_irq(bcmpmu, PMU_IRQ_BATINS);
	bcmpmu->unmask_irq(bcmpmu, PMU_IRQ_BATRM);
	bcmpmu->unmask_irq(bcmpmu, PMU_IRQ_MBOV);
	bcmpmu->unmask_irq(bcmpmu, PMU_IRQ_MBOV_DIS);

#ifdef CONFIG_MFD_BCMPMU_DBG
	ret = sysfs_create_group(&pdev->dev.kobj, &bcmpmu_batt_dbg_attr_group);
#endif
	ret = sysfs_create_group(&pdev->dev.kobj, &bcmpmu_batt_attr_group);

	return 0;

err:
	return ret;
}

static int __devexit bcmpmu_batt_remove(struct platform_device *pdev)
{
	struct bcmpmu *bcmpmu = pdev->dev.platform_data;
	struct bcmpmu_batt *pbatt = bcmpmu->battinfo;

	power_supply_unregister(&pbatt->batt);

	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_BATINS);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_BATRM);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_GBAT_PLUG_IN);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_SMPL_INT);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_MBTEMPLOW);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_MBTEMPHIGH);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_CHGERRDIS);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_MBOV);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_MBOV_DIS);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_MBWV_R_10S_WAIT);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_BBLOW);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_LOWBAT);
	bcmpmu->unregister_irq(bcmpmu, PMU_IRQ_VERYLOWBAT);

	#if defined(CONFIG_SEC_BATT_EXT_ATTRS)
	{
		int i=0;
		for(i=0; i < ARRAY_SIZE(ss_batt_ext_attrs) ; i++)
		{
			device_remove_file(pbatt->batt.dev, &ss_batt_ext_attrs[i]);
		}
	}
	#endif
#ifdef CONFIG_MFD_BCMPMU_DBG
	sysfs_remove_group(&pdev->dev.kobj, &bcmpmu_batt_attr_group);
#endif
	sysfs_remove_group(&pdev->dev.kobj, &bcmpmu_batt_attr_group);

	return 0;
}

static struct platform_driver bcmpmu_batt_driver = {
	.driver = {
		.name = "bcmpmu_batt",
	},
	.probe = bcmpmu_batt_probe,
	.remove = __devexit_p(bcmpmu_batt_remove),
};

static int __init bcmpmu_batt_init(void)
{
	return platform_driver_register(&bcmpmu_batt_driver);
}
module_init(bcmpmu_batt_init);

static void __exit bcmpmu_batt_exit(void)
{
	platform_driver_unregister(&bcmpmu_batt_driver);
}
module_exit(bcmpmu_batt_exit);

MODULE_DESCRIPTION("BCM PMIC battery driver");
MODULE_LICENSE("GPL");
