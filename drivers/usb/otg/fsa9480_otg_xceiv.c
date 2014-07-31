/*****************************************************************************
*  Copyright 2001 - 2011 Broadcom Corporation.  All rights reserved.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/bcmpmu.h>
#include <linux/io.h>
#include <mach/io_map.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/bcm_hsotgctrl.h>
#include "fsa9480_otg_xceiv.h"
//#include "fsa9480_otg_adp.h"
#include <linux/usb/gadget_cust.h>

#include <linux/fsa9480.h>

#define OTGCTRL1_VBUS_ON 0xDC
#define OTGCTRL1_VBUS_OFF 0xD8

#define HOST_TO_PERIPHERAL_DELAY_MS 1000
#define PERIPHERAL_TO_HOST_DELAY_MS 100
#define USBLDO_RAMP_UP_DELAY_IN_MS 2

#define DELAY_OF_USB_DETECTION 100

enum fsa9480_event_t {
	/* events for usb driver */
	FSA9480_USB_EVENT_USB_DETECTION,
	FSA9480_USB_EVENT_IN,
	FSA9480_USB_EVENT_RM,
	FSA9480_USB_EVENT_ADP_CHANGE,
	FSA9480_USB_EVENT_ADP_SENSE_END,
	FSA9480_USB_EVENT_ADP_CALIBRATION_DONE,
	FSA9480_USB_EVENT_ID_CHANGE,
	FSA9480_USB_EVENT_VBUS_VALID,
	FSA9480_USB_EVENT_VBUS_INVALID,
	FSA9480_USB_EVENT_SESSION_VALID,
	FSA9480_USB_EVENT_SESSION_INVALID,
	FSA9480_USB_EVENT_SESSION_END_INVALID,
	FSA9480_USB_EVENT_SESSION_END_VALID,
	FSA9480_USB_EVENT_VBUS_OVERCURRENT,
	FSA9480_USB_EVENT_RIC_C_TO_FLOAT,
	FSA9480_USB_EVENT_CHGDET_LATCH,
	/* events for battery charging */
	FSA9480_CHRGR_EVENT_CHGR_DETECTION,
	FSA9480_CHRGR_EVENT_CHRG_CURR_LMT,
	FSA9480_CHRGR_EVENT_CHRG_RESUME_VBUS,
	FSA9480_CHRGR_EVENT_MBOV,
	FSA9480_CHRGR_EVENT_USBOV,
	FSA9480_CHRGR_EVENT_MBTEMP,
	FSA9480_CHRGR_EVENT_EOC,
	FSA9480_CHRGR_EVENT_CHRG_STATUS,
	FSA9480_CHRGR_EVENT_CAPACITY,
	/* events for fuel gauge */
	FSA9480_FG_EVENT_FGC,
	/*Events for JIG*/
	FSA9480_JIG_EVENT_USB,
	FSA9480_JIG_EVENT_UART,
	FSA9480_EVENT_MAX,
};


struct fsa9480_event_notifier {
	u32 event_id;
	struct blocking_notifier_head notifiers;
};

static struct otg_transceiver *local_otg_xceiver;


struct fsa9480_event_notifier fsa9480_event[FSA9480_EVENT_MAX];
//dh0318.lee
static void send_usb_event( enum fsa9480_event_t ievent, void *para)
{
#if 0	
struct power_supply *ps;
	union power_supply_propval propval;
	struct bcmpmu_accy *paccy = (struct bcmpmu_accy *)pmu->accyinfo;

	if (paccy->usb_cb.callback != NULL) {
		paccy->usb_cb.callback(paccy->bcmpmu, event, para,
				       paccy->usb_cb.clientdata);
		pr_accy(FLOW, "%s, event=%d, val=%p\n", __func__, event, para);
	}
#endif	
	NPRINTK("ievent=%d\n",ievent);
	blocking_notifier_call_chain(&fsa9480_event[ievent].notifiers, ievent, para);
#if 0
	if (event != BCMPMU_USB_EVENT_USB_DETECTION)
		return;
	/* update power supply usb property  */
	pr_accy(FLOW, "%s, usb change, usb_type=0x%X\n",
		__func__, paccy->bcmpmu->usb_accy_data.usb_type);
	ps = power_supply_get_by_name("usb");
	if (ps) {
		propval.intval = paccy->bcmpmu->usb_accy_data.usb_type;
		ps->set_property(ps, POWER_SUPPLY_PROP_TYPE, &propval);
		if (paccy->det_state == USB_CONNECTED)
			propval.intval = 1;
		else
			propval.intval = 0;
		ps->set_property(ps, POWER_SUPPLY_PROP_ONLINE, &propval);

#if defined(CONFIG_TSP_SETTING_FOR_TA_USB_IN)
		if(propval.intval==1)
		{
			// usb in
			tsp_charger_type_status = 1;//TSP Charging[JG]
			set_tsp_for_ta_detect(1);//TSP Charging[JG]			
                }
		else
		{
			// usb out
			tsp_charger_type_status = 0;//TSP Charging[JG]
			set_tsp_for_ta_detect(0);//TSP Charging[JG]			

		}
#endif		
		power_supply_changed(ps);
	}
#endif	
}

void send_usb_attach_event()
{
	//	DWC_WORKQ_SCHEDULE(core_if->wq_otg, w_init_core,
	//	   core_if, "init core");
enum bcmpmu_usb_type_t usb_type = PMU_CHRGR_TYPE_SDP;

	NPRINTK("\n");
		send_usb_event( FSA9480_USB_EVENT_USB_DETECTION, &usb_type);
	
}
EXPORT_SYMBOL_GPL(send_usb_attach_event);

void send_usb_detach_event()
{
	//	DWC_WORKQ_SCHEDULE(core_if->wq_otg, w_init_core,
	//	   core_if, "init core");
//enum bcmpmu_usb_type_t usb_type = PMU_CHRGR_TYPE_NONE;
	NPRINTK("\n");
		send_usb_event(FSA9480_USB_EVENT_SESSION_INVALID, NULL);
	
}
EXPORT_SYMBOL_GPL(send_usb_detach_event);

int fsa9480_add_notifier(u32 event_id, struct notifier_block *notifier)
{
	
	if (unlikely(event_id >= FSA9480_EVENT_MAX)) {
		printk("%s: Invalid event id\n", __func__);
		return -EINVAL;
	}
	
	return blocking_notifier_chain_register(&fsa9480_event[event_id].notifiers, notifier);
}
EXPORT_SYMBOL_GPL(fsa9480_add_notifier);


int fsa9480_remove_notifier(u32 event_id, struct notifier_block *notifier)
{
	
	if (unlikely(event_id >= FSA9480_EVENT_MAX)) {
		printk("%s: Invalid event id\n", __func__);
		return -EINVAL;
	}

	return blocking_notifier_chain_unregister(&fsa9480_event[event_id].notifiers, notifier);
}

EXPORT_SYMBOL_GPL(fsa9480_remove_notifier);

static int fsa9480_otg_xceiv_set_vbus(struct otg_transceiver *otg, bool enabled)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(otg->dev);
	int stat;

	/* The order of these operations has temporarily been
	 * swapped due to overcurrent issue caused by slow I2C
	 * operations. I2C operations take >200ms to complete */
	bcm_hsotgctrl_phy_set_vbus_stat(enabled);

	if (enabled && fsa9480_otg_xceiv_check_id_gnd(xceiv_data)) {
		dev_info(xceiv_data->dev, "Turning on VBUS\n");
		xceiv_data->vbus_enabled = true;
#if 0
		stat = xceiv_data->bcmpmu->usb_set(xceiv_data->bcmpmu,BCMPMU_USB_CTRL_VBUS_ON_OFF, 1);
#else
		/*Samsung should add a code to set VBus on/off*/
#endif
	
	} else {
		dev_info(xceiv_data->dev, "Turning off VBUS\n");
		xceiv_data->vbus_enabled = false;
#if 0		
		stat =xceiv_data->bcmpmu->usb_set(xceiv_data->bcmpmu,BCMPMU_USB_CTRL_VBUS_ON_OFF, 0);
#else
	/*Samsung should add a code to set VBus on/off*/
#endif
	}

	if (stat < 0)
		dev_warn(xceiv_data->dev, "Failed to set VBUS\n");

	return stat;
}


bool fsa9480_otg_xceiv_check_id_gnd(struct fsa9480_otg_xceiv_data
				   *xceiv_data)
{
	unsigned int data = 0;
	bool id_gnd = false;
/*
	bcmpmu_usb_get(xceiv_data->bcmpmu, BCMPMU_USB_CTRL_GET_ID_VALUE, &data);
	id_gnd = (data == PMU_USB_ID_GROUND);
	//Samsung should add a code to check USB ID value from muic
	return id_gnd;
*/
	return id_gnd;	//return 'false' all the time. change this function if the model support OTG.
}


bool fsa9480_otg_xceiv_check_id_rid_a(struct fsa9480_otg_xceiv_data
				  *xceiv_data)
{
	unsigned int data = 0;
	bool id_rid_a = false;

	//bcmpmu_usb_get(xceiv_data->bcmpmu, BCMPMU_USB_CTRL_GET_ID_VALUE, &data);
	/*Samsung should add a code to check USB ID value from muic*/
	id_rid_a = (data == PMU_USB_ID_RID_A);

	return id_rid_a;
}


bool fsa9480_otg_xceiv_check_id_rid_b(struct fsa9480_otg_xceiv_data
				  *xceiv_data)
{
	unsigned int data = 0;
	bool id_rid_b = false;

	//bcmpmu_usb_get(xceiv_data->bcmpmu, BCMPMU_USB_CTRL_GET_ID_VALUE, &data);
	/*Samsung should add a code to check USB ID value from muic*/
	id_rid_b = (data == PMU_USB_ID_RID_B);

	return id_rid_b;
}

bool fsa9480_otg_xceiv_check_id_rid_c(struct fsa9480_otg_xceiv_data
				  *xceiv_data)
{
	unsigned int data = 0;
	bool id_rid_c = false;

	//bcmpmu_usb_get(xceiv_data->bcmpmu, BCMPMU_USB_CTRL_GET_ID_VALUE, &data);
	/*Samsung should add a code to check USB ID value from muic*/
	id_rid_c = (data == PMU_USB_ID_RID_C);

	return id_rid_c;
}

static void fsa9480_otg_xceiv_shutdown(struct otg_transceiver *otg)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(otg->dev);

	if (xceiv_data) {
		/* De-initialize OTG core and PHY */
		bcm_hsotgctrl_phy_deinit();
		xceiv_data->otg_xceiver.xceiver.state = OTG_STATE_UNDEFINED;
		if (!xceiv_data->otg_enabled) {
			if (wake_lock_active
			    (&xceiv_data->otg_xceiver.xceiver_wake_lock))
				wake_unlock(&xceiv_data->otg_xceiver.xceiver_wake_lock);

			if (xceiv_data->bcm_hsotg_regulator &&
				    xceiv_data->regulator_enabled) {
				/* This should have no effect for most of
				 * our platforms as "always on" parameter is set
				 */
				regulator_disable(xceiv_data->bcm_hsotg_regulator);
				xceiv_data->regulator_enabled = false;
			}
		}
	}
}



static void fsa9480_otg_xceiv_set_def_state(
	struct fsa9480_otg_xceiv_data *xceiv_data, bool default_host)
{

	if (xceiv_data) {
		if (default_host)
			xceiv_data->otg_xceiver.xceiver.state =
				    OTG_STATE_A_IDLE;
		else
			xceiv_data->otg_xceiver.xceiver.state =
				    OTG_STATE_B_IDLE;
	}

}
static int fsa9480_otg_xceiv_start(struct otg_transceiver *otg)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(otg->dev);
	bool id_default_host = false;

	if (xceiv_data) {

		if (!xceiv_data->otg_enabled) {
			wake_lock(&xceiv_data->otg_xceiver.xceiver_wake_lock);

			if (xceiv_data->bcm_hsotg_regulator &&
				    !xceiv_data->regulator_enabled) {
				regulator_enable(xceiv_data->bcm_hsotg_regulator);
				/* Give 2ms to ramp up USBLDO */
				mdelay(USBLDO_RAMP_UP_DELAY_IN_MS);
				xceiv_data->regulator_enabled = true;
			}
		}

		id_default_host = 	fsa9480_otg_xceiv_check_id_gnd(xceiv_data) ||
							fsa9480_otg_xceiv_check_id_rid_a(xceiv_data);
		/* Initialize OTG core and PHY */
		bcm_hsotgctrl_phy_init(!id_default_host);

		fsa9480_otg_xceiv_set_def_state(xceiv_data,id_default_host);

	} else
		return -EINVAL;

	return 0;
}

static int fsa9480_otg_xceiv_set_delayed_adp(struct otg_transceiver *otg)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(otg->dev);

	if (!xceiv_data)
		return -EINVAL;

	xceiv_data->otg_xceiver.otg_vbus_off = true;

	return 0;
}

static int fsa9480_otg_xceiv_set_srp_reqd_handler(struct otg_transceiver *otg)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(otg->dev);

	if (!xceiv_data)
		return -EINVAL;

	xceiv_data->otg_xceiver.otg_srp_reqd = true;

	return 0;
}


static int fsa9480_otg_xceiv_set_otg_enable(struct otg_transceiver *otg,
					   bool enable)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(otg->dev);

	if (!xceiv_data)
		return -EINVAL;

	xceiv_data->otg_enabled = enable;
	return 0;
}

static int fsa9480_otg_xceiv_pullup_on(struct otg_transceiver *otg, bool on)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(otg->dev);

	if (!xceiv_data)
		return -EINVAL;

	bcm_hsotgctrl_phy_set_non_driving(!on);

	return 0;
}

static void fsa9480_otg_xceiv_suspend_core(void)
{
	struct fsa9480_otg_xceiv_data *xceiv_data;

	if (NULL == local_otg_xceiver)
		return;

	xceiv_data = dev_get_drvdata(local_otg_xceiver->dev);
	atomic_notifier_call_chain(&xceiv_data->otg_xceiver.
			xceiver.notifier, USB_EVENT_SUSPEND_CORE, NULL);
}

static void fsa9480_otg_xceiv_wakeup_core(void)
{
	struct fsa9480_otg_xceiv_data *xceiv_data;

	if (NULL == local_otg_xceiver)
		return;

	xceiv_data = dev_get_drvdata(local_otg_xceiver->dev);
	atomic_notifier_call_chain(&xceiv_data->otg_xceiver.
			xceiver.notifier, USB_EVENT_WAKEUP_CORE, NULL);
}

static int fsa9480_otg_xceiv_set_suspend(struct otg_transceiver *otg,
					int suspend)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(otg->dev);

	if (!xceiv_data)
		return -EINVAL;

	if (!xceiv_data->otg_enabled && suspend)
		bcm_hsotgctrl_handle_bus_suspend(fsa9480_otg_xceiv_suspend_core,
					fsa9480_otg_xceiv_wakeup_core);

	return 0;
}
static void fsa9480_otg_xceiv_select_host_mode(struct fsa9480_otg_xceiv_data
					      *xceiv_data, bool enable)
{
	if (enable) {
		dev_info(xceiv_data->dev, "Switching to Host\n");
		xceiv_data->host = true;
		bcm_hsotgctrl_set_phy_off(false);
		msleep(PERIPHERAL_TO_HOST_DELAY_MS);
		bcm_hsotgctrl_phy_set_id_stat(false);
	} else {
		dev_info(xceiv_data->dev, "Switching to Peripheral\n");
		bcm_hsotgctrl_phy_set_id_stat(true);
		if (xceiv_data->host) {
			xceiv_data->host = false;
			msleep(HOST_TO_PERIPHERAL_DELAY_MS);
		}
	}
}

static int fsa9480_otg_xceiv_vbus_notif_handler(struct notifier_block *nb,
					       unsigned long value, void *data)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    container_of(nb, struct fsa9480_otg_xceiv_data,
			 fsa9480_otg_vbus_validity_notifier);
	bool vbus_status = 0;

	if (!xceiv_data)
		return -EINVAL;

#if 0
	bcmpmu_usb_get(xceiv_data->bcmpmu, BCMPMU_USB_CTRL_GET_VBUS_STATUS,
		       &vbus_status);
#else
	/*Samsung should add a code to get VBus status*/
#endif
//We assume that the vbus was down for this time, so it will call fsa9480_otg_vbus_a_invalid_work all the time
	queue_work(xceiv_data->fsa9480_otg_work_queue,
		   vbus_status 	? &xceiv_data->fsa9480_otg_vbus_valid_work 
		   				: &xceiv_data->fsa9480_otg_vbus_a_invalid_work);

	return 0;
}


static int fsa9480_otg_xceiv_chg_detection_notif_handler(struct notifier_block
							*nb,
							unsigned long value,
							void *data)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    container_of(nb, struct fsa9480_otg_xceiv_data,
			 fsa9480_otg_chg_detection_notifier);
	bool usb_charger_type = false;

	NPRINTK("\n");
	if (!xceiv_data || !data)
		return -EINVAL;

	usb_charger_type = *(unsigned int *)data ? true : false;

	NPRINTK("usb_charger_type=%d\n", usb_charger_type);
	if (usb_charger_type){
		queue_work(xceiv_data->fsa9480_otg_work_queue,
			   &xceiv_data->fsa9480_otg_chg_detect_work);
		}

	return 0;
}

static int fsa9480_otg_xceiv_id_chg_notif_handler(struct notifier_block *nb,
						 unsigned long value,
						 void *data)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    container_of(nb, struct fsa9480_otg_xceiv_data,
			 fsa9480_otg_id_chg_notifier);

	if (xceiv_data) {
		queue_work(xceiv_data->fsa9480_otg_work_queue,
			   &xceiv_data->fsa9480_otg_id_status_change_work);
	} else
		return -EINVAL;

	return 0;
}


static int fsa9480_otg_xceiv_set_peripheral(struct otg_transceiver *otg,
					   struct usb_gadget *gadget)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(otg->dev);
	int status = 0;
	bool id_default_host = false;

	dev_dbg(xceiv_data->dev, "Setting Peripheral\n");
	otg->gadget = gadget;

	id_default_host = fsa9480_otg_xceiv_check_id_gnd(xceiv_data) ||
		  fsa9480_otg_xceiv_check_id_rid_a(xceiv_data);

	if (!id_default_host) {
		if (xceiv_data->otg_enabled &&
			(fsa9480_otg_xceiv_check_id_rid_b(xceiv_data) == false)) { /* No SRP if RID_B */
			/* REVISIT. Shutdown uses sequence for lowest power
			 * and does not meet timing so don't do that in OTG mode
			 * for now. Just do SRP for ADP startup */
			fsa9480_otg_xceiv_do_srp(xceiv_data);
		} else {
			int data;
#if 0
			bcmpmu_usb_get(xceiv_data->bcmpmu,BCMPMU_USB_CTRL_GET_USB_TYPE, &data);
#else
		/*Samsung should add a code to get USB Type*/
#endif
			if ((data != PMU_USB_TYPE_SDP)
			    && (data != PMU_USB_TYPE_CDP)) {
				/* Shutdown the core */
				atomic_notifier_call_chain(&xceiv_data->otg_xceiver.xceiver.notifier,
							   USB_EVENT_NONE,
							   NULL);//calls dwc_otg_xceiv_nb_callback
			}
		}

	} else {
		bcm_hsotgctrl_phy_set_id_stat(false);
		/* Come up connected  */
		bcm_hsotgctrl_phy_set_non_driving(false);
	}

	return status;
}


static int fsa9480_otg_xceiv_set_vbus_power(struct otg_transceiver *otg,
					   unsigned int ma)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(otg->dev);

#if 0
	return bcmpmu_usb_set(xceiv_data->bcmpmu,BCMPMU_USB_CTRL_CHRG_CURR_LMT, ma);
#else
	/*Samsung should add a code to set vbus current limitation*/
#endif

}

static int fsa9480_otg_xceiv_set_host(struct otg_transceiver *otg,
				     struct usb_bus *host)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(otg->dev);
	int status = 0;

	dev_dbg(xceiv_data->dev, "Setting Host\n");
	otg->host = host;

	if (host) {
		if (xceiv_data->otg_enabled) {
			/* Wake lock forever in OTG build */
			wake_lock(&xceiv_data->otg_xceiver.xceiver_wake_lock);
			/* Do calibration probe */
#if 0 //ignore OTG functionality			
			fsa9480_otg_do_adp_calibration_probe(xceiv_data);
#endif	
		}
	

		if (fsa9480_otg_xceiv_check_id_gnd(xceiv_data) ||
			  fsa9480_otg_xceiv_check_id_rid_a(xceiv_data)) {
			bcm_hsotgctrl_phy_set_id_stat(false);
			bcm_hsotgctrl_phy_set_non_driving(false);
		} else
			bcm_hsotgctrl_phy_set_id_stat(true);
	}
	return status;
}


static ssize_t fsa9480_otg_xceiv_wake_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct usb_gadget *gadget;
	ssize_t result = 0;
	unsigned int val;
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(dev);
	int error;

	gadget = xceiv_data->otg_xceiver.xceiver.gadget;

	result = sscanf(buf, "%u\n", &val);
	if (result != 1) {
		result = -EINVAL;
	} else if (val == 0) {
		dev_warn(xceiv_data->dev, "Illegal value\n");
	} else {
		dev_info(xceiv_data->dev, "Waking up host\n");
		error = usb_gadget_wakeup(gadget);
		if (error)
			dev_err(xceiv_data->dev, "Failed to issue wakeup\n");
	}

	return result < 0 ? result : count;
}


static DEVICE_ATTR(wake, S_IWUSR, NULL, fsa9480_otg_xceiv_wake_store);

static ssize_t fsa9480_otg_xceiv_vbus_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", xceiv_data->vbus_enabled ? "1" : "0");
}

static ssize_t fsa9480_otg_xceiv_vbus_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct usb_hcd *hcd;
	ssize_t result = 0;
	unsigned int val;
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(dev);
	int error;

	hcd = bus_to_hcd(xceiv_data->otg_xceiver.xceiver.host);

	result = sscanf(buf, "%u\n", &val);
	if (result != 1) {
		result = -EINVAL;
	} else if (val == 0) {
		dev_info(xceiv_data->dev, "Clearing PORT_POWER feature\n");
		error = hcd->driver->hub_control(hcd, ClearPortFeature,
						 USB_PORT_FEAT_POWER, 1, NULL,
						 0);
		if (error)
			dev_err(xceiv_data->dev,
				"Failed to clear PORT_POWER feature\n");
	} else {
		dev_info(xceiv_data->dev, "Setting PORT_POWER feature\n");
		error = hcd->driver->hub_control(hcd, SetPortFeature,
						 USB_PORT_FEAT_POWER, 1, NULL,
						 0);
		if (error)
			dev_err(xceiv_data->dev,
				"Failed to set PORT_POWER feature\n");
	}

	return result < 0 ? result : count;
}

static DEVICE_ATTR(vbus, S_IRUGO | S_IWUSR, fsa9480_otg_xceiv_vbus_show,
		   fsa9480_otg_xceiv_vbus_store);


static ssize_t fsa9480_otg_xceiv_host_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", xceiv_data->host ? "1" : "0");
}

static ssize_t fsa9480_otg_xceiv_host_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	ssize_t result = 0;
	unsigned int val;
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(dev);

	result = sscanf(buf, "%u\n", &val);
	if (result != 1)
		result = -EINVAL;
	else
		fsa9480_otg_xceiv_select_host_mode(xceiv_data, !!val);

	return result < 0 ? result : count;
}


static DEVICE_ATTR(host, S_IRUGO | S_IWUSR, fsa9480_otg_xceiv_host_show,
		   fsa9480_otg_xceiv_host_store);

#ifdef CONFIG_USB_OTG
static void fsa9480_otg_xceiv_srp_failure_handler(unsigned long param)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    (struct fsa9480_otg_xceiv_data *)param;
	schedule_delayed_work(&xceiv_data->fsa9480_otg_delayed_adp_work,
			      msecs_to_jiffies(100));
}
static void fsa9480_otg_xceiv_sess_end_srp_timer_handler(unsigned long param)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    (struct fsa9480_otg_xceiv_data *)param;
	schedule_work(&xceiv_data->fsa9480_otg_sess_end_srp_work);
}

#endif

static void fsa9480_otg_xceiv_delayed_adp_handler(struct work_struct *work)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    container_of(work, struct fsa9480_otg_xceiv_data,
			 fsa9480_otg_delayed_adp_work.work);
	dev_info(xceiv_data->dev, "Do ADP probe\n");
#if 0 //ignore OTG functionality			
	fsa9480_otg_do_adp_probe(xceiv_data);
#endif
	xceiv_data->otg_xceiver.otg_vbus_off = false;
}


static void fsa9480_otg_xceiv_vbus_invalid_handler(struct work_struct *work)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    container_of(work, struct fsa9480_otg_xceiv_data,
			 fsa9480_otg_vbus_invalid_work);
	dev_info(xceiv_data->dev, "Vbus invalid\n");

	if (xceiv_data->otg_enabled) {
		/* Need to discharge Vbus quickly to session invalid level */
#if 0
		bcmpmu_usb_set(xceiv_data->bcmpmu, BCMPMU_USB_CTRL_DISCHRG_VBUS,1);
#else
	/*Samsung should add a code to discharge VBus*/
#endif
	}
}

static void fsa9480_otg_xceiv_vbus_valid_handler(struct work_struct *work)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    container_of(work, struct fsa9480_otg_xceiv_data,
			 fsa9480_otg_vbus_valid_work);
	dev_info(xceiv_data->dev, "Vbus valid\n");

#ifdef CONFIG_USB_OTG
	del_timer_sync(&xceiv_data->otg_xceiver.srp_failure_timer);
	del_timer_sync(&xceiv_data->otg_xceiver.sess_end_srp_timer);
#endif
}

static void fsa9480_otg_xceiv_vbus_a_invalid_handler(struct work_struct *work)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    container_of(work, struct fsa9480_otg_xceiv_data,
			 fsa9480_otg_vbus_a_invalid_work);

	dev_info(xceiv_data->dev, "A session invalid\n");

	if (!bcm_hsotgctrl_get_clk_count())
		bcm_hsotgctrl_en_clock(true);

	/* Inform the core of session invalid level  */
	bcm_hsotgctrl_phy_set_vbus_stat(false);

	if (xceiv_data->otg_enabled) {
		
		/* Stop Vbus discharge */
#if 0
		bcmpmu_usb_set(xceiv_data->bcmpmu, BCMPMU_USB_CTRL_DISCHRG_VBUS,0);
#else
	/*Samsung shoud add a code to stop Vbus discharge*/
#endif

		if (fsa9480_otg_xceiv_check_id_gnd(xceiv_data)) {
			/* Use n-1 method for ADP rise time comparison */
#if 0
			bcmpmu_usb_set(xceiv_data->bcmpmu,BCMPMU_USB_CTRL_SET_ADP_COMP_METHOD, 1);
#else
			/*Samsung should add a code to set ADP*/
#endif
			if (xceiv_data->otg_xceiver.otg_vbus_off)
				schedule_delayed_work(&xceiv_data->fsa9480_otg_delayed_adp_work,
					msecs_to_jiffies
					  (T_NO_ADP_DELAY_MIN_IN_MS));
			else{
			#if 0 //ignore OTG function
				fsa9480_otg_do_adp_probe(xceiv_data);
                        #endif
                        }
		} else if (!fsa9480_otg_xceiv_check_id_rid_a(xceiv_data)) {
			if (xceiv_data->otg_xceiver.otg_srp_reqd) {
				/* Start Session End SRP timer */
				xceiv_data->otg_xceiver.sess_end_srp_timer.
				    expires =
				    jiffies +
				    msecs_to_jiffies
				    (T_SESS_END_SRP_START_IN_MS);
				add_timer(&xceiv_data->otg_xceiver.
					  sess_end_srp_timer);
			} else{
			#if 0 //ignore OTG function
				fsa9480_otg_do_adp_sense(xceiv_data);
			#endif
				}
		}
	} else {
		bool id_default_host = false;

		id_default_host =
			fsa9480_otg_xceiv_check_id_gnd(xceiv_data) ||
			fsa9480_otg_xceiv_check_id_rid_a(xceiv_data);

		if (!id_default_host) {
			atomic_notifier_call_chain(&xceiv_data->otg_xceiver.xceiver.notifier,
						   USB_EVENT_NONE, NULL);//calls dwc_otg_xceiv_nb_callback()
		}
	}
}

void fsa9480_otg_xceiv_do_srp(struct fsa9480_otg_xceiv_data *xceiv_data)
{
#ifdef CONFIG_USB_OTG
	if (xceiv_data->otg_xceiver.xceiver.gadget &&
		    xceiv_data->otg_xceiver.xceiver.gadget->ops &&
		    xceiv_data->otg_xceiver.xceiver.gadget->ops->wakeup &&
		    xceiv_data->otg_enabled) {

		bool vbus_status = 0;


#if 0
		bcmpmu_usb_get(xceiv_data->bcmpmu,
			BCMPMU_USB_CTRL_GET_VBUS_STATUS, &vbus_status);
#else
		/*Samsugn should add a code to get the VBUS Status*/
#endif


		/* Should do SRP only if Vbus is not valid */
		if (!vbus_status) {
			bcm_hsotgctrl_phy_set_non_driving(false);
			/* Do SRP */
			xceiv_data->otg_xceiver.xceiver.gadget->
			    ops->wakeup(xceiv_data->otg_xceiver.xceiver.gadget);
			/* Start SRP failure timer to do ADP probes
			 * if it expires
			 */
			xceiv_data->otg_xceiver.srp_failure_timer.expires =
				  jiffies +
				  msecs_to_jiffies(T_SRP_FAILURE_MAX_IN_MS);
			add_timer(&xceiv_data->otg_xceiver.srp_failure_timer);

			/* SRP initiated. Clear the flag */
			xceiv_data->otg_xceiver.otg_srp_reqd = false;
		}
	}
#endif
}



static void fsa9480_otg_xceiv_sess_end_srp_handler(struct work_struct *work)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    container_of(work, struct fsa9480_otg_xceiv_data,
			 fsa9480_otg_sess_end_srp_work);

	fsa9480_otg_xceiv_do_srp(xceiv_data);
}



static void fsa9480_otg_xceiv_vbus_a_valid_handler(struct work_struct *work)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    container_of(work, struct fsa9480_otg_xceiv_data,
			 fsa9480_otg_vbus_a_valid_work);
	dev_info(xceiv_data->dev, "A session valid\n");
}


static void fsa9480_otg_xceiv_id_change_handler(struct work_struct *work)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    container_of(work, struct fsa9480_otg_xceiv_data,
			 fsa9480_otg_id_status_change_work);
	bool id_gnd = false;
	bool id_rid_a = false;
	bool id_rid_c = false;

	dev_info(xceiv_data->dev, "ID change detected\n");
	id_gnd = fsa9480_otg_xceiv_check_id_gnd(xceiv_data);
	id_rid_a = fsa9480_otg_xceiv_check_id_rid_a(xceiv_data);
	id_rid_c = fsa9480_otg_xceiv_check_id_rid_c(xceiv_data);

	bcm_hsotgctrl_phy_set_id_stat(!(id_gnd || id_rid_a));

	/* If ID is gnd, we need to turn on Vbus within 200ms
	 * If ID is RID_A/B/C/FLOAT then we should not turn it on
	 */
	fsa9480_otg_xceiv_set_vbus(&xceiv_data->otg_xceiver.xceiver, 
								id_gnd ? true : false);

	if (!id_rid_c)
		msleep(HOST_TO_PERIPHERAL_DELAY_MS);

	if (id_gnd || id_rid_a || id_rid_c) {
		bcm_hsotgctrl_phy_deinit();
		xceiv_data->otg_xceiver.xceiver.state = OTG_STATE_UNDEFINED;
		atomic_notifier_call_chain(&xceiv_data->otg_xceiver.xceiver.
					   notifier, USB_EVENT_ID, NULL);//calls dwc_otg_xceiv_nb_callback()
	}
}



static void fsa9480_otg_xceiv_chg_detect_handler(struct work_struct *work)
{
	struct fsa9480_otg_xceiv_data *xceiv_data =
	    container_of(work, struct fsa9480_otg_xceiv_data,fsa9480_otg_chg_detect_work);

	NPRINTK("otg_enabled=%d\n", xceiv_data->otg_enabled);
	if (xceiv_data->otg_enabled) {
		/* Core is already up so just set the Vbus status */
		bcm_hsotgctrl_phy_set_vbus_stat(true);
	} else {
		bool id_default_host = false;

		id_default_host =
			fsa9480_otg_xceiv_check_id_gnd(xceiv_data) ||
			fsa9480_otg_xceiv_check_id_rid_a(xceiv_data);

		if (!id_default_host && xceiv_data->otg_xceiver.xceiver.gadget){
			NPRINTK("Starting UDC Core\n");
			atomic_notifier_call_chain(&xceiv_data->otg_xceiver.xceiver.notifier,
						   USB_EVENT_VBUS, NULL);// then this calls dwc_otg_xceiv_nb_callback().
		} else if (!id_default_host && !xceiv_data->otg_xceiver.xceiver.gadget){
			msleep(DELAY_OF_USB_DETECTION);
			queue_work(xceiv_data->fsa9480_otg_work_queue,
				   &xceiv_data->fsa9480_otg_chg_detect_work);
			return ;
		}
	}
}

static void fsa9480_otg_free_regulator(struct fsa9480_otg_xceiv_data *xceiv_data)
{
	if (xceiv_data && xceiv_data->bcm_hsotg_regulator &&
		    xceiv_data->regulator_enabled) {
		regulator_disable(xceiv_data->bcm_hsotg_regulator);
		xceiv_data->regulator_enabled = false;
		regulator_put(xceiv_data->bcm_hsotg_regulator);
	}
}

static int __devinit fsa9480_otg_xceiv_probe(struct platform_device *pdev)
{
	int error = 0;
//dh0318.lee	
	int i=0;
	struct fsa9480_otg_xceiv_data *xceiv_data;
//	struct bcmpmu *bcmpmu = pdev->dev.platform_data;

	NPRINTK("++\n");

	xceiv_data = kzalloc(sizeof(*xceiv_data), GFP_KERNEL);
	if (!xceiv_data) {
		dev_warn(&pdev->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}
	for(i=0;i<FSA9480_EVENT_MAX;i++){
   		fsa9480_event[i].event_id = i;
		BLOCKING_INIT_NOTIFIER_HEAD(&fsa9480_event[i].notifiers);
	} 

	/* REVISIT: Currently there isn't a way to obtain
	 * regulator string associated with USB. Hardcode for now
	 */
	xceiv_data->bcm_hsotg_regulator =
		regulator_get(NULL, "hv2ldo_uc");

	if (IS_ERR(xceiv_data->bcm_hsotg_regulator)) {
		dev_warn(&pdev->dev, "Failed to get regulator handle\n");
		kfree(xceiv_data);
		return -ENODEV;
	}

	/* Enable USB LDO */
	regulator_enable(xceiv_data->bcm_hsotg_regulator);
	/* Give 2ms to ramp up USBLDO */
	mdelay(USBLDO_RAMP_UP_DELAY_IN_MS);
	xceiv_data->regulator_enabled = true;

	xceiv_data->dev = &pdev->dev;
//	xceiv_data->bcmpmu = bcmpmu;
	xceiv_data->otg_xceiver.xceiver.dev = xceiv_data->dev;
	xceiv_data->otg_xceiver.xceiver.label = "bcmpmu_otg_xceiv";
	xceiv_data->host = false;
	xceiv_data->vbus_enabled = false;

	/* Create a work queue for OTG work items */
	xceiv_data->fsa9480_otg_work_queue = create_workqueue("bcm_otg_events");
	if (xceiv_data->fsa9480_otg_work_queue == NULL) {
		dev_warn(&pdev->dev,
			 "BCM OTG events work queue creation failed\n");
		fsa9480_otg_free_regulator(xceiv_data);
		kfree(xceiv_data);
		return -ENOMEM;
	}

	/* Create one work item per deferrable function */
	INIT_WORK(&xceiv_data->fsa9480_otg_vbus_invalid_work,
		  fsa9480_otg_xceiv_vbus_invalid_handler);
	INIT_WORK(&xceiv_data->fsa9480_otg_vbus_valid_work,
		  fsa9480_otg_xceiv_vbus_valid_handler);
	INIT_WORK(&xceiv_data->fsa9480_otg_vbus_a_invalid_work,
		  fsa9480_otg_xceiv_vbus_a_invalid_handler);
	INIT_WORK(&xceiv_data->fsa9480_otg_vbus_a_valid_work,
		  fsa9480_otg_xceiv_vbus_a_valid_handler);
	INIT_WORK(&xceiv_data->fsa9480_otg_id_status_change_work,
		  fsa9480_otg_xceiv_id_change_handler);
	INIT_WORK(&xceiv_data->fsa9480_otg_chg_detect_work,
		  fsa9480_otg_xceiv_chg_detect_handler);
	INIT_WORK(&xceiv_data->fsa9480_otg_sess_end_srp_work,
		  fsa9480_otg_xceiv_sess_end_srp_handler);
	INIT_DELAYED_WORK(&xceiv_data->fsa9480_otg_delayed_adp_work,
			  fsa9480_otg_xceiv_delayed_adp_handler);

	xceiv_data->otg_xceiver.xceiver.state = OTG_STATE_UNDEFINED;
	xceiv_data->otg_xceiver.xceiver.set_vbus = fsa9480_otg_xceiv_set_vbus;
	xceiv_data->otg_xceiver.xceiver.set_peripheral = fsa9480_otg_xceiv_set_peripheral;
	xceiv_data->otg_xceiver.xceiver.set_power =	fsa9480_otg_xceiv_set_vbus_power;
	xceiv_data->otg_xceiver.xceiver.set_host = fsa9480_otg_xceiv_set_host;
	xceiv_data->otg_xceiver.xceiver.shutdown = fsa9480_otg_xceiv_shutdown;
	xceiv_data->otg_xceiver.xceiver.init = fsa9480_otg_xceiv_start;
	xceiv_data->otg_xceiver.xceiver.set_delayed_adp =  fsa9480_otg_xceiv_set_delayed_adp;
	xceiv_data->otg_xceiver.xceiver.set_srp_reqd =  fsa9480_otg_xceiv_set_srp_reqd_handler;
	xceiv_data->otg_xceiver.xceiver.set_otg_enable = fsa9480_otg_xceiv_set_otg_enable;
	xceiv_data->otg_xceiver.xceiver.pullup_on = fsa9480_otg_xceiv_pullup_on;
	xceiv_data->otg_xceiver.xceiver.set_suspend =fsa9480_otg_xceiv_set_suspend;

	ATOMIC_INIT_NOTIFIER_HEAD(&xceiv_data->otg_xceiver.xceiver.notifier);
	/*
			after initializing xceiv_data->otg_xceiver.xceiver.notifier by  ATOMIC_INIT_NOTIFIER_HEAD(),
			dwc_otg_cil_init() registers core_if->otg_xceiv_nb to have dwc_otg_xceiv_nb_callback
			by calling atomic_notifier_chain_register(). So fsa9480_otg_xceive can deliver a event to broadcom otg driver
	*/

	xceiv_data->fsa9480_otg_vbus_validity_notifier.notifier_call =
	    fsa9480_otg_xceiv_vbus_notif_handler;

	
	/*Samsung Shoud add a code to register the notification block
	BCMPMU_USB_EVENT_VBUS_VALID
	BCMPMU_USB_EVENT_SESSION_INVALID
	*/
	fsa9480_add_notifier(FSA9480_USB_EVENT_VBUS_VALID,
			    &xceiv_data->fsa9480_otg_vbus_validity_notifier);
	fsa9480_add_notifier(FSA9480_USB_EVENT_SESSION_INVALID,
			    &xceiv_data->fsa9480_otg_vbus_validity_notifier);


	xceiv_data->fsa9480_otg_id_chg_notifier.notifier_call =
	    fsa9480_otg_xceiv_id_chg_notif_handler;
	
	/*
		Samsung Shoud add a code to register the notification block
		BCMPMU_USB_EVENT_ID_CHANGE
	*/
	fsa9480_add_notifier(FSA9480_USB_EVENT_ID_CHANGE,
			    &xceiv_data->fsa9480_otg_id_chg_notifier);

	xceiv_data->fsa9480_otg_chg_detection_notifier.notifier_call =
	    fsa9480_otg_xceiv_chg_detection_notif_handler;
	
	/*Samsung Should add a code to register the */
	fsa9480_add_notifier(FSA9480_USB_EVENT_USB_DETECTION,
			    &xceiv_data->fsa9480_otg_chg_detection_notifier);

	wake_lock_init(&xceiv_data->otg_xceiver.xceiver_wake_lock,
		       WAKE_LOCK_SUSPEND, "otg_xcvr_wakelock");

#ifdef CONFIG_USB_OTG
	init_timer(&xceiv_data->otg_xceiver.srp_failure_timer);
	xceiv_data->otg_xceiver.srp_failure_timer.data =
	    (unsigned long)xceiv_data;
	xceiv_data->otg_xceiver.srp_failure_timer.function =
	    fsa9480_otg_xceiv_srp_failure_handler;

	init_timer(&xceiv_data->otg_xceiver.sess_end_srp_timer);
	xceiv_data->otg_xceiver.sess_end_srp_timer.data =
	    (unsigned long)xceiv_data;
	xceiv_data->otg_xceiver.sess_end_srp_timer.function =
	    fsa9480_otg_xceiv_sess_end_srp_timer_handler;

	error = fsa9480_otg_adp_init(xceiv_data);
	if (error)
		goto error_attr_host;
#endif

	otg_set_transceiver(&xceiv_data->otg_xceiver.xceiver);

	platform_set_drvdata(pdev, xceiv_data);

	error = device_create_file(&pdev->dev, &dev_attr_host);
	if (error) {
		dev_warn(&pdev->dev, "Failed to create HOST file\n");
		goto error_attr_host;
	}

	error = device_create_file(&pdev->dev, &dev_attr_vbus);
	if (error) {
		dev_warn(&pdev->dev, "Failed to create VBUS file\n");
		goto error_attr_vbus;
	}

	error = device_create_file(&pdev->dev, &dev_attr_wake);
	if (error) {
		dev_warn(&pdev->dev, "Failed to create WAKE file\n");
		goto error_attr_wake;
	}

	/* Check if we should default to A-device */
	xceiv_data->otg_xceiver.xceiver.default_a =
	    fsa9480_otg_xceiv_check_id_gnd(xceiv_data) ||
	    fsa9480_otg_xceiv_check_id_rid_a(xceiv_data);

	fsa9480_otg_xceiv_set_def_state(xceiv_data,	xceiv_data->otg_xceiver.xceiver.default_a);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	NPRINTK("--\n");

	return 0;

error_attr_wake:
	device_remove_file(xceiv_data->dev, &dev_attr_vbus);

error_attr_vbus:
	device_remove_file(xceiv_data->dev, &dev_attr_host);

error_attr_host:
	wake_lock_destroy(&xceiv_data->otg_xceiver.xceiver_wake_lock);
	destroy_workqueue(xceiv_data->fsa9480_otg_work_queue);
	fsa9480_otg_free_regulator(xceiv_data);
	kfree(xceiv_data);
	return error;
}

static int __exit fsa9480_otg_xceiv_remove(struct platform_device *pdev)
{
	struct fsa9480_otg_xceiv_data *xceiv_data = platform_get_drvdata(pdev);

	xceiv_data->otg_xceiver.xceiver.state = OTG_STATE_UNDEFINED;
	device_remove_file(xceiv_data->dev, &dev_attr_wake);
	device_remove_file(xceiv_data->dev, &dev_attr_vbus);
	device_remove_file(xceiv_data->dev, &dev_attr_host);

	pm_runtime_disable(&pdev->dev);

	if (wake_lock_active(&xceiv_data->otg_xceiver.xceiver_wake_lock)) {
		wake_unlock(&xceiv_data->otg_xceiver.xceiver_wake_lock);
		wake_lock_destroy(&xceiv_data->otg_xceiver.xceiver_wake_lock);
	}

#if 0
	/* Remove notifiers */
	bcmpmu_remove_notifier(BCMPMU_USB_EVENT_VBUS_VALID,
			       &xceiv_data->bcm_otg_vbus_validity_notifier);
	bcmpmu_remove_notifier(BCMPMU_USB_EVENT_SESSION_INVALID,
			       &xceiv_data->bcm_otg_vbus_validity_notifier);
	bcmpmu_remove_notifier(BCMPMU_USB_EVENT_ID_CHANGE,
			       &xceiv_data->bcm_otg_id_chg_notifier);
	bcmpmu_remove_notifier(BCMPMU_USB_EVENT_USB_DETECTION,
			       &xceiv_data->bcm_otg_chg_detection_notifier);
#else
	/*Samsung should add a code to remove the events */
	fsa9480_remove_notifier(FSA9480_USB_EVENT_VBUS_VALID,
			       &xceiv_data->fsa9480_otg_vbus_validity_notifier);
	fsa9480_remove_notifier(FSA9480_USB_EVENT_SESSION_INVALID,
			       &xceiv_data->fsa9480_otg_vbus_validity_notifier);
	fsa9480_remove_notifier(FSA9480_USB_EVENT_ID_CHANGE,
			       &xceiv_data->fsa9480_otg_id_chg_notifier);
	fsa9480_remove_notifier(FSA9480_USB_EVENT_USB_DETECTION,
			       &xceiv_data->fsa9480_otg_chg_detection_notifier);

#endif

	destroy_workqueue(xceiv_data->fsa9480_otg_work_queue);
	fsa9480_otg_free_regulator(xceiv_data);
	kfree(xceiv_data);
	bcm_hsotgctrl_phy_deinit();
	return 0;
}

static int fsa9480_otg_xceiv_runtime_suspend(struct device *dev)
{
	int status = 0;
	struct fsa9480_otg_xceiv_data *xceiv_data = dev_get_drvdata(dev);

	if ((xceiv_data->otg_xceiver.xceiver.state !=
	     OTG_STATE_UNDEFINED) || xceiv_data->otg_enabled) {
		/* Don't allow runtime suspend if USB is active
		 * or in OTG mode */
		status = -EBUSY;
	} else {
		/* Allow runtime suspend since USB is not active */
		status = 0;
	}

	return status;
}

static int fsa9480_otg_xceiv_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops fsa9480_otg_xceiv_pm_ops = {
	.runtime_suspend = fsa9480_otg_xceiv_runtime_suspend,
	.runtime_resume = fsa9480_otg_xceiv_runtime_resume,
};

static struct platform_driver fsa9480_otg_xceiv_driver = {
	.probe = fsa9480_otg_xceiv_probe,
	.remove = __exit_p(fsa9480_otg_xceiv_remove),
	.driver = {
		   .name = "fsa9480_otg_xceiv",
		   .owner = THIS_MODULE,
		   .pm = &fsa9480_otg_xceiv_pm_ops,
		   },
};

static int __init fsa9480_otg_xceiv_init(void)
{
	pr_info("FSA9480 USB OTG Transceiver Driver\n");

	return platform_driver_register(&fsa9480_otg_xceiv_driver);
}

subsys_initcall(fsa9480_otg_xceiv_init);

static void __exit fsa9480_otg_xceiv_exit(void)
{
	platform_driver_unregister(&fsa9480_otg_xceiv_driver);
}

module_exit(fsa9480_otg_xceiv_exit);

MODULE_AUTHOR("Samsung");
MODULE_DESCRIPTION("FSA9480 USB OTG transceiver driver");
MODULE_LICENSE("GPL");
