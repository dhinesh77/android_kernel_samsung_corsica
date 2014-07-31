/*
 * Driver for AuthenTec fingerprint sensor
 *
 * Copyright (C) 2011 AuthenTec, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/freezer.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/completion.h>
#include <linux/irq.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/authfp_defs.h>
#include <linux/authfp_dev.h>
#include <linux/authfp.h>

#define ABORT_SEQ_TAG_WRITE 0x54495257
#define ABORT_SEQ_TAG_READ 0x44414552
#define ABORT_SEQ_TAG_DELAY 0x59414c44
#define ABORT_SEQ_TAG_END 0x2e444e45
#define MAX_READ_ON_ABORT (42*1024)
#define MAX_TIME_OUT (1000)

#define MAX_USER_BUF_LEN (1024*100)

#if defined(CONFIG_HAS_EARLYSUSPEND)
#define ES_NONE                 0 
#define ES_RESUMED           1
#define ES_SUSPENDING_OP_ABORTED           2
#define ES_SUSPENDING_OP_NOT_ABORTED   3
#endif

#define FP_DEBUG 0

#if FP_DEBUG
#define FPDBG(fmt, args...) printk(KERN_INFO fmt, ## args)
#else
#define FPDBG(fmt, args...)
#endif

struct authfp_data {
	struct list_head list;
	struct miscdevice authfp_misc_dev;
	struct authfp_data_bus data_bus;
	struct device *dev;
	spinlock_t lock;
	struct mutex open_mutex;
	struct mutex ioctl_mutex;
	struct semaphore ist_semaphore;
	struct mutex sensor_access;
	struct input_dev *input_dev;
	struct authfp_platform_data platform_data;
	struct task_struct *ist_thread;
	void *power_context;
	s32 open_counter;
	bool operation_aborted;
	struct tr_init_params_v2 init_params;

	/*transfer information*/
	u32 bytes_expected;
	u32 n_shot;
	u32 read_policy;
	u32 options;

	u32 power_mode;
	u32 read_timeout; /*in milisecond*/

	struct list_head buffer_list;
	atomic_t int_signal;
	wait_queue_head_t wait_queue;

	struct completion ist_completion;
	bool terminate_ist;

	char dev_name[64];

#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
	wait_queue_head_t suspend_wait_queue;
	atomic_t earlysuspend_status;
#endif

	/*Flag to enable the feature*/
	bool enable_write_on_read;
	/*Flag to buffer writes if write_on_read feature enabled*/
	bool buffer_writes;

	void *extension;
};

struct authfp_packet {
	struct list_head list;
	size_t len;
	u8 data[1] __attribute__((aligned(32)));
};


LIST_HEAD(context_list);


static inline u32 check_and_read_u32(u8 **addr,
				u32 *buf_len, u32 *value)
{
	if (*buf_len < sizeof(u32))
		return 1;
	*buf_len -= sizeof(u32);
	*value = (u32)(((*addr)[3]<<24)|((*addr)[2]<<16)
		|((*addr)[1]<<8)|((*addr)[0]<<0));
	*addr += sizeof(u32);
	return 0;
}

static inline u32 read_u32(u8 **addr)
{
	u32 ret = (u32)(((*addr)[3]<<24)|((*addr)[2]<<16)
		|((*addr)[1]<<8)|((*addr)[0]<<0));
	*addr += sizeof(u32);
	return ret;
}

static inline void write_u32(u8 **addr, u32 value)
{
	**addr = (u8)(((value)>>0)&0xFF);
	*(*addr+1) = (u8)(((value)>>8)&0xFF);
	*(*addr+2) = (u8)(((value)>>16)&0xFF);
	*(*addr+3) = (u8)(((value)>>24)&0xFF);
	(*addr) += sizeof(u32);
	return;
}

int run_abort_sequence(struct authfp_data *data)
{
	int status = 0;
	u8 *cursor;
	u32 len_field;
	u32 length;
	u32 delayms;
	u8 *buf;
	bool done = false;
	struct authfp_data_bus *bus = &data->data_bus;

	if (!data->init_params.abort_sequence_len)
		return 0;

	cursor = (u8 *)data->init_params.abort_sequence;
	FPDBG("[AUTHFP] %s : 0x%x\n",__FUNCTION__, read_u32(&cursor));        
	while (!done) {
		switch (read_u32(&cursor)) {
		case  ABORT_SEQ_TAG_READ:
                	FPDBG( "[AUTHFP] %s : ABORT_SEQ_TAG_READ\n",__FUNCTION__);
			len_field = read_u32(&cursor);
			length = read_u32(&cursor);
			buf = kmalloc(length, GFP_KERNEL);
			if (!buf)
				return -ENOMEM;
			status = bus->bops->bus_read(
						bus->device,
						buf,
						length);
			kfree(buf);
			if (status)
				return -EIO;
			break;
		case ABORT_SEQ_TAG_WRITE:
                	printk(KERN_INFO "[AUTHFP] %s : ABORT_SEQ_TAG_WRITE\n",__FUNCTION__);
			len_field = read_u32(&cursor);
			status = bus->bops->bus_write(
						bus->device,
						cursor,
						len_field);
			if (status)
				return -EIO;
			cursor += len_field;
			break;
		case ABORT_SEQ_TAG_DELAY:
                	FPDBG("[AUTHFP] %s : ABORT_SEQ_TAG_DELAY\n",__FUNCTION__);            
			len_field = read_u32(&cursor);
			delayms = read_u32(&cursor);
			msleep(delayms);
			break;
		case ABORT_SEQ_TAG_END:
                	printk(KERN_INFO "[AUTHFP] %s : ABORT_SEQ_TAG_END\n",__FUNCTION__);                        
			done = true;
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

int authfp_suspend(struct device *dev)
{
	int status = 0;
	struct authfp_data *data = dev_get_drvdata(dev);
	u32 power_mode;

	printk(KERN_INFO "[AUTHFP] %s : power_mode: %d\n",__FUNCTION__, data->power_mode);
	
	if (mutex_lock_interruptible(&data->sensor_access)) {
		dev_err(dev, "failed to acquire the mutex\n");
		return -EINTR;
	}

	if (data->enable_write_on_read)
		data->data_bus.bops->bus_disable_write_buffering(
			data->data_bus.device, true);

        if (data->power_mode == POWER_MODE_ACTIVE){
		data->operation_aborted = 1;        
#if defined(CONFIG_HAS_EARLYSUSPEND)
		atomic_set(&data->earlysuspend_status, ES_SUSPENDING_OP_ABORTED);
#endif
	}
#if defined(CONFIG_HAS_EARLYSUSPEND)
	else {
		atomic_set(&data->earlysuspend_status, ES_SUSPENDING_OP_NOT_ABORTED);
	}
#endif	

	if (data->power_mode == POWER_MODE_ACTIVE) {

            status = run_abort_sequence(data);
            power_mode = POWER_MODE_SLEEP;

		if (!status)
			data->power_mode = power_mode;
	}

	mutex_unlock(&data->sensor_access);
	return status;
}

int authfp_resume(struct device *dev)
{
	printk(KERN_INFO "[AUTHFP] %s\n",__FUNCTION__);    
	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void authfp_early_suspend(struct early_suspend *handler)
{
	struct authfp_data *data;
	data = container_of(handler, struct authfp_data, early_suspend);
	authfp_suspend(data->dev);
}

static void authfp_late_resume(struct early_suspend *handler)
{
	struct authfp_data *data;
	data = container_of(handler, struct authfp_data, early_suspend);
	authfp_resume(data->dev);
	atomic_set(&data->earlysuspend_status, ES_RESUMED);
	wake_up_interruptible(&data->suspend_wait_queue);
}
#endif

static void flush_packet_list(struct authfp_data *data)
{
	struct list_head temp;
	struct authfp_packet *p;
	struct authfp_packet *next;

	spin_lock(&data->lock);
	INIT_LIST_HEAD(&temp);
	list_splice(&data->buffer_list, &temp);
	INIT_LIST_HEAD(&data->buffer_list);
	spin_unlock(&data->lock);

	list_for_each_entry_safe(p, next, &temp, list) {
		kfree(p);
	}
}

static int convert_abort_sequence(struct tr_init_params_v2 *init_params)
{
	u8 aucAbortSeq[MAX_ABORT_SEQ_LEN_V1] = {0};
	u8 *pos = init_params->abort_sequence;

	if (!init_params->abort_sequence_len)
		return 0;

	if (init_params->abort_sequence_len > MAX_ABORT_SEQ_LEN_V1)
		return -EINVAL;

	memcpy(aucAbortSeq, init_params->abort_sequence,
		init_params->abort_sequence_len);

	write_u32(&pos, ABORT_SEQ_TAG_READ);
	write_u32(&pos, sizeof(u32));
	write_u32(&pos, 0);
	write_u32(&pos, ABORT_SEQ_TAG_WRITE);
	write_u32(&pos, init_params->abort_sequence_len);

	memcpy(pos, aucAbortSeq, init_params->abort_sequence_len);
	pos += init_params->abort_sequence_len;

	write_u32(&pos, ABORT_SEQ_TAG_END);

	init_params->abort_sequence_len = pos - init_params->abort_sequence;

	return 0;

}

static int check_abort_sequence(struct tr_init_params_v2 *init_params)
{
	u8 *cursor = (u8 *)init_params->abort_sequence;
	u32 abort_seq_len = init_params->abort_sequence_len;
	bool done = false;
	u32 len_field;
	u32 length;
	u32 delayms;
	u32 value = 0;
	u32 remaining = abort_seq_len;

	if (!abort_seq_len)
		return 0;

	if (abort_seq_len > MAX_ABORT_SEQ_LEN_V2)
		return -EINVAL;

	while (!done) {
		if (check_and_read_u32(&cursor, &remaining, &value))
			break;
		switch (value) {
		case  ABORT_SEQ_TAG_READ:
			if (check_and_read_u32(&cursor, &remaining, &len_field))
				break;
			if (len_field != sizeof(u32))
				return -EINVAL;
			if (check_and_read_u32(&cursor, &remaining, &length))
				break;
			if (length > (u32)MAX_READ_ON_ABORT)
				return -EINVAL;
			break;
		case ABORT_SEQ_TAG_WRITE:
			if (check_and_read_u32(&cursor, &remaining, &len_field))
				break;
			cursor += len_field;
			break;
		case ABORT_SEQ_TAG_DELAY:
			if (check_and_read_u32(&cursor, &remaining, &len_field))
				break;
			if (len_field != sizeof(u32))
				return -EINVAL;
			if (check_and_read_u32(&cursor, &remaining, &delayms))
				break;
			if (delayms > (u32)MAX_TIME_OUT)
				return -EINVAL;
			break;
		case ABORT_SEQ_TAG_END:
			done = true;
			break;
		default:
			return -EINVAL;
		}
	}

	return done ? 0 : -EINVAL;

}

static int ioctl_reset(struct authfp_data *data,
	struct tr_reset_ioctl_buffer *ioctl_buf)
{

	if (data->enable_write_on_read) {
		if (mutex_lock_interruptible(&data->sensor_access)) {
			dev_err(data->dev, "locking mutex failed\n");
			return -EINTR;
		}
		data->data_bus.bops->bus_disable_write_buffering(
					data->data_bus.device, true);
		mutex_unlock(&data->sensor_access);
	}

	spin_lock(&data->lock);
	data->operation_aborted = 0;
	data->bytes_expected = 0;
	data->n_shot = 0;
	data->read_policy = 0;
	data->options = 0;
	data->enable_write_on_read = false;
	data->buffer_writes = false;
	spin_unlock(&data->lock);

	atomic_set(&data->int_signal, 0);
	data->read_timeout = 0;

	flush_packet_list(data);

	return 0;
}

static int ioctl_disconnect(struct authfp_data *data,
	struct tr_connect_ioctl_buffer_v2 *ioctl_buf)
{
	int status = ioctl_reset(data, NULL);
	return status;
}

static int ioctl_connect(struct authfp_data *data,
	struct tr_connect_ioctl_buffer_v2 *ioctl_buf)
{
	int status = ioctl_reset(data, NULL);
	if (status)
		return status;

	data->enable_write_on_read =
		(ioctl_buf->protocol_id == FP_PROTOCOL_1750);

	if (ioctl_buf->params.version == 1) {
		memcpy(&data->init_params, &ioctl_buf->params,
			sizeof(struct tr_init_params_v1));
		status = convert_abort_sequence(&data->init_params);
	} else if (ioctl_buf->params.version == 2) {
		memcpy(&data->init_params, &ioctl_buf->params,
			sizeof(struct tr_init_params_v2));
		status = check_abort_sequence(&data->init_params);
	} else
		status = -EINVAL;

	return status;
}

static int ioctl_check_protocol(struct authfp_data *data,
	struct tr_check_protocol_ioctl_buffer *ioctl_buf)
{
	if (ioctl_buf->protocol_id == FP_PROTOCOL_1750
		|| ioctl_buf->protocol_id == FP_PROTOCOL_0850)
		return 0;

	dev_err(data->dev, "invalid protocol id %d\n",
		ioctl_buf->protocol_id);
	ioctl_buf->protocol_id = 0;
	return -EINVAL;
}

static int ioctl_get_device_info(struct authfp_data *data,
	struct tr_device_info_ioctl_buf *ioctl_buf)
{

	if (ioctl_buf->requested_version == 1) {
		struct driver_info_v1 info;

		if (ioctl_buf->buffer_size != sizeof(struct driver_info_v1)) {
			dev_err(data->dev,
				"invalid device info size %d\n",
				ioctl_buf->buffer_size);
			return -EINVAL;
		}

		info.version = 1;
		info.struct_size = sizeof(struct driver_info_v1);
		info.driver_version = AUTHFP_VERSION;
		info.interface_type = data->platform_data.interface_type;
		info.interface_speed = data->platform_data.interface_speed;
		info.support_sensor_poweroff
			= (data->platform_data.support_power_off) ? 1 : 0;
		memcpy(ioctl_buf->buffer, &info, sizeof(info));
		return 0;
	}

	return -EINVAL;
}

static int ioctl_get_device_power(struct authfp_data *data,
	struct tr_device_power_ioctl_buffer *ioctl_buf)
{
	ioctl_buf->power_mode = data->power_mode;
	return 0;
}

static int ioctl_set_device_power(struct authfp_data *data,
	struct tr_device_power_ioctl_buffer *ioctl_buf)
{
	u32 current_power_mode;
	u32 requested_power_mode = ioctl_buf->power_mode;
	u32 power_mode_to_save = requested_power_mode;
	int status = 0;

	if (mutex_lock_interruptible(&data->sensor_access)) {
		dev_err(data->dev, "mutex lock failed in setting\n");
		return -EINTR;
	}

	current_power_mode = data->power_mode;

	switch (requested_power_mode) {
	case POWER_MODE_OFF:
		if (data->platform_data.support_power_off) {
			status = authfp_power_off(data->power_context,
					&data->platform_data);
		} else {
			power_mode_to_save = POWER_MODE_SLEEP;
		}
		break;

	case POWER_MODE_SLEEP:
	case POWER_MODE_ACTIVE:
		if (POWER_MODE_OFF == current_power_mode) {
			status = authfp_power_on(data->power_context,
				&data->platform_data);
		}
		break;

	case POWER_MODE_STANDBY:
		status = 0;
		break;

	default:
		break;
	}

	if (status == 0)
		data->power_mode = power_mode_to_save;

	mutex_unlock(&data->sensor_access);
	return status;
}

static int ioctl_setup_transfer(struct authfp_data *data,
	struct tr_setup_transfer_ioctl_buffer *ioctl_buf)
{
	if (data->enable_write_on_read) {
		if (mutex_lock_interruptible(&data->sensor_access)) {
			dev_err(data->dev, "locking mutex failed\n");
			return -EINTR;
		}
		data->data_bus.bops->bus_disable_write_buffering(
					data->data_bus.device, false);
		mutex_unlock(&data->sensor_access);
	}

	spin_lock(&data->lock);
	data->bytes_expected = ioctl_buf->packet_len;
	data->n_shot = ioctl_buf->nshots;
	data->read_policy = ioctl_buf->read_policy;
	data->options = ioctl_buf->option_flags;
	data->buffer_writes = false;
	spin_unlock(&data->lock);

	flush_packet_list(data);

	return 0;
}

static int ioctl_cancel_transfer(struct authfp_data *data,
	struct tr_cancel_ioctl_buffer *ioctl_buf)
{
	wake_up_interruptible(&data->wait_queue);
	return 0;
}

static int ioctl_set_timeout(struct authfp_data *data,
	struct tr_timeout_ioctl_buffer *ioctl_buf)
{
	data->read_timeout = ioctl_buf->timeout;
	return 0;
}

static int ioctl_output_key_event(struct authfp_data *data,
	struct tr_key_event *event)
{
	input_report_key(data->input_dev, event->key_code, event->key_event);
        input_sync(data->input_dev);
        
        if(event->key_event == 0)
        {
            switch(event->key_code) {
                case KEY_UP:
                    printk(KERN_INFO "[AUTHFP] KEY_UP\n");
                break;
                case KEY_LEFT:
                    printk(KERN_INFO "[AUTHFP] KEY_LEFT\n");
                break;
                case KEY_RIGHT:
                    printk(KERN_INFO "[AUTHFP] KEY_RIGHT\n");
                break;
                case KEY_DOWN:
                    printk(KERN_INFO "[AUTHFP] KEY_DOWN\n");
                break;
                default:
                    printk(KERN_INFO "[AUTHFP] UnKnown keycode=%d\n", event->key_code);
                break;
            }
        }       
        
	return 0;
}

static int ioctl_ext(struct authfp_data *data,
	struct tr_ext_ioctl_buffer *ioctl_ext_buf)
{
	int status = 0;

	switch (ioctl_ext_buf->header.operation_id) {
	case TROPID_SEND_KEY_EVENT:
		status = ioctl_output_key_event(data,
				(struct tr_key_event *)ioctl_ext_buf->buffer);
		break;
	default:
		status = -EINVAL;
		break;
	}
	return status;
}

static int authfp_open(struct inode *inode, struct file *file)
{
	int status;
	struct authfp_data *data = NULL;

	printk(KERN_INFO "[AUTHFP] %s\n",__FUNCTION__);

	list_for_each_entry(data, &context_list, list) {
		if (data->authfp_misc_dev.minor == iminor(inode))
			break;
	}

	if (!data)
		return -EFAULT;

	file->private_data = data;

	if (mutex_lock_interruptible(&data->open_mutex)) {
		dev_err(data->dev, "mutex lock failed in open\n");
		return -EINTR;
	}

	if (data->open_counter == 0) {
		data->open_counter = 1;
		data->operation_aborted = 0;
		status = 0;
	} else {
		status = -EBUSY;
	}

	mutex_unlock(&data->open_mutex);

	return status;
}

static int authfp_release(struct inode *inode, struct file *file)
{
	struct authfp_data *data = file->private_data;

	printk(KERN_INFO "[AUTHFP] %s\n",__FUNCTION__);

	ioctl_disconnect(data, NULL);
	data->open_counter = 0;

	return 0;
}

static int get_packet(struct authfp_data *data, size_t len,
	struct authfp_packet **out)
{
	int status;
	struct authfp_packet *data_packet;

	data_packet = kmalloc(offsetof(struct authfp_packet, data) + len,
			GFP_KERNEL);
	if (!data_packet)
		return -ENOMEM;

	INIT_LIST_HEAD(&data_packet->list);
	data_packet->len = len;

	if (mutex_lock_interruptible(&data->sensor_access)) {
		dev_err(data->dev, "mutex lock failed\n");
		status = -EINTR;
		goto exit_get_packet;
	}

	if (data->power_mode != POWER_MODE_OFF)
		status = data->data_bus.bops->bus_read(data->data_bus.device,
					data_packet->data, len);
	else
		status = -EIO;

	mutex_unlock(&data->sensor_access);

exit_get_packet:

	if (status) {
		kfree(data_packet);
		data_packet = 0;
	}

	*out = data_packet;
	return status;
}

static int ready_for_read(struct authfp_data *data, u32 policy, u32 options)
{
	int r = 0;

	if (options & TR_FLAG_RETURN_INTERRUPT_PACKET)
		r = atomic_read(&data->int_signal);

	if (!r && (policy == RP_READ_ON_INT)) {
		spin_lock(&data->lock);
		r = !list_empty(&data->buffer_list);
		spin_unlock(&data->lock);
	}

	return r;
}

static int write_packet(char *buf, size_t buf_size,
		const u8 *data, size_t data_size)
{
	int status = 0;

	if (data_size > buf_size || data_size > MAX_USER_BUF_LEN)
		return -EFAULT;

	status = copy_to_user(buf, data, data_size);
	if (status != 0)
		return -EFAULT;
	else
		return data_size;
}

static ssize_t authfp_read(struct file *file, char *buf,
	size_t count, loff_t *f_pos)
{
	struct authfp_data *data = file->private_data;
	struct authfp_packet *data_packet = NULL;
	int status = 0;
	u32 bytes_expected;
	u32 options;
	u32 policy;

	if (data->operation_aborted)
		return -EAGAIN;

	spin_lock(&data->lock);
	bytes_expected = data->bytes_expected;
	options = data->options;
	policy = data->read_policy;
	spin_unlock(&data->lock);

	if (policy != RP_READ_ON_REQ) {
		status = wait_event_interruptible_timeout(data->wait_queue,
					ready_for_read(data, policy, options),
					msecs_to_jiffies(data->read_timeout));
		if (status >= 0)
			status = 0;
		else
			return 0;
	}

	if ((status == 0) && (options & TR_FLAG_RETURN_INTERRUPT_PACKET)) {

		if (atomic_xchg(&data->int_signal, 0)) {
			static const u8 int_packet[] = { 0x80, 0x2f, 0x5a,
				0x0a, 0x00, 0x00, 0x00, 0x16, 0x01, 0x01 };
			status = write_packet(buf, count,
				int_packet, sizeof(int_packet));
		}
	}

	if ((status == 0) && (policy == RP_READ_ON_INT)) {

		spin_lock(&data->lock);

		if (!list_empty(&data->buffer_list)) {
			data_packet = list_entry(data->buffer_list.next,
						struct authfp_packet, list);
			list_del(data->buffer_list.next);
		}

		spin_unlock(&data->lock);

		if (data_packet) {
			if (data_packet->len <= count) {
				status = write_packet(buf, count,
					data_packet->data, data_packet->len);
			} else {
				dev_err(data->dev,
					"data larger than requested\n");
				status = -EFAULT;
			}
			kfree(data_packet);
		}   
	}

	if ((status == 0) && (policy == RP_READ_ON_REQ)) {

		status = get_packet(data, bytes_expected, &data_packet);
		if (!status) {
			status = write_packet(buf, count,
					data_packet->data, data_packet->len);
			kfree(data_packet);
		}
	}

	return status;
}

static ssize_t authfp_write(struct file *file, const char  *buf,
	size_t count, loff_t *f_pos)
{
	struct authfp_data *data = file->private_data;
	u8 *tx_buf = 0;
	int status = 0;

	if (data->operation_aborted) {
		dev_err(data->dev, "operation aborted\n");
		status = -EAGAIN;
		goto exit;
	}

	if (count > MAX_USER_BUF_LEN) {
		dev_err(data->dev, "Writing exceeds maximum\n");
		status = -EINVAL;
		goto exit;
	}

	tx_buf = kmalloc(count, GFP_KERNEL);
	if (!tx_buf) {
		dev_err(data->dev,
			"failed to kmalloc memory for tx buf\n");
		status = -ENOMEM;
		goto exit;
	}

	status = copy_from_user(tx_buf, buf, count);
	if (status != 0) {
		dev_err(data->dev,
			"failed to copy user data to tx buf\n");
		status = -EFAULT;
		goto write_exit;
	}
       
	if (mutex_lock_interruptible(&data->sensor_access)) {
		dev_err(data->dev, "mutex lock failed in write\n");
		status = -EINTR;
		goto write_exit;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	if (atomic_read(&data->earlysuspend_status)
			== ES_SUSPENDING_OP_NOT_ABORTED) {
		data->operation_aborted = 1;
		status = -EAGAIN;
		mutex_unlock(&data->sensor_access);
		goto write_exit;
	}
#endif

	if (data->power_mode != POWER_MODE_OFF) {
		if (data->enable_write_on_read
			&& data->buffer_writes)
			data->data_bus.bops->bus_enable_write_buffering(
				data->data_bus.device);

		status = data->data_bus.bops->bus_write(data->data_bus.device,
					tx_buf, count);
		if (data->enable_write_on_read
			&& !status && data->bytes_expected)
			data->buffer_writes = true;
	} else {
		dev_err(data->dev, "sensor is not active\n");
		status = -EIO;
	}

	mutex_unlock(&data->sensor_access);

	if (status != 0) {
		status = -EIO;
		goto write_exit;
	}

	status = count;

write_exit:
	kfree(tx_buf);
exit:
	return status;
}

static int check_ioctl_command(unsigned int cmd, unsigned long arg)
{
	u32 hdr_size = 0;
	u32 buf_size = 0;

	if (cmd == FP_IOCTL_GET_DEVINFO) {
		struct tr_device_info_ioctl_buf inf_header;
		hdr_size = offsetof(struct tr_device_info_ioctl_buf, buffer);

		buf_size = sizeof(inf_header);

		if (copy_from_user(&inf_header,
				(const void __user *)arg,
				buf_size))
			return -EINVAL;

		if (inf_header.header.size != inf_header.buffer_size+hdr_size)
			return -EINVAL;

		return 0;
	} else if (cmd == FP_IOCTL_TRANSPORT_EXT) {
		int i;
		struct tr_ext_ioctl_buffer ioctl_header;

		static const struct {
			unsigned int operation_id;
			unsigned int size;
		} operations[] = {
			{TROPID_SEND_MOUSE_EVENT,
				sizeof(struct tr_mouse_event)
				+ offsetof(struct tr_ext_ioctl_buffer, buffer)},
			{TROPID_SEND_KEY_EVENT,
				sizeof(struct tr_key_event)
				+ offsetof(struct tr_ext_ioctl_buffer, buffer)}
			};

		hdr_size = offsetof(struct tr_ext_ioctl_buffer, buffer);

		buf_size = sizeof(ioctl_header);

		if (copy_from_user(&ioctl_header,
				(const void __user *)arg, buf_size))
			return -EINVAL;

		if (ioctl_header.header.size != ioctl_header.buf_len+hdr_size)
			return -EINVAL;

		for (i = 0; i < ARRAY_SIZE(operations); i++) {
			if (operations[i].operation_id
				== ioctl_header.header.operation_id
				&& operations[i].size
				== ioctl_header.header.size)
				return 0;
		}

		return -EINVAL;
	} else {
		struct tr_ioctl_header header;
		int i;

		static const struct {
			unsigned int code;
			unsigned int size;
		} commands[] = {
			{FP_IOCTL_TRANSPORT_CONNECT,
				sizeof(struct tr_connect_ioctl_buffer_v1)},
			{FP_IOCTL_TRANSPORT_CONNECT,
				sizeof(struct tr_connect_ioctl_buffer_v2)},
			{FP_IOCTL_TRANSPORT_DISCONNECT,
				sizeof(struct tr_connect_ioctl_buffer_v1)},
			{FP_IOCTL_TRANSPORT_DISCONNECT,
				sizeof(struct tr_connect_ioctl_buffer_v2)},
			{FP_IOCTL_TRANSPORT_RESET,
				sizeof(struct tr_reset_ioctl_buffer)},
			{FP_IOCTL_CHECK_PROTOCOL,
				sizeof(struct tr_check_protocol_ioctl_buffer)},
			{FP_IOCTL_GET_DEVICE_POWER,
				sizeof(struct tr_device_power_ioctl_buffer)},
			{FP_IOCTL_SET_DEVICE_POWER,
				sizeof(struct tr_device_power_ioctl_buffer)},
			{FP_IOCTL_SETUP_TRANSFER,
				sizeof(struct tr_setup_transfer_ioctl_buffer)},
			{FP_IOCTL_CANCEL_TRANSFER,
				sizeof(struct tr_cancel_ioctl_buffer)},
			{FP_IOCTL_SET_TIMEOUT,
				sizeof(struct tr_timeout_ioctl_buffer)},
			};

		buf_size = sizeof(header);

		if (copy_from_user(&header,
				(const void __user *)arg, buf_size))
			return -EINVAL;

		for (i = 0; i < ARRAY_SIZE(commands); i++) {
			if (commands[i].code == cmd
				&& commands[i].size == header.size)
				return 0;
		}

		return -EINVAL;
	}

}

static long authfp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct tr_ioctl_header header;
	struct tr_ioctl_header *ioctl_buffer_header = NULL;
	int status;
	void *ioctl_buffer = NULL;
	u32 buf_size = 0;

	struct authfp_data *data = file->private_data;

	if (check_ioctl_command(cmd, arg)) {
		dev_err(data->dev, "invalid ioctl parameters\n");
		return -EINVAL;
	}

	if (cmd != FP_IOCTL_CANCEL_TRANSFER)
		if (mutex_lock_interruptible(&data->ioctl_mutex)) {
			dev_err(data->dev, "mutex lock failed\n");
			return -EINTR;
		}

	buf_size = sizeof(header);

	if (copy_from_user(&header, (const void __user *)arg, buf_size)) {
		status = -EFAULT;
		goto ioctl_out;
	}

	ioctl_buffer = kmalloc(header.size, GFP_KERNEL);
	if (!ioctl_buffer) {
		status = -ENOMEM;
		goto ioctl_out;
	}

	if (header.size > MAX_USER_BUF_LEN) {
		status = -EINVAL;
		goto ioctl_out;
	}

	if (copy_from_user(ioctl_buffer,
			(const void __user *)arg, header.size)) {
		status = -EFAULT;
		goto ioctl_out;
	}

	ioctl_buffer_header = (struct tr_ioctl_header *) ioctl_buffer;
	ioctl_buffer_header->ret_code = 0;

	if (data->operation_aborted) {
		if ((cmd != FP_IOCTL_TRANSPORT_RESET)
			&& (cmd != FP_IOCTL_TRANSPORT_DISCONNECT)
			&& (cmd != FP_IOCTL_TRANSPORT_CONNECT)) {
			status = -EAGAIN;
			goto ioctl_copy2user;
		}
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	if (atomic_read(&data->earlysuspend_status) 
			== ES_SUSPENDING_OP_NOT_ABORTED) {
		data->operation_aborted = 1;
		if (cmd != FP_IOCTL_TRANSPORT_RESET) {
			status = -EAGAIN;
			goto ioctl_copy2user;
		}
	}
#endif

	switch (cmd) {
	case FP_IOCTL_TRANSPORT_CONNECT:
		status = ioctl_connect(data, ioctl_buffer);
		break;
	case FP_IOCTL_TRANSPORT_DISCONNECT:
		status = ioctl_disconnect(data, ioctl_buffer);
		break;
	case FP_IOCTL_TRANSPORT_RESET:
	#if defined(CONFIG_HAS_EARLYSUSPEND)
		if (atomic_read(&data->earlysuspend_status) != ES_RESUMED) {
			status = wait_event_interruptible(data->suspend_wait_queue, 
						(atomic_read(&data->earlysuspend_status)==ES_RESUMED));
			if (status) {
				status = -EPERM;
				break;
			}
		}
	#endif
		status = ioctl_reset(data, ioctl_buffer);
		break;
	case FP_IOCTL_CHECK_PROTOCOL:
		status = ioctl_check_protocol(data, ioctl_buffer);
		break;
	case FP_IOCTL_GET_DEVINFO:
		status = ioctl_get_device_info(data, ioctl_buffer);
		break;
	case FP_IOCTL_GET_DEVICE_POWER:
		status = ioctl_get_device_power(data, ioctl_buffer);
		break;
	case FP_IOCTL_SET_DEVICE_POWER:
		status = ioctl_set_device_power(data, ioctl_buffer);
		break;
	case FP_IOCTL_SETUP_TRANSFER:
		status = ioctl_setup_transfer(data, ioctl_buffer);
		break;
	case FP_IOCTL_CANCEL_TRANSFER:
		status = ioctl_cancel_transfer(data, ioctl_buffer);;
		break;
	case FP_IOCTL_SET_TIMEOUT:
		status = ioctl_set_timeout(data, ioctl_buffer);
		break;
	case FP_IOCTL_TRANSPORT_EXT:
		status = ioctl_ext(data, ioctl_buffer);
		break;
	default:
		dev_err(data->dev, "invalid IOCTL\n");
		status = -EINVAL;
		break;
	};


ioctl_copy2user:
	if (ioctl_buffer_header->ret_code == 0)
		ioctl_buffer_header->ret_code = status;

	/* copy the data to the user */
	if (copy_to_user((void __user *)arg,
			(const void *)ioctl_buffer, header.size))  {
		dev_err(data->dev, "copy_to_user from ioctl failed\n");
		status = -EINVAL;
	}

ioctl_out:
	if (ioctl_buffer != NULL)
		kfree(ioctl_buffer);

	if (cmd != FP_IOCTL_CANCEL_TRANSFER)
		mutex_unlock(&data->ioctl_mutex);

	return status;

}



static irqreturn_t authfp_isr(int irq, void *dev_id)
{
	struct authfp_data *data = dev_id;
	int int_value;

        if(data->bytes_expected)
            printk(KERN_INFO "[AUTHFP] %s: %d\n",__FUNCTION__, data->bytes_expected);
        
	int_value = gpio_get_value(data->platform_data.interrupt_gpio);
	if (!int_value)
		return IRQ_NONE;

	up(&data->ist_semaphore);
	return IRQ_HANDLED;
}

static int authfp_ist(void *dev_id)
{
	int status;
	u32 bytes_expected;
	u32 options;
	u32 policy;
	struct authfp_data *data = dev_id;
	struct authfp_packet *packet;
	unsigned long flags;

	while (1) {

		status = down_interruptible(&data->ist_semaphore);

		if (status) {
			if (!try_to_freeze()) {
				spin_lock_irqsave(&current->sighand->siglock,
							flags);
				recalc_sigpending();
				spin_unlock_irqrestore(
					&current->sighand->siglock, flags);
			}
			continue;
		}
		if (data->terminate_ist) {
			complete_and_exit(&data->ist_completion, 0);
			break;
		}

		spin_lock(&data->lock);
		bytes_expected = data->bytes_expected;
		options = data->options;
		policy = data->read_policy;
		spin_unlock(&data->lock);

		if (options & TR_FLAG_RETURN_INTERRUPT_PACKET) {
			atomic_set(&data->int_signal, 1);
			wake_up_interruptible(&data->wait_queue);
		}

		switch (policy) {
		case RP_DO_NOT_READ:
		case RP_READ_ON_REQ:
			break;
		case RP_READ_ON_INT:
			status = get_packet(data, bytes_expected, &packet);
			if (!status) {
				spin_lock(&data->lock);
				list_add_tail(&packet->list,
					&data->buffer_list);
				spin_unlock(&data->lock);
				wake_up_interruptible(&data->wait_queue);
			}
			break;
		default:
			dev_err(data->dev, "unknown read policy %d\n",
				policy);
			break;

		}
	}

	return 0;
}

static int register_nav_device(struct authfp_data *data, char *name)
{
	int status = 0;
	int i;

	static const unsigned short authfp_keycodes[] = {
		KEY_LEFT,
		KEY_RIGHT,
		KEY_UP,
		KEY_DOWN,
		BTN_LEFT,
		BTN_RIGHT,
		KEY_ENTER,
		BTN_MOUSE
	};

	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		dev_err(data->dev, "input_allocate_device failed\n");
		return -ENOMEM;
	}

	data->input_dev->name = name;
	data->input_dev->phys = NULL;
	data->input_dev->id.bustype = BUS_HOST;
	data->input_dev->id.vendor = 0x0001;
	data->input_dev->id.product = 0x3344;
	data->input_dev->id.version = 0xae70;
	data->input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_REL);
	data->input_dev->relbit[0] = BIT(REL_X) | BIT(REL_Y);
	data->input_dev->keycode = (void *)authfp_keycodes;
	data->input_dev->keycodesize = sizeof(authfp_keycodes[0]);
	data->input_dev->keycodemax = ARRAY_SIZE(authfp_keycodes);
	for (i = 0; i < ARRAY_SIZE(authfp_keycodes); i++)
		set_bit(authfp_keycodes[i], data->input_dev->keybit);

	status = input_register_device(data->input_dev);
	if (status < 0) {
		dev_err(data->dev, "input_register_device failed: %d\n",
			status);
		input_free_device(data->input_dev);
		return status;
	}

	return 0;
}

static void unregister_nav_device(struct authfp_data *data)
{
	input_unregister_device(data->input_dev);
	input_free_device(data->input_dev);

	return;
}

void *authfp_get_extension(struct device *dev)
{
	return ((struct authfp_data *)dev_get_drvdata(dev))->extension;
}


const static struct file_operations authfp_fops = {
	.owner = THIS_MODULE,
	.open = authfp_open,
	.release = authfp_release,
	.read = authfp_read,
	.write = authfp_write,
	.unlocked_ioctl = authfp_ioctl,
};

static void authfp_uninit(struct authfp_data *data)
{

	struct authfp_data *pos;

#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif

	data->terminate_ist = 1;
	up(&data->ist_semaphore);
	wait_for_completion(&data->ist_completion);

	free_irq(gpio_to_irq(data->platform_data.interrupt_gpio), data);
	gpio_free(data->platform_data.interrupt_gpio);

	authfp_power_uninit(data->power_context, &data->platform_data);
	unregister_nav_device(data);
	misc_deregister(&data->authfp_misc_dev);
	flush_packet_list(data);

	list_for_each_entry(pos, &context_list, list) {
		if (pos == data) {
			list_del(&pos->list);
			break;
		}
	}

	kfree(data);
	return;
}

__devinit int authfp_probe(struct device *dev,
	const struct authfp_data_bus *data_bus,
	size_t extension_size)
{
	struct authfp_data *data = NULL;
	int status;
	int flags;
	u8 label[32] = {0};

	printk(KERN_INFO "[AUTHFP] %s\n",__FUNCTION__);
    
	data = kzalloc(sizeof(struct authfp_data) + extension_size, GFP_KERNEL);
	if (!data) {
		dev_err(dev, "failed to allocate authfp_data memory\n");
		status = -ENOMEM;
		goto out;
	}

	data->extension = (void *)(data + 1);
	data->data_bus = *data_bus;
	data->dev = dev;
	data->platform_data =
		*(struct authfp_platform_data *)dev->platform_data;

	spin_lock_init(&data->lock);
	mutex_init(&data->open_mutex);
	mutex_init(&data->ioctl_mutex);
	mutex_init(&data->sensor_access);
	sema_init(&data->ist_semaphore, 0);

	atomic_set(&data->int_signal, 0);
	init_waitqueue_head(&data->wait_queue);

	data->enable_write_on_read = false;
	data->buffer_writes = false;

	data->open_counter = 0;
	data->operation_aborted = 0;
	data->power_mode = data->platform_data.support_power_off ?
			POWER_MODE_OFF : POWER_MODE_SLEEP;
	INIT_LIST_HEAD(&data->buffer_list);

	if (data->platform_data.sensor_index == 0)
		memcpy((void *)data->dev_name,
			AUTHFP_NAME, sizeof(AUTHFP_NAME));
	else
		sprintf((void *)data->dev_name, "aes%d",
			data->platform_data.sensor_index);

	data->authfp_misc_dev.name = data->dev_name;
	data->authfp_misc_dev.minor = MISC_DYNAMIC_MINOR;
	data->authfp_misc_dev.fops = &authfp_fops;
	status = misc_register(&data->authfp_misc_dev);
	if (status) {
		dev_err(dev, "input_register_device failed: %d\n",
			status);
		status =  -ENODEV;
		goto out_free_drv_data;
	}

	/*INT GPIO */
	sprintf(label, "fp_int_%d", data->platform_data.sensor_index);
	status = gpio_request(data->platform_data.interrupt_gpio, label);
	if (status < 0) {
		dev_err(dev, "gpio_request failed: %d\n", status);
		status =  -ENODEV;
		goto out_deregister_misc;
	}
	status = gpio_direction_input(data->platform_data.interrupt_gpio);
	if (status < 0) {
		dev_err(dev, "gpio_direction_input failed: %d\n",
			status);
		status =  -ENODEV;
		goto out_free_gpio;
	}

	/* IRQ */
	flags = data->platform_data.interrupt_polarity ?
		IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
        
        irq_set_irq_type(gpio_to_irq(data->platform_data.interrupt_gpio), flags);
        
	status = request_irq(gpio_to_irq(data->platform_data.interrupt_gpio),
		authfp_isr, IRQF_SHARED | flags, "authfp_int", data);
	if (status < 0) {
		dev_err(dev, "request_irq failed: %d\n", status);
		status =  -ENODEV;
		goto out_free_gpio;
	}

	printk(KERN_INFO "[AUTHFP] IRQ_NO: %d,%d\n", gpio_to_irq(data->platform_data.interrupt_gpio), flags);

	status = authfp_power_init(&data->power_context, &data->platform_data);
	if (status != 0) {
		dev_err(dev, "authfp_power_init failed: %d\n",
			status);
		status =  -ENODEV;
		goto out_free_irq;
	}

	status = register_nav_device(data, data->dev_name);
	if (status < 0) {
		dev_err(dev, "register_nav_device failed: %d\n",
			status);
		status =  -ENODEV;
		goto out_power_uninit;
	}

	/*IST thread*/
	init_completion(&data->ist_completion);
	data->terminate_ist = 0;
	data->ist_thread = kthread_create(authfp_ist, data, "authfp_ist");
	if (IS_ERR(data->ist_thread)) {
		dev_err(dev, "failed to create thread\n");
		status = -ENODEV;
		goto out_unregister_nav_dev;
	}

	dev_set_drvdata(dev, data);
	list_add_tail(&data->list, &context_list);

	wake_up_process(data->ist_thread);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 10;
	data->early_suspend.suspend = authfp_early_suspend;
	data->early_suspend.resume = authfp_late_resume;
	register_early_suspend(&data->early_suspend);

	atomic_set(&data->earlysuspend_status, ES_NONE);
	init_waitqueue_head(&data->suspend_wait_queue);
#endif

	return 0;

out_unregister_nav_dev:
	unregister_nav_device(data);
out_power_uninit:
	authfp_power_uninit(data->power_context, &data->platform_data);
out_free_irq:
	free_irq(gpio_to_irq(data->platform_data.interrupt_gpio), data);
out_free_gpio:
	gpio_free(data->platform_data.interrupt_gpio);
out_deregister_misc:
	misc_deregister(&data->authfp_misc_dev);
out_free_drv_data:
	kfree(data);
out:
	return status;
}

int authfp_remove(struct device *dev)
{
	struct authfp_data *data = dev_get_drvdata(dev);

	authfp_uninit(data);

	return 0;
}



MODULE_DESCRIPTION("AuthenTec fingerprint sensor driver");
MODULE_AUTHOR("AuthenTec");
MODULE_LICENSE("GPL");
