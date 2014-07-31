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
#include <linux/spi/spi.h>
#include <linux/time.h>

#include <linux/authfp_defs.h>
#include <linux/authfp_dev.h>
#include <linux/authfp.h>


#define MAX_BUFFERED_LEN 1024

#define FP_DEBUG 0
#define FP_DEBUG_TIME 0

#if FP_DEBUG
#define FPDBG(fmt, args...) printk(KERN_INFO fmt, ## args)
#else
#define FPDBG(fmt, args...)
#endif

struct buffered_write_info {
	bool buffering_enabled;
	u8 buffered_bytes[MAX_BUFFERED_LEN];
	u32 buffered_len;
};

static u8 *authfp_spi_rx=NULL;
#if FP_DEBUG_TIME
static int secdif;
static int secdif2;
static struct timeval tv_time1;
static struct timeval tv_time2;
#endif

static int authfp_spi_exchange_frame(struct spi_device *spi,
		const u8 *tx, u8 *rx, unsigned len)
{
        int ret=0;
        
        FPDBG("[AUTHFP] %s : rx=0x%08x , len=%d\n",__FUNCTION__, (u32)rx, len);         
        
	if (rx == 0) 
        {
            struct spi_message m;
            struct spi_transfer t = {0};
#if FP_DEBUG_TIME
            do_gettimeofday(&tv_time1);
#endif
            spi_message_init(&m);

            t.tx_buf = tx;
            t.rx_buf = authfp_spi_rx;
            t.len = len;
            spi_message_add_tail(&t, &m);

            ret = spi_sync(spi, &m);
#if FP_DEBUG_TIME
            do_gettimeofday(&tv_time2);
            secdif = tv_time2.tv_usec - tv_time1.tv_usec;
            secdif2 = tv_time2.tv_sec - tv_time1.tv_sec;
            printk(KERN_INFO "[AUTHFP] spi len=%d sync time-write sec=%d, us=%d\n", len, secdif2, secdif);    
#endif
            if (ret !=0)
             printk(KERN_ERR "[AUTHFP] %s :  write ret=%d\n",__FUNCTION__, ret);    

            return ret; 
	}
        else 
        {
            struct spi_message m;
            struct spi_transfer t = {0};
#if FP_DEBUG_TIME
            do_gettimeofday(&tv_time1);
#endif
            spi_message_init(&m);
            t.tx_buf = tx;
            t.rx_buf = rx;
            t.len = len;
            spi_message_add_tail(&t, &m);

            ret = spi_sync(spi, &m);
#if FP_DEBUG_TIME
            do_gettimeofday(&tv_time2);
            secdif = tv_time2.tv_usec - tv_time1.tv_usec;
            secdif2 = tv_time2.tv_sec - tv_time1.tv_sec;
            printk(KERN_INFO "[AUTHFP] spi len=%d sync time-read sec=%d, us=%d\n", len, secdif2, secdif);    
#endif
            if (ret !=0)
                printk(KERN_ERR "[AUTHFP] %s :  read ret=%d\n",__FUNCTION__, ret);    

            return ret;
	}
}

static int authfp_spi_exchange(struct spi_device *spi, const u8 *tx,
		u8 *rx, size_t len, unsigned max_frame_size)
{
	int status = 0;
	unsigned i;
	unsigned frames;
	unsigned rest;

#if 0 //temp
        max_frame_size = 60; 
#endif

#if 0 //power consumption test
       for(i=0;i<len;i++)
       {
            if(i>7)
                break;
            printk(KERN_ERR "[AUTHFP] authfp_spi_exchange-send:  %0x\n", tx[i]);  
       }
#endif

	frames = len / max_frame_size;
	rest = len % max_frame_size;

	for (i = 0; i < frames; i++) {
		status = authfp_spi_exchange_frame(spi, tx, rx, max_frame_size);
		if (status)
			return status;

		if (tx)
			tx += max_frame_size;
		if (rx)
			rx += max_frame_size;
	}
	if (rest)
		status = authfp_spi_exchange_frame(spi, tx, rx, rest);

	return status;
}

static int authfp_spi_write(void *device, const u8 *buf, size_t len)
{
	int status = 0;
	struct spi_device *spi = (struct spi_device *) device;
	struct authfp_platform_data *platform_data;
	struct buffered_write_info *buffered_writes;

	platform_data = (struct authfp_platform_data *) spi->dev.platform_data;
	buffered_writes = (struct buffered_write_info *)authfp_get_extension(
				&((struct spi_device *)device)->dev);

	if (buffered_writes->buffering_enabled) {
		if (buffered_writes->buffered_len + len <= MAX_BUFFERED_LEN) {
			memcpy(buffered_writes->buffered_bytes
				+ buffered_writes->buffered_len,
				buf, len);
			buffered_writes->buffered_len += len;
		} else {
			return -ENOMEM;
		}
	} else {
		/*there are previously buffered write, then flush it*/
		if (buffered_writes->buffered_len) {
			status = authfp_spi_exchange(device,
					buffered_writes->buffered_bytes,
					0,
					buffered_writes->buffered_len,
					platform_data->max_frame_size);
			if (status)
				return status;

			buffered_writes->buffered_len = 0;
		}
		status = authfp_spi_exchange(device, buf, 0, len,
				platform_data->max_frame_size);
		}

	return status;
}

static int authfp_spi_read(void *device, u8 *buf, size_t len)
{
	struct spi_device *spi = (struct spi_device *) device;
	struct authfp_platform_data *platform_data;
	struct buffered_write_info *buffered_writes;

	platform_data = (struct authfp_platform_data *) spi->dev.platform_data;
	buffered_writes = (struct buffered_write_info *)authfp_get_extension(
				&((struct spi_device *)device)->dev);

	memset(buf, 0xff, len);

	if (buffered_writes->buffered_len) {
		/*only tunnel buffered writes into read when they fit a read*/
		if (buffered_writes->buffered_len <= len) {
			memcpy(buf, buffered_writes->buffered_bytes,
				buffered_writes->buffered_len);
			buffered_writes->buffered_len = 0;
		}
	}

	return authfp_spi_exchange(device, buf, buf, len,
			platform_data->max_frame_size);
}

static void authfp_spi_enable_write_buffering(void *device)
{
	struct buffered_write_info *buffered_writes;

	buffered_writes = (struct buffered_write_info *)authfp_get_extension(
				&((struct spi_device *)device)->dev);
	buffered_writes->buffering_enabled = true;

	return;
}

static void authfp_spi_disable_write_buffering(void *device, bool discard)
{
	struct buffered_write_info *buffered_writes;

	buffered_writes = (struct buffered_write_info *)authfp_get_extension(
				&((struct spi_device *)device)->dev);
	buffered_writes->buffering_enabled = false;

	if (discard)
		buffered_writes->buffered_len = 0;

	return;
}


static const struct authfp_bus_ops authfp_spi_bus_ops = {
	.bus_write = authfp_spi_write,
	.bus_read = authfp_spi_read,
	.bus_enable_write_buffering = authfp_spi_enable_write_buffering,
	.bus_disable_write_buffering = authfp_spi_disable_write_buffering,
};

static int __devinit authfp_spi_probe(struct spi_device *spi)
{
	int status;

	const struct authfp_data_bus data_bus = {
		.bops = &authfp_spi_bus_ops,
		.device = spi,
	};

	dev_info(&spi->dev, "authfp_spi_probe\n");

	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 8;
	status = spi_setup(spi);
	if (status < 0) {
		dev_err(&spi->dev, "spi_setup failed %d\n", status);
		return status;
	}

	return authfp_probe(&spi->dev, &data_bus,
			sizeof(struct buffered_write_info));

}


static int authfp_spi_remove(struct spi_device *spi)
{
	dev_info(&spi->dev, "authfp_spi_remove\n");
	return authfp_remove(&spi->dev);
}

#if !defined(CONFIG_HAS_EARLYSUSPEND)
static int authfp_spi_suspend(struct device *dev)
{
	return authfp_suspend(dev);
}

static int authfp_spi_resume(struct device *dev)
{
	return authfp_resume(dev);
}


static const struct dev_pm_ops authfp_spi_pm_ops = {
	.suspend = authfp_spi_suspend,
	.resume = authfp_spi_resume,
};
#endif

static struct spi_driver authfp_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.bus    = &spi_bus_type,
		.name = AUTHFP_NAME,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
		.pm = &authfp_spi_pm_ops,
#endif
	},
	.probe = authfp_spi_probe,
	.remove = __devexit_p(authfp_spi_remove),
};

static int __init authfp_spi_init(void)
{
	printk(KERN_INFO "[AUTHFP] %s\n",__FUNCTION__);

        authfp_spi_rx = kmalloc(4*MAX_BUFFERED_LEN, GFP_KERNEL);
         if ( authfp_spi_rx==NULL )
                    printk(KERN_ERR "[AUTHFP] %s : kmalloc error\n",__FUNCTION__);    

	return spi_register_driver(&authfp_spi_driver);
}

static void __exit authfp_spi_exit(void)
{
        kfree(authfp_spi_rx);
        authfp_spi_rx  = NULL;
        
	spi_unregister_driver(&authfp_spi_driver);
}


MODULE_DESCRIPTION("AuthenTec fingerprint sensor SPI bus driver");
MODULE_AUTHOR("AuthenTec");
MODULE_LICENSE("GPL");

module_init(authfp_spi_init);
module_exit(authfp_spi_exit);
