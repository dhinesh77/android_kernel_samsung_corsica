/*******************************************************************************
Copyright 2012 Broadcom Corporation.  All rights reserved.

Unless you and Broadcom execute a separate written software license agreement
governing use of this software, this software is licensed to you under the
terms of the GNU General Public License version 2, available at
http://www.gnu.org/copyleft/gpl.html (the "GPL").

Notwithstanding the above, under no circumstances may you combine this software
in any way with any other Broadcom software provided under a license other than
the GPL, without Broadcom's express prior written consent.
*******************************************************************************/

#include <asm/io.h>
#include <linux/string.h> /* memset */
#include <linux/slab.h> /* kmalloc, kfree */
#include <linux/list.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/err.h>

#include <mach/rdb/brcm_rdb_sysmap.h>
#include <mach/rdb/brcm_rdb_v3d.h>
#include <mach/rdb/brcm_rdb_pwrmgr.h>
#include <plat/scu.h>
#include <plat/clock.h>

#include "session.h"
#include "driver.h"
#include "device.h"

#define MEMORY_BIT V3D_INTCTL_INT_OUTOMEM_SHIFT
/*#define MEMORY_BIT V3D_INTCTL_INT_SPILLUSE_SHIFT*/
#define DUMMY_BYTES (32 << 10)
#define DELAY_SWITCHOFF_MS 10
#define DELAY_FREE_BIN_MEMORY_MS 100


/* ================================================================ */

static irqreturn_t interrupt_handler(int irq, void *context);


/* ================================================================ */

static inline uint32_t read(const v3d_device_t *instance, unsigned int offset)
{
	MY_ASSERT(instance->driver, instance->on != 0);
	return ioread32((uint8_t __iomem *) instance->registers.base + offset);
}

static inline void write(const v3d_device_t *instance, unsigned int offset, uint32_t value)
{
	MY_ASSERT(instance->driver, instance->on != 0);
	iowrite32(value, (uint8_t __iomem *) instance->registers.base + offset);
}


/* ================================================================ */

static void supply_binning_memory(v3d_device_t *instance, v3d_bin_memory_t *memory, size_t bytes)
{
#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "%s: Supplying %08x (%1u/%1u)\n", __func__, memory->physical, instance->out_of_memory.index.in_use, instance->out_of_memory.index.allocated);
#endif
	MY_ASSERT(instance->driver, memory->physical != 0);
	write(instance, V3D_BPOA_OFFSET, memory->physical);
	write(instance, V3D_BPOS_OFFSET, bytes);

	/* The BPOS write doesn't take effect immediately */
	while ((read(instance, V3D_PCS_OFFSET) & V3D_PCS_BMOOM_MASK) != 0)
		;

	write(instance, V3D_INTCTL_OFFSET, 1 << MEMORY_BIT);
	write(instance, V3D_INTENA_OFFSET, 1 << MEMORY_BIT);
}

static void allocate_bin_memory(struct work_struct *work)
{
	v3d_device_t     *instance = container_of(work, v3d_device_t, out_of_memory.bin_allocation);
	v3d_driver_job_t *job      = instance->in_progress.job;
	v3d_bin_memory_t  memory;
	unsigned long     flags;

#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "%s: %u/%u/%u\n", __func__, instance->out_of_memory.index.in_use, instance->out_of_memory.index.allocated, sizeof(instance->out_of_memory.memory) / sizeof(instance->out_of_memory.memory[0]));
#endif
	if (instance->out_of_memory.index.allocated == sizeof(instance->out_of_memory.memory) / sizeof(instance->out_of_memory.memory[0])) {
		printk(KERN_ERR "V3D: Out of overspill binning memory entries - job cancelled!\n");
		v3d_device_job_cancel(instance, job, 0);
		return;
	}

	memory.virtual = dma_alloc_coherent(instance->device, BIN_BLOCK_BYTES, &memory.physical, GFP_DMA);
	if (memory.virtual == NULL) {
		printk(KERN_ERR "V3D: Unable to allocate overspill binning memory (used %u/%u) - job cancelled!\n",
			instance->out_of_memory.index.in_use, instance->out_of_memory.index.allocated);
		v3d_device_job_cancel(instance, job, 0);
		return;
	}
	++instance->out_of_memory.index.total.allocated;

	/* Do we have an active job */
	flags = v3d_driver_issued_lock(instance->driver);
	if (instance->in_progress.job == NULL) {
#ifdef VERBOSE_DEBUG
		printk(KERN_ERR "%s: No job running\n", __func__);
#endif
		v3d_driver_issued_unlock(instance->driver, flags);
		dma_free_coherent(
			instance->device,
			BIN_BLOCK_BYTES,
			memory.virtual,
			memory.physical);
		++instance->out_of_memory.index.total.freed;
		return;
	}

	instance->out_of_memory.memory[instance->out_of_memory.index.allocated] = memory;
	++instance->out_of_memory.index.allocated;

	/* We've allocated the memory, but we only supply it if the current job is stalled */
	if ((read(instance, V3D_INTCTL_OFFSET) & V3D_INTCTL_INT_OUTOMEM_MASK) != 0
		&& (read(instance, V3D_INTENA_OFFSET) & V3D_INTENA_EI_OUTOMEM_MASK) == 0) {
		++instance->out_of_memory.index.in_use;
              	supply_binning_memory(instance, &memory, BIN_BLOCK_BYTES);
	}
	v3d_driver_issued_unlock(instance->driver, flags);
}

static void release_bin_memory(v3d_device_t *instance)
{
	instance->out_of_memory.index.in_use = 0;
}

static void free_bin_memory(v3d_device_t *instance)
{
	unsigned int i;
	for (i = 0 ; i < instance->out_of_memory.index.allocated ; ++i, ++instance->out_of_memory.index.total.freed)
		dma_free_coherent(
			instance->device,
			BIN_BLOCK_BYTES,
			instance->out_of_memory.memory[i].virtual,
			instance->out_of_memory.memory[i].physical);
	instance->out_of_memory.index.in_use = 0;
	instance->out_of_memory.index.allocated = 0;
}

static void free_bin_memory_shim(struct work_struct *work)
{
	v3d_device_t *instance = container_of(work, v3d_device_t, free_bin_memory.work);
	mutex_lock(&instance->power);
	if (instance->on == 0)
		free_bin_memory(instance);
	mutex_unlock(&instance->power);
}


/* ================================================================ */

/* TODO: Removed reset value writes - we reset often */
static void initialise_registers(v3d_device_t *instance, int reenable_interrupt)
{
	/* Disable L2 cache */
	write(instance, V3D_L2CACTL_OFFSET, 1 << V3D_L2CACTL_L2CDIS_SHIFT);

#if 0
	/* Reset control list executors */
	write(instance, V3D_CT0CS_OFFSET, 1 << V3D_CT0CS_CTRSTA_SHIFT);
	write(instance, V3D_CT1CS_OFFSET, 1 << V3D_CT1CS_CTRSTA_SHIFT);

	/* Clear flush and frame counts */
	write(instance, V3D_RFC_OFFSET, 1);
	write(instance, V3D_BFC_OFFSET, 1);

	/* Clear slice caches */
	write(instance, V3D_SLCACTL_OFFSET, 0x0f0f0f0f);
#endif

	/* Reset reserved VPM user reservation */
	write(instance, V3D_VPMBASE_OFFSET, instance->mode == v3d_mode_render ? 0U : (64 * 64) >> 8);

#if 0
	/* No limits for binning / rendering VPM allocations */
	write(instance, V3D_VPACNTL_OFFSET, 0);
#endif

	/* Set global debug enable */
	write(instance, V3D_DBCFG_OFFSET, 1);

	/* Supply over-spill binning memory */
	write(instance, V3D_BPOA_OFFSET, instance->dummy.physical);
	write(instance, V3D_BPOS_OFFSET, DUMMY_BYTES);
	write(instance, V3D_BPOS_OFFSET, 0U);
	mb();

	/* The BPOS write doesn't take effect immediately */
	while ((read(instance, V3D_PCS_OFFSET) & V3D_PCS_BMOOM_MASK) != 0)
		;

	/* Set-up interrupts */
	write(instance, V3D_INTDIS_OFFSET, 0xf);
	write(instance, V3D_INTCTL_OFFSET, 0xf);
	if (reenable_interrupt != 0) {
		write(instance, V3D_INTENA_OFFSET, 0
			| 1 << MEMORY_BIT
			/*| 1 << V3D_INTCTL_INT_FLDONE_SHIFT*/
			| 1 << V3D_INTCTL_INT_FRDONE_SHIFT);
	}
	write(instance, V3D_DBQITC_OFFSET, 0xffff);
	write(instance, V3D_DBQITE_OFFSET, 0xffff);

	/* Clear user program counts */
	write(instance, V3D_SRQCS_OFFSET, (1 << V3D_SRQCS_QPURQCM_SHIFT) | (1 << V3D_SRQCS_QPURQCC_SHIFT));
	mb();
}		

static void reset(v3d_device_t *instance, int reenable_interrupt)
{
	write(instance, V3D_INTDIS_OFFSET, 0xf);
	clk_reset(instance->clock);
	initialise_registers(instance, reenable_interrupt);
}

static void enable_clock(v3d_device_t *instance)
{
	int status;
	MY_ASSERT(instance->driver, IS_ERR_OR_NULL(instance->clock) == 0);
	status = clk_enable(instance->clock);
	MY_ASSERT(instance->driver, status == 0);
	pi_mgr_qos_request_update(&instance->qos_node, 0);
	status = pi_mgr_dfs_request_update(&instance->dfs_node, PI_OPP_TURBO);
	MY_ASSERT(instance->driver, status == 0);
	clk_reset(instance->clock);
}

static void disable_clock(v3d_device_t *instance)
{
	int status;
	status = pi_mgr_dfs_request_update(&instance->dfs_node, PI_MGR_DFS_MIN_VALUE);
	MY_ASSERT(instance->driver, status == 0);
	pi_mgr_qos_request_update(&instance->qos_node, PI_MGR_QOS_DEFAULT_VALUE);
	clk_disable(instance->clock);
}

/* Power mutex must be held by the caller of power_on() and power_off() */
static void power_on(v3d_device_t *instance)
{
#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "On\n");
#endif
	enable_clock(instance);
	scu_standby(0);
	mb();
	instance->on = 1;
	initialise_registers(instance, 1);
}

static void power_off(v3d_device_t *instance)
{
#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "Off\n");
#endif
	instance->on = 0;
	disable_clock(instance);
	scu_standby(1);
}


/* ================================================================ */

static void run_user_job(v3d_device_t *instance, v3d_driver_job_t *job)
{
	unsigned int i;
#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "%s: Issuing user job %p ID %08x (q %08x)\n", __func__, job, job->user_job.job_id, read(instance, V3D_SRQCS_OFFSET));
#endif
	MY_ASSERT(instance->driver, job != NULL);
	MY_SPINLOCK_CHECK(&instance->driver->job.issued.lock, 1);
	MY_ASSERT(instance->driver, instance->in_progress.job == NULL);
	instance->in_progress.job = job;
		reset(instance, 1);
	job->start = ktime_get();
	for (i = 0 ; i < job->user_job.user_cnt ; ++i) {
		write(instance, V3D_SRQUL_OFFSET, job->user_job.v3d_srqul[i]);
		write(instance, V3D_SRQUA_OFFSET, job->user_job.v3d_srqua[i]);
		write(instance, V3D_SRQPC_OFFSET, job->user_job.v3d_srqpc[i]);
	}
	write(instance, V3D_SCRATCH_OFFSET, (uint32_t) job);
}

static void run_bin_render_job(v3d_device_t *instance, v3d_driver_job_t *job)
{
	v3d_bin_memory_t memory;
	MY_ASSERT(instance->driver, job != NULL);
	MY_SPINLOCK_CHECK(&instance->driver->job.issued.lock, 1);
	MY_ASSERT(instance->driver, instance->in_progress.job == NULL);
	instance->in_progress.job = job;
#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "%s: ID j %p:%08x hws %03x %08x - %08x %08x - %08x\n", __func__,
		job,
		job->user_job.job_id,
		read(instance, V3D_PCS_OFFSET),
		job->user_job.v3d_ct0ca, job->user_job.v3d_ct0ea,
		job->user_job.v3d_ct1ca, job->user_job.v3d_ct1ea);
#endif

reset(instance, 0);
	job->start = ktime_get();
	if ((job->user_job.job_type & V3D_JOB_BIN) != 0) {
		if (job->user_job.v3d_ct0ca != job->user_job.v3d_ct0ea) {
			MY_ASSERT(instance->driver, job->user_job.v3d_ct0ca != 0);
			MY_ASSERT(instance->driver, job->user_job.v3d_ct0ea != 0);
			write(instance, V3D_CT0CA_OFFSET, job->user_job.v3d_ct0ca);
			write(instance, V3D_CT0EA_OFFSET, job->user_job.v3d_ct0ea);
		} else
			job->user_job.job_type &= ~V3D_JOB_BIN;
	}
	if ((job->user_job.job_type & V3D_JOB_REND) != 0) {
		if (job->user_job.v3d_ct1ca != job->user_job.v3d_ct1ea) {
			MY_ASSERT(instance->driver, job->user_job.v3d_ct1ca != 0);
			MY_ASSERT(instance->driver, job->user_job.v3d_ct1ea != 0);
			write(instance, V3D_CT1CA_OFFSET, job->user_job.v3d_ct1ca);
			write(instance, V3D_CT1EA_OFFSET, job->user_job.v3d_ct1ea);
		} else {
#ifdef VERBOSE_DEBUG
			printk(KERN_ERR "%s: Job %p has no rendering\n", __func__, job);
#endif
			job->user_job.job_type &= ~V3D_JOB_REND;
		}
	}
	write(instance, V3D_SCRATCH_OFFSET, (uint32_t) job);

	memory.physical = instance->dummy.physical;
	memory.virtual  = instance->dummy.virtual;
	supply_binning_memory(instance, &memory, DUMMY_BYTES);

	write(instance, V3D_INTENA_OFFSET, 0
		| 1 << MEMORY_BIT
		/*| 1 << V3D_INTCTL_INT_FLDONE_SHIFT*/
		| 1 << V3D_INTCTL_INT_FRDONE_SHIFT);
}

#ifdef WORKAROUND_GFXH16
static int gfxh16thread(void *context)
{
	v3d_device_t *instance = (v3d_device_t *) context;
	while (msleep(60), kthread_should_stop() == 0) {
		int status = mutex_lock(&instance->power);
		if (status != 0)
			break;
		if (instance->on != 0) {
			/* Clear caches - see GFXH-16 */
			write(instance, V3D_SLCACTL_OFFSET, 0x0f0f0f0f);
		}
		mutex_unlock(&instance->power);
	}
	return 0;
}
#endif

static void set_mode(v3d_device_t *instance, v3d_mode_t mode)
{
	if (mode != instance->mode)
		instance->mode = mode;
}


/* ================================================================ */

static void switch_off(struct work_struct *work)
{
	v3d_device_t *instance = container_of(work, v3d_device_t, switch_off.work);
	mutex_lock(&instance->power);
	if (instance->idle != 0) {
		MY_ASSERT(instance->driver, instance->in_progress.job == NULL);
		if (instance->out_of_memory.index.allocated != 0)
			schedule_delayed_work(&instance->free_bin_memory, msecs_to_jiffies(DELAY_FREE_BIN_MEMORY_MS));
		power_off(instance);
		mutex_unlock(&instance->suspend);
	}
	mutex_unlock(&instance->power);
}


/* ================================================================ */

/* Power mutex must be held by the caller */
static v3d_driver_job_t *run_job(v3d_device_t * instance)
{
	unsigned long      flags;
	v3d_driver_job_t *job;
	if (instance->idle == 0)
		return NULL;
	/* Check for any jobs */
	flags = v3d_driver_issued_lock(instance->driver);
	job = v3d_driver_job_get(instance->driver, V3D_JOB_USER | V3D_JOB_BIN_REND);
	if (job != NULL) {
		job->state = V3DDRIVER_JOB_ACTIVE;
		job->device = instance;
	}
	v3d_driver_issued_unlock(instance->driver, flags);
	if (job == NULL)
		return NULL;

	/* Avoid a race with switch_off */
	if (instance->idle != 0) {
		cancel_delayed_work(&instance->switch_off);
		cancel_delayed_work(&instance->free_bin_memory);
	}
	instance->idle = 0; /* Can't be switched-off from here on */

	/* Do we need to switch on? */
	if (instance->on == 0) {
		mutex_lock(&instance->suspend); /* TODO: mutex_lock_interruptible */
		instance->mode = job->user_job.job_type == V3D_JOB_USER ? v3d_mode_user : v3d_mode_render;
		power_on(instance);
	}
	MY_ASSERT(instance->driver, instance->on != 0);

	/* Post the job to the hardware */
	flags = v3d_driver_issued_lock(instance->driver);
	if (job->user_job.job_type == V3D_JOB_USER) {
		set_mode(instance, v3d_mode_user);
		run_user_job(instance, job);
	} else {
		set_mode(instance, v3d_mode_render);
		run_bin_render_job(instance, job);
	}
	v3d_driver_issued_unlock(instance->driver, flags);
	return job;
}


/* ================================================================ */

void v3d_device_delete(v3d_device_t *instance)
{
	switch (instance->initialised) {
	case 8:
		mutex_lock(&instance->suspend);
		free_irq(instance->irq, instance);
		flush_delayed_work(&instance->switch_off);
		flush_delayed_work(&instance->free_bin_memory);
		free_bin_memory(instance);

	case 7:
		pi_mgr_qos_request_remove(&instance->qos_node);

	case 6:
		pi_mgr_dfs_request_remove(&instance->dfs_node);

	case 5:
#ifdef WORKAROUND_GFXH16
		kthread_stop(instance->gfxh16thread);
#endif

	case 4:
#if 0
		kthread_stop(instance->thread);
		complete(&instance->post_event);
#endif

	case 3:
		destroy_workqueue(instance->out_of_memory.work_queue);

	case 2:
		dma_free_coherent(
			instance->device,
			DUMMY_BYTES,
			instance->dummy.virtual,
			instance->dummy.physical);

	case 1:
		iounmap(&instance->registers.base);

	case 0:
		clk_put(instance->clock);
		kfree(instance);
		break;
	}
}

static void reset_state(v3d_device_t *instance)
{
	instance->in_progress.job = NULL;
}

v3d_device_t *v3d_device_create(
	v3d_driver_t *driver,
	struct device *device,
	uint32_t register_base,
	unsigned int irq)
{
	v3d_device_t *instance = kmalloc(sizeof(v3d_device_t), GFP_KERNEL);
	int            status;
	if (instance == NULL)
		return NULL;
	instance->initialised = 0;

	/* Unconditional initialisation */
	instance->mode = v3d_mode_undefined;
	instance->out_of_memory.index.in_use    = 0;
	instance->out_of_memory.index.allocated = 0;
	instance->out_of_memory.index.total.allocated = 0;
	instance->out_of_memory.index.total.freed = 0;
	INIT_WORK(&instance->out_of_memory.bin_allocation, allocate_bin_memory);
	memset(
		&instance->out_of_memory.memory[0],
		0,
		sizeof(instance->out_of_memory.memory));

	instance->driver = driver;
	instance->device = device;
	instance->idle = 1;
	mutex_init(&instance->suspend);
	mutex_init(&instance->power);
	instance->on = 0;
	INIT_DELAYED_WORK(&instance->switch_off, switch_off);
		INIT_DELAYED_WORK(&instance->free_bin_memory, free_bin_memory_shim);

	reset_state(instance);

	instance->clock = clk_get(NULL, "v3d_axi_clk");
	MY_ASSERT(instance->driver, instance->clock != NULL);

	/* Initialisation that could fail follows */
	instance->registers.base = (void __iomem *) ioremap_nocache(register_base, SZ_4K);
	if (instance->registers.base == NULL)
		return printk(KERN_ERR "%s:%d fail\n", __func__, __LINE__), v3d_device_delete(instance), NULL;
	++instance->initialised;

	instance->dummy.virtual = dma_alloc_coherent(
		instance->device,
		DUMMY_BYTES,
		&instance->dummy.physical,
		GFP_DMA);
	if (instance->dummy.virtual == NULL)
		return printk(KERN_ERR "%s:%d fail\n", __func__, __LINE__), v3d_device_delete(instance), NULL;
	++instance->initialised;

	instance->out_of_memory.work_queue = create_singlethread_workqueue("bin_alloc");
	if (instance->out_of_memory.work_queue == NULL)
		return printk(KERN_ERR "%s:%d fail\n", __func__, __LINE__), v3d_device_delete(instance), NULL;
	++instance->initialised;

#if 0
	instance->thread = kthread_run(&thread, instance, "v3d_device");
	if (IS_ERR(instance->thread))
		return printk(KERN_ERR "%s:%d fail\n", __func__, __LINE__), v3d_device_delete(instance), NULL;
#endif
	++instance->initialised;

#ifdef WORKAROUND_GFXH16
	instance->gfxh16thread = kthread_run(&gfxh16thread, instance, "v3d_device_gfxh16");
	if (IS_ERR(instance->gfxh16thread))
		return printk(KERN_ERR "%s:%d fail\n", __func__, __LINE__), v3d_device_delete(instance), NULL;
#endif
	++instance->initialised;

	memset(&instance->qos_node, 0, sizeof(instance->qos_node));
	memset(&instance->dfs_node, 0, sizeof(instance->dfs_node));
	status = pi_mgr_qos_add_request(
		&instance->qos_node,
		"v3d",
		PI_MGR_PI_ID_ARM_CORE,
		PI_MGR_QOS_DEFAULT_VALUE);
	if (status != 0)
		return printk(KERN_ERR "%s:%d fail\n", __func__, __LINE__), v3d_device_delete(instance), NULL;
	++instance->initialised;

	status = pi_mgr_dfs_add_request(
		&instance->dfs_node,
		"v3d",
		PI_MGR_PI_ID_MM,
		PI_MGR_DFS_MIN_VALUE);
	if (status != 0)
		return printk(KERN_ERR "%s:%d fail\n", __func__, __LINE__), v3d_device_delete(instance), NULL;
	++instance->initialised;

	instance->irq = irq;
	status = request_irq(instance->irq, interrupt_handler, IRQF_TRIGGER_HIGH, "v3d", instance);
	if (status != 0)
		return printk(KERN_ERR "%s:%d fail\n", __func__, __LINE__), v3d_device_delete(instance), NULL;
	++instance->initialised;

	return instance;
}


/* ================================================================ */

void v3d_device_job_posted(v3d_device_t *instance)
{
	int               status;
	if (instance->idle == 0)
		return; /* Job already on the queue, so no race */

	status = mutex_lock_interruptible(&instance->power);
	if (status != 0)
		return;
	(void) run_job(instance);
	mutex_unlock(&instance->power);
}


/* ================================================================ */

void v3d_device_register_read(v3d_device_t *instance, uint32_t *destination, unsigned int offset32, unsigned int n32)
{
	int was_on;
	unsigned int i;
	mutex_lock(&instance->power);
	/* Do we need to switch on? */
	if (instance->on == 0) {
		mutex_lock(&instance->suspend); /* TODO: mutex_lock_interruptible */
		power_on(instance);
		was_on = 0;
	} else
		was_on = 1;
	MY_ASSERT(instance->driver, instance->on != 0);

	for (i = 0 ; i < n32 ; ++i)
		destination[i] = read(instance, sizeof(uint32_t) * (offset32 + i));

	/* Switch off */
	if (was_on == 0) {
		power_off(instance);
		mutex_unlock(&instance->suspend);
	}
	mutex_unlock(&instance->power);
}


/* ================================================================ */

/* Returns Complete? */
static int handle_bin_render_interrupt(v3d_device_t *instance, uint32_t status)
{
	int complete_bin_render = 0;

	/* Acknowledge the interrupts that we'll service */
	write(instance, V3D_INTCTL_OFFSET, status);

	if ((status & ((1 << V3D_INTCTL_INT_FLDONE_SHIFT) | (1 << V3D_INTCTL_INT_FRDONE_SHIFT))) != 0) {
		/* Binning completed */
		if (instance->in_progress.job->user_job.job_type == V3D_JOB_BIN
			|| (status & (1 << V3D_INTCTL_INT_FRDONE_SHIFT)) != 0) {
			complete_bin_render = 1;
			write(instance, V3D_INTDIS_OFFSET, 1 << MEMORY_BIT);
		}
	} else
		if ((status & (1 << MEMORY_BIT)) != 0) {
			/* Need more overflow binning memory */
			MY_ASSERT(instance->driver, instance->in_progress.job != NULL);
			if (instance->out_of_memory.index.in_use != instance->out_of_memory.index.allocated) {
				v3d_bin_memory_t *memory = &instance->out_of_memory.memory[instance->out_of_memory.index.in_use++];
			supply_binning_memory(instance, memory, BIN_BLOCK_BYTES);
			} else {
				/* Need another allocation */
				write(instance, V3D_INTDIS_OFFSET, 1 << MEMORY_BIT);
				queue_work(instance->out_of_memory.work_queue, &instance->out_of_memory.bin_allocation);
			}
		}
		
		if (complete_bin_render != 0 && instance->out_of_memory.index.in_use != 0) {
		instance->in_progress.job->binning_bytes =
			DUMMY_BYTES
			+ instance->out_of_memory.index.in_use*BIN_BLOCK_BYTES
			- read(instance, V3D_BPCS_OFFSET);
	}
	return complete_bin_render;
}

int handle_user_interrupt(v3d_device_t *instance, uint32_t qpu_status)
{
	/* User job(s) completed */
	uint32_t user_fifo = read(instance, V3D_SRQCS_OFFSET);
	unsigned int completed = user_fifo >> V3D_SRQCS_QPURQCC_SHIFT
		& V3D_SRQCS_QPURQCC_MASK >> V3D_SRQCS_QPURQCC_SHIFT;
	MY_ASSERT(instance->driver, (qpu_status & 1 << V3D_SRQCS_QPURQERR_SHIFT) == 0);
#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "%s: s %08x c %1x\n", __func__, user_fifo, completed);
#endif

	/* Acknowledge the interrupts that we'll service */
	if (qpu_status != 0)
		write(instance, V3D_DBQITC_OFFSET, qpu_status);

	/* Complete user jobs */
	if (completed == instance->in_progress.job->user_job.user_cnt)
		return 1;
#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "%s: Completed ..%4x\n", __func__, completed);
#endif
	return 0;
}

static uint32_t get_status(v3d_device_t *instance, uint32_t *status, uint32_t *qpu_status)
{
	uint32_t enable = read(instance, V3D_INTENA_OFFSET);
	*status    = read(instance, V3D_INTCTL_OFFSET) & enable;
	*qpu_status = read(instance, V3D_DBQITC_OFFSET);
	return *status | *qpu_status;
}

/* Returns Issued? */
static int run_next_job(v3d_device_t *instance)
{
	/* Job complete */
	v3d_driver_job_t *job;
	unsigned long      flags;
	/* Issue the next one */
	flags = v3d_driver_issued_lock(instance->driver);
	job = v3d_driver_job_get(instance->driver, V3D_JOB_USER | V3D_JOB_BIN_REND);
	if (job != NULL) {
#ifdef VERBOSE_DEBUG
		printk(KERN_ERR "%s: Issuing job %p\n", __func__, job);
#endif
		job->device = instance;
		job->state = V3DDRIVER_JOB_ACTIVE;

		/* Post the job to the hardware */
		if (job->user_job.job_type == V3D_JOB_USER) {
			set_mode(instance, v3d_mode_user);
			run_user_job(instance, job);
		} else {
			set_mode(instance, v3d_mode_render);
			run_bin_render_job(instance, job);
		}
	}
	v3d_driver_issued_unlock(instance->driver, flags);
	return job != NULL;
}

static void check_idle(v3d_device_t *instance, uint32_t status)
{
	/* Update Idle - we only go from busy to idle here */
	if (instance->in_progress.job == NULL
		&& (status & (1 << MEMORY_BIT)) == 0) {
		instance->idle = 1;
		schedule_delayed_work(&instance->switch_off, msecs_to_jiffies(DELAY_SWITCHOFF_MS));
	}
}

static irqreturn_t interrupt_handler(int irq, void *context)
{
	v3d_device_t     *instance  = (v3d_device_t *) context;
	v3d_driver_job_t *job;
	uint32_t          status, qpu_status;
	int               complete = 0;
	unsigned long     flags;
	MY_ASSERT(instance->driver, instance->idle == 0);
	flags = v3d_driver_issued_lock(instance->driver);
	job = instance->in_progress.job;
	while (get_status(instance, &status, &qpu_status) != 0) {
#ifdef VERBOSE_DEBUG
		printk(KERN_ERR "%s: s %08x q %08x j %p s %d %d\n", __func__, status, qpu_status, job, job == NULL ? 0 : job->state, job == NULL ? 0 : job->status);
#endif
		if (status != 0)
			complete |= handle_bin_render_interrupt(instance, status);
		if (qpu_status != 0)
			complete |= handle_user_interrupt(instance, qpu_status);

		if (complete != 0) {
			release_bin_memory(instance);
			instance->in_progress.job = NULL;
		}
	}
	if (complete != 0)
		v3d_session_job_complete(job, 0);
	v3d_driver_issued_unlock(instance->driver, flags);

	if (complete != 0) {
		v3d_session_job_release(job, __func__); /* Remove completion reference */

		(void) run_next_job(instance);

		/* Update Idle - we only go from busy to idle here */
		check_idle(instance, status);
	}
	return IRQ_HANDLED;
}


/* ================================================================ */

void v3d_device_job_cancel(v3d_device_t *instance, v3d_driver_job_t *job, int flush)
{
	unsigned long flags;
#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "%s: Timeout, idle %d\n", __func__, instance->idle);
#endif
	flags = v3d_driver_issued_lock(instance->driver);
#if 1
	if (job == instance->in_progress.job) {
		printk(KERN_ERR
			"%s: j %p s %d idle %d 0 %08x - %08x 1 %08x - %08x",
			__func__,
			job, job->state, instance->idle,
			job->user_job.v3d_ct0ca, job->user_job.v3d_ct0ea,
			job->user_job.v3d_ct1ca, job->user_job.v3d_ct1ea);
		debug_dump(instance->driver);
	}
#endif
	if (job != instance->in_progress.job) {
#ifdef VERBOSE_DEBUG
		printk(KERN_ERR "%s(%p s %d %d r %1d): Next job (%p/%d) already running\n", __func__, job, job->state, job->status, job->waiters.refcount.counter, instance->in_progress.job, instance->in_progress.job == NULL ? 0 : instance->in_progress.job->state);
#endif
		v3d_driver_issued_unlock(instance->driver, flags);
		return;
	}
	if (job->state == V3DDRIVER_JOB_STATUS) {
		printk(KERN_ERR "%s(%p s %d r %1d): Job completed\n", __func__, job, job->state, job->waiters.refcount.counter);
		v3d_driver_issued_unlock(instance->driver, flags);
		return;
	}

	instance->in_progress.job = NULL;
	write(instance, V3D_INTDIS_OFFSET, 0xf);
	v3d_session_job_complete(job, -ECANCELED);
	v3d_driver_issued_unlock(instance->driver, flags);
	
		reset(instance, 1);
	if (flush != 0)
		flush_workqueue(instance->out_of_memory.work_queue);
	release_bin_memory(instance);
	v3d_session_job_release(job, __func__); /* Remove completion reference */
	reset_state(instance);

	if (run_next_job(instance) == 0) {
		/* Safe out of interrupt context, as we're idle    */
		check_idle(instance, 0U);
	}
}


/* ================================================================ */

void v3d_device_suspend(v3d_device_t *instance)
{
	mutex_lock(&instance->suspend);
}

void v3d_device_resume(v3d_device_t *instance)
{
	mutex_unlock(&instance->suspend);
}


/* ================================================================ */

void v3d_device_counters_enable(v3d_device_t *instance, uint32_t enables)
{
	/* Configure the counters to record the specified quantities */
	unsigned int i, counter;
	uint32_t     shift;
	for (i = 0, shift = enables, counter = 0 ; shift != 0 && counter < 16 ; ++i, shift >>= 1)
		if ((shift & 1) != 0) {
			unsigned int offset = (V3D_PCTRS1_OFFSET - V3D_PCTRS0_OFFSET)*counter++;
			write(instance, V3D_PCTRS0_OFFSET + offset, i);
		}
	write(instance, V3D_PCTRE_OFFSET, (1U << 31) | ((1U << counter) - 1U)); /* Enable counters */
	write(instance, V3D_PCTRC_OFFSET, 0xffU); /* Clear all counters */
}

void v3d_device_counters_disable(v3d_device_t *instance)
{
	write(instance, V3D_PCTRE_OFFSET, 0U); /* Disable all counters */
}

void v3d_device_counters_add(v3d_device_t *instance, uint32_t *counters)
{
	unsigned int i;
	for (i = 0 ; i < 16 ; ++i)
		counters[i] += read(instance, V3D_PCTR0_OFFSET + 4*i);
}
