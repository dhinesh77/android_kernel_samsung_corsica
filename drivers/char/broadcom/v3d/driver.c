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

#include <linux/slab.h> /* kmalloc, kfree */
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <asm/uaccess.h>
#include "session.h"
#include "device.h"
#include "driver.h"


/* ================================================================ */

void v3d_driver_delete(v3d_driver_t *instance)
{
	switch (instance->initialised) {
	case 1:
		v3d_driver_delete_proc_entries(instance);

	case 0:
		kfree(instance);
		break;
	}
}

void v3d_driver_reset_statistics(v3d_driver_t *instance)
{
	statistics_initialise(&instance->bin_render.queue);
	statistics_initialise(&instance->bin_render.run);
	statistics_initialise(&instance->bin_render.binning_bytes);
	statistics_initialise(&instance->user.queue);
	statistics_initialise(&instance->user.run);
	instance->total_run = 0;
	instance->start = ktime_get();
}

v3d_driver_t *v3d_driver_create(void)
{
	v3d_driver_t *instance = kmalloc(sizeof(v3d_driver_t), GFP_KERNEL);
	if (instance == NULL)
		return NULL;
	instance->initialised = 0;

	/* Unconditional initialisation */
	spin_lock_init(&instance->job.free.lock);
	spin_lock_init(&instance->job.issued.lock);

	{
		unsigned int i;
		INIT_LIST_HEAD(&instance->job.free.list);
		for (i = 0 ; i < sizeof(instance->job.free_jobs) / sizeof(instance->job.free_jobs[0]) ; ++i)
			list_add_tail(&instance->job.free_jobs[i].link, &instance->job.free.list);
		sema_init(&instance->job.free.count, i);
	}
	INIT_LIST_HEAD(&instance->job.issued.user);
	INIT_LIST_HEAD(&instance->job.issued.bin_render);
	INIT_LIST_HEAD(&instance->job.issued.exclusive.user);
	INIT_LIST_HEAD(&instance->job.issued.exclusive.bin_render);
	mutex_init(&instance->job.issued.exclusive.lock);
	instance->job.issued.exclusive.owner = NULL;

	instance->device = NULL;

	v3d_driver_reset_statistics(instance);

	mutex_init(&instance->session_lock);
	memset(&instance->sessions, 0, sizeof(instance->sessions));

	/* Stuff that can fail */
	if (v3d_driver_create_proc_entries(instance) != 0)
		return v3d_driver_delete(instance), NULL;
	++instance->initialised;

	return instance;
}


/* ================================================================ */

static int change_session_entry(v3d_driver_t *instance, v3d_session_t *match, v3d_session_t *replace)
{
	unsigned int i;
	for (i = 0 ; i < sizeof(instance->sessions) / sizeof(instance->sessions[0]) ; ++i)
		if (instance->sessions[i] == match)
			return instance->sessions[i] = replace, 1;
	return 0;
}

int v3d_driver_add_session(v3d_driver_t *instance, struct v3d_session_tag *session)
{
	int found;
	mutex_lock(&instance->session_lock);
	found = change_session_entry(instance, NULL, session);
	mutex_unlock(&instance->session_lock);
	return found != 0 ? 0 : -ENOMEM;
}

void v3d_driver_remove_session(v3d_driver_t *instance, struct v3d_session_tag *session)
{
	mutex_lock(&instance->session_lock);
	(void) change_session_entry(instance, session, NULL);
	mutex_unlock(&instance->session_lock);
}


/* ================================================================ */

static v3d_driver_job_t *remove_head(struct list_head *list)
{
	v3d_driver_job_t *job;
	if (list_empty(list) != 0)
		return NULL;
	job = list_first_entry(list, v3d_driver_job_t, link);
	list_del(&job->link);
	return job;
}

/* Requires Instance->Job.Posted.Lock to be held */
v3d_driver_job_t *v3d_driver_job_get(v3d_driver_t *instance, unsigned int required)
{
	v3d_driver_job_t *job = NULL;
	int               exclusive;

	/* See if we've completed everything queued */
	/* when the exclusive lock was requested */
	MY_SPINLOCK_CHECK(&instance->job.issued.lock, 1);
	exclusive = instance->job.issued.exclusive.owner != NULL
		&& instance->job.issued.exclusive.bin_render_count == 0
		&& instance->job.issued.exclusive.user_count      == 0;

	if ((required & V3D_JOB_USER) != 0)
		job = remove_head(exclusive != 0 ? &instance->job.issued.exclusive.user      : &instance->job.issued.user);
	if (job == NULL && (required & V3D_JOB_BIN_REND) != 0)
		job = remove_head(exclusive != 0 ? &instance->job.issued.exclusive.bin_render : &instance->job.issued.bin_render);

	/* Update counts for switching to exclusive queues */
	if (job != NULL && instance->job.issued.exclusive.owner != NULL && exclusive == 0) {
		if (job->user_job.job_type == V3D_JOB_USER)
			--instance->job.issued.exclusive.user_count;
		else
			--instance->job.issued.exclusive.bin_render_count;
	}
	MY_ASSERT(instance, job == NULL || job->state == V3DDRIVER_JOB_ISSUED);
	return job;
}

void v3d_driver_job_complete(v3d_driver_t *instance, v3d_driver_job_t *job)
{
	/* The job is complete and not on any lists */
	/* - no locking required to examine it */
	/* Return the job to Free.List */
	unsigned long flags;
	MY_ASSERT(instance, job != NULL);
	MY_ASSERT(instance, job->state == V3DDRIVER_JOB_INVALID || job->state == V3DDRIVER_JOB_STATUS);
	MY_SPINLOCK_CHECK(&instance->job.issued.lock, 0);
	job->end = ktime_get();
	if (job->state > V3DDRIVER_JOB_ACTIVE) {
		unsigned int queue = ktime_us_delta(job->start, job->queued);
		unsigned int run   = ktime_us_delta(job->end,   job->start);
		instance->total_run += run;
		if (job->user_job.job_type == V3D_JOB_USER) {
			statistics_add(&instance->user.queue, queue);
			statistics_add(&instance->user.run,   run);
			v3d_session_add_statistics(job->session.instance, 1 /* User? */, queue, run, 0U);
		} else {
			statistics_add(&instance->bin_render.queue, queue);
			statistics_add(&instance->bin_render.run,   run);
			statistics_add(&instance->bin_render.binning_bytes, job->binning_bytes);
			v3d_session_add_statistics(job->session.instance, 0 /* User? */, queue, run, job->binning_bytes);
		}
	}

	MY_SPINLOCK_CHECK(&instance->job.free.lock, 0);
	spin_lock_irqsave(&instance->job.free.lock, flags);
	list_add_tail(&job->link, &instance->job.free.list);
	job->state = V3DDRIVER_JOB_INVALID;
	spin_unlock_irqrestore(&instance->job.free.lock, flags);
	up(&instance->job.free.count);
}

int v3d_driver_job_cancel(v3d_driver_t *instance, v3d_driver_job_t *job)
{
	MY_ASSERT(instance, job != NULL);
	MY_SPINLOCK_CHECK(&instance->job.issued.lock, 1);
	if (job->state == V3DDRIVER_JOB_ISSUED) {
		/* Hasn't yet made it to the hardware, so we can dequeue */
		list_del(&job->link);
		return 1;
	}
	return 0;
}


/* ================================================================ */

void v3d_driver_exclusive_start(v3d_driver_t *instance, struct v3d_session_tag *session)
{
	mutex_lock(&instance->job.issued.exclusive.lock);
	MY_ASSERT(instance, instance->job.issued.exclusive.owner == NULL);
	instance->job.issued.exclusive.owner = session;

	/* Count the number of jobs outstanding */
	instance->job.issued.exclusive.bin_render_count = 0;
	instance->job.issued.exclusive.user_count      = 0;
#if 0
	flags = v3d_driver_issued_lock(instance);
	{
		struct list_head *current;
		list_for_each(current, &instance->job.issued.bin_render)
			++instance->job.issued.exclusive.bin_render_count;
		list_for_each(current, &instance->job.issued.user)
			++instance->job.issued.exclusive.user_count;
	}
	v3d_driver_issued_unlock(instance, flags);
#endif
}

int v3d_driver_exclusive_stop(v3d_driver_t *instance, struct v3d_session_tag *session)
{
	unsigned long flags;
	int           empty;
	if (instance->job.issued.exclusive.owner != session)
		return -EPERM;

	flags = v3d_driver_issued_lock(instance);
	empty = list_empty(&instance->job.issued.exclusive.user) != 0
		&& list_empty(&instance->job.issued.exclusive.bin_render) != 0;
	v3d_driver_issued_unlock(instance, flags);
	if (empty == 0)
		return -EBUSY;

	instance->job.issued.exclusive.owner = NULL;
	mutex_unlock(&instance->job.issued.exclusive.lock);

	/* We'll be Idle now, but the normal lists may not be empty */
	v3d_device_job_posted(instance->device);
#if 0
	{
		unsigned int i;
		for (i = 0 ; i < instance->job.time_index ; ++i) {
			printk(KERN_ERR "j %2u q %9u r %9u\n",
				i, instance->job.times[i].queue, instance->job.times[i].run);
		}
	}
#endif
	return 0;
}

int v3d_driver_job_post(
	v3d_driver_t        *instance,
	v3d_session_t       *session,
	const v3d_job_post_t *user_job)
{
	v3d_driver_job_t *job;
	unsigned long     flags;
	struct list_head *list;
	int status;

	/* Get a free job structure */
	status = down_interruptible(&instance->job.free.count);
	if (status != 0)
		return -ERESTARTSYS;
	spin_lock_irqsave(&instance->job.free.lock, flags);
	job = list_first_entry(&instance->job.free.list, v3d_driver_job_t, link);
	list_del(&job->link);
	spin_unlock_irqrestore(&instance->job.free.lock, flags);

	/* Initialise and copy-in the user-side job information */
	init_waitqueue_head(&job->wait_for_completion);
	kref_init(&job->waiters); /* Starts at 1 - Completion reference */
	job->state = V3DDRIVER_JOB_INVALID;
	job->session.instance = session;
	job->queued = job->start = ktime_get();
	job->binning_bytes = 0U;

	if ((uint32_t) user_job < PAGE_OFFSET) {
		int status = copy_from_user(&job->user_job, user_job, sizeof(job->user_job));
		if (status != 0) {
			printk(KERN_ERR "%s: j %p failed copy", __func__, job);
			v3d_driver_job_complete(instance, job);
			return status;
		}
	} else
		job->user_job = *user_job;

	if (session == instance->job.issued.exclusive.owner)
		list = job->user_job.job_type == V3D_JOB_USER ? &instance->job.issued.exclusive.user : &instance->job.issued.exclusive.bin_render;
	else
		list = job->user_job.job_type == V3D_JOB_USER ? &instance->job.issued.user           : &instance->job.issued.bin_render;

	if (list_empty(&instance->job.free.list))
		printk(KERN_ERR "%s: free list empty!\n", __func__);

	/* Queue it */
	spin_lock_irqsave(&instance->job.issued.lock, flags);
	v3d_session_job_issued(job);
	list_add_tail(&job->link, list);
	spin_unlock_irqrestore(&instance->job.issued.lock, flags);

	/* Kick consumer(s) */
	v3d_driver_kick_consumers(instance);
	return 0;
}


/* ================================================================ */
/* The following functions are hard-coded for a single V3D block    */

void v3d_driver_add_device(v3d_driver_t *instance, struct v3d_device_tag *device)
{
	MY_ASSERT(instance, instance->device == NULL);
	instance->device = device;
}

void v3d_driver_kick_consumers(v3d_driver_t *instance)
{
	v3d_device_job_posted(instance->device);
}

void v3d_driver_counters_enable(v3d_driver_t *instance, uint32_t enables)
{
	v3d_device_counters_enable(instance->device, enables);
}

void v3d_driver_counters_disable(v3d_driver_t *instance)
{
	v3d_device_counters_disable(instance->device);
}


/* ================================================================ */

static void debug_output(char *buffer)
{
	char *start;
	char replace;
	do {
		for (start = buffer ; *buffer != '\n' && *buffer != 0 ; ++buffer)
			;
		replace = *buffer;
		*buffer++ = 0;
		printk(KERN_ERR "%s", start);
	} while (replace != 0);
}

void debug_dump(v3d_driver_t *driver)
{
	static char buffer[16 << 10];
	(void) device_read(driver, buffer, sizeof(buffer));
	debug_output(buffer);
	(void) jobs_read(driver, buffer, sizeof(buffer));
	debug_output(buffer);
}
