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
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/wait.h>

#include "driver.h"
#include "device.h"
#include "session.h"


/* ================================================================ */

v3d_session_t *v3d_session_create(v3d_driver_t *driver, const char *name)
{
	v3d_session_t *instance = (v3d_session_t *) kmalloc(sizeof(v3d_session_t), GFP_KERNEL);
	if (instance == NULL)
		return NULL;

	instance->initialised = 0;

	/* Unconditional initialisation */
	instance->driver = driver;
	instance->name   = name;
	instance->last_id = 0;
	INIT_LIST_HEAD(&instance->issued.list);
	instance->performance_counter.enables = 0;

	v3d_session_reset_statistics(instance);

	/* Initialisation that can fail */
	if (v3d_driver_add_session(instance->driver, instance) != 0)
		return v3d_session_delete(instance), NULL;
	++instance->initialised;

	return instance;
}

void v3d_session_delete(v3d_session_t *instance)
{
	switch (instance->initialised) {
	case 1:
		/* Cancel all our jobs */
		v3d_session_jobs_abort(instance);

		/* Ensure that any exclusive lock is released */
		(void) v3d_driver_exclusive_stop(instance->driver, instance);

		v3d_driver_remove_session(instance->driver, instance);

	case 0:
		kfree(instance);
		break;
	}
}

void v3d_session_reset_statistics(v3d_session_t *instance)
{
	instance->start = ktime_get();
	instance->total_run = 0;
	statistics_initialise(&instance->bin_render.queue);
	statistics_initialise(&instance->bin_render.run);

	statistics_initialise(&instance->user.queue);
	statistics_initialise(&instance->user.run);

	statistics_initialise(&instance->binning_bytes);
}

void v3d_session_add_statistics(v3d_session_t *instance, int user, unsigned int queue, unsigned int run, unsigned int binning_bytes)
{
	instance->total_run += run;
	if (user != 0) {
		statistics_add(&instance->user.queue, queue);
		statistics_add(&instance->user.run,   run);
		statistics_add(&instance->binning_bytes, binning_bytes);
	} else {
		statistics_add(&instance->bin_render.queue, queue);
		statistics_add(&instance->bin_render.run,   run);
	}
}


/* ================================================================ */

int v3d_session_job_post(
	v3d_session_t *instance,
	const v3d_job_post_t *user_job)
{
	return v3d_driver_job_post(instance->driver, instance, user_job);
}


/* ================================================================ */

void remove_job(struct kref *reference)
{
	v3d_driver_job_t *job = container_of(reference, v3d_driver_job_t, waiters);
	MY_ASSERT(job->session.instance->driver, job != NULL);
}

void v3d_session_job_reference(struct v3d_driver_job_tag *job, const char *name)
{
	MY_ASSERT(job->session.instance->driver, job != NULL);
#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "%s: j %p %d %s\n", __func__, job, job->waiters.refcount.counter, name);
#endif
	MY_SPINLOCK_CHECK(&job->session.instance->driver->job.issued.lock, 1);
	kref_get(&job->waiters); /* Prevents freeing on completion */
}

void v3d_session_job_release(struct v3d_driver_job_tag *job, const char *name)
{
	v3d_session_t *instance = job->session.instance;
	int            last;
	MY_ASSERT(job->session.instance->driver, job != NULL);
	MY_ASSERT(job->session.instance->driver, job->state != V3DDRIVER_JOB_INVALID);
	MY_SPINLOCK_CHECK(&instance->driver->job.issued.lock, 0);
#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "%s: j %p %d %s\n", __func__, job, job->waiters.refcount.counter, name);
#endif
	last = kref_put(&job->waiters, &remove_job);
	if (last != 0) {
#ifdef VERBOSE_DEBUG
		printk(KERN_ERR "%s: j %p %s - completing\n", __func__, job, name);
#endif
		v3d_driver_job_complete(instance->driver, job);
	}
}


/* ================================================================ */

void v3d_session_job_issued(v3d_driver_job_t *job)
{
	v3d_session_t *instance = job->session.instance;
	MY_ASSERT(job->session.instance->driver, job != NULL);
	MY_SPINLOCK_CHECK(&instance->driver->job.issued.lock, 1);
	instance->last_id = (int32_t) job->user_job.job_id;
	job->state = V3DDRIVER_JOB_ISSUED;
	list_add_tail(&job->session.link, &instance->issued.list);
}

void v3d_session_job_complete(v3d_driver_job_t *job, int status)
{
	v3d_session_t *instance;
	MY_ASSERT(job->session.instance->driver, job != NULL);
#ifdef VERBOSE_DEBUG
	printk(KERN_ERR "%s: j %p s %d:%d\n", __func__, job, job->state, status);
#endif
	instance = job->session.instance;
	MY_SPINLOCK_CHECK(&instance->driver->job.issued.lock, 1);
	MY_ASSERT(job->session.instance->driver, job->state == V3DDRIVER_JOB_ACTIVE || status != 0);
	list_del(&job->session.link);
	job->state = V3DDRIVER_JOB_STATUS;
	job->status = status;
	wake_up_all(&job->wait_for_completion);
	if (instance->performance_counter.enables != 0 && status == 0)
		v3d_device_counters_add(job->device, &instance->performance_counter.count[0]);
}

void v3d_session_jobs_abort(v3d_session_t *instance)
{
	int cancelled = 0;
	unsigned long flags;
	/* Wait on all previous jobs, if any - only really required when multiple V3D blocks exist */
	v3d_driver_job_t *job;
	flags = v3d_driver_issued_lock(instance->driver);
	while (list_empty(&instance->issued.list) == 0) {
		job = list_first_entry(&instance->issued.list, v3d_driver_job_t, session.link);
		cancelled = v3d_driver_job_cancel(instance->driver, job);
		if (cancelled != 0) {
			v3d_session_job_complete(job, -ECANCELED);
			v3d_driver_issued_unlock(instance->driver, flags);
		} else {
			v3d_session_job_reference(job, __func__);
			v3d_driver_issued_unlock(instance->driver, flags);
			v3d_device_job_cancel(job->device, job, 1);
		}
		v3d_session_job_release(job, __func__);
		flags = v3d_driver_issued_lock(instance->driver);
	}
	v3d_driver_issued_unlock(instance->driver, flags);
}

static void wait_job(v3d_session_t *instance, v3d_driver_job_t *job)
{
	int remaining_time;
	int state;
	MY_ASSERT(instance->driver, job != NULL);
	MY_SPINLOCK_CHECK(&instance->driver->job.issued.lock, 0);
	do {
		state = job->state;
		remaining_time = wait_event_timeout(job->wait_for_completion, job->state == V3DDRIVER_JOB_STATUS, msecs_to_jiffies(JOB_TIMEOUT_MS));
	} while (remaining_time == 0 && state == V3DDRIVER_JOB_ISSUED && (printk(KERN_ERR "%s: j %p not yet issued, rewaiting\n", __func__, job), 1));
	if (remaining_time == 0) {
		printk(KERN_ERR "V3D job %p timed-out in state %d, active %p", job, job->state, job->device->in_progress.job);
		debug_dump(instance->driver);
		v3d_device_job_cancel(job->device, job, 1);
	}
}

int32_t v3d_session_wait(v3d_session_t *instance)
{
	int32_t       id = instance->last_id;
	unsigned long flags;

	/* Wait on all previous jobs, if any - only required when multiple V3D blocks exist */
	do {
		v3d_driver_job_t *job = NULL;
		flags = v3d_driver_issued_lock(instance->driver);
		if (list_empty(&instance->issued.list) == 0)
			job = list_first_entry(&instance->issued.list, v3d_driver_job_t, session.link);
		if (job == NULL || id - (int32_t) job->user_job.job_id < 0) {
			v3d_driver_issued_unlock(instance->driver, flags);
			return id;
		}

		v3d_session_job_reference(job, __func__); /* Prevents freeing on completion */
		v3d_driver_issued_unlock(instance->driver, flags);
		wait_job(instance, job);
		v3d_session_job_release(job, __func__); /* Remove waiting reference */
	} while (1);
}

int v3d_session_counters_enable(v3d_session_t *instance, uint32_t enables)
{
	if (instance->performance_counter.enables != 0)
		return -1;
	instance->performance_counter.enables = enables;
	memset(
		&instance->performance_counter.count[0],
		0,
		sizeof(instance->performance_counter.count));
	v3d_driver_counters_enable(instance->driver, enables);
	return 0;
}

int v3d_session_counters_disable(v3d_session_t *instance)
{
	if (instance->performance_counter.enables == 0)
		return -1;
	instance->performance_counter.enables = 0;
	return 0;
}

const uint32_t *v3d_session_counters_get(v3d_session_t *instance)
{
	return instance->performance_counter.enables == 0 ? &instance->performance_counter.count[0] : NULL;
}
