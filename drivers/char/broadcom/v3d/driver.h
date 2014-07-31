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

/* Driver Overview                                                  */

/* A job is logically submitted by the session object to the driver */
/* It is queued on both the session list and driver list.           */
/* Devices remove a job from the driver's issued list, but leave it */
/* on the session list. The user may wait on any job on the session */
/* list. Jobs are removed from the session list only when complete. */
/* Jobs are reference counted, being returned to the free list once */
/* no longer being referenced. The reference count is initially one */
/* The initial reference is released upon completion. Waiters take  */
/* a reference before waiting.                                      */

/* Each job has one of four possible states:                        */
/*  FREE   - On the free list, no references are held on the job    */
/*  ISSUED - On the driver work list and its session's list         */
/*  ACTIVE - Running on a device, removed from the driver work list */
/*  STATUS - Completed. Not on any lists, just waiting for all      */
/*           references to be released (waiters) before freeing     */

/* The resources (lists, jobs ..) for each state are protected by a */
/* spinlock:                                                        */
/*  FREE   - v3d_driver_t::job.free.lock                            */
/*  ISSUED & ACTIVE                                                 */
/*         - v3d_driver_t::job.issued.lock                          */
/*  STATUS - No locks, just the reference count                     */

/* Job link usage by state:                                         */
/* State    FREE           ISSUED          ACTIVE      STATUS       */
/* Session  -              issued.list  ----->         -            */
/* Driver   job.free.list  job.issued.     -           -            */
/*                         [exclusive.]                             */
/*                         user/bin_render                          */

/* When the ISR completes a jobs, the next one will be queued, if   */
/* one is available. Otherwise, the device will set a time-out to   */
/* power-down (currently 100ms). If no jobs are available, the      */
/* device will power-down when this time-out expires.               */

/* Waiters also have a time-out, which starts once they see the job */
/* starting to run. This is currently 2s. If the waiter times-out,  */
/* it cancels the job by reseting the device (and running the       */
/* next job, if any). Care has to be taken to ensure that there     */
/* isn't a race between a job completing normally and being         */
/* cancelled.                                                       */

/* There are only two thread-level synchronisation instances        */
/* - mutexes used to:                                               */
/* 1. Power:    prevents multiple users from switching on a device; */
/* 2. Suspend:  prevents suspension until teh block is off.         */

#ifndef V3D_DRIVER_H_
#define V3D_DRIVER_H_

#include <stddef.h> /* offsetof */
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/wait.h> /* wait_queue_head_t */
#include <linux/list.h>
#include <linux/kref.h>
#include <linux/broadcom/v3d.h>

#include "session.h"
#include "statistics.h"

#define V3D_DEV_NAME	"v3d"
#define V3D_VERSION_STR	"2.1.0\n"

#if 1
#define MY_ASSERT(i, c) \
do { \
	if (!(c)) { \
		printk(KERN_ERR "Assert failed %s:%d: %s\n", __func__, __LINE__, #c); \
		debug_dump(i); \
		BUG_ON(!#c); \
	} \
} while (0)
#else
#define MY_ASSERT(c) BUG_ON(!(c))
#endif

#ifdef CONFIG_DEBUG_SPINLOCK
#define MY_SPINLOCK_CHECK(l, s) \
do { \
	if (spin_is_locked(l) != (s)) \
		printk(KERN_ERR "Assert failed %s:%d: spinlock state %d (not %d)\n", __func__, __LINE__, spin_is_locked(l), (s)); \
} while (0)
#else
#define MY_SPINLOCK_CHECK(l, s)
#endif


struct v3d_device_tag;
struct v3d_session_tag;
struct task_struct;

typedef struct v3d_driver_job_tag {
	v3d_job_post_t         user_job;
	struct list_head       link;
	struct {
		struct list_head      link;
		struct v3d_session_tag *instance;
	} session;
	struct v3d_device_tag   *device;
	wait_queue_head_t      wait_for_completion;
	struct kref            waiters;
#define V3DDRIVER_JOB_INVALID     0
#define V3DDRIVER_JOB_FREE        1
#define V3DDRIVER_JOB_ISSUED      2
#define V3DDRIVER_JOB_ACTIVE      3
#define V3DDRIVER_JOB_STATUS      4
	int                    state;
	int                    status;
	ktime_t                queued;
	ktime_t                start;
	ktime_t                end;
	unsigned int           binning_bytes;

	struct {
		int      enable;
		uint32_t count[16];
	} performance_counter;
} v3d_driver_job_t;

struct v3d_device_tag;
typedef struct v3d_driver_tag {
	unsigned int initialised;

	struct {
		struct {
			struct semaphore count;
			spinlock_t       lock;
			struct list_head list;
		} free;
		struct {
			spinlock_t       lock; /* For all lists */
			struct list_head bin_render;
			struct list_head user;
			struct {
				struct mutex          lock;
				struct v3d_session_tag *owner;
				unsigned int          bin_render_count;
				unsigned int          user_count;
				struct list_head      bin_render;
				struct list_head      user;
			} exclusive;
		} issued;

#define FREE_JOBS 128
		v3d_driver_job_t free_jobs[FREE_JOBS];
	} job;

	ktime_t        start;
	uint32_t       total_run;
	struct {
		statistics_t queue;
		statistics_t run;
		statistics_t binning_bytes;
	} bin_render;
	struct {
		statistics_t queue;
		statistics_t run;
	} user;

	struct v3d_device_tag *device;

#define MAX_SESSIONS 64
	struct mutex            session_lock;
	struct v3d_session_tag *sessions[MAX_SESSIONS];

	struct {
		unsigned int           initialised;
		struct proc_dir_entry *directory;
		struct proc_dir_entry *status;
		struct proc_dir_entry *session;
		struct proc_dir_entry *abort;
		struct proc_dir_entry *device;
		struct proc_dir_entry *version;
		void                  *buffer;
	} proc;
} v3d_driver_t;


extern v3d_driver_t *v3d_driver_create(void);
extern void v3d_driver_add_device(
	v3d_driver_t       *instance,
	struct v3d_device_tag *device);
extern void v3d_driver_delete(v3d_driver_t *instance);

extern int  v3d_driver_job_post(
	v3d_driver_t *instance,
	struct v3d_session_tag *session,
	const v3d_job_post_t *user_job);

extern void v3d_driver_exclusive_start(v3d_driver_t *instance, struct v3d_session_tag *session);
extern int  v3d_driver_exclusive_stop(v3d_driver_t  *instance, struct v3d_session_tag *session);

extern void v3d_driver_reset_statistics(v3d_driver_t *instance);

/* For V3dDevice to use */
static inline unsigned long v3d_driver_issued_lock(v3d_driver_t *instance)
{
	unsigned long flags;
	spin_lock_irqsave(&instance->job.issued.lock, flags);
	return flags;
}
static inline void v3d_driver_issued_unlock(v3d_driver_t *instance, unsigned long flags)
{
	spin_unlock_irqrestore(&instance->job.issued.lock, flags);
}
extern v3d_driver_job_t *v3d_driver_job_get(
	v3d_driver_t *instance,
	unsigned int   required /* if non-zero, specifies job type required */);

/* For V3dSession use */
extern void v3d_driver_job_complete(
	v3d_driver_t     *instance,
	v3d_driver_job_t *job);
/* Returns success? */
/* The issued lock must be held */
extern int v3d_driver_job_cancel(
	v3d_driver_t     *instance,
	v3d_driver_job_t *job);
extern int v3d_driver_add_session(v3d_driver_t *instance, struct v3d_session_tag *session);
extern void v3d_driver_remove_session(v3d_driver_t *instance, struct v3d_session_tag *session);
extern void v3d_driver_kick_consumers(v3d_driver_t *instance);

/* Performance counters */
extern void v3d_driver_counters_enable(v3d_driver_t *instance, uint32_t enables);
extern void v3d_driver_counters_disable(v3d_driver_t *instance);

/* For V3dDriver internal use */
extern int  v3d_driver_create_proc_entries(v3d_driver_t *instance);
extern void v3d_driver_delete_proc_entries(v3d_driver_t *instance);
extern unsigned int session_read(v3d_driver_t *driver, char  *buffer, unsigned int bytes);
extern unsigned int abort_read(v3d_driver_t *driver, char *buffer, unsigned int bytes);
extern unsigned int status_read(v3d_driver_t *driver, char  *buffer, unsigned int bytes);
extern unsigned int device_read(v3d_driver_t *driver, char  *buffer, unsigned int bytes);
extern unsigned int jobs_read(v3d_driver_t *driver, char  *buffer, unsigned int bytes);
extern unsigned int version_read(v3d_driver_t *driver, char *buffer, unsigned int bytes);

/* Debug supprot */
extern void debug_dump(v3d_driver_t *);


#endif /* ifndef V3D_DRIVER_H_ */
