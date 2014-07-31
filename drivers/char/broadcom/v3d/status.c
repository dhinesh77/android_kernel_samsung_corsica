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

#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/math64.h>
#include <linux/list.h>

#include "driver.h"
#include "device.h"
#include "status.h"

#define STATUS_OUTPUT_BYTES (1 << 16)


/* ================================================================ */

typedef struct {
	char          *buffer;
	unsigned int   allocated;
	unsigned int   bytes;
	v3d_driver_t  *driver;
	unsigned int (*get_output)(v3d_driver_t *driver, char *buffer, unsigned int bytes);
} proc_entry_t;

static proc_entry_t session = {
	NULL,
	0,
	0,
	NULL,
	&session_read
};
static proc_entry_t abort = {
	NULL,
	0,
	0,
	NULL,
	&abort_read
};
static proc_entry_t status = {
	NULL,
	0,
	0,
	NULL,
	&status_read
};
static proc_entry_t device = {
	NULL,
	0,
	0,
	NULL,
	&device_read
};
static proc_entry_t jobs = {
	NULL,
	0,
	0,
	NULL,
	&jobs_read
};
static proc_entry_t version = {
	V3D_VERSION_STR,
	sizeof(V3D_VERSION_STR) - 1,
	sizeof(V3D_VERSION_STR) - 1,
	NULL,
	&version_read
};


/* ================================================================ */

struct register_block_tag {
	unsigned int offset32;
	unsigned int n32;
} register_block[] = {
	{ 0x0000 / 4, 0x3c / 4 },
	{ 0x0100 / 4, 0x3c / 4 },
	{ 0x0300 / 4, 0x14 / 4 },
	{ 0x0410 / 4, 0x40 / 4 },
	{ 0x0500 / 4, 0x08 / 4 },
	{ 0x0670 / 4, 0x90 / 4 },
	{ 0x0e00 / 4, 0x40 / 4 },
	{ 0x0f00 / 4, 0x24 / 4 }
};

static uint32_t register_buffer[0x200 /* Must be nig enough for the list above */];


/* ================================================================ */

static int proc_entry_verbose_read(
	char  *buffer,
	char **start,
	off_t  offset,
	int    bytes,
	int   *eof,
	void  *context)
{
	proc_entry_t *instance = (proc_entry_t *) context;
	v3d_driver_t *driver;
	unsigned int copy;
	if (instance == NULL) {
		*eof = 1;
		return 0;
	}
	driver = instance->driver;
	if (offset == 0) {
		if (instance->buffer == NULL) {
			*eof = 1;
			return 0;
		}

		/* Read the full output into our buffer and set the length */
		instance->bytes = (*instance->get_output)(driver, instance->buffer, instance->allocated);
	}

	if (offset >= instance->allocated) {
		*eof = 1;
		return 0; /* No bytes written */
	}

	copy = min(instance->bytes - (unsigned int) offset, (unsigned int) bytes);
	*eof = copy == instance->bytes - offset ? 1 : 0;
	memcpy(*start = buffer, instance->buffer + offset, copy);
	return copy;
}

int dummy_read(
	char  *buffer,
	char **start,
	off_t  offset,
	int    bytes,
	int   *eof,
	void  *data)
{
	*eof = 1;
	return 0;
}

int dummy_write(
	struct file  *file,
	const char   *buffer,
	unsigned long bytes,
	void         *data)
{
	return 0;
}

struct proc_dir_entry *proc_entry_create(
	const char            *name,
	int                    permission,
	struct proc_dir_entry *directory,
	void                  *context)
{
	struct proc_dir_entry *entry = create_proc_entry(name, permission, directory);
	if (entry == NULL)
		return NULL;

	entry->read_proc  = (permission & (S_IRUSR | S_IRGRP)) != 0 ? proc_entry_verbose_read : &dummy_read;
	entry->write_proc = &dummy_write;
	entry->data       = context;
	return entry;
}

void proc_entry_delete(const char *name, struct proc_dir_entry *directory)
{
	remove_proc_entry(name, directory);
}


/* ================================================================ */

static unsigned int my_snprintf(
	char        *buffer,
	unsigned int count,
	const char  *format,
	...)
{
	va_list args;
	int     bytes;
	va_start(args, format);
	bytes = vsnprintf(buffer, count, format, args);
	va_end(args);
	if (bytes < 0 || bytes >= count /* Not enough space */)
		return 0;
	return (unsigned int) bytes;
}

static unsigned int hex_dump_line(
	char           *buffer,
	unsigned int    bytes,
	const char     *prefix,
	unsigned int    print_offset,
	const uint32_t *source,
	unsigned int    from,
	unsigned int    to)
{
	unsigned int i, offset;
	offset = my_snprintf(
		buffer,
		bytes,
		"%s%04x:",
		prefix,
		print_offset);
	for (i = 0 ; i < from ; ++i)
		offset += my_snprintf(
			buffer + offset,
			bytes - offset,
			"         ");
	for (; i < to ; ++i)
		offset += my_snprintf(
			buffer + offset,
			bytes - offset,
			" %08x",
			source[i]);
	return offset += my_snprintf(
		buffer + offset,
		bytes - offset,
		"\n");
}

static unsigned int hex_dump32(
	char           *buffer,
	unsigned int    bytes,
	const char     *prefix,
	unsigned int    print_offset,
	const uint32_t *source,
	unsigned int    n32)
{
	static const unsigned int width = 8 * sizeof(uint32_t);
	unsigned int i, offset = 0;
	unsigned int from   = print_offset % width;
	unsigned int from32 = from / sizeof(uint32_t);
	print_offset -= from;
	source -= from32;
	n32    += from32;
	for (i = 0 ; i < n32 * sizeof(uint32_t) ; i += width, source += width / sizeof(uint32_t), print_offset += width, from32 = 0) {
		offset += hex_dump_line(
			buffer + offset,
			bytes - offset,
			prefix,
			print_offset,
			source,
			from32,
			min(width / sizeof(uint32_t), n32 - i / sizeof(uint32_t)));
	}
	return offset;
}


/* ================================================================ */

unsigned int statistics_output(
	char         *buffer,
	unsigned int  bytes,
	const char   *indent,
	unsigned int  samples,
	unsigned int  mean,
	unsigned int  standard_deviation,
	unsigned int  minimum,
	unsigned int  maximum)
{
	if (samples == 0)
		return my_snprintf(buffer, bytes, "%sNo samples\n", indent);

	if (minimum == maximum)
		return my_snprintf(
			buffer, bytes,
			"%s     %8u (%u sample%s)\n",
			indent,
			mean,
			samples, samples == 1 ? "" : "s");

	return my_snprintf(
		buffer, bytes,
		"%sMean %8u SD %8u Minimum %8u Maximum %8u (%u sample%s)\n",
		indent,
		mean,
		standard_deviation,
		minimum,
		maximum,
		samples, samples == 1 ? "" : "s");
}


/* ================================================================ */

unsigned int version_read(v3d_driver_t *driver, char *buffer, unsigned int bytes)
{
	return bytes; /* Already there */
}

typedef struct {
	uint64_t     mean;
	uint64_t     standard_deviation;
	unsigned int minimum;
	unsigned int maximum;
	unsigned int samples;
} calculated_statistics_t;

static void calculate_job_statistics(
	calculated_statistics_t *instance,
	statistics_t           *statistics)
{
	statistics_calculate(
		statistics,
		1, &instance->mean,
		1, &instance->standard_deviation,
		&instance->minimum, &instance->maximum, &instance->samples);
}

static uint32_t rounding_division64(uint64_t numerator, uint64_t denominator)
{
	return div64_u64(numerator + denominator / 2, denominator);
}

static int output_job_statistics(
	const char                     *description,
	const calculated_statistics_t *statistics,
	char                           *buffer,
	unsigned int                    bytes,
	unsigned int                   *offset,
	unsigned int                    elapsed)
{
	unsigned int job_rate;
	if (statistics[0].samples == 0 && statistics[1].samples == 0)
		return 0;

	job_rate = elapsed / 1000 != 0 ? rounding_division64(10000ULL * (uint64_t) statistics[1].samples, elapsed / 1000) : 0;
	*offset += my_snprintf(
		buffer + *offset, bytes - *offset,
		"  %-16s(%3u.%01u/s)\n",
		description,
		job_rate / 10, job_rate % 10);

	*offset += statistics_output(
		buffer + *offset, bytes - *offset, "   Queue to run (us): ",
		statistics[0].samples,
		(unsigned int) statistics[0].mean,
		(unsigned int) statistics[0].standard_deviation,
		statistics[0].minimum, statistics[0].maximum);
	*offset += statistics_output(
		buffer + *offset, bytes - *offset, "   Run time     (us): ",
		statistics[1].samples,
		(unsigned int) statistics[1].mean,
		(unsigned int) statistics[1].standard_deviation,
		statistics[1].minimum, statistics[1].maximum);
	return 1;
}

unsigned int status_read(v3d_driver_t *driver, char *buffer, unsigned int bytes)
{
	/* Overall stats */
	ktime_t       now     = ktime_get();
	unsigned int  elapsed = ktime_us_delta(now, driver->start);
	unsigned int  run     = driver->total_run;
	unsigned int  load    = elapsed == 0 ? 0 : rounding_division64(1000ULL * (uint64_t) run, (uint64_t) elapsed);
	unsigned int  offset;
	unsigned long flags;
	calculated_statistics_t calculated_statistics[5];
	statistics_t           statistics[5];

	/* Copy-out the raw statistics data to minimise the time we're locked */
	spin_lock_irqsave(&driver->job.issued.lock, flags);
	statistics[0] = driver->bin_render.queue;
	statistics[1] = driver->bin_render.run;
	statistics[2] = driver->user.queue;
	statistics[3] = driver->user.run;
	statistics[4] = driver->bin_render.binning_bytes;
	v3d_driver_reset_statistics(driver);
	spin_unlock_irqrestore(&driver->job.issued.lock, flags);

	/* Now calculate the statistics */
	calculate_job_statistics(&calculated_statistics[0], &statistics[0]);
	calculate_job_statistics(&calculated_statistics[1], &statistics[1]);
	calculate_job_statistics(&calculated_statistics[2], &statistics[2]);
	calculate_job_statistics(&calculated_statistics[3], &statistics[3]);
	calculate_job_statistics(&calculated_statistics[4], &statistics[4]);

	offset = my_snprintf(
		buffer, bytes,
		" Overall                      load %3u.%01u%%\n", load / 10, load % 10);
	output_job_statistics("Bin/Render jobs", &calculated_statistics[0], buffer, bytes, &offset, elapsed);
	if (calculated_statistics[4].samples != 0)
		offset += statistics_output(
			buffer + offset, bytes - offset, "   Binning bytes    : ",
			calculated_statistics[4].samples,
			(unsigned int) calculated_statistics[4].mean,
			(unsigned int) calculated_statistics[4].standard_deviation,
			calculated_statistics[4].minimum, calculated_statistics[4].maximum);
	output_job_statistics("User jobs",       &calculated_statistics[2], buffer, bytes, &offset, elapsed);
	return offset;
}

static unsigned int list_entries(struct list_head *list)
{
	unsigned int n = 0;
	struct list_head *entry;
	list_for_each(entry, list)
		++n;
	return n;
}

unsigned int device_read(v3d_driver_t *driver, char *buffer, unsigned int bytes)
{
	v3d_device_t *device = driver->device;
	v3d_driver_job_t *job;
	unsigned int  i, offset;
	unsigned long flags;
	uint32_t     *registers;
	struct register_block_tag *block;

	flags = v3d_driver_issued_lock(driver);
	job = device->in_progress.job;
	if (job != NULL)
		v3d_session_job_reference(job, __func__);
	v3d_driver_issued_unlock(driver, flags);

	/* Read all the registers that we care about */
	for (i = 0, block = register_block, registers = register_buffer ; i < sizeof(register_block) / sizeof(register_block[0]) ; ++i, registers += block++->n32)
		v3d_device_register_read(device, registers, block->offset32, block->n32);

	offset = my_snprintf(
		buffer, bytes,
		"Device\n"
		" %s, %s\n"
		" Job %p %s %c\n"
		" Binning allocated %u times, freed %u times, have %u\n"
		" Register dump:\n",
		device->on != 0 ? "On" : "Off",
		device->idle != 0 ? "idle" : "running",
		job,
		job == NULL ? "" : "state",
		job == NULL ? ' ' : '0' + job->state,
		device->out_of_memory.index.total.allocated,
		device->out_of_memory.index.total.freed,
		device->out_of_memory.index.allocated);
	if (job != NULL)
		v3d_session_job_release(job, __func__);

	/* Dump out the registers */
	for (i = 0, block = register_block, registers = register_buffer ; i < sizeof(register_block) / sizeof(register_block[0]) ; ++i, registers += block++->n32)
		offset += hex_dump32(
					buffer + offset,
					bytes - offset,
					"  ",
					block->offset32 * sizeof(uint32_t),
					registers,
					block->n32);

	{
		unsigned long flags = v3d_driver_issued_lock(driver);
		unsigned int  queue[4];
		queue[0] = list_entries(&driver->job.issued.exclusive.user);
		queue[1] = list_entries(&driver->job.issued.user);
		queue[2] = list_entries(&driver->job.issued.exclusive.bin_render);
		queue[3] = list_entries(&driver->job.issued.bin_render);
		v3d_driver_issued_unlock(driver, flags);
		offset += my_snprintf(
		buffer + offset, bytes - offset,
		"Driver\n"
		" Queues\n"
		"  User:       exclusive %u, normal %u entries\n"
		"  Bin/render: exclusive %u, normal %u entries\n",
		queue[0], queue[1], queue[2], queue[3]);
	}
	return offset;
}

unsigned int session_read(v3d_driver_t *driver, char *buffer, unsigned int bytes)
{
	unsigned int offset;
	unsigned int i, load, elapsed;
	ktime_t      now;

	if (driver == NULL)
		return my_snprintf(buffer, bytes, "No V3D driver\n");

	offset = my_snprintf(buffer, bytes, "V3D sessions\n");

	/* Session stats */
	for (i = 0 ; i < sizeof(driver->sessions) / sizeof(driver->sessions[0]) ; ++i)
		if (driver->sessions[i] != NULL) {
			v3d_session_t *session = driver->sessions[i];
			calculated_statistics_t calculated_statistics[5];
			statistics_t   statistics[5];
			unsigned int   run = session->total_run;
			v3d_driver_job_t *job;
			unsigned long  flags;

			now     = ktime_get();
			elapsed = ktime_us_delta(now, session->start);
			load    = elapsed == 0 ? 0 : rounding_division64(1000ULL * (uint64_t) run, (uint64_t) elapsed);

			offset += my_snprintf(
				buffer + offset, bytes - offset,
				" Session %-20s load %3u.%01u%%\n",
				session->name != NULL ? session->name : "unknown",
				load / 10, load % 10);

			/* Copy-out the raw statistics data to minimise the time we're locked */
			flags = v3d_driver_issued_lock(driver);
			statistics[0] = session->bin_render.queue;
			statistics[1] = session->bin_render.run;
			statistics[2] = session->user.queue;
			statistics[3] = session->user.run;
			statistics[4] = session->binning_bytes;
			v3d_session_reset_statistics(session);

			/* Get queue info */
			if (list_empty(&session->issued.list) == 0) {
				offset += my_snprintf(
					buffer + offset, bytes - offset,
					"  Jobs\n");
				list_for_each_entry(job, &session->issued.list, session.link)
					offset += my_snprintf(
						buffer + offset, bytes - offset,
						"   j %p ID %8x s %d r %d\n",
						job, job->user_job.job_id, job->state, job->waiters.refcount.counter);
			}
			v3d_driver_issued_unlock(driver, flags);

			/* Now calculate the statistics */
			calculate_job_statistics(&calculated_statistics[0], &statistics[0]);
			calculate_job_statistics(&calculated_statistics[1], &statistics[1]);
			calculate_job_statistics(&calculated_statistics[2], &statistics[2]);
			calculate_job_statistics(&calculated_statistics[3], &statistics[3]);
			calculate_job_statistics(&calculated_statistics[4], &statistics[4]);

			output_job_statistics("Bin/Render jobs", &calculated_statistics[0], buffer, bytes, &offset, elapsed);
			if (calculated_statistics[4].samples != 0)
				offset += statistics_output(
					buffer + offset, bytes - offset, "   Binning bytes    : ",
					calculated_statistics[4].samples,
					(unsigned int) calculated_statistics[4].mean,
					(unsigned int) calculated_statistics[4].standard_deviation,
					calculated_statistics[4].minimum, calculated_statistics[4].maximum);
			output_job_statistics("User jobs",       &calculated_statistics[2], buffer, bytes, &offset, elapsed);
		}

	offset += status_read(driver, buffer + offset, bytes - offset);
	return offset;
}


/* ================================================================ */

unsigned int abort_read(v3d_driver_t *driver, char *buffer, unsigned int bytes)
{
	unsigned int i;
	for (i = 0 ; i < sizeof(driver->sessions) / sizeof(driver->sessions[0]) ; ++i)
		if (driver->sessions[i] != NULL) {
			v3d_session_t *session = driver->sessions[i];
			v3d_session_jobs_abort(session);
		}
	return my_snprintf(
		buffer, bytes,
		"Aborted all jobs\n");
}


/* ================================================================ */

unsigned int jobs_read(v3d_driver_t *driver, char *buffer, unsigned int bytes)
{
	unsigned int offset;
	unsigned int i, n = 0;
	offset = my_snprintf(buffer, bytes, "Jobs\n");
	for (i = 0 ; i < sizeof(driver->job.free_jobs) / sizeof(driver->job.free_jobs[0]) ; ++i) {
		const v3d_driver_job_t *job = &driver->job.free_jobs[i];
		if (job->waiters.refcount.counter != 0 || job->state != V3DDRIVER_JOB_INVALID) {
			++n;
			offset += my_snprintf(
				buffer + offset,
				bytes - offset,
				" %p: ID %08x state %1d type %1d ref %u\n",
				job,
				job->user_job.job_id,
				job->state,
				job->user_job.job_type,
				job->waiters.refcount.counter);
			if ((job->user_job.job_type & (V3D_JOB_BIN | V3D_JOB_REND)) != 0)
				offset += my_snprintf(
					buffer + offset,
					bytes - offset,
					"     0 %08x - %08x  1 %08x - %08x\n",
					job->user_job.v3d_ct0ca,
					job->user_job.v3d_ct0ea,
					job->user_job.v3d_ct1ca,
					job->user_job.v3d_ct1ea);
			if ((job->user_job.job_type & (V3D_JOB_USER)) != 0) {
				unsigned int j;
				for (j = 0 ; j < job->user_job.user_cnt ; ++j)
					offset += my_snprintf(
						buffer + offset,
						bytes - offset,
						"     PC %08x  UA %08x +%6x\n",
						job->user_job.v3d_srqpc,
						job->user_job.v3d_srqua,
						job->user_job.v3d_srqul);
			}
		}
	}

	if (n == 0)
		offset += my_snprintf(
			buffer + offset,
			bytes - offset,
			" All idle\n");

		offset += my_snprintf(
			buffer + offset,
			bytes - offset,
			" %u available\n",
			driver->job.free.count.count);

	return offset;
}


/* ================================================================ */

int v3d_driver_create_proc_entries(v3d_driver_t *instance)
{
	abort.driver = session.driver = status.driver = device.driver = jobs.driver = version.driver = instance;

	instance->proc.initialised = 0;

	instance->proc.buffer = kmalloc(STATUS_OUTPUT_BYTES, GFP_KERNEL);
	if (instance->proc.buffer == NULL)
		return v3d_driver_delete_proc_entries(instance), -1;
	abort.buffer = status.buffer = session.buffer = device.buffer = jobs.buffer = instance->proc.buffer;
	abort.allocated = status.allocated = session.allocated = device.allocated = jobs.allocated = STATUS_OUTPUT_BYTES;
	++instance->proc.initialised;

	instance->proc.directory = proc_mkdir(V3D_DEV_NAME, NULL);
	if (instance->proc.directory == NULL)
		return v3d_driver_delete_proc_entries(instance), -5;
	++instance->proc.initialised;

	instance->proc.status = proc_entry_create("version", S_IRUSR | S_IRGRP | S_IROTH, instance->proc.directory, &version);
	if (instance->proc.status == NULL)
		return v3d_driver_delete_proc_entries(instance), -6;
	++instance->proc.initialised;

	instance->proc.status = proc_entry_create("status", S_IRUSR | S_IRGRP | S_IROTH, instance->proc.directory, &status);
	if (instance->proc.status == NULL)
		return v3d_driver_delete_proc_entries(instance), -7;
	++instance->proc.initialised;

	instance->proc.session = proc_entry_create("session", S_IRUSR | S_IRGRP | S_IROTH, instance->proc.directory, &session);
	if (instance->proc.session == NULL)
		return v3d_driver_delete_proc_entries(instance), -8;
	++instance->proc.initialised;

	instance->proc.device = proc_entry_create("device", S_IRUSR | S_IRGRP | S_IROTH, instance->proc.directory, &device);
	if (instance->proc.device == NULL)
		return v3d_driver_delete_proc_entries(instance), -9;
	++instance->proc.initialised;

	instance->proc.device = proc_entry_create("jobs", S_IRUSR | S_IRGRP | S_IROTH, instance->proc.directory, &jobs);
	if (instance->proc.device == NULL)
		return v3d_driver_delete_proc_entries(instance), -10;
	++instance->proc.initialised;

	instance->proc.abort = proc_entry_create("abort", S_IRUSR | S_IRGRP | S_IROTH, instance->proc.directory, &abort);
	if (instance->proc.abort == NULL)
		return v3d_driver_delete_proc_entries(instance), -11;
	++instance->proc.initialised;

	return 0;
}

void v3d_driver_delete_proc_entries(v3d_driver_t *instance)
{
	switch (instance->proc.initialised) {
	case 8:
		proc_entry_delete("abort", instance->proc.directory);

	case 7:
		proc_entry_delete("jobs", instance->proc.directory);

	case 6:
		proc_entry_delete("device", instance->proc.directory);

	case 5:
		proc_entry_delete("session", instance->proc.directory);

	case 4:
		proc_entry_delete("status", instance->proc.directory);

	case 3:
		proc_entry_delete("version", instance->proc.directory);

	case 2:
		remove_proc_entry(V3D_DEV_NAME, NULL);

	case 1:
		kfree(instance->proc.buffer);

	case 0:
		break;
	}
}
