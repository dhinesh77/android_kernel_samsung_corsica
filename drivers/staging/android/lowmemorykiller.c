/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_adj values will get killed. Specify the
 * minimum oom_adj values in /sys/module/lowmemorykiller/parameters/adj and the
 * number of free pages in /sys/module/lowmemorykiller/parameters/minfree. Both
 * files take a comma separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill processes
 * with a oom_adj value of 8 or higher when the free memory drops below 4096 pages
 * and kill processes with a oom_adj value of 0 or higher when the free memory
 * drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
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
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/notifier.h>
#ifdef CONFIG_ZRAM_FOR_ANDROID
#include <linux/swap.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mm_inline.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <asm/atomic.h>
#include <linux/proc_fs.h>

/* Minimum swapfree pages for running kcompcached */
#define MIN_FREESWAP_PAGE 1280 // 5MB
#endif /* CONFIG_ZRAM_FOR_ANDROID */

static uint32_t lowmem_debug_level = 2;
static int lowmem_adj[6] = {
	0,
	1,
	6,
	12,
};
static int lowmem_adj_size = 4;
static size_t lowmem_minfree[6] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};
static int lowmem_minfree_size = 4;
#ifdef CONFIG_ZRAM_FOR_ANDROID
#define _KCOMPCACHE_DEBUG 0

#if _KCOMPCACHE_DEBUG 
#define lss_dbg(x...) printk("lss: " x)
#else
#define lss_dbg(x...) 
#endif

extern atomic_t early_suspended;
extern atomic_t kswapd_thread_on;

struct soft_reclaim {
	unsigned long nr_total_soft_reclaimed;
	unsigned long nr_total_soft_scanned;
	unsigned long nr_last_soft_reclaimed;
	unsigned long nr_last_soft_scanned;
	int nr_empty_reclaimed;

	atomic_t kcompcached_running;
	atomic_t need_to_reclaim;
	atomic_t lmk_running;
	atomic_t kcompcached_enable;	
	struct task_struct *kcompcached;
};

static struct soft_reclaim s_reclaim = {
	.nr_total_soft_reclaimed = 0,
	.nr_total_soft_scanned = 0,
	.nr_last_soft_reclaimed = 0,
	.nr_last_soft_scanned = 0,
	.nr_empty_reclaimed = 0,
	.kcompcached = NULL,
};

#endif /* CONFIG_ZRAM_FOR_ANDROID */
static struct task_struct *lowmem_deathpending;
static unsigned long lowmem_deathpending_timeout;

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			printk(x);			\
	} while (0)

static int
task_notify_func(struct notifier_block *self, unsigned long val, void *data);

static struct notifier_block task_nb = {
	.notifier_call	= task_notify_func,
};

static int
task_notify_func(struct notifier_block *self, unsigned long val, void *data)
{
	struct task_struct *task = data;

	if (task == lowmem_deathpending) {
		lowmem_print(2, "lowmem_shrink %s/%d is dead!\n",
				task->comm, task->pid);
		lowmem_deathpending = NULL;
	}

	return NOTIFY_OK;
}

static int lowmem_shrink(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *p;
	struct task_struct *selected = NULL;
	int rem = 0;
	int tasksize;
	int i;
	int min_adj = OOM_ADJUST_MAX + 1;
	int selected_tasksize = 0;
	int selected_oom_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free = global_page_state(NR_FREE_PAGES);
	int other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM);

	/*
	 * If CMA is enabled, then do not count free pages
	 * from CMA region and also ignore CMA pages that are
	 * allocated for files.
	 */
#ifdef CONFIG_CMA
	int cma_free, cma_file;

	cma_free = global_page_state(NR_FREE_CMA_PAGES);
	cma_file = global_page_state(NR_CMA_INACTIVE_FILE)
			+ global_page_state(NR_CMA_ACTIVE_FILE);

	other_free -= cma_free;
	other_file -= cma_file;
#endif

#ifdef CONFIG_ZRAM_FOR_ANDROID
	other_file -= total_swapcache_pages;
#endif

	/*
	 * If we already have a death outstanding, then
	 * bail out right away; indicating to vmscan
	 * that we have nothing further to offer on
	 * this pass.
	 *
	 */
	if (lowmem_deathpending &&
	    time_before_eq(jiffies, lowmem_deathpending_timeout))
		return 0;

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		if (other_free < lowmem_minfree[i] &&
		    other_file < lowmem_minfree[i]) {
			min_adj = lowmem_adj[i];
			break;
		}
	}
	if (sc->nr_to_scan > 0)
		lowmem_print(3, "lowmem_shrink %lu, %x, ofree %d %d, ma %d\n",
			     sc->nr_to_scan, sc->gfp_mask, other_free, other_file,
			     min_adj);
	rem = global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
	/*
	 * If CMA is enabled, We will also free up contiguous
	 * allocations done by processes (We cannot free up DMA
	 * allocations that go from CMA region, but we can't count
	 * DMA and PMEM allocations separately right now, so we take
	 * the total.
	 */
#ifdef CONFIG_CMA
	rem += global_page_state(NR_CONTIG_PAGES);
#endif
	if (sc->nr_to_scan <= 0 || min_adj == OOM_ADJUST_MAX + 1) {
		lowmem_print(5, "lowmem_shrink %lu, %x, return %d\n",
			     sc->nr_to_scan, sc->gfp_mask, rem);
		return rem;
	}
	selected_oom_adj = min_adj;
	
#ifdef CONFIG_ZRAM_FOR_ANDROID
	atomic_set(&s_reclaim.lmk_running, 1);
#endif	

	read_lock(&tasklist_lock);
	for_each_process(p) {
		struct mm_struct *mm;
		struct signal_struct *sig;
		int oom_adj;

		task_lock(p);
		mm = p->mm;
		sig = p->signal;
		if (!mm || !sig) {
			task_unlock(p);
			continue;
		}
		oom_adj = sig->oom_adj;
		if (oom_adj < min_adj) {
			task_unlock(p);
			continue;
		}
		tasksize = get_mm_rss(mm);
		task_unlock(p);
		if (tasksize <= 0)
			continue;
		if (selected) {
			if (oom_adj < selected_oom_adj)
				continue;
			if (oom_adj == selected_oom_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_adj = oom_adj;
		lowmem_print(2, "select %d (%s), adj %d, size %d, to kill\n",
			     p->pid, p->comm, oom_adj, tasksize);
	}
	if (selected) {
		lowmem_print(1, "send sigkill to %d (%s), adj %d, size %d"
				" with ofree %d %d, cfree %d %d ma %d\n",
			     selected->pid, selected->comm,
			     selected_oom_adj, selected_tasksize,
			     other_free, other_file, cma_free, cma_file,
			     min_adj);
		lowmem_deathpending = selected;
		lowmem_deathpending_timeout = jiffies + HZ;
		force_sig(SIGKILL, selected);
		rem -= selected_tasksize;
	}
	lowmem_print(4, "lowmem_shrink %lu, %x, return %d\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
	read_unlock(&tasklist_lock);
	
#ifdef CONFIG_ZRAM_FOR_ANDROID
	atomic_set(&s_reclaim.lmk_running, 0);
#endif
	
	return rem;
}

static struct shrinker lowmem_shrinker = {
	.shrink = lowmem_shrink,
	.seeks = DEFAULT_SEEKS * 16
};

#ifdef CONFIG_ZRAM_FOR_ANDROID
void could_cswap(void)
{
	struct sysinfo i;
		
	if (unlikely(s_reclaim.kcompcached == NULL)) {
		return;
	}

	if (likely(atomic_read(&s_reclaim.kcompcached_enable) == 0)) {
		return;
	}
	
	if (atomic_read(&s_reclaim.need_to_reclaim) != 1 ||
		atomic_read(&s_reclaim.lmk_running) == 1 ||
		atomic_read(&kswapd_thread_on) == 1) {
		return;
	}
	
	si_swapinfo(&i);
	if (i.freeswap < MIN_FREESWAP_PAGE) {
		return;
	}	

	if (idle_cpu(task_cpu(s_reclaim.kcompcached)) && this_cpu_loadx(4) == 0) {
		if (atomic_read(&s_reclaim.kcompcached_running) == 0) {
			lss_dbg("wakeup kcompcached\n");
			wake_up_process(s_reclaim.kcompcached);
			atomic_set(&s_reclaim.kcompcached_running, 1);
		}
	}
}

inline void enable_soft_reclaim(void)
{
	atomic_set(&s_reclaim.kcompcached_enable, 1);
}

inline void disable_soft_reclaim(void)
{
	atomic_set(&s_reclaim.kcompcached_enable, 0);
}

inline void need_soft_reclaim(void)
{
	atomic_set(&s_reclaim.need_to_reclaim, 1);
				}

inline void cancel_soft_reclaim(void)
{
	atomic_set(&s_reclaim.need_to_reclaim, 0);
			}

static int soft_reclaim(void)
{
	int nid;
	int i;
	unsigned long nr_soft_reclaimed;
	unsigned long nr_soft_scanned;
	unsigned long nr_reclaimed = 0;

	for_each_node_state(nid, N_HIGH_MEMORY) {
		pg_data_t *pgdat = NODE_DATA(nid);
		for (i = 0; i <= 1; i++) {
			struct zone *zone = pgdat->node_zones + i;
			if (!populated_zone(zone))
				continue;
			if (zone->all_unreclaimable)
				continue;

			nr_soft_scanned = 0;
			nr_soft_reclaimed = mem_cgroup_soft_limit_reclaim(zone,
						0, GFP_KERNEL,
						&nr_soft_scanned);

			s_reclaim.nr_last_soft_reclaimed = nr_soft_reclaimed;
			s_reclaim.nr_last_soft_scanned = nr_soft_scanned;
			s_reclaim.nr_total_soft_reclaimed += nr_soft_reclaimed;
			s_reclaim.nr_total_soft_scanned += nr_soft_scanned;

			nr_reclaimed += nr_soft_reclaimed;
	}
}

	lss_dbg("soft reclaimed %ld pages\n", nr_reclaimed);
	return nr_reclaimed;
}

static int do_compcache(void * nothing)
{
	int ret;
	set_freezable();

	for ( ; ; ) {
		ret = try_to_freeze();
		if (kthread_should_stop())
			break;

		if (soft_reclaim() == 0)
			s_reclaim.nr_empty_reclaimed++;
		else
			s_reclaim.nr_empty_reclaimed = 0;

		if (likely(atomic_read(&early_suspended) == 1)) {
			if (s_reclaim.nr_empty_reclaimed > 2)
				disable_soft_reclaim();
			else
				continue;
		}

		cancel_soft_reclaim();

		atomic_set(&s_reclaim.kcompcached_running, 0);
        set_current_state(TASK_INTERRUPTIBLE);
        schedule();
		}

	return 0;
				}

static ssize_t kcompcache_store(struct class *class, struct class_attribute *attr,
                const char *buf, size_t count)
{
	ssize_t ret = -EINVAL;
	long val;

	ret = strict_strtol(buf, 10, &val);
	if (ret == 0 && val == 0) {
		s_reclaim.nr_total_soft_reclaimed = 0;
		s_reclaim.nr_total_soft_scanned = 0;
		s_reclaim.nr_last_soft_reclaimed = 0;
		s_reclaim.nr_last_soft_scanned = 0;
				}

	return count;
			}

static ssize_t kcompcache_show(struct class *class,
                struct class_attribute *attr, char *buf)
{
    int len = 0;

    len += snprintf(buf + len, PAGE_SIZE, "nr_total_soft_reclaimed\t: %ld pages\n", s_reclaim.nr_total_soft_reclaimed);
    len += snprintf(buf + len, PAGE_SIZE, "nr_total_soft_scanned\t: %ld pages\n", s_reclaim.nr_total_soft_scanned);
    len += snprintf(buf + len, PAGE_SIZE, "nr_last_soft_reclaimed\t: %ld pages\n", s_reclaim.nr_last_soft_reclaimed);
    len += snprintf(buf + len, PAGE_SIZE, "nr_last_soft_scanned\t: %ld pages\n", s_reclaim.nr_last_soft_scanned);
    len += snprintf(buf + len, PAGE_SIZE, "nr_empty_reclaimed\t: %d times\n", s_reclaim.nr_empty_reclaimed);
    len += snprintf(buf + len, PAGE_SIZE, "early_suspended\t\t: %d \n", atomic_read(&early_suspended));
    len += snprintf(buf + len, PAGE_SIZE, "kswapd_thread_on\t: %d \n", atomic_read(&kswapd_thread_on));
    len += snprintf(buf + len, PAGE_SIZE, "need_to_reclaim\t\t: %d \n", atomic_read(&s_reclaim.need_to_reclaim));
    len += snprintf(buf + len, PAGE_SIZE, "lmk_running\t: %d \n", atomic_read(&s_reclaim.lmk_running));
    len += snprintf(buf + len, PAGE_SIZE, "kcompcached_enable\t: %d \n", atomic_read(&s_reclaim.kcompcached_enable));
    len += snprintf(buf + len, PAGE_SIZE, "kcompcached_running\t: %d \n", atomic_read(&s_reclaim.kcompcached_running));

    return (ssize_t) len;
}

static CLASS_ATTR(kcompcache, 0600, kcompcache_show, kcompcache_store);
static struct class *kcompcache_class;
#endif /* CONFIG_ZRAM_FOR_ANDROID */

static int __init lowmem_init(void)
{
	task_free_register(&task_nb);
	register_shrinker(&lowmem_shrinker);

#ifdef CONFIG_ZRAM_FOR_ANDROID
    kcompcache_class = class_create(THIS_MODULE, "kcompcache");
    if (IS_ERR(kcompcache_class)) {
		pr_err("%s: couldn't create kcompcache sysfs class.\n", __func__);
		goto error_create_kcompcache_class;
		}

	if (class_create_file(kcompcache_class, &class_attr_kcompcache) < 0) {
		pr_err("%s: couldn't create kcompcache sysfs file.\n", __func__);
		goto error_create_kcompcache_class_file;
	}
	
    s_reclaim.kcompcached = kthread_run(do_compcache, NULL, "kcompcached");
    if (IS_ERR(s_reclaim.kcompcached)) {
		/* failure at boot is fatal */
		BUG_ON(system_state == SYSTEM_BOOTING); 
    }
	set_user_nice(s_reclaim.kcompcached, 15);
	atomic_set(&s_reclaim.need_to_reclaim, 0);
	atomic_set(&s_reclaim.kcompcached_running, 0);
	enable_soft_reclaim();
	
		return 0;

error_create_kcompcache_class_file:
    class_remove_file(kcompcache_class, &class_attr_kcompcache);
error_create_kcompcache_class:
    class_destroy(kcompcache_class);
#endif /* CONFIG_ZRAM_FOR_ANDROID */

	return 0;
}

static void __exit lowmem_exit(void)
{
	unregister_shrinker(&lowmem_shrinker);
	task_free_unregister(&task_nb);

#ifdef CONFIG_ZRAM_FOR_ANDROID
	if (s_reclaim.kcompcached) {
		cancel_soft_reclaim();
		kthread_stop(s_reclaim.kcompcached);
		s_reclaim.kcompcached = NULL;
	}

	if (kcompcache_class) {
		class_remove_file(kcompcache_class, &class_attr_kcompcache);
		class_destroy(kcompcache_class);
	}
#endif /* CONFIG_ZRAM_FOR_ANDROID */
}

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
module_param_array_named(adj, lowmem_adj, int, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);

module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

