// SPDX-License-Identifier: GPL-2.0
/*
 * The Kyber I/O scheduler. Controls latency by throttling queue depths using
 * scalable techniques.
 *
 * Copyright (C) 2017 Facebook
 */

#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/hrtimer.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/sbitmap.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include <trace/events/block.h>

#include "elevator.h"
#include "blk.h"
#include "blk-mq.h"
#include "blk-mq-debugfs.h"
#include "blk-mq-sched.h"
#include "blk-mq-tag.h"

#define CREATE_TRACE_POINTS
#include <trace/events/kyber.h>

/*
 * Scheduling domains: the device is divided into multiple domains based on the
 * request type.
 */
enum {
	KYBER_READ,
	KYBER_WRITE,
	KYBER_DISCARD,
	KYBER_OTHER,
	KYBER_NUM_DOMAINS,
};

static const char *kyber_domain_names[] = {
	[KYBER_READ] = "READ",
	[KYBER_WRITE] = "WRITE",
	[KYBER_DISCARD] = "DISCARD",
	[KYBER_OTHER] = "OTHER",
};

enum {
	/*
	 * In order to prevent starvation of synchronous requests by a flood of
	 * asynchronous requests, we reserve 25% of requests for synchronous
	 * operations.
	 */
	KYBER_ASYNC_PERCENT = 75,
};

/* QueueTopia state uses elv.priv[1]; Kyber keeps its domain token in priv[0]. */
enum {
	KYBER_QT_TRACKED	= BIT(0),
	KYBER_QT_ADMITTED	= BIT(1),
	KYBER_QT_RETRY		= BIT(2),
	KYBER_QT_TERMINAL	= BIT(3),
};

#define KYBER_QT_CREDIT_UNIT	((u64)NSEC_PER_SEC)
#define KYBER_QT_DEFAULT_RATE	1000000U
#define KYBER_QT_DEFAULT_BURST	128U
#define KYBER_QT_DEFAULT_CAP	128U
#define KYBER_QT_MAX_RATE	1000000U
#define KYBER_QT_MAX_BURST	4096U
#define KYBER_QT_MAX_CAP	65536U
#define KYBER_QT_MAX_CONFIRM	64U

enum kyber_qt_mode {
	KYBER_QT_TRACK,
	KYBER_QT_DRAIN,
};

/*
 * Maximum device-wide depth for each scheduling domain.
 *
 * Even for fast devices with lots of tags like NVMe, you can saturate the
 * device with only a fraction of the maximum possible queue depth. So, we cap
 * these to a reasonable value.
 */
static const unsigned int kyber_depth[] = {
	[KYBER_READ] = 256,
	[KYBER_WRITE] = 128,
	[KYBER_DISCARD] = 64,
	[KYBER_OTHER] = 16,
};

/*
 * Default latency targets for each scheduling domain.
 */
static const u64 kyber_latency_targets[] = {
	[KYBER_READ] = 2ULL * NSEC_PER_MSEC,
	[KYBER_WRITE] = 10ULL * NSEC_PER_MSEC,
	[KYBER_DISCARD] = 5ULL * NSEC_PER_SEC,
};

/*
 * Batch size (number of requests we'll dispatch in a row) for each scheduling
 * domain.
 */
static const unsigned int kyber_batch_size[] = {
	[KYBER_READ] = 16,
	[KYBER_WRITE] = 8,
	[KYBER_DISCARD] = 1,
	[KYBER_OTHER] = 1,
};

/*
 * Requests latencies are recorded in a histogram with buckets defined relative
 * to the target latency:
 *
 * <= 1/4 * target latency
 * <= 1/2 * target latency
 * <= 3/4 * target latency
 * <= target latency
 * <= 1 1/4 * target latency
 * <= 1 1/2 * target latency
 * <= 1 3/4 * target latency
 * > 1 3/4 * target latency
 */
enum {
	/*
	 * The width of the latency histogram buckets is
	 * 1 / (1 << KYBER_LATENCY_SHIFT) * target latency.
	 */
	KYBER_LATENCY_SHIFT = 2,
	/*
	 * The first (1 << KYBER_LATENCY_SHIFT) buckets are <= target latency,
	 * thus, "good".
	 */
	KYBER_GOOD_BUCKETS = 1 << KYBER_LATENCY_SHIFT,
	/* There are also (1 << KYBER_LATENCY_SHIFT) "bad" buckets. */
	KYBER_LATENCY_BUCKETS = 2 << KYBER_LATENCY_SHIFT,
};

/*
 * We measure both the total latency and the I/O latency (i.e., latency after
 * submitting to the device).
 */
enum {
	KYBER_TOTAL_LATENCY,
	KYBER_IO_LATENCY,
};

static const char *kyber_latency_type_names[] = {
	[KYBER_TOTAL_LATENCY] = "total",
	[KYBER_IO_LATENCY] = "I/O",
};

/*
 * Per-cpu latency histograms: total latency and I/O latency for each scheduling
 * domain except for KYBER_OTHER.
 */
struct kyber_cpu_latency {
	atomic_t buckets[KYBER_OTHER][2][KYBER_LATENCY_BUCKETS];
};

/*
 * There is a same mapping between ctx & hctx and kcq & khd,
 * we use request->mq_ctx->index_hw to index the kcq in khd.
 */
struct kyber_ctx_queue {
	/*
	 * Used to ensure operations on rq_list and kcq_map to be an atmoic one.
	 * Also protect the rqs on rq_list when merge.
	 */
	spinlock_t lock;
	struct list_head rq_list[KYBER_NUM_DOMAINS];
} ____cacheline_aligned_in_smp;

struct kyber_queue_data {
	struct request_queue *q;
	dev_t dev;

	/*
	 * Each scheduling domain has a limited number of in-flight requests
	 * device-wide, limited by these tokens.
	 */
	struct sbitmap_queue domain_tokens[KYBER_NUM_DOMAINS];

	/*
	 * Async request percentage, converted to per-word depth for
	 * sbitmap_get_shallow().
	 */
	unsigned int async_depth;

	struct kyber_cpu_latency __percpu *cpu_latency;

	/* Timer for stats aggregation and adjusting domain tokens. */
	struct timer_list timer;

	/* QueueTopia admission state shared by all hardware queues. */
	spinlock_t qt_lock;
	struct hrtimer qt_wake_timer;
	struct hrtimer qt_sample_timer;
	struct work_struct qt_sample_work;
	bool qt_enabled;
	u32 qt_control;		/* 0=observe, 1=fixed gate, 2=adaptive. */
	bool qt_stopping;
	u32 qt_rate;		/* First admissions per second. */
	u32 qt_initial_rate;
	u32 qt_max_rate;
	u32 qt_alpha;		/* Recovery increase, requests per second. */
	u32 qt_beta;		/* Drain multiplier, percent. */
	u32 qt_epsilon;		/* Growth trigger, requests per second. */
	u32 qt_confirm;
	u32 qt_sample_ms;
	u32 qt_burst;		/* Maximum accumulated rate credits. */
	u32 qt_cap;		/* Maximum admitted, unterminated requests. */
	u32 qt_low;
	u32 qt_high;
	enum kyber_qt_mode qt_mode;
	u64 qt_credit;		/* Rate credits scaled by NSEC_PER_SEC. */
	u64 qt_last_ns;
	u64 qt_prev_ns;
	u64 qt_prev_q;
	u64 qt_sample_elapsed_ns;
	u64 qt_prev_domain_blocked;
	s64 qt_growth_rate;
	bool qt_sample_valid;
	u32 qt_growth_streak;
	u32 qt_drain_streak;
	u64 qt_generated;
	u64 qt_admitted;
	u64 qt_completed;
	u64 qt_exit_new;
	u64 qt_exit_admitted;
	u64 qt_rate_blocked;
	u64 qt_cap_blocked;
	u64 qt_domain_blocked;
	u64 qt_requeues;
	u64 qt_b_new;
	u64 qt_q_adm;
	u64 qt_b_retry;

	unsigned int latency_buckets[KYBER_OTHER][2][KYBER_LATENCY_BUCKETS];

	unsigned long latency_timeout[KYBER_OTHER];

	int domain_p99[KYBER_OTHER];

	/* Target latencies in nanoseconds. */
	u64 latency_targets[KYBER_OTHER];
};

struct kyber_hctx_data {
	spinlock_t lock;
	struct list_head rqs[KYBER_NUM_DOMAINS];
	struct list_head retry_rqs;
	unsigned int cur_domain;
	unsigned int batching;
	struct kyber_ctx_queue *kcqs;
	struct sbitmap kcq_map[KYBER_NUM_DOMAINS];
	struct sbq_wait domain_wait[KYBER_NUM_DOMAINS];
	struct sbq_wait_state *domain_ws[KYBER_NUM_DOMAINS];
	atomic_t wait_index[KYBER_NUM_DOMAINS];
};

static unsigned long kyber_qt_rq_state(struct request *rq)
{
	return (unsigned long)rq->elv.priv[1];
}

static void kyber_qt_set_rq_state(struct request *rq, unsigned long state)
{
	rq->elv.priv[1] = (void *)state;
}

static enum hrtimer_restart kyber_qt_wake_timer_fn(struct hrtimer *timer)
{
	struct kyber_queue_data *kqd =
		container_of(timer, struct kyber_queue_data, qt_wake_timer);

	if (!READ_ONCE(kqd->qt_stopping))
		blk_mq_run_hw_queues(kqd->q, true);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart kyber_qt_sample_timer_fn(struct hrtimer *timer)
{
	struct kyber_queue_data *kqd =
		container_of(timer, struct kyber_queue_data, qt_sample_timer);

	if (!READ_ONCE(kqd->qt_stopping))
		schedule_work(&kqd->qt_sample_work);

	return HRTIMER_NORESTART;
}

static bool kyber_qt_set_rate_locked(struct kyber_queue_data *kqd, u32 rate,
				     u64 now)
{
	if (rate == kqd->qt_rate)
		return false;

	if (rate < kqd->qt_rate)
		kqd->qt_credit = 0;
	kqd->qt_rate = rate;
	kqd->qt_last_ns = now;
	return true;
}

static void kyber_qt_sample_workfn(struct work_struct *work)
{
	struct kyber_queue_data *kqd =
		container_of(work, struct kyber_queue_data, qt_sample_work);
	unsigned long flags;
	u64 now = ktime_get_ns();
	u64 q_adm, b_new, growth = 0, drain_rate = 0, previous_q;
	u64 domain_blocked, sample_elapsed = 0, sample_period;
	u32 next_rate, current_rate = 0;
	s64 growth_rate = 0;
	bool valid_sample = false, domain_pressure = false;
	bool rate_changed = false, run_queues = false;
	bool draining = false;

	spin_lock_irqsave(&kqd->qt_lock, flags);
	if (READ_ONCE(kqd->qt_stopping))
		goto out;

	q_adm = kqd->qt_q_adm;
	b_new = kqd->qt_b_new;
	domain_blocked = kqd->qt_domain_blocked;
	previous_q = kqd->qt_prev_q;
	if (kqd->qt_prev_ns && now > kqd->qt_prev_ns) {
		sample_elapsed = now - kqd->qt_prev_ns;
		sample_period = (u64)kqd->qt_sample_ms * NSEC_PER_MSEC;
		valid_sample = sample_elapsed <= 2 * sample_period;
		if (valid_sample) {
			if (q_adm > previous_q)
				growth = div64_u64((q_adm - previous_q) *
						   NSEC_PER_SEC, sample_elapsed);
			else if (q_adm < previous_q)
				drain_rate = div64_u64((previous_q - q_adm) *
						       NSEC_PER_SEC, sample_elapsed);
		}
	}
	kqd->qt_sample_elapsed_ns = sample_elapsed;
	kqd->qt_sample_valid = valid_sample;
	growth_rate = (s64)growth - (s64)drain_rate;
	kqd->qt_growth_rate = growth_rate;
	if (valid_sample && growth > kqd->qt_epsilon)
		kqd->qt_growth_streak = min(kqd->qt_growth_streak + 1,
					    kqd->qt_confirm);
	else
		kqd->qt_growth_streak = 0;
	if (!valid_sample)
		kqd->qt_drain_streak = 0;
	else if (domain_blocked > kqd->qt_prev_domain_blocked)
		domain_pressure = true;

	if (kqd->qt_control < 2) {
		kqd->qt_mode = KYBER_QT_TRACK;
	} else if (q_adm == 0 && b_new == 0) {
		kqd->qt_mode = KYBER_QT_TRACK;
		kqd->qt_growth_streak = 0;
		kqd->qt_drain_streak = 0;
		rate_changed = kyber_qt_set_rate_locked(kqd,
							kqd->qt_initial_rate, now);
	} else if (kqd->qt_mode == KYBER_QT_TRACK) {
		bool pressure = q_adm >= kqd->qt_high || domain_pressure ||
			kqd->qt_growth_streak >= kqd->qt_confirm;

		if (pressure) {
			kqd->qt_mode = KYBER_QT_DRAIN;
			kqd->qt_drain_streak = 0;
			next_rate = div_u64((u64)kqd->qt_rate * kqd->qt_beta,
					    100);
			next_rate = max_t(u32, next_rate, 1);
			rate_changed = kyber_qt_set_rate_locked(kqd, next_rate, now);
		} else if (valid_sample && b_new && q_adm <= kqd->qt_low &&
			   growth <= kqd->qt_epsilon) {
			next_rate = kqd->qt_rate + kqd->qt_alpha;
			next_rate = min(next_rate, kqd->qt_max_rate);
			rate_changed = kyber_qt_set_rate_locked(kqd, next_rate, now);
		}
	} else {
		if (valid_sample && q_adm <= kqd->qt_low &&
		    growth <= kqd->qt_epsilon)
			kqd->qt_drain_streak = min(kqd->qt_drain_streak + 1,
						   kqd->qt_confirm);
		else
			kqd->qt_drain_streak = 0;

		if (kqd->qt_drain_streak >= kqd->qt_confirm) {
			kqd->qt_mode = KYBER_QT_TRACK;
			kqd->qt_drain_streak = 0;
		} else if (valid_sample && q_adm > kqd->qt_low &&
			   drain_rate <= kqd->qt_epsilon) {
			next_rate = div_u64((u64)kqd->qt_rate * kqd->qt_beta,
					    100);
			next_rate = max_t(u32, next_rate, 1);
			rate_changed = kyber_qt_set_rate_locked(kqd, next_rate, now);
		}
	}

	kqd->qt_prev_q = q_adm;
	kqd->qt_prev_ns = now;
	if (valid_sample || !kqd->qt_prev_domain_blocked)
		kqd->qt_prev_domain_blocked = domain_blocked;
	current_rate = kqd->qt_rate;
	draining = kqd->qt_mode == KYBER_QT_DRAIN;
	run_queues = rate_changed && b_new;
out:
	spin_unlock_irqrestore(&kqd->qt_lock, flags);

	if (rate_changed)
		trace_kyber_qt_rate(kqd->dev, current_rate, q_adm, growth_rate,
				    draining);
	if (run_queues && !READ_ONCE(kqd->qt_stopping))
		blk_mq_run_hw_queues(kqd->q, true);
	if (!READ_ONCE(kqd->qt_stopping))
		hrtimer_start(&kqd->qt_sample_timer,
			      ms_to_ktime(READ_ONCE(kqd->qt_sample_ms)),
			      HRTIMER_MODE_REL);
}

static int kyber_domain_wake(wait_queue_entry_t *wait, unsigned mode, int flags,
			     void *key);

static unsigned int kyber_sched_domain(blk_opf_t opf)
{
	switch (opf & REQ_OP_MASK) {
	case REQ_OP_READ:
		return KYBER_READ;
	case REQ_OP_WRITE:
		return KYBER_WRITE;
	case REQ_OP_DISCARD:
		return KYBER_DISCARD;
	default:
		return KYBER_OTHER;
	}
}

static void flush_latency_buckets(struct kyber_queue_data *kqd,
				  struct kyber_cpu_latency *cpu_latency,
				  unsigned int sched_domain, unsigned int type)
{
	unsigned int *buckets = kqd->latency_buckets[sched_domain][type];
	atomic_t *cpu_buckets = cpu_latency->buckets[sched_domain][type];
	unsigned int bucket;

	for (bucket = 0; bucket < KYBER_LATENCY_BUCKETS; bucket++)
		buckets[bucket] += atomic_xchg(&cpu_buckets[bucket], 0);
}

/*
 * Calculate the histogram bucket with the given percentile rank, or -1 if there
 * aren't enough samples yet.
 */
static int calculate_percentile(struct kyber_queue_data *kqd,
				unsigned int sched_domain, unsigned int type,
				unsigned int percentile)
{
	unsigned int *buckets = kqd->latency_buckets[sched_domain][type];
	unsigned int bucket, samples = 0, percentile_samples;

	for (bucket = 0; bucket < KYBER_LATENCY_BUCKETS; bucket++)
		samples += buckets[bucket];

	if (!samples)
		return -1;

	/*
	 * We do the calculation once we have 500 samples or one second passes
	 * since the first sample was recorded, whichever comes first.
	 */
	if (!kqd->latency_timeout[sched_domain])
		kqd->latency_timeout[sched_domain] = max(jiffies + HZ, 1UL);
	if (samples < 500 &&
	    time_is_after_jiffies(kqd->latency_timeout[sched_domain])) {
		return -1;
	}
	kqd->latency_timeout[sched_domain] = 0;

	percentile_samples = DIV_ROUND_UP(samples * percentile, 100);
	for (bucket = 0; bucket < KYBER_LATENCY_BUCKETS - 1; bucket++) {
		if (buckets[bucket] >= percentile_samples)
			break;
		percentile_samples -= buckets[bucket];
	}
	memset(buckets, 0, sizeof(kqd->latency_buckets[sched_domain][type]));

	trace_kyber_latency(kqd->dev, kyber_domain_names[sched_domain],
			    kyber_latency_type_names[type], percentile,
			    bucket + 1, 1 << KYBER_LATENCY_SHIFT, samples);

	return bucket;
}

static void kyber_resize_domain(struct kyber_queue_data *kqd,
				unsigned int sched_domain, unsigned int depth)
{
	depth = clamp(depth, 1U, kyber_depth[sched_domain]);
	if (depth != kqd->domain_tokens[sched_domain].sb.depth) {
		sbitmap_queue_resize(&kqd->domain_tokens[sched_domain], depth);
		trace_kyber_adjust(kqd->dev, kyber_domain_names[sched_domain],
				   depth);
	}
}

static void kyber_timer_fn(struct timer_list *t)
{
	struct kyber_queue_data *kqd = from_timer(kqd, t, timer);
	unsigned int sched_domain;
	int cpu;
	bool bad = false;

	/* Sum all of the per-cpu latency histograms. */
	for_each_online_cpu(cpu) {
		struct kyber_cpu_latency *cpu_latency;

		cpu_latency = per_cpu_ptr(kqd->cpu_latency, cpu);
		for (sched_domain = 0; sched_domain < KYBER_OTHER; sched_domain++) {
			flush_latency_buckets(kqd, cpu_latency, sched_domain,
					      KYBER_TOTAL_LATENCY);
			flush_latency_buckets(kqd, cpu_latency, sched_domain,
					      KYBER_IO_LATENCY);
		}
	}

	/*
	 * Check if any domains have a high I/O latency, which might indicate
	 * congestion in the device. Note that we use the p90; we don't want to
	 * be too sensitive to outliers here.
	 */
	for (sched_domain = 0; sched_domain < KYBER_OTHER; sched_domain++) {
		int p90;

		p90 = calculate_percentile(kqd, sched_domain, KYBER_IO_LATENCY,
					   90);
		if (p90 >= KYBER_GOOD_BUCKETS)
			bad = true;
	}

	/*
	 * Adjust the scheduling domain depths. If we determined that there was
	 * congestion, we throttle all domains with good latencies. Either way,
	 * we ease up on throttling domains with bad latencies.
	 */
	for (sched_domain = 0; sched_domain < KYBER_OTHER; sched_domain++) {
		unsigned int orig_depth, depth;
		int p99;

		p99 = calculate_percentile(kqd, sched_domain,
					   KYBER_TOTAL_LATENCY, 99);
		/*
		 * This is kind of subtle: different domains will not
		 * necessarily have enough samples to calculate the latency
		 * percentiles during the same window, so we have to remember
		 * the p99 for the next time we observe congestion; once we do,
		 * we don't want to throttle again until we get more data, so we
		 * reset it to -1.
		 */
		if (bad) {
			if (p99 < 0)
				p99 = kqd->domain_p99[sched_domain];
			kqd->domain_p99[sched_domain] = -1;
		} else if (p99 >= 0) {
			kqd->domain_p99[sched_domain] = p99;
		}
		if (p99 < 0)
			continue;

		/*
		 * If this domain has bad latency, throttle less. Otherwise,
		 * throttle more iff we determined that there is congestion.
		 *
		 * The new depth is scaled linearly with the p99 latency vs the
		 * latency target. E.g., if the p99 is 3/4 of the target, then
		 * we throttle down to 3/4 of the current depth, and if the p99
		 * is 2x the target, then we double the depth.
		 */
		if (bad || p99 >= KYBER_GOOD_BUCKETS) {
			orig_depth = kqd->domain_tokens[sched_domain].sb.depth;
			depth = (orig_depth * (p99 + 1)) >> KYBER_LATENCY_SHIFT;
			kyber_resize_domain(kqd, sched_domain, depth);
		}
	}
}

static struct kyber_queue_data *kyber_queue_data_alloc(struct request_queue *q)
{
	struct kyber_queue_data *kqd;
	int ret = -ENOMEM;
	int i;

	kqd = kzalloc_node(sizeof(*kqd), GFP_KERNEL, q->node);
	if (!kqd)
		goto err;

	kqd->q = q;
	kqd->dev = disk_devt(q->disk);
	spin_lock_init(&kqd->qt_lock);
	kqd->qt_rate = KYBER_QT_DEFAULT_RATE;
	kqd->qt_initial_rate = KYBER_QT_DEFAULT_RATE;
	kqd->qt_max_rate = KYBER_QT_MAX_RATE;
	kqd->qt_alpha = 10000;
	kqd->qt_beta = 50;
	kqd->qt_epsilon = 500;
	kqd->qt_confirm = 3;
	kqd->qt_sample_ms = 10;
	kqd->qt_burst = KYBER_QT_DEFAULT_BURST;
	kqd->qt_cap = KYBER_QT_DEFAULT_CAP;
	kqd->qt_low = 48;
	kqd->qt_high = 96;
	kqd->qt_mode = KYBER_QT_TRACK;
	kqd->qt_last_ns = ktime_get_ns();
	hrtimer_init(&kqd->qt_wake_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	kqd->qt_wake_timer.function = kyber_qt_wake_timer_fn;
	hrtimer_init(&kqd->qt_sample_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	kqd->qt_sample_timer.function = kyber_qt_sample_timer_fn;
	INIT_WORK(&kqd->qt_sample_work, kyber_qt_sample_workfn);

	kqd->cpu_latency = alloc_percpu_gfp(struct kyber_cpu_latency,
					    GFP_KERNEL | __GFP_ZERO);
	if (!kqd->cpu_latency)
		goto err_kqd;

	timer_setup(&kqd->timer, kyber_timer_fn, 0);

	for (i = 0; i < KYBER_NUM_DOMAINS; i++) {
		WARN_ON(!kyber_depth[i]);
		WARN_ON(!kyber_batch_size[i]);
		ret = sbitmap_queue_init_node(&kqd->domain_tokens[i],
					      kyber_depth[i], -1, false,
					      GFP_KERNEL, q->node);
		if (ret) {
			while (--i >= 0)
				sbitmap_queue_free(&kqd->domain_tokens[i]);
			goto err_buckets;
		}
	}

	for (i = 0; i < KYBER_OTHER; i++) {
		kqd->domain_p99[i] = -1;
		kqd->latency_targets[i] = kyber_latency_targets[i];
	}

	return kqd;

err_buckets:
	free_percpu(kqd->cpu_latency);
err_kqd:
	kfree(kqd);
err:
	return ERR_PTR(ret);
}

static int kyber_init_sched(struct request_queue *q, struct elevator_type *e)
{
	struct kyber_queue_data *kqd;
	struct elevator_queue *eq;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	kqd = kyber_queue_data_alloc(q);
	if (IS_ERR(kqd)) {
		kobject_put(&eq->kobj);
		return PTR_ERR(kqd);
	}
	kqd->qt_enabled = !strcmp(e->elevator_name, "kyber-qt");

	blk_stat_enable_accounting(q);

	blk_queue_flag_clear(QUEUE_FLAG_SQ_SCHED, q);

	eq->elevator_data = kqd;
	q->elevator = eq;
	if (kqd->qt_enabled)
		hrtimer_start(&kqd->qt_sample_timer,
			      ms_to_ktime(kqd->qt_sample_ms), HRTIMER_MODE_REL);

	return 0;
}

static void kyber_exit_sched(struct elevator_queue *e)
{
	struct kyber_queue_data *kqd = e->elevator_data;
	int i;

	WRITE_ONCE(kqd->qt_stopping, true);
	hrtimer_cancel(&kqd->qt_wake_timer);
	hrtimer_cancel(&kqd->qt_sample_timer);
	cancel_work_sync(&kqd->qt_sample_work);
	hrtimer_cancel(&kqd->qt_sample_timer);
	del_timer_sync(&kqd->timer);
	blk_stat_disable_accounting(kqd->q);

	for (i = 0; i < KYBER_NUM_DOMAINS; i++)
		sbitmap_queue_free(&kqd->domain_tokens[i]);
	free_percpu(kqd->cpu_latency);
	kfree(kqd);
}

static void kyber_ctx_queue_init(struct kyber_ctx_queue *kcq)
{
	unsigned int i;

	spin_lock_init(&kcq->lock);
	for (i = 0; i < KYBER_NUM_DOMAINS; i++)
		INIT_LIST_HEAD(&kcq->rq_list[i]);
}

static void kyber_depth_updated(struct blk_mq_hw_ctx *hctx)
{
	struct kyber_queue_data *kqd = hctx->queue->elevator->elevator_data;
	struct blk_mq_tags *tags = hctx->sched_tags;
	unsigned int shift = tags->bitmap_tags.sb.shift;

	kqd->async_depth = (1U << shift) * KYBER_ASYNC_PERCENT / 100U;

	sbitmap_queue_min_shallow_depth(&tags->bitmap_tags, kqd->async_depth);
}

static int kyber_init_hctx(struct blk_mq_hw_ctx *hctx, unsigned int hctx_idx)
{
	struct kyber_hctx_data *khd;
	int i;

	khd = kmalloc_node(sizeof(*khd), GFP_KERNEL, hctx->numa_node);
	if (!khd)
		return -ENOMEM;

	khd->kcqs = kmalloc_array_node(hctx->nr_ctx,
				       sizeof(struct kyber_ctx_queue),
				       GFP_KERNEL, hctx->numa_node);
	if (!khd->kcqs)
		goto err_khd;

	for (i = 0; i < hctx->nr_ctx; i++)
		kyber_ctx_queue_init(&khd->kcqs[i]);

	for (i = 0; i < KYBER_NUM_DOMAINS; i++) {
		if (sbitmap_init_node(&khd->kcq_map[i], hctx->nr_ctx,
				      ilog2(8), GFP_KERNEL, hctx->numa_node,
				      false, false)) {
			while (--i >= 0)
				sbitmap_free(&khd->kcq_map[i]);
			goto err_kcqs;
		}
	}

	spin_lock_init(&khd->lock);

	for (i = 0; i < KYBER_NUM_DOMAINS; i++) {
		INIT_LIST_HEAD(&khd->rqs[i]);
		khd->domain_wait[i].sbq = NULL;
		init_waitqueue_func_entry(&khd->domain_wait[i].wait,
					  kyber_domain_wake);
		khd->domain_wait[i].wait.private = hctx;
		INIT_LIST_HEAD(&khd->domain_wait[i].wait.entry);
		atomic_set(&khd->wait_index[i], 0);
	}
	INIT_LIST_HEAD(&khd->retry_rqs);

	khd->cur_domain = 0;
	khd->batching = 0;

	hctx->sched_data = khd;
	kyber_depth_updated(hctx);

	return 0;

err_kcqs:
	kfree(khd->kcqs);
err_khd:
	kfree(khd);
	return -ENOMEM;
}

static void kyber_exit_hctx(struct blk_mq_hw_ctx *hctx, unsigned int hctx_idx)
{
	struct kyber_hctx_data *khd = hctx->sched_data;
	int i;

	for (i = 0; i < KYBER_NUM_DOMAINS; i++)
		sbitmap_free(&khd->kcq_map[i]);
	kfree(khd->kcqs);
	kfree(hctx->sched_data);
}

static int rq_get_domain_token(struct request *rq)
{
	return (long)rq->elv.priv[0];
}

static void rq_set_domain_token(struct request *rq, int token)
{
	rq->elv.priv[0] = (void *)(long)token;
}

static void rq_clear_domain_token(struct kyber_queue_data *kqd,
				  struct request *rq)
{
	unsigned int sched_domain;
	int nr;

	nr = rq_get_domain_token(rq);
	if (nr != -1) {
		sched_domain = kyber_sched_domain(rq->cmd_flags);
		sbitmap_queue_clear(&kqd->domain_tokens[sched_domain], nr,
				    rq->mq_ctx->cpu);
		rq_set_domain_token(rq, -1);
	}
}

static void kyber_limit_depth(blk_opf_t opf, struct blk_mq_alloc_data *data)
{
	/*
	 * We use the scheduler tags as per-hardware queue queueing tokens.
	 * Async requests can be limited at this stage.
	 */
	if (!op_is_sync(opf)) {
		struct kyber_queue_data *kqd = data->q->elevator->elevator_data;

		data->shallow_depth = kqd->async_depth;
	}
}

static bool kyber_bio_merge(struct request_queue *q, struct bio *bio,
		unsigned int nr_segs)
{
	struct blk_mq_ctx *ctx = blk_mq_get_ctx(q);
	struct blk_mq_hw_ctx *hctx = blk_mq_map_queue(q, bio->bi_opf, ctx);
	struct kyber_hctx_data *khd = hctx->sched_data;
	struct kyber_ctx_queue *kcq = &khd->kcqs[ctx->index_hw[hctx->type]];
	unsigned int sched_domain = kyber_sched_domain(bio->bi_opf);
	struct list_head *rq_list = &kcq->rq_list[sched_domain];
	bool merged;

	spin_lock(&kcq->lock);
	merged = blk_bio_list_merge(hctx->queue, rq_list, bio, nr_segs);
	spin_unlock(&kcq->lock);

	return merged;
}

static void kyber_qt_track_request(struct kyber_queue_data *kqd,
				   struct request *rq)
{
	unsigned long flags, state;

	if (!kqd->qt_enabled || !(rq->rq_flags & RQF_ELVPRIV))
		return;

	spin_lock_irqsave(&kqd->qt_lock, flags);
	state = kyber_qt_rq_state(rq);
	if (!(state & KYBER_QT_TRACKED)) {
		state |= KYBER_QT_TRACKED;
		kyber_qt_set_rq_state(rq, state);
		kqd->qt_generated++;
		kqd->qt_b_new++;
	}
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
}

static void kyber_qt_finish_request(struct kyber_queue_data *kqd,
				    struct request *rq, bool completed)
{
	unsigned long flags, state;

	if (!kqd->qt_enabled)
		return;

	spin_lock_irqsave(&kqd->qt_lock, flags);
	state = kyber_qt_rq_state(rq);
	if (!(state & KYBER_QT_TRACKED) || (state & KYBER_QT_TERMINAL))
		goto out;

	if (state & KYBER_QT_RETRY) {
		if (kqd->qt_b_retry)
			kqd->qt_b_retry--;
		state &= ~KYBER_QT_RETRY;
	}
	if (state & KYBER_QT_ADMITTED) {
		if (kqd->qt_q_adm)
			kqd->qt_q_adm--;
		if (completed)
			kqd->qt_completed++;
		else
			kqd->qt_exit_admitted++;
	} else {
		if (kqd->qt_b_new)
			kqd->qt_b_new--;
		kqd->qt_exit_new++;
	}
	state |= KYBER_QT_TERMINAL;
	kyber_qt_set_rq_state(rq, state);
out:
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
}

static void kyber_qt_requeue_request(struct kyber_queue_data *kqd,
				     struct request *rq)
{
	unsigned long flags, state;

	if (!kqd->qt_enabled)
		return;

	spin_lock_irqsave(&kqd->qt_lock, flags);
	state = kyber_qt_rq_state(rq);
	if ((state & KYBER_QT_TRACKED) && (state & KYBER_QT_ADMITTED) &&
	    !(state & (KYBER_QT_RETRY | KYBER_QT_TERMINAL))) {
		state |= KYBER_QT_RETRY;
		kyber_qt_set_rq_state(rq, state);
		kqd->qt_b_retry++;
		kqd->qt_requeues++;
	}
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
}

static void kyber_prepare_request(struct request *rq)
{
	rq_set_domain_token(rq, -1);
	kyber_qt_set_rq_state(rq, 0);
}

static void kyber_insert_requests(struct blk_mq_hw_ctx *hctx,
				  struct list_head *rq_list, bool at_head)
{
	struct kyber_hctx_data *khd = hctx->sched_data;
	struct kyber_queue_data *kqd = hctx->queue->elevator->elevator_data;
	struct request *rq, *next;

	list_for_each_entry_safe(rq, next, rq_list, queuelist) {
		unsigned int sched_domain = kyber_sched_domain(rq->cmd_flags);
		struct kyber_ctx_queue *kcq = &khd->kcqs[rq->mq_ctx->index_hw[hctx->type]];
		struct list_head *head = &kcq->rq_list[sched_domain];

		kyber_qt_track_request(kqd, rq);
		if (kqd->qt_enabled &&
		    (kyber_qt_rq_state(rq) & KYBER_QT_RETRY)) {
			spin_lock(&khd->lock);
			if (at_head)
				list_move(&rq->queuelist, &khd->retry_rqs);
			else
				list_move_tail(&rq->queuelist, &khd->retry_rqs);
			spin_unlock(&khd->lock);
			continue;
		}
		spin_lock(&kcq->lock);
		trace_block_rq_insert(rq);
		if (at_head)
			list_move(&rq->queuelist, head);
		else
			list_move_tail(&rq->queuelist, head);
		sbitmap_set_bit(&khd->kcq_map[sched_domain],
				rq->mq_ctx->index_hw[hctx->type]);
		spin_unlock(&kcq->lock);
	}
}

static void kyber_finish_request(struct request *rq)
{
	struct kyber_queue_data *kqd = rq->q->elevator->elevator_data;

	rq_clear_domain_token(kqd, rq);
	kyber_qt_finish_request(kqd, rq, false);
}

static void kyber_requeue_request(struct request *rq)
{
	struct kyber_queue_data *kqd = rq->q->elevator->elevator_data;

	rq_clear_domain_token(kqd, rq);
	kyber_qt_requeue_request(kqd, rq);
}

static void add_latency_sample(struct kyber_cpu_latency *cpu_latency,
			       unsigned int sched_domain, unsigned int type,
			       u64 target, u64 latency)
{
	unsigned int bucket;
	u64 divisor;

	if (latency > 0) {
		divisor = max_t(u64, target >> KYBER_LATENCY_SHIFT, 1);
		bucket = min_t(unsigned int, div64_u64(latency - 1, divisor),
			       KYBER_LATENCY_BUCKETS - 1);
	} else {
		bucket = 0;
	}

	atomic_inc(&cpu_latency->buckets[sched_domain][type][bucket]);
}

static void kyber_completed_request(struct request *rq, u64 now)
{
	struct kyber_queue_data *kqd = rq->q->elevator->elevator_data;
	struct kyber_cpu_latency *cpu_latency;
	unsigned int sched_domain;
	u64 target;

	if (kqd->qt_enabled) {
		kyber_qt_finish_request(kqd, rq, true);
		return;
	}

	sched_domain = kyber_sched_domain(rq->cmd_flags);
	if (sched_domain == KYBER_OTHER)
		return;

	cpu_latency = get_cpu_ptr(kqd->cpu_latency);
	target = kqd->latency_targets[sched_domain];
	add_latency_sample(cpu_latency, sched_domain, KYBER_TOTAL_LATENCY,
			   target, now - rq->start_time_ns);
	add_latency_sample(cpu_latency, sched_domain, KYBER_IO_LATENCY, target,
			   now - rq->io_start_time_ns);
	put_cpu_ptr(kqd->cpu_latency);

	timer_reduce(&kqd->timer, jiffies + HZ / 10);
}

struct flush_kcq_data {
	struct kyber_hctx_data *khd;
	unsigned int sched_domain;
	struct list_head *list;
};

static bool flush_busy_kcq(struct sbitmap *sb, unsigned int bitnr, void *data)
{
	struct flush_kcq_data *flush_data = data;
	struct kyber_ctx_queue *kcq = &flush_data->khd->kcqs[bitnr];

	spin_lock(&kcq->lock);
	list_splice_tail_init(&kcq->rq_list[flush_data->sched_domain],
			      flush_data->list);
	sbitmap_clear_bit(sb, bitnr);
	spin_unlock(&kcq->lock);

	return true;
}

static void kyber_flush_busy_kcqs(struct kyber_hctx_data *khd,
				  unsigned int sched_domain,
				  struct list_head *list)
{
	struct flush_kcq_data data = {
		.khd = khd,
		.sched_domain = sched_domain,
		.list = list,
	};

	sbitmap_for_each_set(&khd->kcq_map[sched_domain],
			     flush_busy_kcq, &data);
}

static int kyber_domain_wake(wait_queue_entry_t *wqe, unsigned mode, int flags,
			     void *key)
{
	struct blk_mq_hw_ctx *hctx = READ_ONCE(wqe->private);
	struct sbq_wait *wait = container_of(wqe, struct sbq_wait, wait);

	sbitmap_del_wait_queue(wait);
	blk_mq_run_hw_queue(hctx, true);
	return 1;
}

static int kyber_get_domain_token(struct kyber_queue_data *kqd,
				  struct kyber_hctx_data *khd,
				  struct blk_mq_hw_ctx *hctx,
				  unsigned int sched_domain)
{
	struct sbitmap_queue *domain_tokens = &kqd->domain_tokens[sched_domain];
	struct sbq_wait *wait = &khd->domain_wait[sched_domain];
	struct sbq_wait_state *ws;
	int nr;

	nr = __sbitmap_queue_get(domain_tokens);

	/*
	 * If we failed to get a domain token, make sure the hardware queue is
	 * run when one becomes available. Note that this is serialized on
	 * khd->lock, but we still need to be careful about the waker.
	 */
	if (nr < 0 && list_empty_careful(&wait->wait.entry)) {
		ws = sbq_wait_ptr(domain_tokens,
				  &khd->wait_index[sched_domain]);
		khd->domain_ws[sched_domain] = ws;
		sbitmap_add_wait_queue(domain_tokens, ws, wait);

		/*
		 * Try again in case a token was freed before we got on the wait
		 * queue.
		 */
		nr = __sbitmap_queue_get(domain_tokens);
	}

	/*
	 * If we got a token while we were on the wait queue, remove ourselves
	 * from the wait queue to ensure that all wake ups make forward
	 * progress. It's possible that the waker already deleted the entry
	 * between the !list_empty_careful() check and us grabbing the lock, but
	 * list_del_init() is okay with that.
	 */
	if (nr >= 0 && !list_empty_careful(&wait->wait.entry)) {
		ws = khd->domain_ws[sched_domain];
		spin_lock_irq(&ws->wait.lock);
		sbitmap_del_wait_queue(wait);
		spin_unlock_irq(&ws->wait.lock);
	}

	return nr;
}

static bool kyber_qt_admit(struct kyber_queue_data *kqd, struct request *rq)
{
	unsigned long flags, state;
	u64 now, elapsed, max_credit, add, delay_ns = 0;
	u64 q_adm = 0;
	const char *blocked_reason = NULL;
	u32 rate = 0;
	bool admitted = true;

	if (!(rq->rq_flags & RQF_ELVPRIV))
		return true;

	spin_lock_irqsave(&kqd->qt_lock, flags);
	state = kyber_qt_rq_state(rq);
	if (!(state & KYBER_QT_TRACKED))
		goto out;

	if (state & KYBER_QT_ADMITTED) {
		if (state & KYBER_QT_RETRY) {
			if (kqd->qt_b_retry)
				kqd->qt_b_retry--;
			state &= ~KYBER_QT_RETRY;
			kyber_qt_set_rq_state(rq, state);
		}
		goto out;
	}
	if (!kqd->qt_control) {
		if (kqd->qt_b_new)
			kqd->qt_b_new--;
		kqd->qt_q_adm++;
		kqd->qt_admitted++;
		state |= KYBER_QT_ADMITTED;
		kyber_qt_set_rq_state(rq, state);
		goto out;
	}

	now = ktime_get_ns();
	max_credit = (u64)kqd->qt_burst * KYBER_QT_CREDIT_UNIT;
	if (now > kqd->qt_last_ns) {
		/* Bound multiplication even after a long idle period. */
		elapsed = min_t(u64, now - kqd->qt_last_ns, NSEC_PER_SEC);
		add = elapsed * kqd->qt_rate;
		if (kqd->qt_credit >= max_credit)
			kqd->qt_credit = max_credit;
		else if (add >= max_credit - kqd->qt_credit)
			kqd->qt_credit = max_credit;
		else
			kqd->qt_credit += add;
		kqd->qt_last_ns = now;
	}

	if (kqd->qt_q_adm >= kqd->qt_cap) {
		kqd->qt_cap_blocked++;
		blocked_reason = "cap";
		admitted = false;
	} else if (!kqd->qt_rate ||
		   kqd->qt_credit < KYBER_QT_CREDIT_UNIT) {
		kqd->qt_rate_blocked++;
		blocked_reason = "rate";
		if (kqd->qt_rate)
			delay_ns = DIV_ROUND_UP_ULL(KYBER_QT_CREDIT_UNIT -
						    kqd->qt_credit,
						    kqd->qt_rate);
		admitted = false;
	} else {
		kqd->qt_credit -= KYBER_QT_CREDIT_UNIT;
		if (kqd->qt_b_new)
			kqd->qt_b_new--;
		kqd->qt_q_adm++;
		kqd->qt_admitted++;
		state |= KYBER_QT_ADMITTED;
		kyber_qt_set_rq_state(rq, state);
	}
	q_adm = kqd->qt_q_adm;
	rate = kqd->qt_rate;
out:
	spin_unlock_irqrestore(&kqd->qt_lock, flags);

	if (blocked_reason)
		trace_kyber_qt_block(kqd->dev, blocked_reason, rate, q_adm);
	if (delay_ns && !READ_ONCE(kqd->qt_stopping))
		hrtimer_start(&kqd->qt_wake_timer,
			      ns_to_ktime(max_t(u64, delay_ns, 1)),
			      HRTIMER_MODE_REL);
	return admitted;
}

static void kyber_qt_domain_blocked(struct kyber_queue_data *kqd)
{
	unsigned long flags;
	u64 q_adm;
	u32 rate;

	spin_lock_irqsave(&kqd->qt_lock, flags);
	kqd->qt_domain_blocked++;
	q_adm = kqd->qt_q_adm;
	rate = kqd->qt_rate;
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
	trace_kyber_qt_block(kqd->dev, "domain", rate, q_adm);
}

static struct request *
kyber_dispatch_cur_domain(struct kyber_queue_data *kqd,
			  struct kyber_hctx_data *khd,
			  struct blk_mq_hw_ctx *hctx)
{
	struct list_head *rqs;
	struct request *rq;
	int nr;

	rqs = &khd->rqs[khd->cur_domain];

	/*
	 * If we already have a flushed request, then we just need to get a
	 * token for it. Otherwise, if there are pending requests in the kcqs,
	 * flush the kcqs, but only if we can get a token. If not, we should
	 * leave the requests in the kcqs so that they can be merged. Note that
	 * khd->lock serializes the flushes, so if we observed any bit set in
	 * the kcq_map, we will always get a request.
	 */
	rq = list_first_entry_or_null(rqs, struct request, queuelist);
	if (rq) {
		nr = kyber_get_domain_token(kqd, khd, hctx, khd->cur_domain);
		if (nr >= 0) {
			if (kqd->qt_enabled && !kyber_qt_admit(kqd, rq)) {
				sbitmap_queue_clear(&kqd->domain_tokens[khd->cur_domain],
						    nr, rq->mq_ctx->cpu);
				return NULL;
			}
			khd->batching++;
			rq_set_domain_token(rq, nr);
			list_del_init(&rq->queuelist);
			return rq;
		} else {
			if (kqd->qt_enabled)
				kyber_qt_domain_blocked(kqd);
			trace_kyber_throttled(kqd->dev,
					      kyber_domain_names[khd->cur_domain]);
		}
	} else if (sbitmap_any_bit_set(&khd->kcq_map[khd->cur_domain])) {
		nr = kyber_get_domain_token(kqd, khd, hctx, khd->cur_domain);
		if (nr >= 0) {
			kyber_flush_busy_kcqs(khd, khd->cur_domain, rqs);
			rq = list_first_entry(rqs, struct request, queuelist);
			if (kqd->qt_enabled && !kyber_qt_admit(kqd, rq)) {
				sbitmap_queue_clear(&kqd->domain_tokens[khd->cur_domain],
						    nr, rq->mq_ctx->cpu);
				return NULL;
			}
			khd->batching++;
			rq_set_domain_token(rq, nr);
			list_del_init(&rq->queuelist);
			return rq;
		} else {
			if (kqd->qt_enabled)
				kyber_qt_domain_blocked(kqd);
			trace_kyber_throttled(kqd->dev,
					      kyber_domain_names[khd->cur_domain]);
		}
	}

	/* There were either no pending requests or no tokens. */
	return NULL;
}

static struct request *kyber_dispatch_request(struct blk_mq_hw_ctx *hctx)
{
	struct kyber_queue_data *kqd = hctx->queue->elevator->elevator_data;
	struct kyber_hctx_data *khd = hctx->sched_data;
	struct request *rq;
	int i;

	spin_lock(&khd->lock);

	/* Retries retain admission and do not consume new rate credits. */
	if (kqd->qt_enabled && !list_empty(&khd->retry_rqs)) {
		unsigned int sched_domain;

		rq = list_first_entry(&khd->retry_rqs, struct request, queuelist);
		sched_domain = kyber_sched_domain(rq->cmd_flags);
		i = kyber_get_domain_token(kqd, khd, hctx, sched_domain);
		if (i >= 0) {
			if (kyber_qt_admit(kqd, rq)) {
				rq_set_domain_token(rq, i);
				list_del_init(&rq->queuelist);
				khd->batching = 0;
				goto out;
			}
			sbitmap_queue_clear(&kqd->domain_tokens[sched_domain], i,
					    rq->mq_ctx->cpu);
		} else {
			kyber_qt_domain_blocked(kqd);
		}
	}

	/*
	 * First, if we are still entitled to batch, try to dispatch a request
	 * from the batch.
	 */
	if (khd->batching < kyber_batch_size[khd->cur_domain]) {
		rq = kyber_dispatch_cur_domain(kqd, khd, hctx);
		if (rq)
			goto out;
	}

	/*
	 * Either,
	 * 1. We were no longer entitled to a batch.
	 * 2. The domain we were batching didn't have any requests.
	 * 3. The domain we were batching was out of tokens.
	 *
	 * Start another batch. Note that this wraps back around to the original
	 * domain if no other domains have requests or tokens.
	 */
	khd->batching = 0;
	for (i = 0; i < KYBER_NUM_DOMAINS; i++) {
		if (khd->cur_domain == KYBER_NUM_DOMAINS - 1)
			khd->cur_domain = 0;
		else
			khd->cur_domain++;

		rq = kyber_dispatch_cur_domain(kqd, khd, hctx);
		if (rq)
			goto out;
	}

	rq = NULL;
out:
	spin_unlock(&khd->lock);
	return rq;
}

static bool kyber_has_work(struct blk_mq_hw_ctx *hctx)
{
	struct kyber_hctx_data *khd = hctx->sched_data;
	int i;

	for (i = 0; i < KYBER_NUM_DOMAINS; i++) {
		if (!list_empty_careful(&khd->rqs[i]) ||
		    sbitmap_any_bit_set(&khd->kcq_map[i]))
			return true;
	}

	return !list_empty_careful(&khd->retry_rqs);
}

#define KYBER_LAT_SHOW_STORE(domain, name)				\
static ssize_t kyber_##name##_lat_show(struct elevator_queue *e,	\
				       char *page)			\
{									\
	struct kyber_queue_data *kqd = e->elevator_data;		\
									\
	return sprintf(page, "%llu\n", kqd->latency_targets[domain]);	\
}									\
									\
static ssize_t kyber_##name##_lat_store(struct elevator_queue *e,	\
					const char *page, size_t count)	\
{									\
	struct kyber_queue_data *kqd = e->elevator_data;		\
	unsigned long long nsec;					\
	int ret;							\
									\
	ret = kstrtoull(page, 10, &nsec);				\
	if (ret)							\
		return ret;						\
									\
	kqd->latency_targets[domain] = nsec;				\
									\
	return count;							\
}
KYBER_LAT_SHOW_STORE(KYBER_READ, read);
KYBER_LAT_SHOW_STORE(KYBER_WRITE, write);
#undef KYBER_LAT_SHOW_STORE

#define KYBER_LAT_ATTR(op) __ATTR(op##_lat_nsec, 0644, kyber_##op##_lat_show, kyber_##op##_lat_store)
static struct elv_fs_entry kyber_sched_attrs[] = {
	KYBER_LAT_ATTR(read),
	KYBER_LAT_ATTR(write),
	__ATTR_NULL
};
#undef KYBER_LAT_ATTR

static ssize_t kyber_qt_rate_show(struct elevator_queue *e, char *page)
{
	struct kyber_queue_data *kqd = e->elevator_data;
	unsigned long flags;
	u32 rate;

	if (!kqd->qt_enabled)
		return -EOPNOTSUPP;
	spin_lock_irqsave(&kqd->qt_lock, flags);
	rate = kqd->qt_rate;
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
	return sysfs_emit(page, "%u\n", rate);
}

static ssize_t kyber_qt_rate_store(struct elevator_queue *e,
				   const char *page, size_t count)
{
	struct kyber_queue_data *kqd = e->elevator_data;
	unsigned long flags;
	u32 rate;
	int ret;

	if (!kqd->qt_enabled)
		return -EOPNOTSUPP;
	ret = kstrtouint(page, 10, &rate);
	if (ret)
		return ret;
	if (rate > KYBER_QT_MAX_RATE)
		return -EINVAL;

	spin_lock_irqsave(&kqd->qt_lock, flags);
	if (rate > kqd->qt_max_rate) {
		spin_unlock_irqrestore(&kqd->qt_lock, flags);
		return -EINVAL;
	}
	kqd->qt_rate = rate;
	kqd->qt_initial_rate = rate;
	kqd->qt_credit = 0;
	kqd->qt_last_ns = ktime_get_ns();
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
	blk_mq_run_hw_queues(kqd->q, true);
	return count;
}

static ssize_t kyber_qt_cap_show(struct elevator_queue *e, char *page)
{
	struct kyber_queue_data *kqd = e->elevator_data;
	unsigned long flags;
	u32 cap;

	if (!kqd->qt_enabled)
		return -EOPNOTSUPP;
	spin_lock_irqsave(&kqd->qt_lock, flags);
	cap = kqd->qt_cap;
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
	return sysfs_emit(page, "%u\n", cap);
}

static ssize_t kyber_qt_cap_store(struct elevator_queue *e,
				  const char *page, size_t count)
{
	struct kyber_queue_data *kqd = e->elevator_data;
	unsigned long flags;
	u32 cap;
	int ret;

	if (!kqd->qt_enabled)
		return -EOPNOTSUPP;
	ret = kstrtouint(page, 10, &cap);
	if (ret)
		return ret;
	if (!cap || cap > KYBER_QT_MAX_CAP)
		return -EINVAL;

	spin_lock_irqsave(&kqd->qt_lock, flags);
	if (cap <= kqd->qt_high) {
		spin_unlock_irqrestore(&kqd->qt_lock, flags);
		return -EINVAL;
	}
	if (cap < kqd->qt_q_adm) {
		spin_unlock_irqrestore(&kqd->qt_lock, flags);
		return -EBUSY;
	}
	kqd->qt_cap = cap;
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
	blk_mq_run_hw_queues(kqd->q, true);
	return count;
}

static ssize_t kyber_qt_burst_show(struct elevator_queue *e, char *page)
{
	struct kyber_queue_data *kqd = e->elevator_data;
	unsigned long flags;
	u32 burst;

	if (!kqd->qt_enabled)
		return -EOPNOTSUPP;
	spin_lock_irqsave(&kqd->qt_lock, flags);
	burst = kqd->qt_burst;
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
	return sysfs_emit(page, "%u\n", burst);
}

static ssize_t kyber_qt_burst_store(struct elevator_queue *e,
				    const char *page, size_t count)
{
	struct kyber_queue_data *kqd = e->elevator_data;
	unsigned long flags;
	u64 max_credit;
	u32 burst;
	int ret;

	if (!kqd->qt_enabled)
		return -EOPNOTSUPP;
	ret = kstrtouint(page, 10, &burst);
	if (ret)
		return ret;
	if (!burst || burst > KYBER_QT_MAX_BURST)
		return -EINVAL;

	spin_lock_irqsave(&kqd->qt_lock, flags);
	kqd->qt_burst = burst;
	max_credit = (u64)burst * KYBER_QT_CREDIT_UNIT;
	kqd->qt_credit = min(kqd->qt_credit, max_credit);
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
	return count;
}

enum kyber_qt_attribute {
	KYBER_QT_CONTROL,
	KYBER_QT_MAX_RATE_ATTR,
	KYBER_QT_ALPHA,
	KYBER_QT_BETA,
	KYBER_QT_EPSILON,
	KYBER_QT_CONFIRM,
	KYBER_QT_SAMPLE_MS,
	KYBER_QT_LOW,
	KYBER_QT_HIGH,
};

static ssize_t kyber_qt_parameter_show(struct elevator_queue *e, char *page,
				       enum kyber_qt_attribute attribute)
{
	struct kyber_queue_data *kqd = e->elevator_data;
	unsigned long flags;
	u32 value;

	if (!kqd->qt_enabled)
		return -EOPNOTSUPP;
	spin_lock_irqsave(&kqd->qt_lock, flags);
	switch (attribute) {
	case KYBER_QT_CONTROL:
		value = kqd->qt_control;
		break;
	case KYBER_QT_MAX_RATE_ATTR:
		value = kqd->qt_max_rate;
		break;
	case KYBER_QT_ALPHA:
		value = kqd->qt_alpha;
		break;
	case KYBER_QT_BETA:
		value = kqd->qt_beta;
		break;
	case KYBER_QT_EPSILON:
		value = kqd->qt_epsilon;
		break;
	case KYBER_QT_CONFIRM:
		value = kqd->qt_confirm;
		break;
	case KYBER_QT_SAMPLE_MS:
		value = kqd->qt_sample_ms;
		break;
	case KYBER_QT_LOW:
		value = kqd->qt_low;
		break;
	case KYBER_QT_HIGH:
		value = kqd->qt_high;
		break;
	default:
		value = 0;
		break;
	}
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
	return sysfs_emit(page, "%u\n", value);
}

static ssize_t kyber_qt_parameter_store(struct elevator_queue *e,
					const char *page, size_t count,
					enum kyber_qt_attribute attribute)
{
	struct kyber_queue_data *kqd = e->elevator_data;
	unsigned long flags;
	bool restart_sample = false, wake_queues = false;
	u32 value;
	int ret;

	if (!kqd->qt_enabled)
		return -EOPNOTSUPP;
	ret = kstrtouint(page, 10, &value);
	if (ret)
		return ret;

	spin_lock_irqsave(&kqd->qt_lock, flags);
	if (kqd->qt_q_adm || kqd->qt_b_new) {
		ret = -EBUSY;
		goto out;
	}
	switch (attribute) {
	case KYBER_QT_CONTROL:
		if (value > 2) {
			ret = -EINVAL;
			goto out;
		}
		kqd->qt_control = value;
		kqd->qt_mode = KYBER_QT_TRACK;
		kqd->qt_growth_streak = 0;
		kqd->qt_drain_streak = 0;
		kqd->qt_prev_ns = 0;
		if (value < 2)
			kyber_qt_set_rate_locked(kqd, kqd->qt_initial_rate,
						 ktime_get_ns());
		wake_queues = true;
		break;
	case KYBER_QT_MAX_RATE_ATTR:
		if (!value || value > KYBER_QT_MAX_RATE ||
		    value < kqd->qt_rate ||
		    value < kqd->qt_initial_rate) {
			ret = -EINVAL;
			goto out;
		}
		kqd->qt_max_rate = value;
		break;
	case KYBER_QT_ALPHA:
		if (value > KYBER_QT_MAX_RATE) {
			ret = -EINVAL;
			goto out;
		}
		kqd->qt_alpha = value;
		break;
	case KYBER_QT_BETA:
		if (!value || value >= 100) {
			ret = -EINVAL;
			goto out;
		}
		kqd->qt_beta = value;
		break;
	case KYBER_QT_EPSILON:
		if (value > KYBER_QT_MAX_RATE) {
			ret = -EINVAL;
			goto out;
		}
		kqd->qt_epsilon = value;
		break;
	case KYBER_QT_CONFIRM:
		if (!value || value > KYBER_QT_MAX_CONFIRM) {
			ret = -EINVAL;
			goto out;
		}
		kqd->qt_confirm = value;
		break;
	case KYBER_QT_SAMPLE_MS:
		if (!value || value > 1000) {
			ret = -EINVAL;
			goto out;
		}
		kqd->qt_sample_ms = value;
		restart_sample = kqd->qt_enabled;
		break;
	case KYBER_QT_LOW:
		if (value >= kqd->qt_high) {
			ret = -EINVAL;
			goto out;
		}
		kqd->qt_low = value;
		break;
	case KYBER_QT_HIGH:
		if (value <= kqd->qt_low || value >= kqd->qt_cap) {
			ret = -EINVAL;
			goto out;
		}
		kqd->qt_high = value;
		break;
	default:
		ret = -EINVAL;
		goto out;
	}
	kqd->qt_mode = KYBER_QT_TRACK;
	kqd->qt_growth_streak = 0;
	kqd->qt_drain_streak = 0;
	kqd->qt_prev_ns = 0;
	kqd->qt_prev_domain_blocked = kqd->qt_domain_blocked;
	kqd->qt_sample_valid = false;
	ret = count;
out:
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
	if (ret < 0)
		return ret;
	if (restart_sample && !READ_ONCE(kqd->qt_stopping))
		hrtimer_start(&kqd->qt_sample_timer,
			      ms_to_ktime(READ_ONCE(kqd->qt_sample_ms)),
			      HRTIMER_MODE_REL);
	if (wake_queues)
		blk_mq_run_hw_queues(kqd->q, true);
	return count;
}

#define KYBER_QT_PARAM_SHOW(name, param) \
static ssize_t kyber_qt_##name##_show(struct elevator_queue *e, char *page) \
{ \
	return kyber_qt_parameter_show(e, page, param); \
}

#define KYBER_QT_PARAM_STORE(name, param) \
static ssize_t kyber_qt_##name##_store(struct elevator_queue *e, \
					       const char *page, size_t count) \
{ \
	return kyber_qt_parameter_store(e, page, count, param); \
}

KYBER_QT_PARAM_SHOW(control, KYBER_QT_CONTROL)
KYBER_QT_PARAM_STORE(control, KYBER_QT_CONTROL)
KYBER_QT_PARAM_SHOW(max_rate, KYBER_QT_MAX_RATE_ATTR)
KYBER_QT_PARAM_STORE(max_rate, KYBER_QT_MAX_RATE_ATTR)
KYBER_QT_PARAM_SHOW(alpha, KYBER_QT_ALPHA)
KYBER_QT_PARAM_STORE(alpha, KYBER_QT_ALPHA)
KYBER_QT_PARAM_SHOW(beta, KYBER_QT_BETA)
KYBER_QT_PARAM_STORE(beta, KYBER_QT_BETA)
KYBER_QT_PARAM_SHOW(epsilon, KYBER_QT_EPSILON)
KYBER_QT_PARAM_STORE(epsilon, KYBER_QT_EPSILON)
KYBER_QT_PARAM_SHOW(confirm, KYBER_QT_CONFIRM)
KYBER_QT_PARAM_STORE(confirm, KYBER_QT_CONFIRM)
KYBER_QT_PARAM_SHOW(sample_ms, KYBER_QT_SAMPLE_MS)
KYBER_QT_PARAM_STORE(sample_ms, KYBER_QT_SAMPLE_MS)
KYBER_QT_PARAM_SHOW(low, KYBER_QT_LOW)
KYBER_QT_PARAM_STORE(low, KYBER_QT_LOW)
KYBER_QT_PARAM_SHOW(high, KYBER_QT_HIGH)
KYBER_QT_PARAM_STORE(high, KYBER_QT_HIGH)
#undef KYBER_QT_PARAM_SHOW
#undef KYBER_QT_PARAM_STORE

static ssize_t kyber_qt_stats_show(struct elevator_queue *e, char *page)
{
	struct kyber_queue_data *kqd = e->elevator_data;
	unsigned long flags;
	ssize_t len;

	if (!kqd->qt_enabled)
		return -EOPNOTSUPP;
	spin_lock_irqsave(&kqd->qt_lock, flags);
	len = sysfs_emit(page,
			 "G=%llu A=%llu C=%llu X_B=%llu X_Q=%llu B_new=%llu Q_adm=%llu\n",
		(unsigned long long)kqd->qt_generated,
		(unsigned long long)kqd->qt_admitted,
		(unsigned long long)kqd->qt_completed,
		(unsigned long long)kqd->qt_exit_new,
		(unsigned long long)kqd->qt_exit_admitted,
		(unsigned long long)kqd->qt_b_new,
		(unsigned long long)kqd->qt_q_adm);
	len += sysfs_emit_at(page, len,
			    "B_retry=%llu rate=%u burst=%u cap=%u\n",
		(unsigned long long)kqd->qt_b_retry,
		kqd->qt_rate, kqd->qt_burst, kqd->qt_cap);
	len += sysfs_emit_at(page, len,
			    "credit_nano_requests=%llu\n",
		(unsigned long long)kqd->qt_credit);
	len += sysfs_emit_at(page, len,
			    "rate_blocked=%llu cap_blocked=%llu domain_blocked=%llu requeues=%llu\n",
		(unsigned long long)kqd->qt_rate_blocked,
		(unsigned long long)kqd->qt_cap_blocked,
		(unsigned long long)kqd->qt_domain_blocked,
		(unsigned long long)kqd->qt_requeues);
	len += sysfs_emit_at(page, len,
			    "mode=%s control=%u growth=%lld\n",
		kqd->qt_mode == KYBER_QT_DRAIN ? "DRAIN" : "TRACK",
		kqd->qt_control, (long long)kqd->qt_growth_rate);
	len += sysfs_emit_at(page, len,
			    "sample_valid=%u sample_elapsed_ns=%llu\n",
		kqd->qt_sample_valid,
		(unsigned long long)kqd->qt_sample_elapsed_ns);
	len += sysfs_emit_at(page, len,
			    "epsilon=%u low=%u high=%u alpha=%u beta=%u\n",
		kqd->qt_epsilon, kqd->qt_low, kqd->qt_high, kqd->qt_alpha,
		kqd->qt_beta);
	len += sysfs_emit_at(page, len,
			    "confirm=%u sample_ms=%u max_rate=%u\n",
		kqd->qt_confirm, kqd->qt_sample_ms, kqd->qt_max_rate);
	spin_unlock_irqrestore(&kqd->qt_lock, flags);
	return len;
}

static struct elv_fs_entry kyber_qt_sched_attrs[] = {
	__ATTR(qt_rate, 0644, kyber_qt_rate_show, kyber_qt_rate_store),
	__ATTR(qt_burst, 0644, kyber_qt_burst_show, kyber_qt_burst_store),
	__ATTR(qt_cap, 0644, kyber_qt_cap_show, kyber_qt_cap_store),
	__ATTR(qt_control, 0644, kyber_qt_control_show,
	       kyber_qt_control_store),
	__ATTR(qt_max_rate, 0644, kyber_qt_max_rate_show,
	       kyber_qt_max_rate_store),
	__ATTR(qt_alpha, 0644, kyber_qt_alpha_show, kyber_qt_alpha_store),
	__ATTR(qt_beta, 0644, kyber_qt_beta_show, kyber_qt_beta_store),
	__ATTR(qt_epsilon, 0644, kyber_qt_epsilon_show,
	       kyber_qt_epsilon_store),
	__ATTR(qt_confirm, 0644, kyber_qt_confirm_show,
	       kyber_qt_confirm_store),
	__ATTR(qt_sample_ms, 0644, kyber_qt_sample_ms_show,
	       kyber_qt_sample_ms_store),
	__ATTR(qt_low, 0644, kyber_qt_low_show, kyber_qt_low_store),
	__ATTR(qt_high, 0644, kyber_qt_high_show, kyber_qt_high_store),
	__ATTR(qt_stats, 0444, kyber_qt_stats_show, NULL),
	__ATTR_NULL
};

#ifdef CONFIG_BLK_DEBUG_FS
#define KYBER_DEBUGFS_DOMAIN_ATTRS(domain, name)			\
static int kyber_##name##_tokens_show(void *data, struct seq_file *m)	\
{									\
	struct request_queue *q = data;					\
	struct kyber_queue_data *kqd = q->elevator->elevator_data;	\
									\
	sbitmap_queue_show(&kqd->domain_tokens[domain], m);		\
	return 0;							\
}									\
									\
static void *kyber_##name##_rqs_start(struct seq_file *m, loff_t *pos)	\
	__acquires(&khd->lock)						\
{									\
	struct blk_mq_hw_ctx *hctx = m->private;			\
	struct kyber_hctx_data *khd = hctx->sched_data;			\
									\
	spin_lock(&khd->lock);						\
	return seq_list_start(&khd->rqs[domain], *pos);			\
}									\
									\
static void *kyber_##name##_rqs_next(struct seq_file *m, void *v,	\
				     loff_t *pos)			\
{									\
	struct blk_mq_hw_ctx *hctx = m->private;			\
	struct kyber_hctx_data *khd = hctx->sched_data;			\
									\
	return seq_list_next(v, &khd->rqs[domain], pos);		\
}									\
									\
static void kyber_##name##_rqs_stop(struct seq_file *m, void *v)	\
	__releases(&khd->lock)						\
{									\
	struct blk_mq_hw_ctx *hctx = m->private;			\
	struct kyber_hctx_data *khd = hctx->sched_data;			\
									\
	spin_unlock(&khd->lock);					\
}									\
									\
static const struct seq_operations kyber_##name##_rqs_seq_ops = {	\
	.start	= kyber_##name##_rqs_start,				\
	.next	= kyber_##name##_rqs_next,				\
	.stop	= kyber_##name##_rqs_stop,				\
	.show	= blk_mq_debugfs_rq_show,				\
};									\
									\
static int kyber_##name##_waiting_show(void *data, struct seq_file *m)	\
{									\
	struct blk_mq_hw_ctx *hctx = data;				\
	struct kyber_hctx_data *khd = hctx->sched_data;			\
	wait_queue_entry_t *wait = &khd->domain_wait[domain].wait;	\
									\
	seq_printf(m, "%d\n", !list_empty_careful(&wait->entry));	\
	return 0;							\
}
KYBER_DEBUGFS_DOMAIN_ATTRS(KYBER_READ, read)
KYBER_DEBUGFS_DOMAIN_ATTRS(KYBER_WRITE, write)
KYBER_DEBUGFS_DOMAIN_ATTRS(KYBER_DISCARD, discard)
KYBER_DEBUGFS_DOMAIN_ATTRS(KYBER_OTHER, other)
#undef KYBER_DEBUGFS_DOMAIN_ATTRS

static int kyber_async_depth_show(void *data, struct seq_file *m)
{
	struct request_queue *q = data;
	struct kyber_queue_data *kqd = q->elevator->elevator_data;

	seq_printf(m, "%u\n", kqd->async_depth);
	return 0;
}

static int kyber_cur_domain_show(void *data, struct seq_file *m)
{
	struct blk_mq_hw_ctx *hctx = data;
	struct kyber_hctx_data *khd = hctx->sched_data;

	seq_printf(m, "%s\n", kyber_domain_names[khd->cur_domain]);
	return 0;
}

static int kyber_batching_show(void *data, struct seq_file *m)
{
	struct blk_mq_hw_ctx *hctx = data;
	struct kyber_hctx_data *khd = hctx->sched_data;

	seq_printf(m, "%u\n", khd->batching);
	return 0;
}

#define KYBER_QUEUE_DOMAIN_ATTRS(name)	\
	{#name "_tokens", 0400, kyber_##name##_tokens_show}
static const struct blk_mq_debugfs_attr kyber_queue_debugfs_attrs[] = {
	KYBER_QUEUE_DOMAIN_ATTRS(read),
	KYBER_QUEUE_DOMAIN_ATTRS(write),
	KYBER_QUEUE_DOMAIN_ATTRS(discard),
	KYBER_QUEUE_DOMAIN_ATTRS(other),
	{"async_depth", 0400, kyber_async_depth_show},
	{},
};
#undef KYBER_QUEUE_DOMAIN_ATTRS

#define KYBER_HCTX_DOMAIN_ATTRS(name)					\
	{#name "_rqs", 0400, .seq_ops = &kyber_##name##_rqs_seq_ops},	\
	{#name "_waiting", 0400, kyber_##name##_waiting_show}
static const struct blk_mq_debugfs_attr kyber_hctx_debugfs_attrs[] = {
	KYBER_HCTX_DOMAIN_ATTRS(read),
	KYBER_HCTX_DOMAIN_ATTRS(write),
	KYBER_HCTX_DOMAIN_ATTRS(discard),
	KYBER_HCTX_DOMAIN_ATTRS(other),
	{"cur_domain", 0400, kyber_cur_domain_show},
	{"batching", 0400, kyber_batching_show},
	{},
};
#undef KYBER_HCTX_DOMAIN_ATTRS
#endif

static struct elevator_type kyber_sched = {
	.ops = {
		.init_sched = kyber_init_sched,
		.exit_sched = kyber_exit_sched,
		.init_hctx = kyber_init_hctx,
		.exit_hctx = kyber_exit_hctx,
		.limit_depth = kyber_limit_depth,
		.bio_merge = kyber_bio_merge,
		.prepare_request = kyber_prepare_request,
		.insert_requests = kyber_insert_requests,
		.finish_request = kyber_finish_request,
		.requeue_request = kyber_finish_request,
		.completed_request = kyber_completed_request,
		.dispatch_request = kyber_dispatch_request,
		.has_work = kyber_has_work,
		.depth_updated = kyber_depth_updated,
	},
#ifdef CONFIG_BLK_DEBUG_FS
	.queue_debugfs_attrs = kyber_queue_debugfs_attrs,
	.hctx_debugfs_attrs = kyber_hctx_debugfs_attrs,
#endif
	.elevator_attrs = kyber_sched_attrs,
	.elevator_name = "kyber",
	.elevator_owner = THIS_MODULE,
};

static struct elevator_type kyber_qt_sched = {
	.ops = {
		.init_sched = kyber_init_sched,
		.exit_sched = kyber_exit_sched,
		.init_hctx = kyber_init_hctx,
		.exit_hctx = kyber_exit_hctx,
		.limit_depth = kyber_limit_depth,
		.bio_merge = kyber_bio_merge,
		.prepare_request = kyber_prepare_request,
		.insert_requests = kyber_insert_requests,
		.finish_request = kyber_finish_request,
		.requeue_request = kyber_requeue_request,
		.completed_request = kyber_completed_request,
		.dispatch_request = kyber_dispatch_request,
		.has_work = kyber_has_work,
		.depth_updated = kyber_depth_updated,
	},
#ifdef CONFIG_BLK_DEBUG_FS
	.queue_debugfs_attrs = kyber_queue_debugfs_attrs,
	.hctx_debugfs_attrs = kyber_hctx_debugfs_attrs,
#endif
	.elevator_attrs = kyber_qt_sched_attrs,
	.elevator_name = "kyber-qt",
	.elevator_owner = THIS_MODULE,
};

static int __init kyber_init(void)
{
	int ret;

	ret = elv_register(&kyber_sched);
	if (ret)
		return ret;
	ret = elv_register(&kyber_qt_sched);
	if (ret)
		elv_unregister(&kyber_sched);
	return ret;
}

static void __exit kyber_exit(void)
{
	elv_unregister(&kyber_qt_sched);
	elv_unregister(&kyber_sched);
}

module_init(kyber_init);
module_exit(kyber_exit);

MODULE_AUTHOR("Omar Sandoval");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Kyber I/O scheduler");
