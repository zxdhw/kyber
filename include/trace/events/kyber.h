/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM kyber

#if !defined(_TRACE_KYBER_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_KYBER_H

#include <linux/blkdev.h>
#include <linux/string.h>
#include <linux/tracepoint.h>

#define DOMAIN_LEN		16
#define LATENCY_TYPE_LEN	8

TRACE_EVENT(kyber_latency,

	TP_PROTO(dev_t dev, const char *domain, const char *type,
		 unsigned int percentile, unsigned int numerator,
		 unsigned int denominator, unsigned int samples),

	TP_ARGS(dev, domain, type, percentile, numerator, denominator, samples),

	TP_STRUCT__entry(
		__field(	dev_t,	dev				)
		__array(	char,	domain,	DOMAIN_LEN		)
		__array(	char,	type,	LATENCY_TYPE_LEN	)
		__field(	u8,	percentile			)
		__field(	u8,	numerator			)
		__field(	u8,	denominator			)
		__field(	unsigned int,	samples			)
	),

	TP_fast_assign(
		__entry->dev		= dev;
		strlcpy(__entry->domain, domain, sizeof(__entry->domain));
		strlcpy(__entry->type, type, sizeof(__entry->type));
		__entry->percentile	= percentile;
		__entry->numerator	= numerator;
		__entry->denominator	= denominator;
		__entry->samples	= samples;
	),

	TP_printk("%d,%d %s %s p%u %u/%u samples=%u",
		  MAJOR(__entry->dev), MINOR(__entry->dev), __entry->domain,
		  __entry->type, __entry->percentile, __entry->numerator,
		  __entry->denominator, __entry->samples)
);

TRACE_EVENT(kyber_adjust,

	TP_PROTO(dev_t dev, const char *domain, unsigned int depth),

	TP_ARGS(dev, domain, depth),

	TP_STRUCT__entry(
		__field(	dev_t,	dev			)
		__array(	char,	domain,	DOMAIN_LEN	)
		__field(	unsigned int,	depth		)
	),

	TP_fast_assign(
		__entry->dev		= dev;
		strlcpy(__entry->domain, domain, sizeof(__entry->domain));
		__entry->depth		= depth;
	),

	TP_printk("%d,%d %s %u",
		  MAJOR(__entry->dev), MINOR(__entry->dev), __entry->domain,
		  __entry->depth)
);

TRACE_EVENT(kyber_throttled,

	TP_PROTO(dev_t dev, const char *domain),

	TP_ARGS(dev, domain),

	TP_STRUCT__entry(
		__field(	dev_t,	dev			)
		__array(	char,	domain,	DOMAIN_LEN	)
	),

	TP_fast_assign(
		__entry->dev		= dev;
		strlcpy(__entry->domain, domain, sizeof(__entry->domain));
	),

	TP_printk("%d,%d %s", MAJOR(__entry->dev), MINOR(__entry->dev),
		  __entry->domain)
);

TRACE_EVENT(kyber_qt_block,

	TP_PROTO(dev_t dev, const char *reason, unsigned int rate, u64 q_adm),

	TP_ARGS(dev, reason, rate, q_adm),

	TP_STRUCT__entry(
		__field(dev_t, dev)
		__array(char, reason, 8)
		__field(unsigned int, rate)
		__field(u64, q_adm)
	),

	TP_fast_assign(
		__entry->dev = dev;
		strscpy(__entry->reason, reason, sizeof(__entry->reason));
		__entry->rate = rate;
		__entry->q_adm = q_adm;
	),

	TP_printk("%d,%d reason=%s rate=%u Q_adm=%llu",
		  MAJOR(__entry->dev), MINOR(__entry->dev), __entry->reason,
		  __entry->rate, (unsigned long long)__entry->q_adm)
);

TRACE_EVENT(kyber_qt_rate,

	TP_PROTO(dev_t dev, unsigned int rate, u64 q_adm, s64 growth,
		 bool draining),

	TP_ARGS(dev, rate, q_adm, growth, draining),

	TP_STRUCT__entry(
		__field(dev_t, dev)
		__field(unsigned int, rate)
		__field(u64, q_adm)
		__field(s64, growth)
		__field(bool, draining)
	),

	TP_fast_assign(
		__entry->dev = dev;
		__entry->rate = rate;
		__entry->q_adm = q_adm;
		__entry->growth = growth;
		__entry->draining = draining;
	),

	TP_printk("%d,%d rate=%u Q_adm=%llu growth=%lld mode=%s",
		  MAJOR(__entry->dev), MINOR(__entry->dev), __entry->rate,
		  (unsigned long long)__entry->q_adm,
		  (unsigned long long)__entry->growth,
		  __entry->draining ? "DRAIN" : "TRACK")
);

#define _TRACE_KYBER_H
#endif /* _TRACE_KYBER_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
