# QueueTopia support in Kyber (Linux 6.1.53)

## Feasibility

The integration is feasible without changing `struct request` or the blk-mq
dispatch core. In this source version Kyber has the needed scheduler callbacks,
uses `rq->elv.priv[0]` for its domain token, and leaves `priv[1]` available for
QueueTopia request state. The implementation registers a second elevator named
`kyber-qt`; the existing `kyber` elevator retains its original behavior.

The implementation is queue scoped. All hctx instances for one request queue
share its credit bucket, admission cap, counters, and controller. It counts
requests carrying elevator-private state, not application operations or bytes.
It does not identify tenants or applications.

## Select and configure

Build `CONFIG_MQ_IOSCHED_KYBER`, then select `kyber-qt` for a queue. Its
attributes are under:

```text
/sys/block/<device>/queue/iosched/
```

`qt_control` selects the behavior:

| Value | Behavior |
|---|---|
| `0` | Observe lifecycle counts and growth; do not enforce rate or cap. Retains Kyber's native latency feedback. Default. |
| `1` | Enforce a fixed GCRA rate and queue-wide admission cap. |
| `2` | Enforce the same gate and adapt the rate with TRACK/DRAIN feedback. |

Changing `qt_control`, `qt_cap`, `qt_burst`, or the adaptive profile requires an
empty tracked queue; the update briefly quiesces the fast path. `qt_rate` can be
changed live; writing it resets the GCRA timestamp and sets both the current
rate and the rate restored at the next idle period. In adaptive mode, the
controller changes the current rate up to
`qt_max_rate`.
The principal parameters are `qt_cap`, `qt_burst`, `qt_low`, `qt_high`,
`qt_alpha`, `qt_beta`, `qt_epsilon`, `qt_epsilon_off`, `qt_confirm`, and
`qt_sample_ms`.
Validation enforces `qt_low < qt_high < qt_cap`, `qt_beta` in 1–99 percent,
`qt_confirm` in 1–64 samples, and a 1–1000 ms sampling interval.

Initial values are rate and maximum rate 2,000,000 requests/s, burst 128,
cap 256, low water 96, high water 192, alpha 10,000 requests/s, beta 90%,
entry and recovery growth thresholds 500 and 250 requests/s, three
confirmation samples, and a 10 ms sample interval. These remain prototype
defaults, not calibrated device settings.

In adaptive mode, confirmed growth, reaching the high watermark, or cap/domain
blocking enters DRAIN and cuts the rate. Further cuts require a complete
`qt_confirm` period above the low watermark without enough queue drain. Recovery
requires `qt_confirm` valid samples below the low watermark, below the recovery
growth threshold, and without cap/domain blocking. TRACK increases the rate by
`qt_alpha` only after the same stability check and when requests are pending.
Each rate change clears controller confirmations; a rate cut also discards
GCRA burst credit. If requests remain admitted for a full confirmation period
without a completion, new admissions stop until a completion is observed;
already admitted retries continue through Kyber's retry path. Idle queues reset
to their configured initial rate.

The growth estimate is the one-interval change in sampled `Q_adm`, divided by
the actual interval elapsed. `qt_confirm` confirms consecutive interval tests;
it does not create independent observations. The recovery threshold is
`qt_epsilon_off`, constrained to be below `qt_epsilon`. These thresholds and
sampling defaults have not been calibrated on the NVMe workload.

The read/write lifecycle counters are sharded per CPU. The active gate uses an
atomic global capacity reservation and a compare-and-swap GCRA timestamp, so it
does not take the old queue-wide spinlock on every request. The cap remains
exact across hctx instances; the rate limiter permits an initial burst of
`qt_burst` requests.

`qt_stats` reports G/A/C, X_B/X_Q, B_new, Q_adm, B_retry, the current rate,
GCRA debt, block counts, controller state, and the current stall confirmation
count. `rate_blocked` and `cap_blocked` count blocked dispatch attempts, not
unique requests. Counter snapshots can be transient while requests move between
CPUs. Trace events
`kyber:kyber_qt_block` and `kyber:kyber_qt_rate` record gate blocks and rate
changes.

## Lifecycle and limits

G increments on the first scheduler insertion. A increments on the first
successful departure from the scheduler. C increments when an admitted request
reaches its completion callback, whether it succeeded or failed. Requeued
requests keep their original admission and cap occupancy; they use a retry lane
and do not consume another first-admission credit. The domain token is reacquired
for each scheduler retry. Active-mode capacity reservations and releases use an
atomic queue-wide counter shared by all hctx instances.

The prototype does not classify lower-driver tag/budget failures, instrument
successful bytes, preserve per-application identity, or define general split,
merge, zoned, multipath, and stacked-device accounting. The initial experiment
should use the plan's fixed-size direct-I/O workload and one ordinary local
namespace. The growth controller uses the sampled Q_adm slope and high/low
watermarks, and treats cap or Kyber domain-token blocking as admission pressure;
it does not infer a specific SSD-internal failure cause.

## Source and verification

The nested Git repository starts with baseline commit `61894a1` (the supplied
Linux 6.1.53 source tree). `git diff` in this directory shows the integration
changes relative to that baseline.

`block/kyber-iosched.o` compiled with `W=1`. The final module was separately
built with `W=1` against the running host kernel's exact 6.1.53 configuration
(`SMP`, `modversions`) and loaded without rebooting.

A QEMU 6.2 TCG guest used four vCPUs, four virtio-blk queues, 512 MiB RAM, and a
disposable 32 MiB disk. The module test exercised observer, fixed, and adaptive
modes; busy observer-to-fixed and fixed-to-observer control changes were
rejected; fixed and adaptive mode each reached `cap_blocked` with cap=4; all
direct writes completed with G=A=C and B_new=Q_adm=0 after drain; and module
unload/reload worked. The guest powered off without a BUG, Oops, or panic.

### NVMe fio pilot

The host tests used `/dev/nvme2n1`, `io_uring`, direct 4 KiB random I/O, 70/30
read/write, eight jobs, 10 seconds measured plus 2 seconds ramp, and the same
16 GiB region at offset 16 GiB. QD128 used `8x16`; QD256 used `8x32`.

| Scheduler / setting | QD | Total IOPS | Read p99 | Write p99 |
|---|---:|---:|---:|---:|
| Kyber, first comparison | 128 | 881k | 1.466 ms | 0.099 ms |
| QT observer, per-CPU accounting | 128 | 885k | 1.466 ms | 0.092 ms |
| QT fixed, rate 2M / cap 256 | 128 | 884k | 1.450 ms | 0.110 ms |
| Kyber, interleaved repeat | 256 | 1,146k | 1.647 ms | 0.249 ms |
| QT fixed, rate 2M / cap 224 | 256 | 1,136k | 1.647 ms | 0.187 ms |
| QT fixed, cap 64 | 256 | 637k | 1.810 ms | 0.922 ms |
| QT adaptive, cap 64 / beta 85 | 256 | 317k | 2.007 ms | 1.548 ms |

Per-CPU sharding recovered observer performance from the earlier 495k IOPS to
885k, close to native Kyber. Across two short QD256 runs per mode, Kyber averaged
1,133k IOPS and QT cap=224 averaged 1,132k. Mean read p99 was 1.638 ms for
Kyber and 1.647 ms for QT; mean write p99 was 0.273 ms and 0.232 ms respectively.
This is a promising write-tail signal for this one read-heavy mixed workload,
not proof of a general advantage. The cap=64 and adaptive cap=64 runs reduced
throughput and did not improve tail latency. The adaptive rate moved between
roughly 277k and 545k IOPS during the run, so the controller still needs more
stability work.

The device is a multipath NVMe namespace: the test used the head node
`nvme2n1`, whose active path was `nvme2c2n1`. This result should not be treated
as a general-purpose device benchmark. The current implementation also counts
all requests in one queue-wide budget and has no per-application or per-cgroup
classification. A stronger QoS test needs competing traffic with distinct
latency objectives and an explicit policy for separating that traffic.

After the runs, SMART still reported `critical_warning=0`, `media_errors=0`,
and nine error-log entries; there were no new NVMe I/O errors. The fio process
was stopped, both the multipath head and active path schedulers were restored to
`none`, and the final module remains loaded with reference count zero. No reboot
was needed.


### Controller follow-up: October 10

The controller was updated to use separate growth entry and recovery thresholds,
include cap blocking in its admission-pressure signal, require stable samples
before recovery/probing, clear accumulated GCRA burst credit on rate cuts, and
pause new admissions after a full confirmation interval without completions.
Manual rate changes now clear old controller confirmations and start a fresh
observation epoch.

The latest module compiled with `W=1` against both the source tree and the
running host kernel configuration. The QEMU TCG run checked the new threshold
validation, drove adaptive mode into DRAIN under a low cap, completed I/O with
zero residual tracked requests, and passed unload/reload. The no-completion
suspension path was not fault-injected.

A short follow-up campaign used the same QD256, 4 KiB, 70/30 read/write
`io_uring` workload. Each fio run completed without errors:

| Scheduler / setting | Runs | Mean IOPS | Read p99 | Write p99 |
|---|---:|---:|---:|---:|
| Kyber, paired set | 2 | 1,139.6k | 1.638 ms | 0.262 ms |
| QT fixed, cap=224, paired set | 2 | 1,119.6k | 1.630 ms | 0.291 ms |
| QT fixed, cap=192 | 1 | 1,071.5k | 1.630 ms | 0.330 ms |
| QT adaptive, cap=224 | 1 | 601.6k | 1.679 ms | 0.758 ms |
| Latest module QT fixed, cap=224 | 1 | 1,133.5k | 1.647 ms | 0.196 ms |
| Latest module Kyber control | 1 | 1,130.3k | 1.647 ms | 0.264 ms |

The two-run set did not reproduce the earlier cap=224 write-tail improvement:
throughput was about 1.8% lower and mean write p99 about 11% higher than Kyber.
The final single paired run favored QT write p99 by about 26% with nearly equal
throughput, but that result is not repeatable evidence yet. Cap=192 reduced
throughput further. Adaptive mode fell to roughly 628k requests/s before the
queue drained and its idle reset restored the initial rate. These runs show the
current defaults need calibration; they do not establish a general QT benefit.

During the latest-module paired run, fio reported zero errors. NVMe SMART
peaked at 44°C, with `critical_warning=0`, `media_errors=0`, and the error-log
entry count unchanged at nine. There were no new NVMe errors, resets, Oops, or
panics. Earlier PolyOS allocation warnings were present in the kernel log but
did not recur during this campaign. The multipath head and active-path
schedulers were restored to `none`; no fio process remains, and the module is
loaded with reference count zero. Raw JSON and adaptive samples are under
`/tmp/queuetopia-nvme2-sharded/`.


### Write-heavy cap tuning: October 10

To find a quick workload-specific win, I swept 30/70, 70/30, and 90/10
read/write ratios at 4 KiB, `io_uring`, eight jobs, QD64 per job (aggregate
QD512), with a 12-second run and 2-second ramp. On the 30/70 workload, fixed
QT cap=128 and rate=2M kept throughput essentially level while reducing the
write tail. Two additional interleaved 20-second pairs reproduced that result:

| Scheduler / setting | Runs | Mean IOPS | Read total-latency p99 | Write total-latency p99 |
|---|---:|---:|---:|---:|
| Kyber | 2 | 1,239.4k | 0.979 ms | 1.745 ms |
| QT fixed, cap=128, rate=2M | 2 | 1,237.5k | 0.918 ms | 1.393 ms |

In these two pairs, QT throughput was 0.15% lower, read p99 was 6.3% lower,
and write p99 was 20.2% lower. `rate_blocked` remained zero while cap blocking
was active, so this gain came from limiting admitted queue depth, not rate
pacing. The sweep also showed why this cap must be selected for the workload:
at 70% reads, cap=128 lowered throughput by about 24%; at 90% reads, cap=224
lowered throughput by about 5.8% while cutting write p99 by about 45%.

For the tested write-heavy profile, configure the active path while its queue
is idle, in this order so the cap constraints remain valid:

```sh
echo kyber-qt > /sys/block/nvme2c2n1/queue/scheduler
p=/sys/block/nvme2c2n1/queue/iosched
echo 32 > "$p/qt_low"
echo 64 > "$p/qt_high"
echo 128 > "$p/qt_cap"
echo 2000000 > "$p/qt_rate"
echo 1 > "$p/qt_control"
```

This is evidence for a useful operating point on this NVMe and this mixed
workload, not a universal default. The read-heavy sweep shows that the same
cap can hurt other mixes. All fio jobs completed without errors. SMART peaked
at 49°C; `critical_warning=0`, `media_errors=0`, and the error-log entry count
stayed at nine. No new device errors, resets, Oops, or panics appeared, and the
active-path and multipath-head schedulers were restored to `none` after the
test. A plot-ready summary of the nine workload-sweep runs and four valid
interleaved samples is in
[`benchmark-data/qt-nvme2-fast-sweep.csv`](benchmark-data/qt-nvme2-fast-sweep.csv).
The interrupted, incomplete run is excluded. The CSV includes per-direction
IOPS plus total-latency and completion-latency p99 in microseconds. Raw JSON
and per-run health snapshots are in `/tmp/qt-fast15/`.
