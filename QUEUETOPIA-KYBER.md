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
| `0` | Observe lifecycle counts and growth; do not enforce rate or cap. Default. |
| `1` | Enforce the fixed token-bucket rate and queue-wide admission cap. |
| `2` | Enforce the same gate and adapt the rate with TRACK/DRAIN feedback. |

Changing `qt_control` and the adaptive profile requires an empty tracked queue.
`qt_rate` can be changed live; writing it resets accumulated credits and sets the
rate used at the next idle period. In adaptive mode, the controller changes the
current rate up to `qt_max_rate`.
The principal parameters are `qt_cap`, `qt_burst`, `qt_low`, `qt_high`,
`qt_alpha`, `qt_beta`, `qt_epsilon`, `qt_confirm`, and `qt_sample_ms`.
Validation enforces `qt_low < qt_high < qt_cap`, `qt_beta` in 1–99 percent,
`qt_confirm` in 1–64 samples, and a 1–1000 ms sampling interval.

Initial values are rate 1,000,000 requests/s, burst 128, cap 128, low water 48,
high water 96, alpha 10,000 requests/s, beta 50%, epsilon 500 requests/s,
three confirmation samples, and a 10 ms sample interval. These are prototype
defaults, not calibrated device settings.

`qt_stats` reports G/A/C, X_B/X_Q, B_new, Q_adm, B_retry, the current rate and
credit, block counts, and controller state. Trace events
`kyber:kyber_qt_block` and `kyber:kyber_qt_rate` record gate blocks and rate
changes.

## Lifecycle and limits

G increments on the first scheduler insertion. A increments on the first
successful departure from the scheduler. C increments when an admitted request
reaches its completion callback, whether it succeeded or failed. Requeued
requests keep their original admission and cap occupancy; they use a retry lane
and do not consume another first-admission credit. The domain token is reacquired
for each scheduler retry. The cap is committed under one queue-level lock across
all hctx instances.

The prototype does not classify lower-driver tag/budget failures, instrument
successful bytes, preserve per-application identity, or define general split,
merge, zoned, multipath, and stacked-device accounting. The initial experiment
should use the plan's fixed-size direct-I/O workload and one ordinary local
namespace. The growth controller uses the sampled Q_adm slope and high/low
watermarks, and treats Kyber domain-token exhaustion as an admission-pressure
signal; it does not infer a specific SSD-internal failure cause.

## Source and verification

The nested Git repository starts with baseline commit `61894a1` (the supplied
Linux 6.1.53 source tree). `git diff` in this directory shows the integration
changes relative to that baseline.

The changed Kyber object compiled with `W=1` after generating an x86_64
`defconfig`. A separate minimal x86-64 kernel built from this commit also
booted in QEMU 6.2 with TCG software emulation, one vCPU, 512 MiB RAM, and a
disposable 32 MiB virtio block device. The smoke test selected `kyber-qt`,
switched `qt_control` through 0, 1, and 2, issued concurrent direct writes
from eight workers, and switched back to native `kyber`. All writes completed;
the final QueueTopia counters were G=A=C=1088 with B_new=Q_adm=0, and the VM
powered off cleanly without a kernel BUG, Oops, or panic.

The module path was also tested in a second QEMU guest configured with
`CONFIG_MODULES=y` and `CONFIG_MQ_IOSCHED_KYBER=m`: it loaded
`kyber-iosched.ko`, selected `kyber-qt`, completed direct I/O, switched to
`mq-deadline`, unloaded the module, then loaded it again and repeated the
selection and I/O successfully. This validates module registration and
unload/reload in the test guest; it does not make that module binary suitable
for the host kernel without building against the host's exact source and
configuration.

This confirms basic boot, scheduler selection, sysfs control, request
accounting, and virtual block I/O only. The run used a single virtual queue;
`cap_blocked` remained zero and the adaptive controller stayed in TRACK without
changing its rate. It does not validate cap saturation, adaptive transitions,
zero-rate retries, error/requeue injection, multi-hctx concurrency, or target
hardware behavior. Validate the target machine's config and kernel patches
before performance measurements, then exercise those cases on that build.
