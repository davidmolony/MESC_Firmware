# CAN Implementation Strategy (No RTOS)

## Purpose

This document defines the CAN strategy for reliable telemetry and control on STM32F405 ESC nodes without reintroducing RTOS scheduling.

Primary goals:
- Transmit POS/VEL at 500 Hz per node with low jitter.
- Receive and apply `CAN_ID_IQREQ` and `CAN_ID_ADC1_2_REQ` robustly.
- Avoid burst/stall behavior observed in previous implementations.
- Preserve FOC control timing as highest priority.
- Preserve USB serial/CLI liveness while CAN traffic is active.

## Constraints and Context

- Runtime remains no-RTOS for the active target path.
- Prior intermittent issues were likely multi-factor: electrical integrity plus servicing/scheduling stalls.
- Trusted drop metric is per-frame sequence accounting in the CAN RX path on receiver side.
- FOC control-loop work must retain highest priority over CAN and USB servicing.

## Strategy

### 0) Priority and Scheduling Guardrails (FOC First)

- FOC control-loop timing is the top runtime priority.
- CAN and USB work must not add blocking behavior to the FOC critical path.
- CAN ISR/callback work is limited to lightweight ingest/flag/counter updates.
- Heavier work (parsing policy, state application, formatting, diagnostics) is deferred to non-ISR context.

### 1) Deterministic POS/VEL TX at 500 Hz

- Use a strict periodic scheduler for POS/VEL transmit every 2 ms.
- Preferred timing source:
  - Hardware timer callback, or
  - a strict 1 kHz loop with a 2 ms divider.
- Send exactly one POS/VEL frame per slot.
- Do not queue historical POS/VEL frames; keep latest-value only.
- Include a sequence counter in payload for receiver-side loss/jitter measurement.

### 2) Single-Node Deterministic Slot Policy

- Each ESC transmits POS/VEL only for its configured node identity.
- Keep a fixed 2 ms transmit slot schedule on that ESC (no opportunistic extra sends).
- Do not burst to catch up if a slot is missed; transmit the latest value at the next slot.
- If multiple ESCs are present on the same bus, arbitration handles coexistence; this firmware does not assume multi-node scheduling inside one ESC.

### 3) Traffic Class Prioritization

- Class A (strict): POS/VEL.
- Class B (best effort): status/temps/debug.
- If TX mailbox is busy at a POS/VEL slot:
  - increment `posvel_slot_miss`,
  - skip catch-up bursts,
  - send fresh data on next slot.
- Class B traffic may be skipped under pressure to protect Class A cadence.

Mailbox pressure policy:
- Reduce effective mailbox depth for low-priority traffic.
- Keep at most one fresh Class B frame pending per class; drop older stale data instead of queueing.
- Do not allow queued low-priority traffic to block the next Class A slot.
- If needed during hardening, operate in a reduced-mailbox strategy where only one mailbox is used for best-effort traffic while Class A attempts immediate send per slot.

### 4) RX Handling for `CAN_ID_IQREQ` and `CAN_ID_ADC1_2_REQ`

- Keep ISR work minimal:
  - parse ID and payload length,
  - update volatile latest-command storage,
  - update counters.
- No dynamic allocation, no blocking waits, and no heavy formatting in ISR.
- Use latest-command semantics (no deep queue for commands).
- Apply commands in the control loop with age/timeouts.
- Track per-ID counters:
  - `rx_ok`
  - `rx_bad_len`
  - `rx_overrun`
  - timeout count

### 4.1) CAN Identity Roles

- `node_id`:
  - Identity of this ESC on the CAN bus.
  - Used as destination matching for addressed packets and as sender identity for ESC-originated telemetry.
- `remote_ADC_can_id`:
  - Identity of the external command source (for example, the brain/controller board) that is allowed to provide remote ADC-style request inputs.
  - Used to gate acceptance of `CAN_ID_ADC1_2_REQ` so only the configured remote source is applied.

Current behavior note:
- `CAN_ID_ADC1_2_REQ` is sender-gated by `remote_ADC_can_id`.
- `CAN_ID_IQREQ` may currently be accepted without the same sender gate in legacy handling; if strict source control is required, apply the same sender-ID gate policy to IQ requests.

### 5) Tight Hardware Acceptance Filtering

- Configure CAN filters to accept only required IDs for this node:
  - `CAN_ID_IQREQ`
  - `CAN_ID_ADC1_2_REQ`
  - required control/session IDs
- Reject unrelated traffic in hardware to reduce ISR load and jitter.

### 6) Timeout and Failsafe Policy

- If command age exceeds timeout:
  - invalidate stale command,
  - fall back to safe default behavior.
- Keep separate timeout windows for IQ request and ADC request paths.

### 7) Continuous Instrumentation

Keep these counters/metrics always available during development and regression tests:

- TX metrics:
  - `posvel_sent`
  - `posvel_slot_miss`
  - `mailbox_busy_count`
- RX metrics:
  - per-ID `rx_ok`, `rx_bad_len`, `rx_overrun`
  - ISR ring high-water mark
- Timing metrics:
  - max inter-frame gap
  - p95/p99 inter-frame gap
  - burst-length histogram
  - per-slot jitter against 2 ms target (`abs(actual_period - 2 ms)`)
  - running jitter accumulator and max jitter

- Dropout metrics:
  - sequence-gap dropout counter
  - longest consecutive dropout run
  - running dropout accumulator

- Service-health metrics:
  - CAN error-active/passive/bus-off counters
  - USB CLI heartbeat/response watchdog counter

## Bus-Load and Stability Strategy

- Expected bus load for a single ESC at 500 Hz POS/VEL is modest at 1 Mbps.
- With multiple ESCs on the same bus, aggregate load is still usually acceptable at 1 Mbps if traffic classes and periodic send discipline are maintained.
- Main risk is not average utilization; main risk is burstiness and service latency spikes.
- Mitigation:
  - deterministic slot scheduler,
  - strict single-slot periodic send policy,
  - no catch-up bursts,
  - drop-old/keep-new for high-rate telemetry.

## Implementation Notes

- Reuse existing CAN ID definitions and helper packing functions where appropriate.
- Keep transport-layer logic separate from command backend logic.
- If command/control over CAN is added, route through the same backend used by USB CLI to preserve one safety/persistence policy.

## Pre-Flight Gate (Before CAN Feature Expansion)

These checks must pass before adding additional CAN features beyond baseline telemetry/command handling:

1. Priority map verification
- FOC path confirmed highest runtime priority.
- CAN/USB priorities configured so neither can starve FOC.

2. ISR budget verification
- CAN RX/TX ISR paths contain only minimal ingest/counter operations.
- No heavy processing or policy logic in ISR context.

3. Mailbox policy verification
- No catch-up bursting.
- Low-priority traffic cannot monopolize mailboxes.
- Class A slot behavior remains deterministic under load.

4. Instrumentation readiness
- Jitter, dropout, and CAN error counters available.
- USB heartbeat/liveness counter available.

5. USB coexistence verification
- USB CLI remains responsive during idle CAN traffic and stressed CAN traffic.

6. FOC timing regression check
- No measurable control-loop timing regression versus CAN-disabled baseline.

## Quantitative Thresholds (Initial)

These are initial acceptance thresholds for bring-up and early regression runs. Tighten as the implementation matures.

1. FOC timing budget
- Control-loop period jitter increase vs CAN-disabled baseline: <= 5% at p99.
- Worst-case control-loop period increase vs baseline: <= 10%.

2. POS/VEL cadence at 500 Hz (2 ms target)
- p99 inter-frame jitter: <= 150 us.
- Maximum inter-frame gap during nominal load: <= 4 ms.
- Maximum inter-frame gap during stress load: <= 6 ms.

3. Dropout limits (sequence-based)
- Nominal load: 0 sequence drops over 60 s window.
- Stress load: <= 0.1% sequence drop rate over 60 s window.
- Longest consecutive dropout run: <= 2 frames.

4. Mailbox/queue pressure limits
- `mailbox_busy_count` growth under nominal load: 0 sustained growth.
- `posvel_slot_miss` rate under stress load: <= 0.1% of slots.

5. RX integrity limits
- `rx_bad_len`: 0 under nominal load.
- `rx_overrun`: 0 under nominal load, and no sustained growth under stress.

6. CAN error-state limits
- Bus-off count: 0 in all acceptance runs.
- Error-passive transitions: 0 in nominal runs; investigate any nonzero during stress.

7. USB coexistence limits
- USB CLI heartbeat misses: 0 over 60 s nominal and stress windows.
- CLI command-response latency (p99) while CAN stress is active: <= 20 ms.

## Validation Plan

1. Baseline run:
- motors disconnected
- no torque command traffic
- no unrelated serial stressors

2. Add stressors one at a time:
- enable torque command traffic
- enable serial side traffic
- enable motor power stage activity
- combine serial + CAN stress to verify coexistence

3. Pass criteria:
- no command timeout anomalies under nominal conditions
- no ISR ring overruns
- bounded max gap for POS/VEL stream
- stable sequence continuity over long windows
- jitter accumulator and p99 jitter meet the quantitative thresholds above
- dropout accumulator and consecutive-dropout limits meet the quantitative thresholds above
- USB CLI heartbeat and command-response latency meet the quantitative thresholds above
