# CAN Implementation Strategy (No RTOS)

## Purpose

This document defines the CAN strategy for reliable telemetry and control on STM32F405 ESC nodes without reintroducing RTOS scheduling.

Primary goals:
- Transmit POS/VEL at 500 Hz per node with low jitter.
- Receive and apply `CAN_ID_IQREQ` and `CAN_ID_ADC1_2_REQ` robustly.
- Avoid burst/stall behavior observed in previous implementations.

## Constraints and Context

- Runtime remains no-RTOS for the active target path.
- Prior intermittent issues were likely multi-factor: electrical integrity plus servicing/scheduling stalls.
- Trusted drop metric is per-frame sequence accounting in the CAN RX path on receiver side.

## Strategy

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

### 4) RX Handling for `CAN_ID_IQREQ` and `CAN_ID_ADC1_2_REQ`

- Keep ISR work minimal:
  - parse ID and payload length,
  - update volatile latest-command storage,
  - update counters.
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

## Validation Plan

1. Baseline run:
- motors disconnected
- no torque command traffic
- no unrelated serial stressors

2. Add stressors one at a time:
- enable torque command traffic
- enable serial side traffic
- enable motor power stage activity

3. Pass criteria:
- no command timeout anomalies under nominal conditions
- no ISR ring overruns
- bounded max gap for POS/VEL stream
- stable sequence continuity over long windows
