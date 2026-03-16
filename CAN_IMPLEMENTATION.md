
# CAN Implementation Status and Roadmap

## What Has Been Completed

- **CAN Command Receive Confirmed (Current Build):**
  - The active firmware build now receives CAN control commands reliably in the no-RTOS path.
  - RX processing is performed in the active interface periodic loop by polling CAN FIFO0 and dispatching frames into the existing command callback logic.
  - IQREQ command reception has been validated on hardware (motor spin confirmed from CAN command input).
  - `status` output now includes `iqreq_rx` so command reception can be confirmed live during testing.

- **No-RTOS CAN Telemetry and Control:**  
  The firmware now operates without an RTOS, using a deterministic scheduler for CAN tasks.
- **POS/VEL Transmission:**  
  - POS/VEL frames are sent at 500 Hz per node using a strict periodic scheduler.
  - Each ESC transmits only for its configured node identity.
  - No catch-up bursts; only the latest value is sent.
  - Sequence counter included for loss/jitter measurement.
- **100Hz Status Frame:**  
  - As of commit `f0a061fa2cb1409fda189c87a192bd22b44178a0`, a 100Hz CAN status frame is sent, including:
    - SPEED (FOC.eHz)
    - BUS_VOLT_CURR (Vbus, Ibus)
    - MOTOR_CURRENT (Idq.q, Idq.d)
    - MOTOR_VOLTAGE (Vdq.q, Vdq.d)
    - TEMP_MOT_MOS1 (Motor_T, MOSu_T)
  - Integrated into the main CAN scheduler.
  - Preliminary testing shows correct operation.
- **Traffic Class Prioritization:**  
  - Class A (POS/VEL) and Class B (status/temps/debug) traffic are prioritized.
  - Mailbox pressure policy prevents low-priority traffic from blocking high-priority slots.
- **RX Command Handling:**  
  - IQREQ and ADC1_2_REQ are received and applied with minimal ISR work.
  - Latest-command semantics, per-ID counters, and timeouts are implemented.
- **Basic Instrumentation:**  
  - Counters for sent frames, slot misses, and blocked transmissions are available.
- **USB/FOC Coexistence:**  
  - FOC loop remains highest priority.
  - USB CLI remains responsive during CAN activity.

## What Remains To Be Completed

- **Comprehensive Instrumentation:**  
  - Implement and expose all metrics listed in the original plan (e.g., jitter histograms, dropout accumulators, p95/p99 timing, CAN error states, USB heartbeat watchdog).
- **Hardware Acceptance Filtering:**  
  - Ensure hardware CAN filters are configured to accept only required IDs for each node.
- **Full RX Path Hardening:**  
  - Confirm sender gating for IQREQ matches ADC1_2_REQ policy.
  - Complete per-ID RX error counters and timeout handling.
- **Stress and Regression Testing:**  
  - Run and document results for all quantitative thresholds (timing, jitter, dropouts, error states, USB latency).
  - Test with multiple ESCs on the same bus.
- **Documentation and Code Cleanup:**  
  - Remove all legacy/conditional code (e.g., #ifdef POSVEL_PLANE).
  - Update and maintain this document as features mature.
- **Pre-Flight Gate Checks:**  
  - Complete all pre-flight checks before expanding CAN features (see original plan for details).

---

## Addendum (2026-03-16): Dual-ESC Shared-Bus Findings

### Summary

- Single-ESC high-throughput operation has now been demonstrated as robust.
- Dual-ESC operation on one shared CAN bus shows repeatable asymmetry and frame-loss under concurrent traffic.
- The current bottleneck appears to be multi-node bus interaction (arbitration/scheduling and possibly physical-layer sensitivity), not a fundamental Teensy-ESC CAN incompatibility.

### Verified Results

- **Single ESC active (node 11):**
  - Sustained ESC POSVEL receive at approximately 500 Hz with counter integrity (`ctr_dup=0`, `ctr_jump=0`, `ctr_missed=0`).
  - Concurrent Teensy command TX remained healthy (zero TX enqueue failures).
  - This validates robust bidirectional throughput on a one-ESC bus.

- **Two ESCs active on one bus (nodes 11 and 12):**
  - At multiple command rates and patterns, one node often degraded while the other remained cleaner.
  - Observed failure signatures included rising counter jumps/missed counts, stale age spikes, and side-specific dropouts.
  - A/B tests (single-target, alternating-target) indicate contention/fairness effects are significant on the shared bus.

- **Control physical test (right branch unplugged/unpowered):**
  - Right branch state could strongly affect overall bus behavior.
  - In at least one test configuration, connecting an unpowered right ESC branch caused loss of expected receive behavior, reinforcing concern about physical-layer robustness and unpowered-node transceiver behavior.

### Interpretation

- The system can do high-throughput CAN with one ESC.
- Degradation emerges when scaling to two ESCs on one bus, likely from a combination of:
  - frame timing collisions between publishers,
  - arbitration priority asymmetry,
  - scheduler phase alignment,
  - and/or branch-level physical effects.

### Recommended Next Steps

- Keep single-bus tuning as an interim path:
  - stagger ESC telemetry phase,
  - reduce avoidable command traffic,
  - continue counter-based integrity monitoring.

- Preferred architecture direction for reliability:
  - move to dual independent CAN channels from Teensy (CAN1 and CAN2), one ESC per bus,
  - each bus with correct termination and known high-impedance behavior for unpowered nodes,
  - retain per-bus telemetry/error counters for validation.

### Practical Conclusion

- Current evidence supports feasibility of robust high-rate Teensy<->ESC CAN for a balancing robot in single-ESC operation.
- For dual-ESC robustness, separate CAN buses are now a strongly justified design direction.

---

## Reproducibility Runbook (2026-03-16)

### Objective

- Reproduce and verify high-throughput CAN behavior, first on a single ESC, then under dual-ESC bus loading.

### Steps Used To Produce Current Results

1. Build and flash ESC firmware (`MESC_F405RG`) with CAN POSVEL payload slot 0 set to a monotonic counter and slot 1 set to velocity.
2. Build and flash Teensy `can_driver` with counter-mode receive diagnostics enabled.
3. Enable counter diagnostics from Teensy USB serial:
  - `posvel_counter on`
4. Run the timed test sequence from Teensy USB serial:
  - `run`
5. Capture and compare these telemetry fields from `CAN_POSVEL_RX`:
  - `left_count` / `right_count`
  - `left_avg_gap_us` / `right_avg_gap_us`
  - `left_ctr_dup`, `left_ctr_jump`, `left_ctr_missed`
  - `right_ctr_dup`, `right_ctr_jump`, `right_ctr_missed`
6. Compare against `CAN_TXQ_SUM` and `CAN_TX_DONE` for Teensy transmit health (`attempts`, `ok`, `fail`).

### Device Firmware Load Reminder

- Teensy device:
  - Project: `can_driver`
  - Build command: `pio run`
  - Flash command (host/tooling dependent): `pio run -t upload`
  - Runtime test commands: `posvel_counter on`, then `run`

- ESC device(s):
  - Project target: `MESC_F405RG`
  - Build+flash script: `scripts/build_and_upload_f405rg.sh`
  - VS Code task alternative: `MESC_F405RG: build + upload`

### Notes For Repeatability

- Use one known wiring/termination configuration per test run and do not change it mid-run.
- Reset Teensy POSVEL stats at test start (already implemented in test entry path).
- Record whether right ESC branch is unplugged, powered, or connected-unpowered for each dataset.
