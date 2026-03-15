
# CAN Implementation Status and Roadmap

## What Has Been Completed

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
