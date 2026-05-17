# MESC_Firmware

MESC Firmware is a field-oriented motor control firmware project for STM32 targets, with shared control logic in MESC_Common and board/runtime integration in platform folders such as MESC_F405RG and MESC_Interface.

## Summary: MESC Fork

- Reliable robotic motor control, removal non-essential runtime layers for the active target.
- Tuning, and persistence on STM32 hardware.
- Robust CAN communication scheduling.
- USB CLI interface supports variable read/write, status, and persistence operations.


## Simplified Build Target

The primary build target is now MESC_F405RG. This build was developed and tested with the assistance of GitHub Copilot (GPT-4.1) in Visual Studio Code, providing code review, refactoring, and documentation support throughout the project:

- Build artifacts are generated under MESC_F405RG/Debug.
- Main firmware image: MESC_F405RG/Debug/MESC_F405RG.elf
- Typical local flow:
	1. Compile (reproducible): ./scripts/build_f405rg.sh
	2. Flash (with fallback connection modes): ./scripts/upload_f405rg.sh
	3. Full flow: ./scripts/build_and_upload_f405rg.sh
    3. motor spin test
    4. CLI test

### Reproducible Build Notes

- Use the wrapper scripts in `scripts/` as the canonical workflow.
- The build wrapper sets a consistent PATH and locale, then performs a clean build by default.
- The build wrapper writes `MESC_F405RG/Debug/build_env.txt` and `MESC_F405RG/Debug/MESC_F405RG.elf.sha256` for run-to-run comparison.
- The upload wrapper retries OpenOCD with progressively safer connection sequences for hard-to-connect targets.
- VS Code tasks are wired to these scripts so terminal runs and task runs stay consistent.


## RTOS Removal (Active Target Runtime)

For the active F405 runtime path, scheduler startup and RTOS task execution were removed to simplify behavior and maintenance.

- No FreeRTOS scheduler startup is used in the active motor-control path.
- Interface and persistence services are retained through MESC_Interface and supporting modules.

## Specifications

- [RTOS_UNWIND.md](RTOS_UNWIND.md): RTOS unwind history, rationale, and validation checkpoints for the simplified runtime path.
- [VSCODE_BUILD_SETUP.md](VSCODE_BUILD_SETUP.md): VS Code task configuration and build/flash workflow details.
- [CLI_IMPLEMENTATION.md](CLI_IMPLEMENTATION.md): USB CLI architecture, command behavior, safety policy, and implementation/validation record.
- [CAN_IMPLEMENTATION.md](CAN_IMPLEMENTATION.md): CAN bus telemetry/control architecture, scheduler, and implementation/testing status.

## CAN Testing

Code in the `can_testing` directory was used to verify CAN bus communication and validate the firmware build during development. This includes Teensy-based test harnesses and monitoring tools for ESC telemetry and command compatibility.

### POSVEL Timebase Follow-Up

The current CAN POSVEL path intentionally syncs position/velocity feedback to incoming `CAN_ID_IQREQ` frames from the Teensy. That sync trigger is important for low-dropout command/feedback timing and should be retained.

However, receiver-only tests showed that autonomous `CAN_ID_POSVEL` telemetry can stop after about 25.56 seconds unless periodic IQREQ traffic is present. The likely cause is the POSVEL scheduler's use of `DWT_CYCCNT / 168` as a `uint32_t` microsecond clock; at 168 MHz, the 32-bit cycle counter wraps at about 25.56 seconds. Incoming IQREQ frames re-anchor the scheduler, which masks the wrap in synchronized operation but creates an unintuitive implicit keepalive requirement.

Potential fix:

- Introduce a wrap-safe monotonic POSVEL scheduler timebase instead of using raw `DWT_CYCCNT / 168` directly.
- Keep the IQREQ-triggered sync/nudge path and immediate POSVEL response for low-latency control correlation.
- Ensure autonomous POSVEL publishing continues when no IQREQ frames are present.
- Update related diagnostics such as POSVEL sync age and velocity `dt` calculations to use the wrap-safe timebase where appropriate.

Before merging that behavior change, repeat CAN dropout validation. At minimum, test autonomous POSVEL for multi-minute runs, IQREQ-synchronized POSVEL at the control cadence, mailbox-full/slot-miss counters, and dual-bus Teensy routing.

## Acknowledgement

Thanks to David Molony, the original author of MESC, for creating and sharing the foundational code.

## CAN Comment Marker Legend

The source now includes inline markers to highlight reliability-focused edits:

- `CAN reliability improvement:` marks code paths that were changed to improve CAN robustness, timing, or observability.
- `CAN reliability improvement baseline:` marks default configuration values used in validated stable runs.

Primary file using these markers in this repo:

- `MESC_Interface/MESC/MESCinterface.c`
