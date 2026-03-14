# RTOS Unwind Handoff

Use this document as the handoff brief for a future Codex session that will simplify the firmware to a single-target, non-RTOS, non-CLI build.

Before starting work from this handoff, read these documents first:
- [VSCODE_BUILD_SETUP.md](VSCODE_BUILD_SETUP.md)
- [VSCODE_BOOTUP_COMMENTS.md](VSCODE_BOOTUP_COMMENTS.md)

## Handoff Prompt

```text
Perform a staged simplification of this firmware project with the following fixed scope:

Target constraints
- Support only `MESC_F405RG`
- Support only the `STM32F405`
- Support only the board config in `Wheely.h`
- Remove the CLI/TTerm-based runtime configuration path after a replacement persistent-memory loader is in place
- Remove the RTOS/FreeRTOS runtime from this target
- Retain motor-driving capability only
- Preserve the ability to read the existing persistent configuration stored in reserved flash/NVM
- Preserve and enable the ability to edit/update persistent configuration and retrieve live runtime data mapped to that persisted model

Requirements
- Work only within this target and do not preserve compatibility with other STM32 families or other board configs unless absolutely required by shared code
- Favor the smallest clean architecture that supports the fixed target
- Keep the interrupt-driven motor-control path intact
- Preserve compatibility with the existing persisted CLI variable set instead of pruning it during the initial migration
- Before each major change, inspect the current code and verify assumptions locally
- Make changes in stages so the project can be validated incrementally
- Use apply_patch for edits
- Do not revert unrelated user changes
- Update build/project files as needed so the RTOS and CLI code are no longer compiled for this target
- At the end, summarize:
  1. what was removed
  2. what persistent-memory mechanism replaced CLI-backed loading
  3. what files remain as the motor-only runtime path
  4. any residual risks or unverified assumptions

Execution order
1. Freeze the target to `MESC_F405RG` + `Wheely.h`
2. Skip variable-set reduction for now and preserve the full existing CLI variable set
3. Create a standalone persistent-memory reader for the existing flash format
4. Validate the new reader against the current CLI-loaded runtime values
5. Integrate the standalone persistent-memory loader into the startup path
6. Make the system no longer depend on CLI/TTerm for required initialization
7. Remove CLI/TTerm/app-style task code from the target build
8. Remove RTOS startup and task creation from the F405 target
9. Remove FreeRTOS/RTOS source and include dependencies from the F405 build configuration
10. Do a final cleanup pass for single-target simplifications
```

## Concrete Implementation Checklist

### 1. Freeze Target to `STM32F405` + `Wheely.h`

Files to inspect/edit:
- [MESC_F405RG/Core/Inc/main.h](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Inc/main.h)
- [MESC_F405RG/Core/Src/main.c](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Src/main.c)
- [MESC_F405RG/Core/Inc/Wheely.h](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Inc/Wheely.h)
- [MESC_F405RG/Core/Inc/MESC_F405.h](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Inc/MESC_F405.h)

Tasks:
- Confirm `Wheely.h` is the sole board-config source of truth.
- Remove or avoid any new logic meant to preserve alternate board configs for this target.
- Document which macros from `Wheely.h` are treated as fixed assumptions.

### 2. Skip Variable-Subset Reduction and Preserve the Existing CLI Variable Set

Files to inspect:
- [MESC_RTOS/MESC/MESCinterface.c](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/MESCinterface.c)
- [MESC_RTOS/TTerm/Core/TTerm_var.c](/Users/owhite/MESC_Firmware/MESC_RTOS/TTerm/Core/TTerm_var.c)
- [MESC_RTOS/TTerm/Core/include/TTerm.h](/Users/owhite/MESC_Firmware/MESC_RTOS/TTerm/Core/include/TTerm.h)

Tasks:
- Do not prune the CLI variable list during the first migration.
- Treat the full existing `TERM_addVar(...)` set in `MESCinterface.c` as the compatibility target for persistence.
- Note any variables that are obviously transient, but do not remove them from the initial persistence plan.

### 3. Create a Standalone Persistent-Memory Reader for the Existing Flash Format

Files to create/edit:
- New file: [MESC_F405RG/Core/Inc/mesc_persist.h](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Inc/mesc_persist.h)
- New file: [MESC_F405RG/Core/Src/mesc_persist.c](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Src/mesc_persist.c)
- Flash I/O reference: [MESC_RTOS/Common/RTOS_flash.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Common/RTOS_flash.c)
- Storage init reference: [MESC_RTOS/Tasks/task_cli.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/task_cli.c)
- CLI variable/storage reference: [MESC_RTOS/TTerm/Core/TTerm_var.c](/Users/owhite/MESC_Firmware/MESC_RTOS/TTerm/Core/TTerm_var.c)
- Variable definitions: [MESC_RTOS/MESC/MESCinterface.c](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/MESCinterface.c)
- Flash location definitions: [MESC_Common/Inc/MESChw_setup.h](/Users/owhite/MESC_Firmware/MESC_Common/Inc/MESChw_setup.h), [MESC_F405RG/STM32F405RGTX_FLASH.ld](/Users/owhite/MESC_Firmware/MESC_F405RG/STM32F405RGTX_FLASH.ld)

Tasks:
- Define a standalone API such as `MESC_PersistLoad(MESC_motor_typedef *m)` or equivalent.
- Read the same reserved flash/NVM region currently used by `TERM_VAR_init(...)`.
- Parse the existing on-flash variable storage format without depending on RTOS task infrastructure.
- Support the full existing persisted CLI variable set for compatibility.

### 4. Validate the New Reader Against the Current CLI-Loaded Runtime Values

Files to inspect/edit:
- [MESC_RTOS/Tasks/task_cli.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/task_cli.c)
- [MESC_RTOS/MESC/MESCinterface.c](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/MESCinterface.c)
- New loader files from step 3

Tasks:
- Run the new standalone reader while the existing CLI/RTOS path still exists.
- Compare loaded values against the current `CMD_varLoad(...)` behavior.
- Verify that important motor/runtime fields match after load.
- Fix any format, naming, typing, or callback-related mismatches before switching startup over.

Step 4 validation checklist (pass/fail):
1. Baseline capture (CLI path)
  - Build/upload current firmware with existing CLI path active.
  - Capture a baseline snapshot of values from `CLI_VALUES.txt` fields, including:
    - Core motor parameters: `par_*`, `FOC_*` setup values
    - Input/safety values: `adc*`, `safe_*`, `input_opt`, `uart_*`
    - Control mode/options: `opt_*`, `speed_*`
  - Mark each captured item as `R` or `R_W` according to `CLI_VALUES.txt`.
2. Reader parse and dataset integrity
  - Open latest dataset with standalone reader.
  - Validate header/footer markers, revision, entry count, and CRC.
  - PASS only if dataset validates and entry count is non-zero.
3. Name and type compatibility
  - For each persisted entry from flash, attempt name match to runtime variable registry.
  - Verify type and type-size match for each mapped variable.
  - PASS only if there are zero unexpected type/size mismatches for intended persisted fields.
4. Value equivalence check
  - Compare standalone-reader values with values loaded by `CMD_varLoad(...)`.
  - Use exact equality for integer/flag fields.
  - Use tolerance-based equality for floating-point fields (document tolerance used).
  - PASS only if all required `R_W` persisted fields match within criteria.
5. Callback side-effect equivalence
  - After applying values, verify derived recomputations are consistent with current behavior:
    - `calculateGains`
    - `calculateVoltageGain`
    - `calculateFlux`
    - `MESCinput_Init`
  - PASS only if post-load runtime state needed for motor operation is equivalent.
6. Access-behavior sanity
  - Confirm read-only (`R`) telemetry/readback fields are not treated as persisted writable targets.
  - Confirm read-write (`R_W`) fields remain writable and loadable.
  - PASS only if RO/RW behavior remains consistent with `CLI_VALUES.txt` intent.
7. Evidence package to keep in repo notes/handoff
  - Dataset revision tested.
  - Mismatch list (name/type/value/callback), if any.
  - Final pass/fail table for required fields.
  - Statement of residual risks before step 5 startup cutover.

### 5. Integrate the Standalone Persistent-Memory Loader into Startup

Files to edit:
- [MESC_F405RG/Core/Src/main.c](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Src/main.c)
- [MESC_Common/Src/MESCfoc.c](/Users/owhite/MESC_Firmware/MESC_Common/Src/MESCfoc.c)
- Possibly [MESC_Common/Src/MESCinput.c](/Users/owhite/MESC_Firmware/MESC_Common/Src/MESCinput.c)
- New loader files from step 3

Tasks:
- Call the new persistent loader at the correct point relative to `motor_init()` and `MESCfoc_Init()`, or explicitly justify the final ordering.
- Recompute derived values after loading where needed:
  - `calculateGains`
  - `calculateVoltageGain`
  - `calculateFlux`
  - `MESCinput_Init`
- Ensure startup still uses the interrupt-driven motor path.

### 6. Remove Dependence on CLI/TTerm for Required Initialization

Files to inspect/edit:
- [MESC_RTOS/Tasks/task_cli.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/task_cli.c)
- [MESC_RTOS/MESC/MESCinterface.c](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/MESCinterface.c)
- [MESC_RTOS/Tasks/init.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/init.c)

Tasks:
- Identify anything still required from `MESCinterface_init()` after the standalone loader exists.
- Move any truly required initialization into core startup or the new persistence layer.
- Leave CLI/TTerm with no mandatory role before removing it.

### 7. Remove CLI/TTerm/App-Style Task Code from Target Build

Files to remove from build and possibly delete or orphan:
- [MESC_RTOS/Tasks/task_cli.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/task_cli.c)
- [MESC_RTOS/Tasks/task_cli.h](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/task_cli.h)
- [MESC_RTOS/MESC/MESCinterface.c](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/MESCinterface.c)
- [MESC_RTOS/MESC/MESCinterface.h](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/MESCinterface.h)
- [MESC_RTOS/MESC/calibrate.c](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/calibrate.c)
- [MESC_RTOS/MESC/calibrate.h](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/calibrate.h)
- [MESC_RTOS/MESC/hfi.c](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/hfi.c)
- [MESC_RTOS/MESC/hfi.h](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/hfi.h)
- [MESC_RTOS/Tasks/apps.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/apps.c)
- [MESC_RTOS/Tasks/apps.h](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/apps.h)
- [MESC_RTOS/Tasks/top.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/top.c)
- [MESC_RTOS/Tasks/top.h](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/top.h)
- [MESC_RTOS/Tasks/task_overlay.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/task_overlay.c)
- [MESC_RTOS/Tasks/task_overlay.h](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/task_overlay.h)
- [MESC_RTOS/TTerm/Core/include/TTerm.h](/Users/owhite/MESC_Firmware/MESC_RTOS/TTerm/Core/include/TTerm.h) and associated TTerm sources, if no longer referenced

Tasks:
- Remove includes and compile references.
- Ensure no core file still includes task/terminal headers.

### 8. Remove RTOS Startup from F405 Target

Files to edit:
- [MESC_F405RG/Core/Src/main.c](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Src/main.c)
- [MESC_F405RG/Core/Src/freertos.c](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Src/freertos.c)
- [MESC_RTOS/Tasks/init.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/init.c)
- [MESC_RTOS/MESC/task_LED.c](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/task_LED.c) if still referenced

Tasks:
- Remove `cmsis_os.h`.
- Remove kernel init, task creation, `init_system()`, `osKernelStart()`, and `StartDefaultTask()`.
- Replace post-init flow with a simple non-RTOS idle loop.
- If LED indication is still desired, reimplement it later as simple polled logic, not a task.

### 9. Remove FreeRTOS/RTOS Build Dependencies from Target

Files to edit:
- [MESC_F405RG/.cproject](/Users/owhite/MESC_Firmware/MESC_F405RG/.cproject)
- [MESC_F405RG/.project](/Users/owhite/MESC_Firmware/MESC_F405RG/.project)
- [MESC_F405RG/MESC_F405RG.ioc](/Users/owhite/MESC_Firmware/MESC_F405RG/MESC_F405RG.ioc)
- [MESC_F405RG/Core/Inc/FreeRTOSConfig.h](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Inc/FreeRTOSConfig.h) if it remains but should no longer be used

Tasks:
- Remove RTOS/FreeRTOS include paths from the active build config.
- Remove `MESC_RTOS/*` source folders from the target build if they are no longer needed.
- Remove `freertos.c` from compilation.
- Regenerate or manually clean CubeIDE configuration as needed.

### 10. Final Single-Target Cleanup

Files to inspect/edit:
- [MESC_F405RG/Core/Src/stm32f4xx_it.c](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Src/stm32f4xx_it.c)
- [MESC_F405RG/Core/Src/MESChw_setup.c](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Src/MESChw_setup.c)
- [MESC_Common/Src/MESCfoc.c](/Users/owhite/MESC_Firmware/MESC_Common/Src/MESCfoc.c)
- [MESC_Common/Src/MESCinput.c](/Users/owhite/MESC_Firmware/MESC_Common/Src/MESCinput.c)

Tasks:
- Remove any leftover dead references to CLI, RTOS, TTerm, or multi-board abstractions that are no longer meaningful.
- Keep only the code needed for `STM32F405` + `Wheely.h` motor operation.
- Confirm the minimal runtime path is:
  - `main.c`
  - F405 peripheral setup
  - `MESChw_setup.c`
  - `MESCmotor.c`
  - `MESCfoc.c`
  - `MESCinput.c`
  - ISR handlers in `stm32f4xx_it.c`

## Validation Checkpoints

- After step 3: the standalone reader should successfully parse the existing persisted flash format.
- After step 4: the standalone reader should agree with the CLI-loaded runtime values.
- After step 5: startup should be able to load persisted configuration without relying on the CLI path, and derived values should still be recomputed correctly.
- After step 6: the firmware should no longer depend on CLI/TTerm for required initialization, and there should be a verified non-CLI runtime control path that can still command the motor.
- After step 8: target should boot without scheduler startup and still retain the intended motor-driving behavior.
- After step 9: build should no longer require FreeRTOS or RTOS task sources, and compile/upload from VS Code should still work.
- Final: motor-driving ISR path remains intact, persisted flash configuration remains readable, and no CLI/RTOS code is required for operation.

## Resume Prompt

```text
Resume work on this firmware simplification from the current repo state.

Current known state:
- Target has already been frozen to `STM32F405` + `Wheely.h`
- VS Code compile/upload now works using the STM32CubeIDE 13.3 toolchain
- Debug operation has not yet been proven end-to-end on this target
- Milestone completed: a documented and repeatable VS Code mechanism exists to build and upload firmware for this target
- The motor still spins on hardware in the current firmware
- A standalone persistent-memory reader has been added in:
  - `MESC_F405RG/Core/Inc/mesc_persist.h`
  - `MESC_F405RG/Core/Src/mesc_persist.c`
- That reader is currently compiled in but dormant; startup still relies on the existing CLI/RTOS load path
- The persistent flash/NVM data must be preserved and remain readable after CLI/RTOS removal
- Important new realization: the motor is currently being commanded through the CLI, so removing CLI requires a replacement runtime control path, not just a config/persistence replacement

Immediate priorities:
1. Validate the standalone persistent-memory reader against the current CLI-loaded runtime values
2. Identify exactly which CLI action/path is currently used to make the motor spin
3. Propose and implement the simplest non-CLI runtime control path so the motor can still be driven before CLI removal
4. Only after that, continue with integrating the persistent loader into startup and making CLI non-essential

Constraints:
- Keep support limited to `MESC_F405RG`, `STM32F405`, and `Wheely.h`
- Preserve motor-driving capability throughout the migration
- Preserve access to existing persisted flash configuration
- Ensure the non-CLI path can read, edit/update, and observe live data corresponding to persisted configuration
- Use apply_patch for edits
- Do not revert unrelated user changes

Before making major changes, inspect the current code and confirm assumptions locally.
```
