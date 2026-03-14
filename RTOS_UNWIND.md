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

Completion of step 4:
`833cc4baaa58ababe459db9a30abfa4356766395`

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

Step 5 implementation status (2026-03-14):
- Implemented in code and flashed to target.
- Startup ordering is now:
  1. `motor_init(&mtr[0])`
  2. `MESCfoc_Init(&mtr[0])`
  3. `MESCinterface_startup_init()`
- `MESCinterface_startup_init()` now performs:
  - Variable-system bring-up for persistence storage access.
  - Variable registration (`populate_vars`) once.
  - Standalone reader apply first, with legacy `CMD_varLoad` fallback.
  - Derived recomputation: `calculateGains`, `calculateVoltageGain`, `calculateFlux`, `MESCinput_Init`.
- `MESCinterface_init()` remains for one-time CLI command registration only.
- Validation updates:
  - Reboot-time verification completed (user-confirmed): persisted values are applied correctly after power-cycle.
- Remaining validation before marking Step 5 complete:
  - Confirm no regression in `get/set/save/load` command behavior across transports.

### 6. Remove Dependence on CLI/TTerm for Required Initialization

Files to inspect/edit:
- [MESC_RTOS/Tasks/task_cli.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/task_cli.c)
- [MESC_RTOS/MESC/MESCinterface.c](/Users/owhite/MESC_Firmware/MESC_RTOS/MESC/MESCinterface.c)
- [MESC_RTOS/Tasks/init.c](/Users/owhite/MESC_Firmware/MESC_RTOS/Tasks/init.c)

Tasks:
- Identify anything still required from `MESCinterface_init()` after the standalone loader exists.
- Move any truly required initialization into core startup or the new persistence layer.
- Leave CLI/TTerm with no mandatory role before removing it.

Step 6 progress (2026-03-14):
- Core initialization decoupled from CLI task startup:
  - `MESCinterface_init(&null_handle)` now runs from `init_system()` before CLI tasks are created.
  - `task_cli.c` no longer calls `MESCinterface_init(...)` per transport task.
- Startup persistence path remains in `main.c` via `MESCinterface_startup_init()`.
- CLI tasks currently remain as transport/runtime shell only.
- Build/upload validation after change: PASS.
- Remaining Step 6 validation before completion:
  - Confirm `get/set/save/load` behavior is unchanged on active CLI transport(s).
  - Confirm there is no duplicate command registration when multiple transports are enabled.

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

Step 7 progress (2026-03-14):
- Partial removal slice completed:
  - Removed app-style command registration (`status`, `log`, `REGISTER_apps`) from `MESCinterface.c`.
  - Removed these modules from active target build lists (`Debug` generated make/object lists):
    - `MESC_RTOS/Tasks/app_template.c`
    - `MESC_RTOS/Tasks/apps.c`
    - `MESC_RTOS/Tasks/task_overlay.c`
    - `MESC_RTOS/Tasks/top.c`
    - `MESC_RTOS/MESC/calibrate.c`
    - `MESC_RTOS/MESC/hfi.c`
- Build/upload after this slice: PASS.
- Hardware validation after upload: motor spin test PASS (user-confirmed).
- Note: CLI/TTerm still present and active; full Step 7 removal is not complete yet.

Step 7 additional slice (2026-03-14, later):
- Removed `MESC_RTOS/Tasks/cana.c` from active target build/link lists.
- Intermediate compile regression was introduced during header decoupling attempt and then corrected (restored `task_cli.h` dependency path).
- Final state after correction:
  - Rebuild: PASS
  - Reflash/verify/reset: PASS
  - CLI/TTerm still present.

Step 7 additional slice (2026-03-14, latest):
- Introduced shared overlay type header `MESC_RTOS/Tasks/overlay_types.h`.
- `task_cli.h` now depends on `overlay_types.h` instead of `task_overlay.h`.
- `task_overlay.h` now consumes `overlay_types.h`.
- `task_can.h` now includes `TTerm.h` directly to provide `TERMINAL_HANDLE` type independently.
- Validation after refactor:
  - Rebuild: PASS
  - Reflash/verify/reset: PASS
  - Motor spin test: PASS (user-confirmed)

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

Step 8 progress (2026-03-14):
- Implemented in `main.c`:
  - Removed `cmsis_os.h` include.
  - Removed default task objects/prototype and `StartDefaultTask()` implementation.
  - Removed scheduler bring-up calls (`osKernelInitialize`, `osThreadNew`, `init_system`, `osKernelStart`).
  - Kept core startup ordering (`motor_init` -> `MESCfoc_Init` -> `MESCinterface_startup_init`) and switched USB device init to direct call in main startup path.
- Active build updates:
  - Removed `Core/Src/freertos.c` from `Debug/Core/Src/subdir.mk` source/object/dep lists.
  - Removed `./Core/Src/freertos.o` from `Debug/objects.list`.
- Validation after Step 8 slice:
  - Rebuild: PASS
  - Reflash/verify/reset: PASS
  - Motor spin test: PASS (user-confirmed)

Remaining for full RTOS removal (handled in step 9):
- FreeRTOS middleware and remaining RTOS task/object references are still present in build configuration and will be removed in dependency-cleanup step.

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

Step 9 progress (2026-03-14, slice 1):
- Active build/link cleanup completed:
  - Removed `MESC_RTOS/Tasks/*` objects from active `Debug` task subdir/object lists.
  - Removed `MESC_RTOS/MESC/task_LED.o` from active link path.
  - Kept `MESC_RTOS/MESC/MESCinterface.o`, `MESC_RTOS/Common/RTOS_flash.o`, and TTerm core objects for persistence startup path.
- Code-side dependency adjustments:
  - `null_handle` ownership moved into `MESCinterface.c` (no dependency on `Tasks/init.o`).
  - Added local USB CDC callback stub in `MESCinterface.c` to satisfy `usbd_cdc_if` link hook after `task_cli.o` removal.
  - Removed CAN command registration from `MESCinterface_init`; retained internal CAN state placeholder for compile compatibility.
- Validation after slice:
  - Rebuild: PASS
  - Reflash/verify/reset: PASS
  - Motor spin test: PASS (user-confirmed)

Step 9 progress (2026-03-14, slice 2):
- Removed all FreeRTOS scheduler objects from `objects.list`:
  - Dropped: `cmsis_os2.o`, `croutine.o`, `event_groups.o`, `list.o`, `queue.o`, `stream_buffer.o`, `tasks.o`, `timers.o`, `port.o`
  - Retained: `heap_4.o` (still needed by `pvPortMalloc` in TTerm/MESCinterface)
- Cleared corresponding `C_SRCS`/`OBJS`/`C_DEPS` entries from three subdir.mk files:
  - `Middlewares/Third_Party/FreeRTOS/Source/subdir.mk`
  - `Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk`
  - `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk`
- Added `vTaskSuspendAll()` / `xTaskResumeAll()` no-op stubs in `MESCinterface.c`
  - `heap_4.c` calls these two functions for malloc thread-safety; they are the only external scheduler symbols it references
  - Safe as no-ops because the FreeRTOS scheduler is never started
- Binary size after slice 2: 79,672 text / 900 data / 84,856 bss
- Validation after slice:
  - Rebuild: PASS
  - Reflash/verify/reset: PASS
  - Motor spin test: PASS (user-confirmed)

Step 9 progress (2026-03-14, metadata):
- Removed all FREERTOS entries from `MESC_F405RG.ioc`:
  - Deleted 7 `FREERTOS.*` parameter lines
  - Removed `Mcu.IP5=FREERTOS`; renumbered IP5–IP16 (formerly IP6–IP17); updated `Mcu.IPNb` 18→17
  - Removed `Mcu.Pin43=VP_FREERTOS_VS_CMSIS_V2`; renumbered Pin43–Pin49 (formerly Pin44–Pin50); updated `Mcu.PinsNb` 51→50
  - Removed `VP_FREERTOS_VS_CMSIS_V2.Mode/Signal` entries
  - Removed `rtos.0.ip=FREERTOS`
- Removed `MESC_RTOS/Tasks` source folder entry from `.cproject` (all task sources already empty)
- No changes to `.project` — `MESC_RTOS` linked folder still needed for active sources (Common, MESC, TTerm)

**Step 9: COMPLETE**

- Validation after metadata slice:
  - Rebuild: PASS (user-performed)
  - Reflash/verify/reset: PASS (user-performed)
  - Motor spin test: PASS (user-confirmed)

> **WARNING — .ioc code generation is a destructive operation.**
> The `.ioc` edits above are bookkeeping only. CubeMX code generation was deliberately NOT run after modifying the `.ioc`.
> Running "Generate Code" from CubeMX would overwrite `main.c` and other hand-modified files, undoing the manual changes made across Steps 7–9.
> The `.ioc` should be treated as a reference document that accurately describes the non-RTOS target state.
> If CubeMX code generation is ever needed in the future, all generated files (`main.c`, `freertos.c`, interrupt handlers, etc.) must be carefully diff-reviewed and reconciled against the manual edits before rebuilding.

### 10. Final Single-Target Cleanup

Files inspected:
- [MESC_F405RG/Core/Src/stm32f4xx_it.c](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Src/stm32f4xx_it.c) — clean, no RTOS/CLI refs
- [MESC_F405RG/Core/Src/MESChw_setup.c](/Users/owhite/MESC_Firmware/MESC_F405RG/Core/Src/MESChw_setup.c) — clean, no RTOS/CLI refs
- [MESC_Common/Src/MESCfoc.c](/Users/owhite/MESC_Firmware/MESC_Common/Src/MESCfoc.c) — clean, no RTOS/CLI refs
- [MESC_Common/Src/MESCinput.c](/Users/owhite/MESC_Firmware/MESC_Common/Src/MESCinput.c) — one stale comment updated

Additional cleanup in `MESC_Interface/MESC/MESCinterface.c` (active compiled file):
- Replaced all `vTaskDelay(N)` → `HAL_Delay(N)` in `CMD_measure` and other handlers (18 calls total)
- Removed all `xSemaphoreGive(port->term_block)` and `xQueueSemaphoreTake(port->term_block, portMAX_DELAY)` yield-point calls (9 pairs)
- Removed now-unused `port_str * port` local variable from `CMD_measure`
- Net effect: if `CMD_measure` is ever invoked over USB CDC, it will spin-wait correctly using HAL_Delay instead of crashing through undefined RTOS scheduler calls

Step 10 progress (2026-03-14):
- Rebuild: PASS (79,672 bytes text — identical to Step 9 slice 2, confirming no binary regression)
- Reflash/verify/reset: PASS
- Motor spin test: PASS (user-confirmed)

**Step 10: COMPLETE**

Post-step rename update (2026-03-14):
- Renamed top-level firmware interface folder: `MESC_RTOS` -> `MESC_Interface`
- Updated active project/build metadata paths (`.cproject`, `.project`, `Debug/makefile`, `Debug/sources.mk`, `Debug/objects.list`, and Debug `subdir.mk` files)
- Rebuild + reflash/verify/reset after rename: PASS
- Motor spin test after rename: PASS (user-confirmed)

AXIS cleanup follow-up (2026-03-14):
- Removed `MESC_Interface/AXIS/` directory (encoder app module not required for F405 motor operation path)
- Removed dormant AXIS hooks from task sources (`Tasks/init.c`, `Tasks/task_cli.c`, `Tasks/apps.c`)
- Updated legacy device-name literal in `TTerm/TTerm_config.h` from `"AXIS"` to `"MESC"`
- Rebuild + reflash/verify/reset after AXIS cleanup: PASS
- Motor spin test after AXIS cleanup: PASS (user-confirmed)

DASH cleanup follow-up (2026-03-14):
- Removed `MESC_Interface/Dash/` directory (not required for F405 motor operation path)
- Removed dormant DASH hooks from task sources (`Tasks/init.c`, `Tasks/task_cli.c`)
- Simplified `TTerm/TTerm_config.h` by removing DASH-specific `TERM_SUPPORT_CWD` branch
- Rebuild + reflash/verify/reset after DASH cleanup: PASS
- Motor spin test after DASH cleanup: PASS (user-confirmed)

Root artifact cleanup follow-up (2026-03-14):
- Removed orphan root file `CCER` (not referenced by build metadata or source includes)
- Rebuild + reflash/verify/reset after `CCER` removal: PASS
- Motor spin test after `CCER` removal: PASS (user-confirmed)

## Validation Checkpoints

Completed checkpoints:
- Step 3: standalone persistence reader parses existing flash format.
- Step 4: reader behavior aligns with CLI-loaded runtime values.
- Step 5: startup persistence loading works without scheduler startup dependencies.
- Step 6: required initialization no longer depends on RTOS task startup path.
- Step 8: target boots without scheduler start and retains motor-driving behavior.
- Step 9: build no longer requires FreeRTOS scheduler/CMSIS wrapper objects or RTOS task sources in the active link path.
- Step 10/final unwind: motor-driving ISR/runtime path remains intact; persisted flash configuration remains readable; all incremental cleanup slices validated with build/flash/spin tests.

Next-phase checkpoints (if continuing cleanup):
- If TTerm is removed, confirm startup persistence load and variable registration parity with current behavior.
- If `RTOS_flash.*` is renamed/refactored, confirm flash read/write compatibility on existing persisted datasets.
- After each further reduction slice: compile PASS, upload/verify PASS, motor spin PASS.

## Resume Prompt

as of git commit: 
`8a3d052c29bdc37aec694d40280fa0b8507cf42e`

```text
Resume work on this firmware simplification from the current repo state.

Current known state:
- Target is frozen to `STM32F405RG` + `Wheely.h`; multi-target runtime/task infrastructure has been removed from the active build path.
- RTOS unwind plan is complete through Step 10, including metadata alignment and post-cleanup hardware validation.
- VS Code compile/upload flow is stable with STM32CubeIDE 13.3 toolchain; repeated build/flash/verify cycles are passing.
- Motor spin is verified on hardware after all major cleanup slices (scheduler removal, task-object removal, metadata cleanup, folder rename, AXIS/DASH cleanup, orphan-file cleanup).
- Top-level interface folder has been renamed from `MESC_RTOS` to `MESC_Interface`.
- Removed modules not needed for current F405 motor path:
  - `MESC_Interface/AXIS/`
  - `MESC_Interface/Dash/`
  - orphan root file `CCER`
- Active persistence/load path is in `MESC_Interface/MESC/MESCinterface.c` startup init.
- FreeRTOS scheduler/CMSIS wrapper objects are removed from link; `heap_4.o` remains to satisfy allocator usage in retained TTerm/persistence path.
- TTerm is still compiled and linked intentionally for variable/persistence plumbing.

Immediate priorities:
1. Decide whether to keep TTerm as a lightweight variable/persistence backend or fully remove it.
2. If removing TTerm, replace `TERM_VAR_*` dependencies in `MESC_Interface/MESC/MESCinterface.c` with a minimal native persistence/parameter registry path.
3. Rename remaining legacy RTOS-prefixed files (for example `RTOS_flash.*`) only after functional parity is preserved.
4. Keep validating each cleanup slice with compile + flash + hardware spin test before proceeding.

Constraints:
- Keep support limited to `MESC_F405RG`, `STM32F405`, and `Wheely.h`.
- Preserve motor-driving capability throughout all changes.
- Preserve readability/compatibility of existing persisted flash configuration.
- Continue non-destructive incremental edits; do not revert unrelated user changes.

Before making major changes, inspect the current code and confirm assumptions locally.
```
