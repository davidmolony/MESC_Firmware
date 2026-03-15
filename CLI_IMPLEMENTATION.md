 # CLI Implementation Plan (Phase 1)

## Objective
Create a safe, maintainable variable update interface over USB CDC for the F405 firmware, without introducing CAN transport in this phase.

## Scope
- In scope:
  - Variable read/write commands over USB CDC
  - Variable listing and metadata visibility
  - Persistence actions (save/load)
  - Runtime safety guards for writes
- Out of scope:
  - CAN transport for variable updates
  - Multi-board abstractions beyond F405

## Baseline Assumptions
- Active target is STM32F405RG with Wheely configuration.
- Runtime variable and persistence logic currently lives in MESC_Interface/MESC/MESCinterface.c.
- Build and flash workflow is stable and validated.

## Step 1: Define Variable Access Contract
Define a transport-agnostic API layer for variable access and persistence operations.

Status (2026-03-14): Implemented in `MESC_Interface/MESC/MESCinterface.h` and `MESC_Interface/MESC/MESCinterface.c`.

Implemented API:
- `MESCinterface_var_count(uint32_t *out_count)`
- `MESCinterface_var_get_by_index(uint32_t index, MESC_VAR_META *out_meta)`
- `MESCinterface_var_get_meta(const char *name, MESC_VAR_META *out_meta)`
- `MESCinterface_var_get(const char *name, char *out_value, uint32_t out_len)`
- `MESCinterface_var_set(const char *name, const char *value)`
- `MESCinterface_var_save(void)`
- `MESCinterface_var_load(void)`

API result enum and metadata struct:
- `MESC_VAR_API_RESULT`
- `MESC_VAR_META`

Validation after Step 1 implementation:
- Compile: PASS
- Upload/verify/reset: PASS
- Motor spin test: PASS (user-confirmed)

Required operations:
- list_vars()
- get_var(name)
- set_var(name, value)
- save_vars()
- load_vars()
- get_status()

Behavior requirements:
- Enforce read/write permissions per variable.
- Enforce min/max bounds.
- Return explicit error codes for not found, read-only, out-of-range, parse error, and unsafe state.

## Step 2: Add Safety Policy
Define and enforce write safety rules.

Status (2026-03-14): Implemented in `MESC_Interface/MESC/MESCinterface.c`.

Implemented policy:
- Central safety gate added for mutating operations (`SET`, `SAVE`, `LOAD`).
- Writes are currently allowed only when motor state is one of:
  - `MOTOR_STATE_INITIALISING`
  - `MOTOR_STATE_TRACKING`
  - `MOTOR_STATE_IDLE`
- Mutating operations in any other state return `MESC_VAR_API_ERR_UNSAFE_STATE`.

Supporting API additions:
- Added `MESC_VAR_API_ERR_UNSAFE_STATE` to `MESC_VAR_API_RESULT`.
- Added `MESCinterface_var_get_status(uint32_t *motor_state, uint8_t *write_allowed, uint8_t *persist_loaded)` for dispatcher/status reporting.

Validation after Step 2 implementation:
- Compile: PASS
- Upload/verify/reset: PASS
- Motor spin test: pending user confirmation

Minimum policy:
- Reject non-critical writes while motor is in unsafe/active drive state unless explicitly whitelisted.
- Allow safe read-only operations at all times.
- Allow persistence load/save only in allowed states.
- Some session-control variables (for example `uart_req`) may be set during an active CLI session, but must be treated as transient and reset to `0` during boot initialization.

Deliverables:
- Central safety check function used by set/save/load paths.
- Clear response messages for denied operations.

## Step 3: Build Transport-Agnostic Dispatcher
Create one command dispatcher/parser that maps commands to the API in Step 1.

Status (2026-03-14): Implemented.

Implemented module:
- `MESC_Interface/MESC/cli_dispatcher.c`
- `MESC_Interface/MESC/cli_dispatcher.h`

Dispatcher entrypoint:
- `MESCcli_dispatch_line(const char *line, char *out, uint32_t out_len)`

Implemented commands:
- `HELP`
- `LIST`
- `GET [name]` (legacy-compatible semantics: bare `GET` == list, `GET <name>` returns only that variable value)
- `SET <name> <value>`
- `SAVE`
- `LOAD`
- `STATUS`

Response contract:
- Success: `OK ...`
- Error: `ERR <code> <reason>`

Build integration updates:
- Added dispatcher source/object to active F405 debug build lists:
  - `MESC_F405RG/Debug/MESC_Interface/MESC/subdir.mk`
  - `MESC_F405RG/Debug/objects.list`

Validation after Step 3 implementation:
- Compile: PASS
- Upload/verify/reset: pending
- Motor spin test: pending

Protocol style:
- Line-based ASCII commands
- Deterministic, machine-parseable responses

Initial command set:
- HELP
- LIST
- GET <name>
- SET <name> <value>
- SAVE
- LOAD
- STATUS

Legacy behavior compatibility note:
- `GET <name>` should return only the value for that variable (example: `GET input_opt`).
- Bare `GET` (no variable argument) should list all existing variables (equivalent to `LIST`).

Response format:
- One-line status prefix per command:
  - OK ...
  - ERR <code> <reason>

## Step 4: Integrate USB CDC Front End
Wire USB CDC receive callback to the dispatcher.

Status (2026-03-14): Implemented in `MESC_F405RG/USB_DEVICE/App/usbd_cdc_if.c`.

Implemented behavior:
- Byte-accumulating RX line buffer with fixed maximum line length
- CR/LF line termination handling
- Malformed control-character rejection
- Overflow detection with deterministic error response
- Dispatch through `MESCcli_dispatch_line(...)`
- One-line response transmit via CDC IN endpoint, with pending-response retry on TX complete callback

Validation after Step 4 implementation:
- Compile: PASS
- Upload/verify/reset: PASS
- Motor spin test: PASS

Requirements:
- Accumulate bytes into line buffer
- Handle CR/LF termination
- Protect against overflow and malformed input
- Return dispatcher responses over CDC transmit path

## Step 5: Persistence and Recompute Integration
Ensure SAVE/LOAD behavior is consistent with current startup logic.

Status (2026-03-14): Implemented in `MESC_Interface/MESC/MESCinterface.c`.

Implemented behavior:
- SAVE path remains `CMD_varSave(...)`, persisting current in-RAM variable values to flash dataset.
- LOAD path keeps `CMD_varLoad(...)` restore behavior and now applies the same post-load recompute flow used at startup.
- Centralized recompute sequence added via `mesc_apply_runtime_recompute()`:
  - `calculateGains(&mtr[0])`
  - `calculateVoltageGain(&mtr[0])`
  - `calculateFlux(&mtr[0])`
  - `MESCinput_Init(&mtr[0])`
- Successful runtime LOAD now sets `mesc_persist_loaded = true` so STATUS reflects loaded configuration state.

Validation after Step 5 implementation:
- Compile: PASS
- Upload/verify/reset: PASS
- Motor spin test: PASS

Requirements:
- SAVE writes current in-RAM values to flash dataset
- LOAD restores values from flash dataset
- After LOAD, recompute dependent runtime values (gains/flux/input init) exactly as required by existing flow

## Step 6: Validation Matrix
Run these checks after each implementation slice:

Status (2026-03-14): Completed (operator validation confirmed).

Build/runtime checks:
- Compile: PASS
- Upload/verify/reset: PASS
- Motor spin: PASS

Functional checks:
- LIST returns expected variables: PASS
- GET returns current value for `GET <name>` and list for bare `GET`: PASS
- SET updates allowed variable and rejects invalid writes: PASS
- SAVE persists update: PASS
- Power cycle + LOAD/startup confirms persistence: PASS

Negative tests:
- Unknown variable name: PASS
- Non-numeric value for numeric variable: PASS
- Out-of-range write: PASS
- Write to read-only variable: PASS
- Command while in disallowed motor state: PASS

Operator USB CLI script (run in safe state unless noted):
- `HELP` -> expect `OK HELP ...`
- `LIST` -> expect `OK LIST <count> ...`
- `GET` -> expect list output equivalent to `LIST`
- `GET input_opt` -> expect `OK <value>`
- `SET input_opt 1` -> expect `OK SET`
- `SET does_not_exist 1` -> expect `ERR NOT_FOUND ...`
- `SET input_opt not_a_number` -> expect `ERR PARSE ...` (or equivalent parse failure)
- `SAVE` -> expect `OK SAVE`
- Power cycle device
- `LOAD` -> expect `OK LOAD`
- `GET input_opt` -> expect persisted value
- `STATUS` while safe -> expect `write_allowed=1`
- `STATUS` while actively driving motor -> expect `write_allowed=0`, then run `SAVE` or `SET ...` and expect `ERR UNSAFE_STATE ...`

## Step 7: Exit Criteria
Phase 1 is complete when:

Status (2026-03-14): Complete.

- USB CLI can safely read/write variables and manage persistence.
- Safety policy is enforced and test-proven.
- Persistence survives reset/power cycle and remains compatible.
- No CAN update path is required for completion.

## Deferred Items (Future Phase)
- CAN transport adapter using the same dispatcher/backend API.
- Extended telemetry and remote update framing over CAN.

## HANDOFF PROMPT

Use this prompt to resume work in a new Codex session with minimal context loss.

```text
Resume implementation of Phase 1 USB CLI variable interface for the MESC firmware repo.

Project context and current state:
- Date checkpoint: 2026-03-14.
- Active hardware target: STM32F405RG with Wheely configuration.
- RTOS unwind milestone is complete and validated with repeated compile/upload/spin-test passes.
- Top-level firmware interface folder is now named MESC_Interface (renamed from MESC_RTOS).
- Removed as unneeded for active F405 motor path:
  - MESC_Interface/AXIS/
  - MESC_Interface/Dash/
  - root file CCER
- Current firmware still links TTerm and heap_4 for variable/persistence plumbing.
- FreeRTOS scheduler/CMSIS wrapper objects are removed from active link path.

Important files:
- MESC_Interface/MESC/MESCinterface.c
- MESC_Interface/MESC/MESCinterface.h
- MESC_F405RG/Core/Src/main.c
- USB_DEVICE/App/usbd_cdc_if.c
- MESC_F405RG/Core/Src/mesc_persist.c
- MESC_F405RG/Core/Inc/mesc_persist.h
- MESC_Interface/Common/RTOS_flash.c
- MESC_Interface/TTerm/Core/include/TTerm.h
- MESC_Interface/TTerm/Core/TTerm_var.c

Current behavior to preserve:
- Build + flash flow works from VS Code tasks.
- Motor spin test currently passes after all cleanup changes.
- Startup persistence load path in MESCinterface startup init is operational.

Phase 1 objective (no CAN in this phase):
- Implement safe variable management over USB CDC only.
- Commands: HELP, LIST, GET <name>, SET <name> <value>, SAVE, LOAD, STATUS.
- Keep transport-neutral backend API so CAN can be added later without backend rewrite.

Implementation requirements:
1) Define backend API layer
  - list_vars, get_var, set_var, save_vars, load_vars, get_status
  - enforce rw permissions and min/max bounds
  - return explicit error codes (not found, read-only, out-of-range, parse error, unsafe-state)

2) Add write safety policy
  - reject non-critical writes while motor is in unsafe/active drive state
  - reads always allowed
  - save/load allowed only in safe states

3) Build parser/dispatcher
  - line-based ASCII
  - deterministic one-line responses:
    - OK <...>
    - ERR <CODE> <reason>

4) Integrate USB CDC RX/TX
  - hook RX callback to line buffer
  - parse on CR/LF
  - guard against overflow and malformed commands
  - emit responses via CDC transmit

5) Persistence parity
  - SAVE writes current RAM values to flash dataset
  - LOAD restores from flash dataset
  - after LOAD, recompute derived runtime values exactly as current startup does

Validation gate after each slice:
- Compile: PASS
- Upload/verify/reset: PASS
- Motor spin: PASS
- Functional checks:
  - LIST returns expected variables
  - GET returns expected value
  - SET updates valid rw variable
  - SAVE persists
  - power cycle + startup confirms persistence
- Negative checks:
  - unknown variable
  - invalid numeric parse
  - out-of-range write
  - write to read-only variable
  - write/load/save in disallowed motor state

Known warnings currently seen in build (pre-existing unless changed):
- TTerm float->double promotion warning in TTerm_var.c
- array-bounds warning in TTerm_AC.c
- HUGE_VAL conversion warnings in MESCinterface variable registration
- unused MX_I2C2_Init warning in main.c
- linker RWX segment warning

Recommended first coding slice:
- Add a new small module under MESC_Interface/MESC for command dispatch + backend wrappers, with no transport glue yet.
- Unit-test-like validation via direct function calls from a temporary debug hook.
- Once stable, connect only USB CDC callback path.

Hard constraints:
- Keep support scoped to MESC_F405RG / STM32F405 / Wheely.
- Do not reintroduce RTOS task startup.
- Do not add CAN interface in this phase.
- Preserve persisted dataset compatibility.
- Keep incremental, test-after-each-slice workflow.
```
