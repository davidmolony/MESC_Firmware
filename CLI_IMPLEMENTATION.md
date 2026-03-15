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
- Active USB CLI implementation currently lives in `MESC_Interface/MESC/MESCinterface.c`, with USB CDC transport glue in `MESC_Interface/USB_DEVICE/App/usbd_cdc_if.c`.
- Build and flash workflow is stable and validated.

## Step 1: Define Variable Access Contract
Define a transport-agnostic API layer for variable access and persistence operations.

Status (2026-03-14): Superseded by current branch implementation.

Current branch reality:
- The active runtime does not use a separate transport-agnostic wrapper API layer.
- Variable access, safety checks, persistence calls, and minimal command dispatch are currently implemented directly in `MESC_Interface/MESC/MESCinterface.c` using existing TTerm variable descriptors and persistence helpers.
- Documentation below is retained as intent, but the validated implementation in this branch is the inline USB CLI path.

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
- Mutating operations in any other state return `ERR UNSAFE` in the CLI transport.
- Exception: `SET uart_req <value>` is allowed outside the general safe states when the requested value is within `-40` to `40`.

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

Current branch reality:
- No separate `cli_dispatcher.c` / `cli_dispatcher.h` module is present in the active tree.
- The active dispatcher is the inline minimal parser in `MESC_Interface/MESC/MESCinterface.c`.

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
- Error: short machine-oriented `ERR ...` forms

Validation after Step 3 implementation:
- Compile: PASS
- Upload/verify/reset: PASS
- Motor spin test: pending user confirmation for this CLI milestone

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

Status (2026-03-14): Implemented across `MESC_Interface/USB_DEVICE/App/usbd_cdc_if.c` and `MESC_Interface/MESC/MESCinterface.c`.

Implemented behavior:
- CDC banner on connect/open: `MESC CLI READY`
- RX forwarding from CDC transport into `USB_CDC_Callback(...)`
- Fixed-size line buffer with command dispatch on Enter
- CR/LF handling with clean newline before response output
- Overflow detection with deterministic error response
- Backspace/delete support
- Left/right cursor editing support
- Up/down RAM-only command history support
- Inline response transmit over CDC

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
- Current recompute sequence is applied inline after runtime LOAD and during startup init:
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

Status (2026-03-14): Partially validated; update as new runtime checks are confirmed.

Build/runtime checks:
- Compile: PASS
- Upload/verify/reset: PASS
- Motor spin: PASS

Functional checks:
- HELP returns expected command summary: PASS
- STATUS returns current state/write/load summary: PASS
- GET returns current value for `GET <name>` and list for bare `GET`: PASS
- LIST returns expected variable names: PASS
- SET updates allowed variable and rejects invalid writes: PASS
- SAVE returns success in allowed state: PASS
- LOAD returns success in allowed state and recomputes runtime values: PASS
- Cursor-aware left/right edit path: PASS (operator-confirmed)
- Command history up/down recall: pending operator confirmation

Negative tests:
- Unknown variable name: PASS
- Non-numeric value for numeric variable: PASS
- Out-of-range write: PASS
- Write to read-only variable: PASS
- Command while in disallowed motor state: PASS
- `uart_req` override outside safe state, within `-40..40`: PASS

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
- `SET uart_req 0` while otherwise unsafe -> expect `OK SET`
- Edit a numeric command with left/right arrows before Enter -> expect successful parse
- `Up` / `Down` arrows -> expect prior command recall / draft restore

## Step 7: Exit Criteria
Phase 1 is complete when:

Status (2026-03-14): Complete.

- USB CLI can safely read/write variables and manage persistence.
- Safety policy is enforced and test-proven.
- Persistence survives reset/power cycle and remains compatible.
- No CAN update path is required for completion.

## Immediate Priorities (Post-Phase-1)

Status (2026-03-14): Active.

Now that Phase 1 is complete, prioritize hardening and maintenance over new CLI feature work.

1) Documentation state alignment
- Keep this document synchronized with actual validated runtime/build state.
- Remove stale assumptions in handoff text before new work starts.

2) Regression gate for every change
- Run this gate before accepting any follow-up change:
  - clean build
  - compile
  - upload/verify/reset
  - motor spin test
  - USB CLI smoke + negative checks

3) Safety/persistence invariants must not regress
- Keep mutating command safety policy behavior unchanged unless explicitly planned.
- Keep SAVE/LOAD dataset compatibility and post-load recompute sequence unchanged.

4) Scope control
- Keep USB as the only CLI transport in this track.
- Defer CAN transport to a later, separate phase.

## Confirmed Policy Decisions (2026-03-14)

These decisions are fixed unless explicitly changed later.

1) Persistence writes are manual only
- `SAVE` remains operator-triggered only.
- No auto-save behavior is introduced in this track.

2) Backward compatibility is not a requirement for upcoming revisions
- Command/response refinements are allowed when they improve safety and maintainability.

3) Error semantics should stay concise
- Prefer short machine-oriented error forms.
- Avoid verbose response text in the CLI transport layer.

4) Response verbosity should stay minimal
- CLI output should not be chatty.
- Additional presentation/verbosity concerns are handled by a higher layer.

## Deferred Items (Future Phase)
- CAN telemetry integration (higher priority within CAN phase).
- CAN transport adapter using the same dispatcher/backend API.
- Extended remote update framing over CAN.

## HANDOFF PROMPT

Use this prompt to resume work in a new Codex session with minimal context loss.

```text
Resume work on post-Phase-1 CLI hardening and maintenance for the MESC firmware repo.

Project context and current state:
- Date checkpoint: 2026-03-14.
- Active hardware target: STM32F405RG with Wheely configuration.
- RTOS unwind milestone is complete and validated with repeated compile/upload/spin-test passes.
- Top-level firmware interface folder is now named MESC_Interface (renamed from MESC_RTOS).
- Removed as unneeded for active F405 motor path:
  - MESC_Interface/AXIS/
  - MESC_Interface/Dash/
  - root file CCER
- Current firmware keeps a lightweight retained variable/persistence plumbing path.
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
- Active CLI path is inline in `MESC_Interface/MESC/MESCinterface.c`, not a separate dispatcher module.

Immediate objective:
- Preserve validated USB CLI behavior and safety policy.
- Enforce repeatable regression validation for any code changes.
- Keep transport-neutral backend API stable so CAN can be added later without backend rewrite.

Confirmed policy constraints for next work:
- `SAVE` is manual only (no auto-save).
- Backward compatibility is optional for future CLI revisions.
- Error responses stay short.
- CLI transport output stays non-chatty.
- In CAN work, telemetry has higher priority than update/control framing.

Hardening requirements:
1) Preserve command contract and response format
  - Commands: HELP, LIST, GET, SET, SAVE, LOAD, STATUS
  - Deterministic one-line responses remain:
    - OK <...>
    - ERR <...>

2) Preserve safety behavior
  - Non-critical writes blocked in disallowed motor states
  - Reads always allowed
  - SAVE/LOAD only in allowed states

3) Preserve persistence parity
  - SAVE writes current RAM values to flash dataset
  - LOAD restores from flash dataset
  - After LOAD, apply required recompute flow

Validation gate after each slice:
- Compile: PASS
- Upload/verify/reset: PASS
- Motor spin: PASS
- USB CLI smoke + negative checks: PASS
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

Recommended first maintenance slice:
- Run a full clean -> compile -> upload -> spin -> USB CLI regression pass.
- Fix only regressions/warnings that affect safety, persistence correctness, or validated runtime behavior.
- Keep unrelated refactors out of this slice.

Recently completed milestone notes:
- Milestone commit: `3f46b23` (`Add interactive USB CLI milestone`)
- USB CLI banner is live.
- Minimal command set is operator-usable.
- Cursor-aware line editing is live.
- RAM-only command history is live.
- `uart_req` has bounded transient override support in unsafe states.

Hard constraints:
- Keep support scoped to MESC_F405RG / STM32F405 / Wheely.
- Do not reintroduce RTOS task startup.
- Do not add CAN interface in this phase.
- Preserve persisted dataset compatibility.
- Keep incremental, test-after-each-slice workflow.
```
