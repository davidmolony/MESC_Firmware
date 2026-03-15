# MESC_Firmware

MESC Firmware is a field-oriented motor control firmware project for STM32 targets, with shared control logic in MESC_Common and board/runtime integration in platform folders such as MESC_F405RG and MESC_Interface.

## Project Summary

- Focus: reliable motor control, tuning, and persistence on STM32 hardware.
- Active, simplified integration path: STM32F405RG target using the MESC_Interface module set.
- USB CLI interface supports variable read/write, status, and persistence operations.
- Recent cleanup reduced legacy complexity and removed non-essential runtime layers for the active target.

## Simplified Build Target

The primary build target is now MESC_F405RG.

- Build artifacts are generated under MESC_F405RG/Debug.
- Main firmware image: MESC_F405RG/Debug/MESC_F405RG.elf
- Typical local flow:
	1. Compile: make -C MESC_F405RG/Debug main-build -j
	2. Flash: openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program MESC_F405RG/Debug/MESC_F405RG.elf verify reset exit"

## RTOS Removal (Active Target Runtime)

For the active F405 runtime path, scheduler startup and RTOS task execution were removed to simplify behavior and maintenance.

- No FreeRTOS scheduler startup is used in the active motor-control path.
- Interface and persistence services are retained through MESC_Interface and supporting modules.
- Variable management and USB CLI remain available, including SAVE/LOAD persistence flow.

## Specifications

- [RTOS_UNWIND.md](RTOS_UNWIND.md): RTOS unwind history, rationale, and validation checkpoints for the simplified runtime path.
- [VSCODE_BUILD_SETUP.md](VSCODE_BUILD_SETUP.md): VS Code task configuration and build/flash workflow details.
- [CLI_IMPLEMENTATION.md](CLI_IMPLEMENTATION.md): USB CLI architecture, command behavior, safety policy, and implementation/validation record.

## Acknowledgement

Thank you to David Molony, the original author of MESC, for creating and sharing the foundation this project is built on.
