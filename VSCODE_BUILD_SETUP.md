# VS Code Build Setup

This document explains what was done to make `MESC_F405RG` compile and upload from VS Code.

## Overview

The working VS Code build path reuses the existing STM32CubeIDE-generated build system rather than replacing it with CMake or a handwritten Makefile.

The key VS Code files are:
- [.vscode/tasks.json](/Users/owhite/MESC_Firmware/.vscode/tasks.json)
- [.vscode/launch.json](/Users/owhite/MESC_Firmware/.vscode/launch.json)

The key CubeIDE-generated build files are:
- [MESC_F405RG/Debug/makefile](/Users/owhite/MESC_Firmware/MESC_F405RG/Debug/makefile)
- [MESC_F405RG/Debug/Core/Src/subdir.mk](/Users/owhite/MESC_Firmware/MESC_F405RG/Debug/Core/Src/subdir.mk)
- [MESC_F405RG/Debug/objects.list](/Users/owhite/MESC_Firmware/MESC_F405RG/Debug/objects.list)

## Main Problems Found

### 1. Plain `make` did not build

Running:

```bash
make -C MESC_F405RG/Debug -j
```

did not compile the project. It only ran cleanup.

The reason is that the generated makefiles ended up with:

```make
.DEFAULT_GOAL := clean
```

even though [MESC_F405RG/Debug/makefile](/Users/owhite/MESC_Firmware/MESC_F405RG/Debug/makefile) also defines:

```make
all: main-build
```

So the VS Code task had to invoke the real target explicitly:

```bash
make -C MESC_F405RG/Debug main-build -j
```

### 2. The wrong GCC was being used

The local shell default compiler was:

```bash
/usr/local/bin/arm-none-eabi-gcc
```

which was:

```text
GNU Arm Embedded Toolchain 10.3.1
```

That compiler failed on this project with:

```text
arm-none-eabi-gcc: error: unrecognized command-line option '-fcyclomatic-complexity'
```

The generated build files were created for the STM32CubeIDE 13.3 toolchain, so VS Code had to be pointed at the same compiler family that CubeIDE was using.

## Toolchain Used

The working toolchain path is:

```text
/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.macos64_1.0.0.202411102158/tools/bin
```

This path is injected into `PATH` by the VS Code tasks before calling `make`.

## VS Code Tasks

The compile task in [.vscode/tasks.json](/Users/owhite/MESC_Firmware/.vscode/tasks.json) was set up to:

1. launch `/bin/zsh -lc`
2. prepend the STM32CubeIDE 13.3 toolchain path to `PATH`
3. run:

```bash
make -C MESC_F405RG/Debug main-build -j
```

This is the important part of the setup. The task must:
- use the CubeIDE toolchain
- call `main-build`, not plain `make`

There is also:
- a `clean` task
- an `upload` task
- a `build + upload` task

## Run And Debug

The Run and Debug dropdown entry was added in [.vscode/launch.json](/Users/owhite/MESC_Firmware/.vscode/launch.json):

- `MESC_F405RG: compile only`

This exists so compile can be launched from the left-side Run and Debug UI, not just from `Terminal -> Run Task`.

There is also a debug entry and a combined build/upload path.

## Upload Path

Upload uses `openocd` with ST-Link and STM32F4 target scripts.

The task is defined in [.vscode/tasks.json](/Users/owhite/MESC_Firmware/.vscode/tasks.json) and uses:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program ${workspaceFolder}/MESC_F405RG/Debug/MESC_F405RG.elf verify reset exit"
```

## Result

With this setup in place:
- VS Code can compile the project
- VS Code can upload the firmware
- hardware was verified to still run and spin the motor

## Notes For Future Changes

- This setup currently depends on STM32CubeIDE-generated files in `MESC_F405RG/Debug`
- If CubeIDE regenerates the project, `subdir.mk`, `objects.list`, and other generated files may change
- If the build is ever migrated away from CubeIDE-generated makefiles, this document should be updated
- If a different STM32CubeIDE version is installed, the toolchain path in [.vscode/tasks.json](/Users/owhite/MESC_Firmware/.vscode/tasks.json) may need to be updated
