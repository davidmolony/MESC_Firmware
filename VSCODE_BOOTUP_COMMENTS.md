# VS Code Bootup Comments

Read this first: [VSCODE_BUILD_SETUP.md](VSCODE_BUILD_SETUP.md)

## Repeatable Build Guidance

1. Add a one-command bootstrap script that regenerates STM32CubeIDE managed build artifacts when `MESC_F405RG/Debug` is missing.
2. Keep the recovery workflow documented in `VSCODE_BUILD_SETUP.md` so users can quickly restore compile from the VS Code dropdown path.
