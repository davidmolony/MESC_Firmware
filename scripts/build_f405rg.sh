#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/MESC_F405RG/Debug"
TOOLCHAIN_BIN="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.macos64_1.0.0.202411102158/tools/bin"

export LC_ALL=C
export LANG=C
export TZ=UTC
export PATH="$TOOLCHAIN_BIN:/usr/local/bin:/usr/local/sbin:/opt/local/bin:/opt/local/sbin:/usr/bin:/bin:/usr/sbin:/sbin:${PATH:-}"

if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  echo "error: arm-none-eabi-gcc not found in PATH"
  exit 1
fi

JOBS="${MESC_JOBS:-$(sysctl -n hw.logicalcpu 2>/dev/null || echo 4)}"
DO_CLEAN=1

if [[ "${1:-}" == "--no-clean" ]]; then
  DO_CLEAN=0
fi

if [[ "$DO_CLEAN" -eq 1 ]]; then
  echo "==> Cleaning build directory"
  make -C "$BUILD_DIR" clean
fi

echo "==> Building MESC_F405RG (jobs=$JOBS)"
make -C "$BUILD_DIR" main-build -j"$JOBS"

echo "==> Build complete: $BUILD_DIR/MESC_F405RG.elf"

{
  echo "timestamp_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "host=$(uname -a)"
  echo "toolchain=$(arm-none-eabi-gcc --version | head -n 1)"
} > "$BUILD_DIR/build_env.txt"

shasum -a 256 "$BUILD_DIR/MESC_F405RG.elf" | tee "$BUILD_DIR/MESC_F405RG.elf.sha256"
echo "==> Wrote: $BUILD_DIR/build_env.txt"
echo "==> Wrote: $BUILD_DIR/MESC_F405RG.elf.sha256"
