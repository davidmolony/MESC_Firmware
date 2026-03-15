#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="$ROOT_DIR/MESC_F405RG/Debug/MESC_F405RG.elf"
OPENOCD_BIN="${OPENOCD_BIN:-openocd}"

if [[ ! -f "$ELF" ]]; then
  echo "error: ELF not found: $ELF"
  echo "hint: run scripts/build_f405rg.sh first"
  exit 1
fi

if pgrep -x openocd >/dev/null 2>&1; then
  echo "error: another openocd process is already running"
  echo "hint: stop the existing debug server before flashing"
  exit 1
fi

run_openocd() {
  local sequence="$1"
  echo "==> OpenOCD sequence: $sequence"
  "$OPENOCD_BIN" -f interface/stlink.cfg -f target/stm32f4x.cfg -c "$sequence"
}

if run_openocd "program $ELF verify reset exit"; then
  echo "==> Upload successful (standard)"
  exit 0
fi

if run_openocd "adapter speed 1000; init; reset halt; program $ELF verify reset exit"; then
  echo "==> Upload successful (reset-halt fallback)"
  exit 0
fi

if run_openocd "adapter speed 100; reset_config srst_only srst_nogate connect_assert_srst; init; reset halt; program $ELF verify reset exit"; then
  echo "==> Upload successful (connect-under-reset fallback)"
  exit 0
fi

echo "error: upload failed after all retry sequences"
exit 1
