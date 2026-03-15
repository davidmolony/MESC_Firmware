#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$ROOT_DIR/scripts/build_f405rg.sh"
"$ROOT_DIR/scripts/upload_f405rg.sh"
