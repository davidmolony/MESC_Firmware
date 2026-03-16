#!/usr/bin/env python3
"""Capture and interpret Teensy balance-mode logs using the OpenAI API.

Usage:
  export OPENAI_API_KEY=...
  python interpret_balance_log.py --port /dev/ttyACM0
  python interpret_balance_log.py --file run.log
  cat run.log | python interpret_balance_log.py
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

from openai import OpenAI


DEFAULT_MODEL = os.getenv("OPENAI_MODEL", "gpt-5")

SYSTEM_PROMPT = (
    "You are an embedded controls log analyst. "
    "Given balance-mode JSONL logs, produce a concise interpretation with: "
    "1) loop timing health, 2) per-motor data-path health, 3) CAN transport health, "
    "4) notable anomalies, 5) practical bottom-line verdict. "
    "Use short bullets. Avoid fluff."
)


def read_log_text(path: Optional[str]) -> str:
    if path:
        with open(path, "r", encoding="utf-8") as f:
            return f.read()

    if not sys.stdin.isatty():
        return sys.stdin.read()

    raise ValueError("Provide --file <path> or pipe log text on stdin.")


def capture_log_from_serial(port: str, baud: int, timeout_s: float) -> str:
    try:
        import serial  # type: ignore
    except ImportError as e:
        raise RuntimeError("Missing dependency: pyserial. Install with `pip install pyserial`.") from e

    lines = []
    start = None
    saw_start = False
    saw_end = False

    with serial.Serial(port=port, baudrate=baud, timeout=0.1) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        input(f"Ready on {port}. Press Enter to send 'run' to Teensy...")
        ser.write(b"run\n")
        ser.flush()
        print("Sent: run")
        print("Capturing serial output...")

        start = time.monotonic()
        while True:
            raw = ser.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    print(line)
                    lines.append(line)
                    if "Balance mode started" in line:
                        saw_start = True
                    if "balance exit:" in line:
                        saw_end = True
                        break

            if time.monotonic() - start > timeout_s:
                break

    if not lines:
        raise RuntimeError("No serial data captured.")
    if not saw_start:
        print("Warning: did not observe 'Balance mode started' in captured output.", file=sys.stderr)
    if not saw_end:
        print("Warning: timed out before 'balance exit' was observed.", file=sys.stderr)

    return "\n".join(lines)


def build_user_prompt(log_text: str) -> str:
    return (
        "Interpret this balancing-robot runtime log. "
        "Be concrete with field names and values where relevant.\n\n"
        "Log:\n"
        f"{log_text.strip()}\n"
    )


def call_api(model: str, log_text: str) -> str:
    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    resp = client.responses.create(
        model=model,
        input=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": build_user_prompt(log_text)},
        ],
    )

    text = getattr(resp, "output_text", "").strip()
    if text:
        return text

    # Fallback for SDK variants without output_text convenience.
    chunks = []
    for item in getattr(resp, "output", []):
        for c in getattr(item, "content", []):
            if getattr(c, "type", "") == "output_text":
                chunks.append(getattr(c, "text", ""))
    return "\n".join(chunks).strip()


def format_api_error(e: Exception) -> str:
    msg = str(e)
    status_code = getattr(e, "status_code", None)
    code = None

    # Best-effort extraction of API error code (varies by SDK version).
    body = getattr(e, "body", None)
    if isinstance(body, dict):
        err = body.get("error")
        if isinstance(err, dict):
            code = err.get("code")
            if not msg and err.get("message"):
                msg = str(err["message"])

    if code == "insufficient_quota":
        return (
            "API quota exceeded (insufficient_quota). "
            "Add credits or update billing at https://platform.openai.com, "
            "then retry with the same key/project."
        )
    if status_code == 429:
        return (
            "API rate limit/quota error (429). "
            "Retry shortly; if it persists, check usage/billing limits."
        )
    if status_code in (401, 403):
        return (
            "API authentication/permission error. "
            "Verify OPENAI_API_KEY and project/org access."
        )
    if status_code:
        return f"API request failed (HTTP {status_code}): {msg}"
    return f"API request failed: {msg}"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Capture and interpret Teensy balance-mode logs with OpenAI API.")
    p.add_argument("--port", help="Serial port for Teensy, e.g. /dev/ttyACM0 (preferred mode).")
    p.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200).")
    p.add_argument(
        "--capture-timeout",
        type=float,
        default=8.0,
        help="Max seconds to wait for run completion on serial capture (default: 8.0).",
    )
    p.add_argument("--file", help="Path to log file. If omitted, reads stdin.")
    p.add_argument("--model", default=DEFAULT_MODEL, help=f"Model name (default: {DEFAULT_MODEL}).")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    if not os.getenv("OPENAI_API_KEY"):
        print("Error: OPENAI_API_KEY is not set.", file=sys.stderr)
        return 2

    if args.port:
        try:
            log_text = capture_log_from_serial(args.port, args.baud, args.capture_timeout)
        except Exception as e:  # pragma: no cover
            print(f"Serial capture error: {e}", file=sys.stderr)
            return 2
    else:
        try:
            log_text = read_log_text(args.file)
        except Exception as e:  # pragma: no cover
            print(f"Input error: {e}", file=sys.stderr)
            return 2

    if not log_text.strip():
        print("Input error: log text is empty.", file=sys.stderr)
        return 2

    try:
        analysis = call_api(args.model, log_text)
    except Exception as e:  # pragma: no cover
        print(f"API error: {format_api_error(e)}", file=sys.stderr)
        return 1

    if not analysis:
        print("API returned no text output.", file=sys.stderr)
        return 1

    print(analysis)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
