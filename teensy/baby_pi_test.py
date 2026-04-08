#!/usr/bin/env python3
"""Minimal OpenAI API smoke test: ask for first 10 digits of pi."""

import os
import sys

from openai import OpenAI


def main() -> int:
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Error: OPENAI_API_KEY is not set.", file=sys.stderr)
        return 2

    model = os.getenv("OPENAI_MODEL", "gpt-5")
    client = OpenAI(api_key=api_key)

    try:
        resp = client.responses.create(
            model=model,
            input="Print only the first 10 digits of pi.",
        )
    except Exception as e:
        print(f"API error: {e}", file=sys.stderr)
        return 1

    text = getattr(resp, "output_text", "").strip()
    if not text:
        chunks = []
        for item in getattr(resp, "output", []):
            for c in getattr(item, "content", []):
                if getattr(c, "type", "") == "output_text":
                    chunks.append(getattr(c, "text", ""))
        text = "\n".join(chunks).strip()

    if not text:
        print("No text returned from API.", file=sys.stderr)
        return 1

    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
