#!/usr/bin/env python3
"""
TWR unwrap log analyzer: Tests 1 & 2 only (no plotting).

Test 1 (freshness):
  - Count exact repeats in pos_L_raw_mrad and pos_R_raw_mrad.
  - Report run-length stats (how long repeated stretches last).

Test 2 (unwrap/gate consistency):
  - Compute Δunwrap per sample and compare with accept flags and d*_wrapped.
  - Checks:
      acc==0  => Δunwrap == 0
      acc==1  => Δunwrap ≈ d_wrapped  (exact match in integer mrad units)
  - Reports mismatch counts and worst-case mismatches.

Assumes the same packet format you posted:
  Header: 32 bytes, magic "TWR1"
  Sample: 29 bytes, format "<hhhhhhBiiihh"
  Trailer CRC32: 4 bytes

Connects to your ESP32 repeater via TCP and processes ONE dump.
"""

import socket
import struct
import zlib
from dataclasses import dataclass
from typing import Dict, List, Tuple

HOST = "twr-repeater.local"
PORT = 9000

MAGIC = b"TWR1"
HEADER_SZ = 32
TRAILER_SZ = 4

SAMPLE_BYTES_EXPECTED = 29
HEADER_FMT = "<4sHHHHIIIII"          # 32 bytes
SAMPLE_FMT = "<hhhhhhBiiihh"         # 29 bytes


@dataclass
class Header:
    magic: bytes
    version: int
    msg_type: int
    sample_rate_hz: int
    sample_bytes: int
    sample_count: int
    start_index: int
    payload_bytes: int
    payload_crc32: int
    header_crc32: int


def recv_exact(s: socket.socket, n: int) -> bytes:
    chunks = []
    got = 0
    while got < n:
        b = s.recv(n - got)
        if not b:
            raise ConnectionError("Socket closed while receiving")
        chunks.append(b)
        got += len(b)
    return b"".join(chunks)


def recv_until_magic(s: socket.socket, magic: bytes) -> None:
    """Simple sync: scan byte-by-byte for MAGIC."""
    buf = bytearray()
    m = len(magic)
    while True:
        b = s.recv(1)
        if not b:
            raise ConnectionError("Socket closed before magic")
        buf += b
        if len(buf) >= m and bytes(buf[-m:]) == magic:
            return


def read_header(s: socket.socket) -> Header:
    recv_until_magic(s, MAGIC)
    rest = recv_exact(s, HEADER_SZ - len(MAGIC))
    raw = MAGIC + rest
    vals = struct.unpack(HEADER_FMT, raw)
    hdr = Header(*vals)

    # Header sanity checks (avoid blocking on nonsense lengths)
    if hdr.magic != MAGIC:
        raise ValueError(f"Bad magic: {hdr.magic!r}")
    if hdr.sample_bytes != SAMPLE_BYTES_EXPECTED:
        raise ValueError(f"Unexpected sample_bytes={hdr.sample_bytes}, expected {SAMPLE_BYTES_EXPECTED}")
    if hdr.payload_bytes != hdr.sample_count * hdr.sample_bytes:
        raise ValueError("Header inconsistent: payload_bytes != sample_count*sample_bytes")
    if hdr.sample_count <= 0 or hdr.sample_count > 500_000:
        raise ValueError(f"Unreasonable sample_count={hdr.sample_count}")
    return hdr


def unpack_samples(payload: bytes, sample_bytes: int) -> Dict[str, List[int]]:
    if sample_bytes != SAMPLE_BYTES_EXPECTED:
        raise ValueError(f"Expected sample_bytes={SAMPLE_BYTES_EXPECTED}, got {sample_bytes}")
    if struct.calcsize(SAMPLE_FMT) != SAMPLE_BYTES_EXPECTED:
        raise ValueError("SAMPLE_FMT size mismatch")

    N = len(payload) // sample_bytes
    if N * sample_bytes != len(payload):
        raise ValueError("Payload length not an integer multiple of sample_bytes")

    posL = [0] * N
    posR = [0] * N
    dL = [0] * N
    dR = [0] * N
    accL = [0] * N
    accR = [0] * N
    unwrapL = [0] * N
    unwrapR = [0] * N

    for i in range(N):
        off = i * sample_bytes
        chunk = payload[off:off + sample_bytes]
        (pL, pR, vL, vR, dl, dr, flags,
         uL, uR, xw, xd, xdp) = struct.unpack(SAMPLE_FMT, chunk)

        posL[i] = pL
        posR[i] = pR
        dL[i] = dl
        dR[i] = dr
        accL[i] = 1 if (flags & 0x01) else 0
        accR[i] = 1 if (flags & 0x02) else 0
        unwrapL[i] = uL
        unwrapR[i] = uR

    return {
        "posL_raw_mrad": posL,
        "posR_raw_mrad": posR,
        "dL_wrapped_mrad": dL,
        "dR_wrapped_mrad": dR,
        "accL": accL,
        "accR": accR,
        "unwrapL_mrad": unwrapL,
        "unwrapR_mrad": unwrapR,
    }


def repeat_stats(x: List[int]) -> Tuple[int, int, float, int]:
    """
    Returns:
      repeats: count of indices i>0 where x[i]==x[i-1]
      max_run: maximum run length of a constant-value stretch
      avg_run: average run length over all constant-value stretches (including length=1)
      run_count: number of runs
    """
    if not x:
        return 0, 0, 0.0, 0

    repeats = 0
    run_lengths = []
    run_len = 1

    for i in range(1, len(x)):
        if x[i] == x[i - 1]:
            repeats += 1
            run_len += 1
        else:
            run_lengths.append(run_len)
            run_len = 1
    run_lengths.append(run_len)

    max_run = max(run_lengths)
    avg_run = sum(run_lengths) / len(run_lengths)
    return repeats, max_run, avg_run, len(run_lengths)


def unwrap_gate_consistency(
    unwrap: List[int], d_wrapped: List[int], acc: List[int], label: str
) -> None:
    """
    Test 2:
      Δunwrap[i] = unwrap[i] - unwrap[i-1]
      Expect:
        acc[i]==0 => Δunwrap[i]==0
        acc[i]==1 => Δunwrap[i]==d_wrapped[i]
    All in integer mrad units, so exact equality should hold if the
    firmware logs are consistent with the described algorithm.
    """
    if not unwrap or len(unwrap) != len(d_wrapped) or len(unwrap) != len(acc):
        raise ValueError(f"{label}: array length mismatch")

    N = len(unwrap)
    mism_reject = 0
    mism_accept = 0
    worst_accept_err = 0
    worst_accept_i = None

    # Track a few example indices for quick inspection
    examples_reject = []
    examples_accept = []

    for i in range(1, N):
        du = unwrap[i] - unwrap[i - 1]
        if acc[i] == 0:
            if du != 0:
                mism_reject += 1
                if len(examples_reject) < 5:
                    examples_reject.append((i, du, d_wrapped[i], unwrap[i - 1], unwrap[i]))
        else:
            err = du - d_wrapped[i]
            if err != 0:
                mism_accept += 1
                ae = abs(err)
                if ae > worst_accept_err:
                    worst_accept_err = ae
                    worst_accept_i = i
                if len(examples_accept) < 5:
                    examples_accept.append((i, du, d_wrapped[i], err, unwrap[i - 1], unwrap[i]))

    total_reject = sum(1 for v in acc[1:] if v == 0)
    total_accept = (N - 1) - total_reject

    print(f"\n[Test 2] {label} unwrap/gate consistency (integer mrad)")
    print(f"  Samples (excluding i=0): {N-1}")
    print(f"  Accept: {total_accept}  Reject: {total_reject}")
    print(f"  Reject mismatches (acc=0 but Δunwrap!=0): {mism_reject}")
    print(f"  Accept mismatches (acc=1 but Δunwrap!=d_wrapped): {mism_accept}")

    if worst_accept_i is not None:
        i = worst_accept_i
        du = unwrap[i] - unwrap[i - 1]
        print(f"  Worst accept mismatch at i={i}:")
        print(f"    Δunwrap={du} mrad, d_wrapped={d_wrapped[i]} mrad, err={du - d_wrapped[i]} mrad")

    if examples_reject:
        print("  Example reject mismatches (i, Δunwrap, d_wrapped, unwrap[i-1], unwrap[i]):")
        for row in examples_reject:
            print(f"    {row}")

    if examples_accept:
        print("  Example accept mismatches (i, Δunwrap, d_wrapped, err, unwrap[i-1], unwrap[i]):")
        for row in examples_accept:
            print(f"    {row}")


def main() -> None:
    # Basic size checks
    if struct.calcsize(HEADER_FMT) != HEADER_SZ:
        raise RuntimeError("HEADER_FMT does not equal 32 bytes")
    if struct.calcsize(SAMPLE_FMT) != SAMPLE_BYTES_EXPECTED:
        raise RuntimeError("SAMPLE_FMT does not equal 29 bytes")

    with socket.create_connection((HOST, PORT), timeout=10) as s:
        s.settimeout(None)
        print(f"Connected to {HOST}:{PORT}, waiting for {MAGIC!r}...")

        hdr = read_header(s)
        print("Header:", hdr)

        payload = recv_exact(s, hdr.payload_bytes)
        trailer_crc = struct.unpack("<I", recv_exact(s, TRAILER_SZ))[0]
        calc_crc = zlib.crc32(payload) & 0xFFFFFFFF

        print(f"CRC header=0x{hdr.payload_crc32:08X} trailer=0x{trailer_crc:08X} calc=0x{calc_crc:08X}")
        if calc_crc != hdr.payload_crc32 or trailer_crc != hdr.payload_crc32:
            raise ValueError("CRC mismatch")
        if hdr.sample_bytes * hdr.sample_count != hdr.payload_bytes:
            raise ValueError("Header length fields inconsistent")

        d = unpack_samples(payload, hdr.sample_bytes)
        N = hdr.sample_count
        print(f"Unpacked {N} samples.")

        # -------------------------
        # Test 1: freshness repeats
        # -------------------------
        posL = d["posL_raw_mrad"]
        posR = d["posR_raw_mrad"]

        repL, maxrunL, avgrunL, runsL = repeat_stats(posL)
        repR, maxrunR, avgrunR, runsR = repeat_stats(posR)

        print("\n[Test 1] Raw wrapped position freshness (exact repeats)")
        print(f"  Left: repeats={repL}/{N-1} ({100.0*repL/(N-1):.2f}%), max_run={maxrunL}, avg_run={avgrunL:.2f}, runs={runsL}")
        print(f"  Right: repeats={repR}/{N-1} ({100.0*repR/(N-1):.2f}%), max_run={maxrunR}, avg_run={avgrunR:.2f}, runs={runsR}")

        # Optional: show a few longest repeated stretches for quick debugging
        def longest_runs(x: List[int], topk: int = 5) -> List[Tuple[int, int, int]]:
            """Return [(start_idx, length, value), ...] sorted by length desc."""
            runs = []
            start = 0
            for i in range(1, len(x) + 1):
                if i == len(x) or x[i] != x[i - 1]:
                    length = i - start
                    runs.append((start, length, x[start]))
                    start = i
            runs.sort(key=lambda r: r[1], reverse=True)
            return runs[:topk]

        print("  Longest constant runs (Left):", longest_runs(posL))
        print("  Longest constant runs (Right):", longest_runs(posR))

        # -----------------------------------
        # Test 2: unwrap vs accept & d_wrapped
        # -----------------------------------
        unwrap_gate_consistency(
            d["unwrapL_mrad"], d["dL_wrapped_mrad"], d["accL"], label="LEFT"
        )
        unwrap_gate_consistency(
            d["unwrapR_mrad"], d["dR_wrapped_mrad"], d["accR"], label="RIGHT"
        )


if __name__ == "__main__":
    main()
    
