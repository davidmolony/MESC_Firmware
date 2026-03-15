#!/usr/bin/env python3
import socket, struct, zlib
from dataclasses import dataclass

import matplotlib.pyplot as plt

HOST = "twr-repeater.local"
PORT = 9000

MAGIC = b"TWR1"
HEADER_SZ = 32
TRAILER_SZ = 4

# Scales (match Teensy fixed-point encoding)
MRAD_TO_RAD = 1e-3
MM_TO_M = 1e-3
MMPS_TO_MPS = 1e-3

LPF_FC_HZ = 20.0  # match firmware

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

def lpf_1pole(x, fs, fc):
    """1st-order low-pass, same structure as firmware (y += alpha*(x-y))."""
    import math
    if not x:
        return []
    dt = 1.0 / fs
    RC = 1.0 / (2.0 * math.pi * fc)
    alpha = dt / (dt + RC)
    y = [0.0] * len(x)
    y[0] = float(x[0])
    for i in range(1, len(x)):
        y[i] = y[i - 1] + alpha * (float(x[i]) - y[i - 1])
    return y

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
    # Read byte-by-byte until we see MAGIC. Simple + robust.
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
    rest = recv_exact(s, HEADER_SZ - 4)
    raw = MAGIC + rest

    fmt = "<4s H H H H I I I I I"
    vals = struct.unpack(fmt, raw)
    return Header(*vals)

def unpack_samples(payload: bytes, sample_bytes: int):
    if sample_bytes != 29:
        raise ValueError(f"Expected sample_bytes=29, got {sample_bytes}")

    sample_fmt = "<hhhhhhBiiihh"
    assert struct.calcsize(sample_fmt) == 29

    # Store as arrays (faster + easier to plot)
    N = len(payload) // sample_bytes
    posL = [0]*N
    posR = [0]*N
    velL = [0]*N
    velR = [0]*N
    dL = [0]*N
    dR = [0]*N
    accL = [0]*N
    accR = [0]*N
    unwrapL = [0]*N
    unwrapR = [0]*N
    x_mm = [0]*N
    xdot_mmps = [0]*N
    xdot_from_pos_mmps = [0]*N

    for i in range(N):
        off = i * sample_bytes
        chunk = payload[off:off+sample_bytes]
        (pL, pR, vL, vR, dl, dr, flags,
         uL, uR, xw, xd, xdp) = struct.unpack(sample_fmt, chunk)

        posL[i] = pL
        posR[i] = pR
        velL[i] = vL
        velR[i] = vR
        dL[i] = dl
        dR[i] = dr
        accL[i] = 1 if (flags & 0x01) else 0
        accR[i] = 1 if (flags & 0x02) else 0
        unwrapL[i] = uL
        unwrapR[i] = uR
        x_mm[i] = xw
        xdot_mmps[i] = xd
        xdot_from_pos_mmps[i] = xdp

    return {
        "posL_mrad": posL,
        "posR_mrad": posR,
        "velL_mrad_s": velL,
        "velR_mrad_s": velR,
        "dL_mrad": dL,
        "dR_mrad": dR,
        "accL": accL,
        "accR": accR,
        "unwrapL_mrad": unwrapL,
        "unwrapR_mrad": unwrapR,
        "x_wheel_mm": x_mm,
        "x_dot_mm_s": xdot_mmps,  # NOTE: this is already ESC vel + LPF from firmware
        "x_dot_from_pos_mm_s": xdot_from_pos_mmps,  # diagnostic
    }

def plot_unwrap(hdr: Header, d: dict):
    fs = hdr.sample_rate_hz
    N = hdr.sample_count
    t = [i / fs for i in range(N)]
    dt = 1.0 / fs

    # Convert to engineering units
    posL = [x * MRAD_TO_RAD for x in d["posL_mrad"]]
    posR = [x * MRAD_TO_RAD for x in d["posR_mrad"]]
    dL   = [x * MRAD_TO_RAD for x in d["dL_mrad"]]
    dR   = [x * MRAD_TO_RAD for x in d["dR_mrad"]]
    unwrapL = [x * MRAD_TO_RAD for x in d["unwrapL_mrad"]]
    unwrapR = [x * MRAD_TO_RAD for x in d["unwrapR_mrad"]]
    x_wheel = [x * MM_TO_M for x in d["x_wheel_mm"]]
    x_dot   = [x * MMPS_TO_MPS for x in d["x_dot_mm_s"]]
    x_dot_from_pos_logged = [x * MMPS_TO_MPS for x in d["x_dot_from_pos_mm_s"]]

    # Raw ESC wheel velocities (per wheel) in rad/s
    velL_raw = [x * MRAD_TO_RAD for x in d["velL_mrad_s"]]
    velR_raw = [x * MRAD_TO_RAD for x in d["velR_mrad_s"]]

    accL = d["accL"]
    accR = d["accR"]

    # --- Figure 1: wrapped positions ---
    plt.figure()
    plt.plot(t, posL, label="pos_L_raw (rad)")
    plt.plot(t, posR, label="pos_R_raw (rad)")
    plt.xlabel("time (s)")
    plt.ylabel("wrapped angle (rad)")
    plt.title("ESC wrapped wheel position")
    plt.grid(True)
    plt.legend()

    # --- Figure 2: wrapped deltas + accept flags ---
    plt.figure()
    plt.plot(t, dL, label="dL_wrapped (rad)")
    plt.plot(t, dR, label="dR_wrapped (rad)")
    rejL_t = [t[i] for i in range(N) if accL[i] == 0]
    rejL_y = [dL[i] for i in range(N) if accL[i] == 0]
    rejR_t = [t[i] for i in range(N) if accR[i] == 0]
    rejR_y = [dR[i] for i in range(N) if accR[i] == 0]
    if rejL_t:
        plt.scatter(rejL_t, rejL_y, s=8, label="rejected L", marker="x")
    if rejR_t:
        plt.scatter(rejR_t, rejR_y, s=8, label="rejected R", marker="x")
    plt.xlabel("time (s)")
    plt.ylabel("delta per tick (rad)")
    plt.title("Wrapped increments and plausibility rejection")
    plt.grid(True)
    plt.legend()

    # --- Figure 3: unwrapped angles ---
    plt.figure()
    plt.plot(t, unwrapL, label="unwrap_L (rad)")
    plt.plot(t, unwrapR, label="unwrap_R (rad)")
    plt.xlabel("time (s)")
    plt.ylabel("unwrapped angle (rad)")
    plt.title("Unwrapped wheel angle (continuous)")
    plt.grid(True)
    plt.legend()

    # --- Figure 4: forward displacement ---
    plt.figure()
    plt.plot(t, x_wheel, label="x_wheel (m)")
    plt.xlabel("time (s)")
    plt.ylabel("position (m)")
    plt.title("Forward wheel displacement")
    plt.grid(True)
    plt.legend()

    # --- NEW Figure 5: wheel velocities from ESC (raw) ---
    plt.figure()
    plt.plot(t, velL_raw, label="vel_L_raw (rad/s) [ESC]")
    plt.plot(t, velR_raw, label="vel_R_raw (rad/s) [ESC]")
    plt.xlabel("time (s)")
    plt.ylabel("wheel velocity (rad/s)")
    plt.title("Raw ESC wheel velocities")
    plt.grid(True)
    plt.legend()

    # --- NEW Figure 6: compare ESC vel vs delta-derived vel (masked) ---
    # delta-derived vel from wrapped increments: w ≈ d/dt
    # Mask out: rejects, and wrap-crossing spikes near ±pi (those are real but not useful for this comparison)
    import math
    wrap_thresh = 2.5  # rad; anything above this is likely a wrap event
    wL_from_d = [0.0]*N
    wR_from_d = [0.0]*N
    wL_from_d_masked = [None]*N
    wR_from_d_masked = [None]*N
    velL_masked = [None]*N
    velR_masked = [None]*N

    for i in range(N):
        wL_from_d[i] = dL[i] / dt
        wR_from_d[i] = dR[i] / dt

        okL = (accL[i] == 1) and (abs(dL[i]) < wrap_thresh)
        okR = (accR[i] == 1) and (abs(dR[i]) < wrap_thresh)

        if okL:
            wL_from_d_masked[i] = wL_from_d[i]
            velL_masked[i] = velL_raw[i]
        if okR:
            wR_from_d_masked[i] = wR_from_d[i]
            velR_masked[i] = velR_raw[i]

    plt.figure()
    plt.plot(t, velL_masked, label="vel_L_raw (rad/s) [ESC]")
    plt.plot(t, wL_from_d_masked, label="dL/dt (rad/s) [masked]")
    plt.plot(t, velR_masked, label="vel_R_raw (rad/s) [ESC]")
    plt.plot(t, wR_from_d_masked, label="dR/dt (rad/s) [masked]")
    plt.xlabel("time (s)")
    plt.ylabel("wheel velocity (rad/s)")
    plt.title("ESC velocity vs delta-derived velocity (masked rejects + wraps)")
    plt.grid(True)
    plt.legend()

    # --- FIXED Figure 7: filtered vs filtered velocity consistency in *forward* motion ---
    # Compute x_dot_from_pos_raw from x_wheel, then LPF it with same cutoff.
    xdp_raw = [0.0] * N
    for i in range(1, N):
        xdp_raw[i] = (x_wheel[i] - x_wheel[i-1]) / dt

    xdp_filt = lpf_1pole(xdp_raw, fs, LPF_FC_HZ)

    plt.figure()
    plt.plot(t, x_dot, label="x_dot (m/s) [ESC vel + LPF (firmware)]")
    plt.plot(t, xdp_filt, label=f"x_dot_from_pos (m/s) [diff(x) + LPF {LPF_FC_HZ:.0f}Hz]")
    plt.xlabel("time (s)")
    plt.ylabel("velocity (m/s)")
    plt.title("Velocity consistency: filtered ESC vs filtered derivative of x_wheel")
    plt.grid(True)
    plt.legend()

    # Optional: keep your original (unfiltered) comparison but label it as diagnostic
    plt.figure()
    plt.plot(t, x_dot, label="x_dot (m/s) [ESC vel + LPF]")
    plt.plot(t, x_dot_from_pos_logged, label="x_dot_from_pos (m/s) [logged]")
    plt.xlabel("time (s)")
    plt.ylabel("velocity (m/s)")
    plt.title("Velocity (diagnostic): filtered ESC vs logged pos-derivative channel")
    plt.grid(True)
    plt.legend()

    # Quick printed stats
    rejL_count = sum(1 for v in accL if v == 0)
    rejR_count = sum(1 for v in accR if v == 0)
    print(f"Reject counts: L={rejL_count}/{N}  R={rejR_count}/{N}")

def main():
    with socket.create_connection((HOST, PORT), timeout=10) as s:
        s.settimeout(None)
        print(f"Connected to {HOST}:{PORT}, waiting for TWR1...")

        hdr = read_header(s)
        print("Header:", hdr)

        payload = recv_exact(s, hdr.payload_bytes)
        trailer_crc = struct.unpack("<I", recv_exact(s, 4))[0]
        calc_crc = zlib.crc32(payload) & 0xFFFFFFFF

        print(f"CRC header=0x{hdr.payload_crc32:08X} trailer=0x{trailer_crc:08X} calc=0x{calc_crc:08X}")
        if calc_crc != hdr.payload_crc32 or trailer_crc != hdr.payload_crc32:
            raise ValueError("CRC mismatch")
        if hdr.sample_bytes * hdr.sample_count != hdr.payload_bytes:
            raise ValueError("Header length fields inconsistent")

        data = unpack_samples(payload, hdr.sample_bytes)
        print(f"Unpacked {hdr.sample_count} samples.")
        plot_unwrap(hdr, data)

        plt.show()

if __name__ == "__main__":
    main()
    
