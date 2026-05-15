#!/usr/bin/env python3
"""
ft_configure_sensor.py
======================
Step 1: Detects whether the sensor is in Modbus mode or continuous-output mode.
Step 2: If in Modbus mode, applies all recommended settings and tare.
Step 3: Waits for the sensor to switch to continuous output, then streams
        live data so you can confirm 200 Hz + heavy filtering.

HOW TO USE
----------
1. POWER-CYCLE the sensor (unplug USB, wait 5 s, replug).
2. Run this script IMMEDIATELY – you have ~30 s:

       python3 P6_RehabilitationRobotics/src/scripts/ft_configure_sensor.py

3. If it says "already in continuous mode", power-cycle again and be faster.

SETTINGS APPLIED
----------------
  0x0002 = 2   Tare  (zero the current load)
  0x0011 = 0   Disable zero tracking  → eliminates slow drift
  0x000F = 0   Disable power-on auto-zero
  0x0012 = 0   Disable stability display  → maximum resolution
  0x0060 = 5   ADC sampling rate → 200 Hz  (was 800 Hz from factory)
  0x0013 = 5   Filter level → 5 (heaviest, maximum smoothing)
"""

import argparse
import struct
import sys
import time

import serial

# ── constants ──────────────────────────────────────────────────────────────────
SLAVES      = [1, 2, 3, 4, 5, 6]
CHAN_NAMES  = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
FORCE_SCALE   = 0.001   # raw int32 / 1000 → N   (100 N rated range)
TORQUE_SCALE  = 0.001   # raw int32 / 1000 → Nm  (10 Nm rated range)
PACKET_SIZE   = 38

# Registers to write (order matters – tare first)
WRITE_CONFIG = [
    (0x0002, 2, "Tare (zero current load)"),
    (0x0011, 0, "Disable zero tracking  → no drift"),
    (0x000F, 0, "Disable power-on auto-zero"),
    (0x0012, 0, "Disable stability display → max resolution"),
    (0x0060, 5, "ADC rate = 200 Hz"),
    (0x0013, 5, "Filter = 5 (heaviest)"),
]

# Registers to read back and verify (skip 0x0002 – tare, write-only action)
VERIFY_CONFIG = [
    (0x0011, 0, "Zero tracking"),
    (0x000F, 0, "Power-on zero range"),
    (0x0012, 0, "Stability display"),
    (0x0060, 5, "Sampling rate"),
    (0x0013, 5, "Filter level"),
]

# ── CRC-16 Modbus ──────────────────────────────────────────────────────────────
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def _build_write(slave, reg, val) -> bytes:
    cmd = bytes([slave, 0x06, reg >> 8, reg & 0xFF, val >> 8, val & 0xFF])
    c = _crc16(cmd)
    return cmd + bytes([c & 0xFF, c >> 8])

def _build_read(slave, reg, count=1) -> bytes:
    cmd = bytes([slave, 0x03, reg >> 8, reg & 0xFF, 0x00, count])
    c = _crc16(cmd)
    return cmd + bytes([c & 0xFF, c >> 8])

# ── Modbus helpers ─────────────────────────────────────────────────────────────
def _write_reg(ser, slave, reg, val) -> bool:
    cmd = _build_write(slave, reg, val)
    ser.reset_input_buffer()
    ser.write(cmd)
    time.sleep(0.06)
    reply = ser.read(8)
    return len(reply) == 8 and reply[:6] == cmd[:6]

def _read_reg(ser, slave, reg):
    """Returns int value or None."""
    cmd = _build_read(slave, reg)
    ser.reset_input_buffer()
    ser.write(cmd)
    time.sleep(0.06)
    reply = ser.read(7)
    if len(reply) == 7 and reply[0] == slave and reply[1] == 0x03 and reply[2] == 2:
        return struct.unpack(">H", reply[3:5])[0]
    return None

def _read_reg32(ser, slave, reg):
    """Read two consecutive 16-bit registers as a signed 32-bit int (3412 byte order)."""
    cmd = _build_read(slave, reg, 2)
    ser.reset_input_buffer()
    ser.write(cmd)
    time.sleep(0.06)
    reply = ser.read(9)   # 1+1+1+4+2
    if len(reply) == 9 and reply[0] == slave and reply[1] == 0x03 and reply[2] == 4:
        lo, hi = struct.unpack(">HH", reply[3:7])
        raw = (hi << 16) | lo
        return struct.unpack(">i", struct.pack(">I", raw))[0]
    return None

# ── detect mode ────────────────────────────────────────────────────────────────
def detect_mode(port):
    """
    Returns "modbus", "continuous", or "unknown".
    Tries Modbus at 115200 first (faster probe), then looks for 0xAA55 at 460800.
    """
    # Probe Modbus
    try:
        with serial.Serial(port, 115200, timeout=0.4) as s:
            s.reset_input_buffer()
            s.write(_build_read(1, 0x0003))   # read decimal places
            time.sleep(0.15)
            r = s.read(7)
            if len(r) == 7 and r[0] == 1 and r[1] == 0x03 and r[2] == 2:
                return "modbus"
    except serial.SerialException:
        pass

    # Probe continuous output
    try:
        with serial.Serial(port, 460800, timeout=0.3) as s:
            s.reset_input_buffer()
            time.sleep(0.1)
            data = s.read(300)
            if b'\xaa\x55' in data:
                return "continuous"
    except serial.SerialException:
        pass

    return "unknown"

# ── read current register state ────────────────────────────────────────────────
def read_current_state(port):
    print("\n  Current register values (slave 1 only):")
    checks = [
        (0x0003, 1, "Decimal places"),
        (0x0013, 1, "Filter level"),
        (0x0060, 1, "Sampling rate (code)"),
        (0x0011, 1, "Zero tracking range"),
        (0x0012, 1, "Stability display range"),
        (0x000F, 1, "Power-on zero range"),
    ]
    sampling_rate_map = {0:"6Hz",1:"12Hz",2:"25Hz",3:"50Hz",4:"100Hz",
                         5:"200Hz",6:"40Hz",7:"800Hz",8:"1600Hz"}
    try:
        with serial.Serial(port, 115200, timeout=0.4) as ser:
            for reg, count, label in checks:
                val = _read_reg(ser, 1, reg)
                if val is not None:
                    extra = ""
                    if reg == 0x0060:
                        extra = f"  ({sampling_rate_map.get(val, '?')})"
                    print(f"    0x{reg:04X}  {label:30s} = {val}{extra}")
                else:
                    print(f"    0x{reg:04X}  {label:30s} = READ FAILED")
    except serial.SerialException as e:
        print(f"    Serial error: {e}")

# ── configure ──────────────────────────────────────────────────────────────────
def configure(port):
    print(f"\n  Writing configuration via Modbus @ 115200 baud …")
    all_ok = True

    try:
        with serial.Serial(port, 115200,
                           bytesize=serial.EIGHTBITS,
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE,
                           timeout=0.4) as ser:

            for i, slave in enumerate(SLAVES):
                name = CHAN_NAMES[i]
                print(f"\n    Slave {slave} ({name})")
                for reg, val, desc in WRITE_CONFIG:
                    ok = _write_reg(ser, slave, reg, val)
                    mark = "✓" if ok else "✗"
                    print(f"      {mark} 0x{reg:04X} ← {val:2d}  {desc}")
                    if not ok:
                        all_ok = False

            print("\n    ── Verify (slave 1 only) ──────────────────────────────")
            for reg, expected, label in VERIFY_CONFIG:
                got = _read_reg(ser, 1, reg)
                if got is None:
                    print(f"      ✗ 0x{reg:04X}  {label}: read failed")
                    all_ok = False
                elif got != expected:
                    print(f"      ✗ 0x{reg:04X}  {label}: expected {expected}, got {got}")
                    all_ok = False
                else:
                    print(f"      ✓ 0x{reg:04X}  {label} = {got}")

    except serial.SerialException as e:
        print(f"  Serial error during configuration: {e}")
        return False

    return all_ok

# ── wait for continuous mode ───────────────────────────────────────────────────
def wait_for_continuous(port, timeout_s=35):
    print(f"\n  Waiting for sensor to switch to continuous output mode …")
    print(f"  (happens automatically ~30 s after power-on)")
    deadline = time.time() + timeout_s
    dots = 0
    while time.time() < deadline:
        try:
            with serial.Serial(port, 460800, timeout=0.15) as s:
                s.reset_input_buffer()
                time.sleep(0.1)
                data = s.read(200)
                if b'\xaa\x55' in data:
                    print(f"\n  ✓ Continuous output detected!")
                    return True
        except serial.SerialException:
            pass
        remaining = int(deadline - time.time())
        print(f"  {remaining:2d}s …", end='\r', flush=True)
        time.sleep(1)
    print()
    return False

# ── live monitor ───────────────────────────────────────────────────────────────
def monitor(port, duration_s=15):
    """Stream continuous output and report Hz + force/torque values."""
    print(f"\n  Monitoring continuous output for {duration_s} s …")
    print(f"  (Expected ~200 Hz after configuration)")
    print()

    sw_tare = None
    total_pkts = 0
    hz_count = 0
    t_start = time.time()
    t_hz    = t_start

    try:
        with serial.Serial(port, 460800,
                           bytesize=serial.EIGHTBITS,
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE,
                           timeout=0.002) as ser:
            buf = bytearray()
            while time.time() - t_start < duration_s:
                w = ser.in_waiting
                chunk = ser.read(max(w, 1))
                if not chunk:
                    continue
                buf.extend(chunk)

                while len(buf) >= PACKET_SIZE:
                    idx = buf.find(b'\xaa\x55')
                    if idx < 0:
                        del buf[:-1]
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < PACKET_SIZE:
                        break

                    pkt = bytes(buf[:PACKET_SIZE])
                    if pkt[2] != 0xFF or pkt[5] != 0x10:
                        del buf[:1]
                        continue
                    crc_calc = _crc16(pkt[:-2])
                    crc_pkt  = struct.unpack_from("<H", pkt, 36)[0]
                    if crc_calc != crc_pkt:
                        del buf[:1]
                        continue

                    ch = list(struct.unpack_from("<6i", pkt, 6))
                    del buf[:PACKET_SIZE]
                    total_pkts += 1
                    hz_count   += 1

                    if sw_tare is None:
                        sw_tare = ch[:]
                        print(f"  Tare snapshot (raw counts): {sw_tare}")
                        print(f"  {'─'*70}")

                    now = time.time()
                    if now - t_hz >= 1.0:
                        hz = hz_count / (now - t_hz)
                        hz_count = 0
                        t_hz = now
                        f = [(ch[i] - sw_tare[i]) * FORCE_SCALE  for i in range(3)]
                        t = [(ch[i] - sw_tare[i]) * TORQUE_SCALE for i in range(3, 6)]
                        print(f"  {hz:5.0f} Hz  │  "
                              f"Fx={f[0]:7.3f}N  Fy={f[1]:7.3f}N  Fz={f[2]:7.3f}N  │  "
                              f"Tx={t[0]:7.4f}Nm  Ty={t[1]:7.4f}Nm  Tz={t[2]:7.4f}Nm")

    except serial.SerialException as e:
        print(f"  Serial error: {e}")
        return

    elapsed = time.time() - t_start
    avg_hz  = total_pkts / elapsed if elapsed > 0 else 0
    print(f"\n  Total: {total_pkts} packets / {elapsed:.1f}s = {avg_hz:.0f} Hz average")

# ── main ───────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="FT sensor configurator + monitor")
    ap.add_argument("--port",         default="/dev/ttyUSB0")
    ap.add_argument("--monitor-only", action="store_true",
                    help="Skip Modbus config, just monitor continuous output")
    ap.add_argument("--duration",     type=int, default=15,
                    help="Monitoring duration in seconds (default: 15)")
    args = ap.parse_args()

    print("=" * 72)
    print("  FT Sensor Configuration + Monitor")
    print("=" * 72)

    if args.monitor_only:
        monitor(args.port, args.duration)
        return

    # ── Step 1: detect mode ───────────────────────────────────────────────────
    print("\nStep 1/4 – Detecting sensor mode …")
    mode = detect_mode(args.port)
    print(f"  → {mode.upper()}")

    if mode == "continuous":
        print("""
  ✗  The sensor is ALREADY in continuous-output mode (460800 baud).
     The 30-second Modbus configuration window has already passed.

  To fix this:
    1. Unplug the USB cable from the computer (or the sensor power)
    2. Wait at least 5 seconds
    3. Plug it back in
    4. Run this script IMMEDIATELY (within ~30 s of power-on)

  Quick re-run command after power-cycle:
    python3 P6_RehabilitationRobotics/src/scripts/ft_configure_sensor.py
""")
        print("  Switching to monitor-only mode to show current data …")
        monitor(args.port, args.duration)
        return

    if mode == "unknown":
        print("  ✗  Cannot communicate with sensor. Check USB cable and port.")
        sys.exit(1)

    # ── Step 2: read current state ────────────────────────────────────────────
    print("\nStep 2/4 – Reading current register values …")
    read_current_state(args.port)

    # ── Step 3: configure ─────────────────────────────────────────────────────
    print("\nStep 3/4 – Applying configuration …")
    ok = configure(args.port)

    if ok:
        print("\n  ✓  All configuration applied and verified.")
    else:
        print("\n  ⚠  Some writes failed – sensor may be partially configured.")
        print("     Try power-cycling and running again.")

    # ── Step 4: monitor ───────────────────────────────────────────────────────
    print("\nStep 4/4 – Waiting for continuous output to start …")
    if wait_for_continuous(args.port):
        monitor(args.port, args.duration)
    else:
        print("  ✗  Continuous output never started. Unexpected.")


if __name__ == "__main__":
    main()
