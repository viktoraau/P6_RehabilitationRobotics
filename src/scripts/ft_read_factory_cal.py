#!/usr/bin/env python3
"""
ft_read_factory_cal.py
======================
Reads all factory calibration registers from the FT sensor via Modbus RTU
and prints per-channel conversion factors.

Must be run within ~30 s of power-on (before the sensor switches to
continuous-output mode at 460800 baud).

Usage
-----
  python3 ft_read_factory_cal.py [--port /dev/ttyUSB0] [--slaves 1 2 3 4 5 6]

Registers read per slave
------------------------
  0x0000/0x0001  Display weight (current reading, sanity check)
  0x0003         Decimal places
  0x000A/0x000B  Full scale (high range)
  0x0013         Filter level (current)
  0x0016/0x0017  Calibrator weight value  ← factory cal reference weight
  0x0018/0x0019  Calibrated internal code ← factory cal ADC count at that weight
  0x001A/0x001B  Total sensor range
  0x001C/0x001D  Average sensor sensitivity (mV/V × 100000)
  0x001E         Slave address (verify)
  0x001F         Software version
  0x0022/0x0023  Device serial number
  0x0060         AD sampling rate (current)

Computed output
---------------
  scale  = calibrator_weight / calibrated_internal_code
           → multiply raw continuous-output int32 counts by this to get
             the sensor's factory-calibrated units (typically N or Nm,
             depending on what the factory used as reference weight)
"""

import argparse
import struct
import time

import serial


# ── CRC-16 Modbus ──────────────────────────────────────────────────────────────
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _build_read(slave: int, reg: int, count: int) -> bytes:
    payload = struct.pack(">BBHH", slave, 0x03, reg, count)
    return payload + struct.pack("<H", _crc16(payload))


def _read_registers(ser, slave: int, reg: int, count: int, retries: int = 3):
    """
    Send a Modbus read request and return list of register values (uint16),
    or None on failure.
    """
    cmd = _build_read(slave, reg, count)
    expected = 5 + count * 2  # 1 addr + 1 fn + 1 byte_count + 2n data + 2 CRC

    for attempt in range(retries):
        ser.reset_input_buffer()
        ser.write(cmd)
        time.sleep(0.05)
        reply = ser.read(expected)

        if len(reply) < expected:
            continue
        if reply[0] != slave or reply[1] != 0x03:
            continue
        if _crc16(reply[:-2]) != struct.unpack_from("<H", reply, -2)[0]:
            continue

        regs = []
        for i in range(count):
            regs.append(struct.unpack_from(">H", reply, 3 + i * 2)[0])
        return regs

    return None


def _regs_to_int32(regs, idx=0) -> int:
    """Combine two consecutive uint16 registers into a signed Int32 (big-endian)."""
    raw = (regs[idx] << 16) | regs[idx + 1]
    # convert to signed
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw


def _regs_to_int32_swapped(regs, idx=0) -> int:
    """
    The manual uses '3412' byte order for 32-bit values:
      register[0] = bytes 3,4 (high word)  → already big-endian per register
      register[1] = bytes 1,2 (low word)
    So combined value = (reg[1] << 16) | reg[0]  i.e. low word first.
    """
    raw = (regs[idx + 1] << 16) | regs[idx]
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw


SAMPLING_RATES = {0: 6, 1: 12, 2: 25, 3: 50, 4: 100, 5: 200, 6: 40, 7: 800, 8: 1600}


def read_slave(ser, slave: int) -> dict | None:
    """Read all calibration registers for one slave. Returns dict or None."""
    results = {}

    def rd(reg, count):
        return _read_registers(ser, slave, reg, count)

    # ── current reading (sanity check) ────────────────────────────────────────
    r = rd(0x0000, 2)
    if r is None:
        return None
    results["display_weight_raw"] = _regs_to_int32(r)

    r = rd(0x0003, 1)
    results["decimal_places"] = r[0] if r else None

    # ── full scale ────────────────────────────────────────────────────────────
    r = rd(0x000A, 2)
    results["full_scale"] = _regs_to_int32(r) if r else None

    # ── filter and sampling ───────────────────────────────────────────────────
    r = rd(0x0013, 1)
    results["filter_level"] = r[0] if r else None

    r = rd(0x0060, 1)
    results["sampling_rate_code"] = r[0] if r else None
    results["sampling_rate_hz"]   = SAMPLING_RATES.get(r[0]) if r else None

    # ── factory calibration registers ─────────────────────────────────────────
    # Read registers 0x0016–0x001D in one block (8 registers = 4 × Int32)
    r = rd(0x0016, 8)
    if r:
        # 0x0016/0x0017: calibrator weight value
        # The manual says 32-bit values use '3412' format: low word in first reg
        results["cal_weight"]       = _regs_to_int32_swapped(r, 0)
        # 0x0018/0x0019: calibrated internal code (ADC count at cal point)
        results["cal_internal_code"] = _regs_to_int32_swapped(r, 2)
        # 0x001A/0x001B: total sensor range
        results["sensor_range"]     = _regs_to_int32_swapped(r, 4)
        # 0x001C/0x001D: average sensitivity (200000 = 2.00000 mV/V)
        results["sensitivity_raw"]  = _regs_to_int32_swapped(r, 6)
        results["sensitivity_mVV"]  = results["sensitivity_raw"] / 1e5
    else:
        results["cal_weight"] = results["cal_internal_code"] = None
        results["sensor_range"] = results["sensitivity_raw"] = None

    # ── identity ──────────────────────────────────────────────────────────────
    r = rd(0x001E, 1)
    results["slave_address"] = r[0] if r else None

    r = rd(0x001F, 1)
    results["fw_version"] = r[0] if r else None

    r = rd(0x0022, 2)
    results["serial_number"] = _regs_to_int32(r) if r else None

    # ── derived scale factor ──────────────────────────────────────────────────
    cw  = results.get("cal_weight")
    cic = results.get("cal_internal_code")
    if cw and cic and cic != 0:
        results["scale_factor"] = cw / cic
    else:
        results["scale_factor"] = None

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Read factory calibration from 6-axis FT sensor (Modbus window)"
    )
    parser.add_argument("--port",   default="/dev/ttyUSB0")
    parser.add_argument("--slaves", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    args = parser.parse_args()

    print(f"Opening {args.port} @ 115200 baud (Modbus RTU)")
    print("NOTE: must be within ~30 s of sensor power-on\n")

    try:
        ser = serial.Serial(
            args.port, 115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,
        )
    except serial.SerialException as exc:
        print(f"ERROR: cannot open port: {exc}")
        return

    labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    all_scales = {}

    for i, slave in enumerate(args.slave if hasattr(args, "slave") else args.slaves):
        label = labels[i] if i < len(labels) else f"CH{slave}"
        print(f"{'─'*60}")
        print(f"Slave {slave:2d}  ({label})")
        print(f"{'─'*60}")

        data = read_slave(ser, slave)
        if data is None:
            print("  ✗  No response – sensor may be in continuous mode already")
            print("     Power-cycle the sensor and run this within 30 s")
            continue

        dp   = data.get("decimal_places", 0) or 0
        dw   = data["display_weight_raw"]
        if dw is not None:
            print(f"  Current reading    : {dw / 10**dp:.{dp}f} (raw={dw})")

        print(f"  Decimal places     : {data.get('decimal_places')}")
        print(f"  Full scale         : {data.get('full_scale')}")
        print(f"  Filter level       : {data.get('filter_level')}  (0=off, 5=max)")
        print(f"  Sampling rate      : {data.get('sampling_rate_hz')} Hz  (code={data.get('sampling_rate_code')})")
        print(f"  Firmware version   : {data.get('fw_version')}")
        print(f"  Serial number      : {data.get('serial_number')}")
        print(f"  Slave address      : {data.get('slave_address')}")
        print()
        print(f"  ── Factory calibration ──")
        print(f"  Calibrator weight  : {data.get('cal_weight')}  (reference load used at factory)")
        print(f"  Cal internal code  : {data.get('cal_internal_code')}  (ADC count at that load)")
        print(f"  Sensor range       : {data.get('sensor_range')}  (rated capacity)")
        print(f"  Sensitivity        : {data.get('sensitivity_mVV'):.5f} mV/V")

        sf = data.get("scale_factor")
        if sf is not None:
            print(f"\n  ★ Scale factor     : {sf:.8f}  (= cal_weight / cal_internal_code)")
            print(f"    raw_count × {sf:.6f}  →  factory units")
            all_scales[label] = sf
        else:
            print("\n  ✗  Scale factor could not be computed (missing cal data)")
        print()

    ser.close()

    # ── summary ────────────────────────────────────────────────────────────────
    if all_scales:
        print("=" * 60)
        print("SUMMARY – scale factors (raw int32 counts → factory units)")
        print("=" * 60)
        for ch, sf in all_scales.items():
            print(f"  {ch}: {sf:.8f}")
        print()
        print("To use in ft_sensor_node, set:")
        scales = list(all_scales.values())
        if scales:
            force_scales  = scales[:3]
            torque_scales = scales[3:]
            print(f"  force_scale  (average of Fx/Fy/Fz): {sum(force_scales)/len(force_scales):.8f}")
            if torque_scales:
                print(f"  torque_scale (average of Tx/Ty/Tz): {sum(torque_scales)/len(torque_scales):.8f}")
        print()
        print("NOTE: these are per-channel 1D gains only.")
        print("Cross-axis coupling is NOT calibrated by this sensor's firmware.")
        print("For full 6×6 decoupling, a custom calibration matrix is needed.")


if __name__ == "__main__":
    main()
