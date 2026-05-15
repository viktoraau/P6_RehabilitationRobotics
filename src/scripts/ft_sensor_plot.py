#!/usr/bin/env python3
"""
6-Axis Force/Torque Sensor – Live Plot with Tare
=================================================
Reads the sensor continuous-output protocol (460800 baud, 8N1) and
displays all 6 channels in a scrolling live plot.

Packet layout (38 bytes, all multi-byte fields little-endian):
  [0:2]   Header        0xAA 0x55
  [2]     Address       0xFF
  [3:5]   Frame length  uint16 (= 38)
  [5]     Command       0x10
  [6:30]  CH1-6 data    6 × int32 (little-endian)
  [30:36] CH1-6 status  6 × uint8
  [36:38] CRC-16        Modbus CRC over bytes [0:36]

Usage
-----
  python ft_sensor_plot.py [--port /dev/ttyUSB0] [--baud 460800]
"""

import argparse
import collections
import struct
import threading
import time

import numpy as np
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

# ── defaults ──────────────────────────────────────────────────────────────────
DEFAULT_PORT   = "/dev/ttyUSB0"
DEFAULT_BAUD   = 460800
PLOT_WINDOW    = 500          # samples kept on screen
UPDATE_INTERVAL_MS = 10       # animation refresh rate (≤ sensor rate)

LABELS  = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
COLORS  = ["tab:blue", "tab:orange", "tab:green",
           "tab:red",  "tab:purple",  "tab:brown"]

# ── packet constants ───────────────────────────────────────────────────────────
HEADER      = bytes([0xAA, 0x55])
ADDR_BYTE   = 0xFF
CMD_BYTE    = 0x10
PACKET_SIZE = 38   # 2+1+2+1+24+6+2


# ── CRC-16 Modbus ──────────────────────────────────────────────────────────────
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


# ── packet parser ──────────────────────────────────────────────────────────────
def _parse(raw: bytes):
    """
    Validate and decode a 38-byte packet.
    Returns (channels: list[int], statuses: list[int]) or None.
    """
    if len(raw) != PACKET_SIZE:
        return None
    if raw[0] != 0xAA or raw[1] != 0x55:
        return None
    if raw[2] != ADDR_BYTE:
        return None
    if raw[5] != CMD_BYTE:
        return None
    calc = _crc16(raw[:-2])
    recv = struct.unpack_from("<H", raw, 36)[0]
    if calc != recv:
        return None
    channels = list(struct.unpack_from("<6i", raw, 6))
    statuses = list(struct.unpack_from("6B",  raw, 30))
    return channels, statuses


# ── serial reader thread ───────────────────────────────────────────────────────
class SensorReader:
    def __init__(self, port: str, baud: int):
        self._port = port
        self._baud = baud
        self._ser  = None
        self._lock = threading.Lock()
        self._raw  = [0] * 6   # latest raw int32 values
        self._tare = [0] * 6   # tare offset
        self._running = False

    # ── public API ──────────────────────────────────────────────────────
    def start(self):
        self._ser = serial.Serial(
            self._port, self._baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.002,          # 2 ms – never blocks longer than this
        )
        self._ser.reset_input_buffer()
        # packet queue: deque of 6-element lists (tare-compensated floats)
        self._queue   = collections.deque(maxlen=2000)
        self._pkt_count = 0
        self._t0        = time.monotonic()
        self._running = True
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def stop(self):
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()

    def tare(self):
        """Set current reading as zero baseline."""
        with self._lock:
            self._tare = list(self._raw)

    def get_values(self) -> list:
        """Return tare-compensated float values for all 6 channels."""
        with self._lock:
            return [float(self._raw[i] - self._tare[i]) for i in range(6)]

    def drain_queue(self):
        """Pop all pending packets – called from the plot thread."""
        with self._lock:
            items = list(self._queue)
            self._queue.clear()
        return items

    @property
    def rate_hz(self) -> float:
        elapsed = time.monotonic() - self._t0
        return self._pkt_count / elapsed if elapsed > 0 else 0.0

    # ── internal ─────────────────────────────────────────────────────────
    def _loop(self):
        buf = bytearray()
        while self._running:
            try:
                # drain whatever is available right now; never block longer
                # than the port timeout (2 ms)
                waiting = self._ser.in_waiting
                chunk   = self._ser.read(max(waiting, 1))
                if not chunk:
                    continue
                buf.extend(chunk)
                # parse every complete packet in the buffer
                while len(buf) >= PACKET_SIZE:
                    idx = buf.find(HEADER)
                    if idx == -1:
                        del buf[:-1]
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < PACKET_SIZE:
                        break
                    result = _parse(bytes(buf[:PACKET_SIZE]))
                    if result is not None:
                        channels, _ = result
                        with self._lock:
                            self._raw = channels
                            vals = [float(channels[i] - self._tare[i])
                                    for i in range(6)]
                            self._queue.append(vals)
                        self._pkt_count += 1
                        del buf[:PACKET_SIZE]
                    else:
                        del buf[:1]
            except serial.SerialException as exc:
                print(f"[sensor] serial error: {exc}")
                self._running = False
                break


# ── plotting ───────────────────────────────────────────────────────────────────
def run_plot(reader: SensorReader):
    # ring buffers – one per channel
    data = [collections.deque([0.0] * PLOT_WINDOW, maxlen=PLOT_WINDOW)
            for _ in range(6)]
    x    = np.arange(PLOT_WINDOW)

    fig, axes = plt.subplots(6, 1, figsize=(11, 9), sharex=True)
    fig.suptitle("6-Axis Force / Torque Sensor – Live", fontsize=11, fontweight="bold")
    plt.subplots_adjust(left=0.10, right=0.97, top=0.94, bottom=0.10, hspace=0.45)

    lines = []
    for i, (ax, label, color) in enumerate(zip(axes, LABELS, COLORS)):
        (ln,) = ax.plot(x, list(data[i]), color=color, linewidth=0.9)
        ax.set_ylabel(label, fontsize=8, rotation=0, labelpad=28)
        ax.tick_params(labelsize=7)
        ax.grid(True, linewidth=0.3, alpha=0.6)
        lines.append(ln)
    axes[-1].set_xlabel("Samples", fontsize=8)

    # rate text (top-right of first axis)
    rate_text = axes[0].text(
        0.99, 0.88, "0 Hz", transform=axes[0].transAxes,
        ha="right", va="top", fontsize=7, color="gray"
    )

    # ── tare button ──────────────────────────────────────────────────────
    ax_btn = plt.axes([0.40, 0.02, 0.20, 0.045])
    btn    = Button(ax_btn, "Tare / Zero",
                    color="lightyellow", hovercolor="khaki")

    def on_tare(_event):
        reader.tare()
        print("[tare] offset applied")
    btn.on_clicked(on_tare)

    # autoscale limits per channel (updated lazily)
    y_min = [0.0] * 6
    y_max = [1.0] * 6
    MARGIN = 0.1

    def update(_frame):
        packets = reader.drain_queue()
        if not packets:
            return

        for vals in packets:
            for i, v in enumerate(vals):
                data[i].append(v)

        for i in range(6):
            arr = np.asarray(data[i])
            lines[i].set_ydata(arr)
            lo, hi = float(arr.min()), float(arr.max())
            span = max(hi - lo, 1e-3)
            new_lo = lo - MARGIN * span
            new_hi = hi + MARGIN * span
            if abs(new_lo - y_min[i]) > span * 0.05 or \
               abs(new_hi - y_max[i]) > span * 0.05:
                y_min[i], y_max[i] = new_lo, new_hi
                axes[i].set_ylim(new_lo, new_hi)

        rate_text.set_text(f"{reader.rate_hz:.1f} Hz")

    ani = animation.FuncAnimation(   # noqa: F841
        fig, update,
        interval=UPDATE_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    finally:
        reader.stop()


# ── entry point ────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="6-Axis FT Sensor live plot (continuous-output protocol)"
    )
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    args = parser.parse_args()

    reader = SensorReader(args.port, args.baud)
    try:
        reader.start()
        print(f"[sensor] connected: {args.port} @ {args.baud} baud")
    except serial.SerialException as exc:
        print(f"[sensor] cannot open port '{args.port}': {exc}")
        available = [p.device for p in serial.tools.list_ports.comports()]
        if available:
            print(f"[sensor] available ports: {available}")
            print(f"[sensor] retry with:  --port {available[0]}")
        else:
            print("[sensor] no serial ports detected")
        return

    run_plot(reader)


if __name__ == "__main__":
    main()
