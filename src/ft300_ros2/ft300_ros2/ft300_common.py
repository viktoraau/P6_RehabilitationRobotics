import struct
import time
import serial
import numpy as np


SLAVE_ID = 0x09


def modbus_crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_start_stream_command() -> bytes:
    payload = bytes([SLAVE_ID, 0x10, 0x01, 0x9A, 0x00, 0x01, 0x02, 0x02, 0x00])
    crc = modbus_crc(payload)
    return payload + struct.pack("<H", crc)


def parse_stream_frame(frame: bytes):
    if len(frame) != 16:
        return None

    if frame[0] != 0x20 or frame[1] != 0x4E:
        return None

    calc_crc = modbus_crc(frame[:14])
    recv_crc = struct.unpack("<H", frame[14:16])[0]
    if calc_crc != recv_crc:
        return None

    raw = struct.unpack("<hhhhhh", frame[2:14])

    fx = raw[0] / 100.0
    fy = raw[1] / 100.0
    fz = raw[2] / 100.0
    mx = raw[3] / 1000.0
    my = raw[4] / 1000.0
    mz = raw[5] / 1000.0

    return np.array([fx, fy, fz, mx, my, mz], dtype=float)


class FT300:
    def __init__(self, port: str, baudrate: int, timeout: float):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        self.buffer = bytearray()

    def start_stream(self):
        self.ser.write(build_start_stream_command())
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.buffer.clear()

    def stop_stream(self):
        try:
            self.ser.write(b"\xFF" * 50)
            time.sleep(0.05)
        except Exception:
            pass

    def close(self):
        try:
            self.stop_stream()
        finally:
            if self.ser.is_open:
                self.ser.close()

    def read_sample(self):
        """
        Return the first valid sample found in the serial stream.
        This is the stable parser to use everywhere.
        """
        data = self.ser.read(64)
        if data:
            self.buffer.extend(data)

        while len(self.buffer) >= 16:
            idx = self.buffer.find(b"\x20\x4E")
            if idx < 0:
                self.buffer.clear()
                return None

            if idx > 0:
                del self.buffer[:idx]

            if len(self.buffer) < 16:
                return None

            frame = bytes(self.buffer[:16])
            sample = parse_stream_frame(frame)
            if sample is not None:
                del self.buffer[:16]
                return sample

            del self.buffer[0]

        return None

    def read_latest_sample(self, max_reads: int = 8):
        """
        Read up to max_reads samples back-to-back and return only the newest one.
        This avoids lag without changing the low-level parser.
        """
        latest = None
        for _ in range(max_reads):
            sample = self.read_sample()
            if sample is None:
                break
            latest = sample
        return latest