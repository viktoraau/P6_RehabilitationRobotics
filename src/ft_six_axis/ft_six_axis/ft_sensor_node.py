#!/usr/bin/env python3
"""
ROS 2 driver node for the 6-axis FT sensor.

Continuous output (460800 baud, 8N1) is published on:
  ~/wrench   geometry_msgs/WrenchStamped   – calibrated F/T
  ~/raw      std_msgs/Float64MultiArray    – raw int32 channel counts

Tare service:
  ~/tare     std_srvs/Trigger

Tare service:  ~/tare  std_srvs/Trigger

Tare strategy (two-tier):
  1. HARDWARE tare  – the node stops the 460800 reader, re-opens the port at
     115200 (Modbus RTU), writes register 0x0002 = 2 (Tare) to every slave,
     then resumes continuous output.  This zeros the sensor internally and
     survives a node restart.
     CONSTRAINT: the sensor only accepts Modbus commands in the first ~30 s
     after power-on.  With auto_tare=true (default) the node does a hardware
     tare automatically at startup, well within that window.
  2. SOFTWARE tare fallback – if hardware Modbus tare fails (sensor already
     in continuous mode), the node captures the current raw counts and
     subtracts them from every published value.  The service still returns
     success=true but the message says 'software tare applied'.  This is
     reset on node restart.

Parameters
----------
port              /dev/ttyUSB0
baud_continuous   460800
baud_modbus       115200
frame_id          ft_sensor
force_scale       0.001  (raw counts ÷ 1000 → Newtons; sensor rated 100 N)
torque_scale      0.001  (raw counts ÷ 1000 → Nm;      sensor rated 10 Nm)
modbus_slaves     [1, 2, 3, 4, 5, 6]
auto_tare         true  (hardware tare at node startup, within the 30 s window)
"""

import collections
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

import serial

# ── continuous-output packet constants ────────────────────────────────────────
_HEADER      = bytes([0xAA, 0x55])
_ADDR_BYTE   = 0xFF
_CMD_BYTE    = 0x10
_PACKET_SIZE = 38          # 2+1+2+1+24+6+2
_NUM_CH      = 6

# ── Modbus tare constants ──────────────────────────────────────────────────────
_MODBUS_REG_MANIPULATE = 0x0002
_MODBUS_VAL_TARE       = 2   # 1=Clear, 2=Tare (subtract current load)


# ── helpers ────────────────────────────────────────────────────────────────────

def _crc16(data: bytes) -> int:
    """CRC-16 Modbus."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _modbus_write_single(slave: int, register: int, value: int) -> bytes:
    """Build a Modbus RTU function-06 (write single register) frame."""
    payload = struct.pack(">BBHH", slave, 0x06, register, value)
    crc     = _crc16(payload)
    return payload + struct.pack("<H", crc)


def _modbus_read_holding(slave: int, register: int, count: int) -> bytes:
    """Build a Modbus RTU function-03 (read holding registers) frame."""
    payload = struct.pack(">BBHH", slave, 0x03, register, count)
    crc = _crc16(payload)
    return payload + struct.pack("<H", crc)


def _regs_to_int32(reg0: int, reg1: int) -> int:
    """
    Combine two uint16 registers into one signed int32.

    The sensor uses 3412 order for 32-bit register values, i.e. first register
    is low word and second register is high word.
    """
    raw = (reg1 << 16) | reg0
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw


def _parse_continuous(raw: bytes):
    """
    Validate and decode one 38-byte continuous-output packet.
    Returns (channels: list[int], statuses: list[int]) or None.
    """
    if len(raw) != _PACKET_SIZE:
        return None
    if raw[0] != 0xAA or raw[1] != 0x55:
        return None
    if raw[2] != _ADDR_BYTE or raw[5] != _CMD_BYTE:
        return None
    if _crc16(raw[:-2]) != struct.unpack_from("<H", raw, 36)[0]:
        return None
    channels = list(struct.unpack_from("<6i", raw, 6))
    statuses = list(struct.unpack_from("6B",  raw, 30))
    return channels, statuses


# ── node ───────────────────────────────────────────────────────────────────────

class FtSensorNode(Node):

    def __init__(self):
        super().__init__("ft_sensor_node")

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter("port",            "/dev/ttyUSB0")
        self.declare_parameter("baud_continuous", 460800)
        self.declare_parameter("baud_modbus",     115200)
        self.declare_parameter("frame_id",        "ft_sensor")
        self.declare_parameter("force_scale",     0.001)  # raw/1000 → N
        self.declare_parameter("torque_scale",    0.001)  # raw/1000 → Nm
        self.declare_parameter("modbus_slaves",   [1, 2, 3, 4, 5, 6])
        self.declare_parameter("auto_tare",       True)
        # The sensor stays in Modbus mode for about 30 s after power-on before
        # it begins the 460800 continuous stream. Do not fall back earlier.
        self.declare_parameter("startup_data_timeout_sec", 35.0)
        self.declare_parameter("modbus_poll_rate_hz", 80.0)

        self._port          = self.get_parameter("port").value
        self._baud_cont     = self.get_parameter("baud_continuous").value
        self._baud_modbus   = self.get_parameter("baud_modbus").value
        self._frame_id      = self.get_parameter("frame_id").value
        self._force_scale   = float(self.get_parameter("force_scale").value)
        self._torque_scale  = float(self.get_parameter("torque_scale").value)
        self._slaves        = list(self.get_parameter("modbus_slaves").value)
        self._auto_tare     = self.get_parameter("auto_tare").value
        self._startup_data_timeout = float(
            self.get_parameter("startup_data_timeout_sec").value)
        self._modbus_poll_rate_hz = float(
            self.get_parameter("modbus_poll_rate_hz").value)

        self.get_logger().info(
            f"Parameters: port={self._port}  force_scale={self._force_scale}  "
            f"torque_scale={self._torque_scale}  auto_tare={self._auto_tare}"
        )

        # software tare offset (fallback when hardware tare window has passed)
        self._sw_tare       = [0.0] * _NUM_CH
        self._sw_tare_lock  = threading.Lock()
        # latest raw counts from the reader thread (for software tare snapshot)
        self._raw_latest    = [0] * _NUM_CH
        # set when the reader thread receives the very first valid packet
        self._first_packet  = threading.Event()

        # ── publishers ────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub_wrench = self.create_publisher(
            WrenchStamped, "~/wrench", sensor_qos)
        self._pub_raw = self.create_publisher(
            Float64MultiArray, "~/raw", sensor_qos)

        # ── tare service ──────────────────────────────────────────────────────
        self._tare_srv = self.create_service(
            Trigger, "~/tare", self._cb_tare)

        # ── serial state ──────────────────────────────────────────────────────
        self._ser            = None
        self._ser_modbus     = None
        self._lock           = threading.Lock()   # guards port open/close
        self._running        = False
        self._running_modbus = False
        self._stream_mode    = "continuous"

        # ── start ─────────────────────────────────────────────────────────────
        self._open_continuous()

        if self._auto_tare:
            # Fire tare in a background thread so the node finishes __init__.
            # 0.5 s is enough to open serial; hardware tare needs to happen
            # before the sensor's 30 s Modbus window closes.
            threading.Thread(target=self._startup_tare, daemon=True).start()

        # If continuous packets never arrive, fall back to Modbus polling.
        threading.Thread(target=self._ensure_data_stream, daemon=True).start()

    # ── serial open / close ───────────────────────────────────────────────────

    def _open_continuous(self):
        try:
            self._ser = serial.Serial(
                self._port, self._baud_cont,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.002,
            )
            self._ser.reset_input_buffer()
            self._running = True
            self._stream_mode = "continuous"
            threading.Thread(target=self._read_loop, daemon=True).start()
            self.get_logger().info(
                f"Opened {self._port} @ {self._baud_cont} baud (continuous mode). "
                "Data will arrive once sensor exits its 30 s Modbus window after power-on."
            )
        except serial.SerialException as exc:
            self.get_logger().error(f"Cannot open serial port: {exc}")

    def _open_modbus_stream(self):
        """Open 115200 Modbus stream and start polling thread."""
        try:
            self._ser_modbus = serial.Serial(
                self._port, self._baud_modbus,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.02,
            )
            self._ser_modbus.reset_input_buffer()
            self._running_modbus = True
            self._stream_mode = "modbus"
            threading.Thread(target=self._modbus_poll_loop, daemon=True).start()
            self.get_logger().warn(
                f"Continuous stream unavailable. Falling back to Modbus polling on "
                f"{self._port} @ {self._baud_modbus} baud."
            )
        except serial.SerialException as exc:
            self.get_logger().error(f"Cannot open Modbus polling port: {exc}")

    def _stop_reading(self):
        """Signal the reader thread to stop and close the port."""
        self._running = False
        time.sleep(0.05)            # allow reader thread to exit its iteration
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def _stop_modbus_polling(self):
        """Stop Modbus polling thread and close port."""
        self._running_modbus = False
        time.sleep(0.05)
        if self._ser_modbus and self._ser_modbus.is_open:
            self._ser_modbus.close()
        self._ser_modbus = None

    def _read_modbus_regs(self, ser, slave: int, reg: int, count: int):
        """Read Modbus holding registers, returns list[int] or None."""
        cmd = _modbus_read_holding(slave, reg, count)
        expected = 5 + count * 2
        ser.reset_input_buffer()
        ser.write(cmd)
        time.sleep(0.003)
        reply = ser.read(expected)

        if len(reply) != expected:
            return None
        if reply[0] != slave or reply[1] != 0x03 or reply[2] != count * 2:
            return None
        if _crc16(reply[:-2]) != struct.unpack_from("<H", reply, -2)[0]:
            return None

        regs = []
        for i in range(count):
            regs.append(struct.unpack_from(">H", reply, 3 + i * 2)[0])
        return regs

    def _modbus_poll_loop(self):
        """Publish channels by polling 0x0000/0x0001 from each slave."""
        period = 1.0 / max(self._modbus_poll_rate_hz, 1.0)
        while self._running_modbus:
            t0 = time.time()
            channels = []
            try:
                for slave in self._slaves:
                    regs = self._read_modbus_regs(self._ser_modbus, slave, 0x0000, 2)
                    if regs is None:
                        channels = []
                        break
                    channels.append(_regs_to_int32(regs[0], regs[1]))

                if len(channels) == _NUM_CH:
                    with self._lock:
                        self._raw_latest = channels
                    if not self._first_packet.is_set():
                        self.get_logger().info(
                            "[modbus_poll] first data received - publishing started"
                        )
                    self._first_packet.set()
                    self._publish(channels)
            except serial.SerialException as exc:
                if self._running_modbus:
                    self.get_logger().error(f"Modbus polling error: {exc}")
                self._running_modbus = False
                break
            except Exception as exc:
                if self._running_modbus:
                    self.get_logger().error(f"Modbus poll loop error: {exc}")

            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

    def _ensure_data_stream(self):
        """
        Wait for startup data. If continuous packets never arrive,
        switch to Modbus polling so ROS topics still publish.
        """
        got_data = self._first_packet.wait(timeout=self._startup_data_timeout)
        if got_data:
            return

        with self._lock:
            if self._stream_mode != "continuous":
                return
            self.get_logger().warn(
                "No continuous packets received after startup timeout. "
                "Switching to Modbus polling fallback."
            )
            self._stop_reading()
            time.sleep(0.1)
            self._open_modbus_stream()

    # ── continuous read loop ──────────────────────────────────────────────────

    def _read_loop(self):
        buf = bytearray()
        while self._running:
            try:
                waiting = self._ser.in_waiting
                chunk   = self._ser.read(max(waiting, 1))
                if not chunk:
                    continue
                buf.extend(chunk)

                while len(buf) >= _PACKET_SIZE:
                    idx = buf.find(_HEADER)
                    if idx == -1:
                        del buf[:-1]
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < _PACKET_SIZE:
                        break
                    result = _parse_continuous(bytes(buf[:_PACKET_SIZE]))
                    if result is not None:
                        channels, _ = result
                        with self._lock:
                            self._raw_latest = channels
                        if not self._first_packet.is_set():
                            self.get_logger().info(
                                "[read_loop] first packet received – publishing started"
                            )
                        self._first_packet.set()   # signal: real data arrived
                        self._publish(channels)
                        del buf[:_PACKET_SIZE]
                    else:
                        del buf[:1]

            except serial.SerialException as exc:
                if self._running:
                    self.get_logger().error(f"Serial read error: {exc}")
                self._running = False
                break
            except Exception as exc:
                if self._running:
                    self.get_logger().error(f"Read loop error: {exc}")

    # ── publish ───────────────────────────────────────────────────────────────

    def _publish(self, channels: list):
        stamp = self.get_clock().now().to_msg()
        fs    = self._force_scale
        ts    = self._torque_scale

        with self._sw_tare_lock:
            sw = list(self._sw_tare)

        # apply software tare offset (zero if hardware tare was used)
        vals = [float(channels[i]) - sw[i] for i in range(_NUM_CH)]

        # WrenchStamped – channels 0-2 → force, 3-5 → torque
        w = WrenchStamped()
        w.header.stamp    = stamp
        w.header.frame_id = self._frame_id
        w.wrench.force.x  = vals[0] * fs
        w.wrench.force.y  = vals[1] * fs
        w.wrench.force.z  = vals[2] * fs
        w.wrench.torque.x = vals[3] * ts
        w.wrench.torque.y = vals[4] * ts
        w.wrench.torque.z = vals[5] * ts
        self._pub_wrench.publish(w)

        # raw counts (useful for calibration / debugging) – no offset applied
        raw = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label  = "channel"
        dim.size   = _NUM_CH
        dim.stride = _NUM_CH
        raw.layout.dim.append(dim)
        raw.data = [float(c) for c in channels]
        self._pub_raw.publish(raw)

    # ── tare service callback ─────────────────────────────────────────────────

    def _cb_tare(self, _request, response):
        self.get_logger().info("Tare requested")
        success, message = self._do_tare()
        response.success = success
        response.message = message
        return response

    # ── startup tare ──────────────────────────────────────────────────────────

    def _startup_tare(self):
        """
        Called in a daemon thread at node startup.
        1. Wait briefly so the serial port is settled.
        2. Try hardware (Modbus) tare – succeeds if within 30 s of power-on.
        3. If hardware tare fails, wait until the first real packet arrives
           (up to 5 s), then apply a software offset against actual data.
        """
        time.sleep(0.5)   # let serial buffer fill
        self.get_logger().info("[auto_tare] attempting hardware tare at startup")
        hw_ok, hw_msg = self._try_hardware_tare()

        if hw_ok:
            with self._sw_tare_lock:
                self._sw_tare = [0.0] * _NUM_CH
            self.get_logger().info(f"[auto_tare] hardware tare OK: {hw_msg}")
            return

        # Hardware tare window has passed – wait for real data then software tare
        self.get_logger().warn(
            f"[auto_tare] hardware tare failed ({hw_msg}); "
            "waiting for first packet to apply software tare …"
        )
        got_data = self._first_packet.wait(timeout=5.0)
        if not got_data:
            self.get_logger().error(
                "[auto_tare] no packets received within 5 s – tare skipped"
            )
            return

        with self._lock:
            raw_now = list(self._raw_latest)
        with self._sw_tare_lock:
            self._sw_tare = [float(v) for v in raw_now]
        self.get_logger().info(
            f"[auto_tare] software tare applied from first packet: {self._sw_tare}"
        )

    # ── tare: try hardware first, fall back to software ───────────────────────

    def _do_tare(self) -> tuple:
        """
        Attempt a hardware Modbus tare (register 0x0002 = 2) on every slave.
        If the sensor is already in continuous-output mode (Modbus window
        expired), fall back to a software offset: capture the current raw
        values and subtract them from every published wrench.
        """
        hw_ok, hw_msg = self._try_hardware_tare()
        if hw_ok:
            # hardware tare succeeded – clear any residual software offset
            with self._sw_tare_lock:
                self._sw_tare = [0.0] * _NUM_CH
            return True, hw_msg

        # hardware tare failed → apply software tare
        self.get_logger().warn(
            f"[tare] Hardware tare unavailable ({hw_msg}). "
            "Applying software offset instead."
        )
        with self._lock:
            raw_now = list(self._raw_latest)
        with self._sw_tare_lock:
            self._sw_tare = [float(v) for v in raw_now]
        self.get_logger().info(
            f"[tare] Software offset applied: {self._sw_tare}"
        )
        return True, (
            f"Software tare applied (hardware tare unavailable: {hw_msg}). "
            "To use hardware tare: power-cycle the sensor and restart the node "
            "with auto_tare:=true, or call tare within 30 s of power-on."
        )

    def _try_hardware_tare(self) -> tuple:
        """
        Stop reader, switch to 115200 Modbus baud, send tare command to all
        slaves, return to continuous mode.  Returns (success, message).
        """
        with self._lock:
            prev_mode = self._stream_mode
            self._stop_reading()
            self._stop_modbus_polling()
            time.sleep(0.1)

            errors = []
            any_ok = False

            try:
                ser = serial.Serial(
                    self._port, self._baud_modbus,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.3,
                )
                ser.reset_input_buffer()

                # Register writes applied once per slave during the Modbus window:
                #   0x0002 = 2  Tare
                #   0x0011 = 0  Disable zero tracking  (fixes drift)
                #   0x000F = 0  Disable power-on auto-zero
                #   0x0012 = 0  Disable stability display (highest resolution)
                #   0x0060 = 5  200 Hz ADC sampling
                #   0x0013 = 5  Heaviest filter (max smoothing)
                _STARTUP_REGS = [
                    (0x0002, 2),   # tare
                    (0x0011, 0),   # disable zero tracking
                    (0x000F, 0),   # disable power-on auto-zero
                    (0x0012, 0),   # disable stability display
                    (0x0060, 5),   # 200 Hz sampling
                    (0x0013, 5),   # heaviest filter
                ]

                for slave in self._slaves:
                    slave_ok = True
                    for reg, val in _STARTUP_REGS:
                        cmd = _modbus_write_single(slave, reg, val)
                        ser.write(cmd)
                        time.sleep(0.05)
                        reply = ser.read(8)
                        if not (len(reply) == 8 and reply[:6] == cmd[:6]):
                            msg = (
                                f"slave {slave} reg 0x{reg:04X}={val}: no valid reply "
                                f"(got {reply.hex() if reply else 'empty'})"
                            )
                            errors.append(msg)
                            self.get_logger().warn(f"[tare] {msg}")
                            slave_ok = False
                    if slave_ok:
                        self.get_logger().info(
                            f"[tare] slave {slave}: tare + settings applied OK")
                        any_ok = True

                ser.close()

            except serial.SerialException as exc:
                errors.append(str(exc))
                self.get_logger().error(f"[tare] serial error: {exc}")

            finally:
                time.sleep(0.1)
                if prev_mode == "modbus":
                    self._open_modbus_stream()
                else:
                    self._open_continuous()

        if not errors:
            return True, f"Hardware tare applied on slaves {self._slaves}"
        if any_ok:
            return False, "partial: " + "; ".join(errors)
        return False, "; ".join(errors)

    # ── cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        with self._lock:
            self._stop_reading()
            self._stop_modbus_polling()
        super().destroy_node()


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FtSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
