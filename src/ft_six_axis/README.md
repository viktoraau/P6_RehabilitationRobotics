# ft_six_axis – 6-Axis FT Sensor ROS 2 Driver

## Quick start

```bash
# build
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ft_six_axis
source install/setup.bash

# run
ros2 launch ft_six_axis ft_sensor.launch.py port:=/dev/ttyUSB0

# tare (any time)
ros2 service call /ft_sensor/tare std_srvs/srv/Trigger {}
```

### Topics

| Topic | Type | Description |
|---|---|---|
| `/ft_sensor/wrench` | `geometry_msgs/WrenchStamped` | Tare-compensated Fx Fy Fz Tx Ty Tz |
| `/ft_sensor/raw` | `std_msgs/Float64MultiArray` | Raw int32 counts, no offset applied |

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial device |
| `baud_continuous` | `460800` | Continuous-output baud |
| `baud_modbus` | `115200` | Modbus baud (used only during tare) |
| `frame_id` | `ft_sensor` | TF frame id on published wrench |
| `force_scale` | `1.0` | Multiply raw counts → Newtons |
| `torque_scale` | `1.0` | Multiply raw counts → Nm |
| `modbus_slaves` | `[1,2,3,4,5,6]` | Slave addresses to tare |
| `auto_tare` | `true` | Tare automatically at node startup |
| `startup_data_timeout_sec` | `35.0` | How long to wait for the 460800 continuous stream before falling back to Modbus polling |

---

## Sensor communication overview

### Serial modes

The sensor has **two operating modes** after power-on:

| Time after power-on | Mode | Baud | Protocol |
|---|---|---|---|
| 0 – ~30 s | **Modbus RTU** | 115 200 | Request/response, read/write registers |
| After 30 s (or if no Modbus traffic) | **Continuous output** | 460 800 | Streams packets automatically, no requests |

The ROS node uses continuous output for publishing. The tare service
temporarily switches back to 115 200 to send a Modbus command, then
returns to continuous mode. This only works if called within the 30 s
window; after that a software offset is applied instead.

Important: the node must wait at least about 30 s after power-on before it
declares continuous mode unavailable. Falling back earlier will keep the
sensor in Modbus traffic and prevent the continuous stream from ever starting.

### Continuous-output packet (38 bytes, little-endian)

```
Byte  0–1   Header         0xAA 0x55  (fixed)
Byte  2     Address        0xFF       (fixed)
Byte  3–4   Frame length   uint16     (= 38)
Byte  5     Command        0x10       (fixed)
Byte  6–29  CH1–CH6 data   6 × int32  (little-endian signed)
Byte 30–35  CH1–CH6 status 6 × uint8
Byte 36–37  CRC-16         Modbus CRC over bytes 0–35
```

Channel order: CH1=Fx, CH2=Fy, CH3=Fz, CH4=Tx, CH5=Ty, CH6=Tz

#### Channel status byte bits

| Bit | 0 | 1 | 2 | 3 | 4 | 5 | 6–15 |
|---|---|---|---|---|---|---|---|
| **Meaning** | 0=sensor OK / 1=disconnected | 0=chip OK / 1=chip fault | 0=unstable / 1=stable | 0=non-zero / 1=at-zero | 0=normal / 1=overload | 0=normal / 1=failed to zero on power-up | reserved |

### Modbus frame formats

**Read registers (function 03H)**
```
[slave] 03 [addr_hi] [addr_lo] [count_hi] [count_lo] [CRC_lo] [CRC_hi]
```

**Write single register (function 06H)**
```
[slave] 06 [addr_hi] [addr_lo] [val_hi] [val_lo] [CRC_lo] [CRC_hi]
```

**Write multiple registers (function 10H)**
```
[slave] 10 [addr_hi] [addr_lo] [count_hi] [count_lo] [byte_count] [data...] [CRC_lo] [CRC_hi]
```

> **Big-endian note:** multi-byte values inside the data payload are big-endian
> (high byte first). 32-bit values span two consecutive registers.
> CRC itself is transmitted little-endian (low byte first).

---

## Modbus register reference

All addresses are 0-indexed (protocol addresses). PLC address = protocol address + 40001.

### Real-time data (read-only)

#### `0x0000–0x0001` — Display weight (Int32)
The current net/gross weight reading with decimal point ignored.
Read 2 registers to get one Int32 value. Example: value `12345678` with
2 decimal places means `123456.78` units.

```
Read:  01 03 00 00 00 02 C4 0B
Reply: 01 03 04 [byte3][byte2][byte1][byte0] [CRC]
```

#### `0x0004–0x0005` — Gross weight (Int32, read-only)
Raw gross weight before tare subtraction.

#### `0x0062–0x0063` — AD internal code (Int32, read-only)
Raw ADC counts directly from the analogue-to-digital converter, before
any calibration or filtering is applied. Use these for building a
calibration matrix.

```
Read:  01 03 00 62 00 02 [CRC]
```

---

### Tare / zeroing

#### `0x0002` — Manipulate (Int16, read/write)
**The main runtime control register.** Write one of:

| Value | Effect |
|---|---|
| `0` | No action |
| `1` | **Clear** – remove tare, return to gross weight |
| `2` | **Tare** – store current load as tare offset (zero the display) |
| `3` | Clear (same as 1) |
| `4` | ×10 display mode |
| `5` | Toggle gross / net weight mode |

Reading this register returns the current display status flags (see status
bits below), **not** the last written value.

```
Tare:   01 06 00 02 00 02 28 0A
Clear:  01 06 00 02 00 01 E9 CA
```

> **Important:** value `2` (Tare) tells the sensor to internally store
> whatever load is currently on the sensor as the new zero. This is a
> hardware offset that survives node restarts. Value `1` (Clear) removes
> that stored tare.

#### `0x0006–0x0007` — Tare weight (Int32, read/write)
Directly read or write the stored tare value in raw counts (decimal
ignored). Writing `0` here manually clears the tare without sending a
Manipulate command.

#### `0x000F` — Power-on zero range (Int16, 0–100 %)
Percentage of full scale within which the sensor automatically zeros
itself when it powers on. Default `2` means it auto-zeros if the startup
load is within ±2 % of full scale. Set to `0` to disable power-on
auto-zero entirely (useful if the sensor is always under load at startup).

```
Disable power-on auto-zero:  01 06 00 0F 00 00 F8 09
```

#### `0x0010` — Manual zero range (Int16, 0–100 %)
Maximum load (as % of full scale) that the operator-triggered zero command
(Manipulate = 2) is allowed to zero. Prevents accidental zeroing under
large load. Default `5` (±5 % FS).

#### `0x0011` — Zero tracking range (Int16)
**Drift source #1.** The sensor continuously watches the signal and if it
stays within this range for long enough, it silently shifts the zero
baseline. This causes the slow drift visible in long recordings.
Value is in raw counts (decimal ignored). Default `10`.

```
Disable zero tracking (recommended for robotics):
  01 06 00 11 00 00 D5 CE
```

---

### Filtering and sampling

#### `0x0013` — Filter level (Int16, 0–5)
Internal low-pass filter applied to the ADC output before the value is
reported. Higher = more smoothing but more phase lag and lower bandwidth.

| Value | Filter |
|---|---|
| `0` | Off (raw) |
| `1` | Light |
| `2` | Medium-light |
| `3` | Medium (default) |
| `4` | Heavy |
| `5` | Heaviest |

For control applications at 100 Hz, value `1` or `2` is usually best.
Value `3+` introduces enough phase lag to destabilise force controllers.

```
Set filter = 1:  01 06 00 13 00 01 B8 0E
Set filter = 0:  01 06 00 13 00 00 79 CE
```

#### `0x0060` — AD sampling rate (Int16)
**Sets the actual hardware sampling rate of the ADC.** This is independent
of the Modbus/continuous-output communication rate.

| Value | Rate |
|---|---|
| `0` | 6 Hz |
| `1` | 12 Hz |
| `2` | 25 Hz |
| `3` | 50 Hz |
| `4` | **100 Hz** (default) |
| `5` | **200 Hz** |
| `6` | 40 Hz |
| `7` | 800 Hz |
| `8` | 1600 Hz |

For force control, 200 Hz (`5`) gives better phase margin than the 100 Hz
default. At very high rates (800+) the noise floor increases significantly.

```
Set 200 Hz:  01 06 00 60 00 05 88 10
Set 100 Hz:  01 06 00 60 00 04 49 D0
```

#### `0x0012` — Stability display range (Int16, 0–99)
Hysteresis on the displayed value: the display only updates when the
reading changes by more than this many display counts. Useful for panel
meters, counterproductive in control. Set to `0` to disable.

```
Disable stability display:  01 06 00 12 00 00 24 0E
```

---

### Calibration registers

#### `0x001A–0x001B` — Total sensor range (Int32)
Sum of the rated capacities of all load cells connected to this
transmitter. Used as the reference for calibration scaling.

#### `0x001C–0x001D` — Average sensor sensitivity (Int32)
The average mV/V output of the load cells. `200000` = 2.00000 mV/V.
Change this if your load cells have a different rated output.

#### `0x0014` — Calibration point select (Int16, 0–5)
Select which calibration point you are about to set.
`0` = zero point, `1–5` = span calibration points.

#### `0x0015` — Calibration execute / status (Int16)
Write `1` to execute calibration at the currently selected point.
Reading returns the countdown in seconds; `0` = done; `-1` = failed.

#### `0x0016–0x0017` — Calibration weight value (Int32)
The actual known weight placed on the sensor during span calibration.
Write this before executing calibration point 1.

---

### Communication settings

#### `0x001E` — Slave address (Int16, 1–32)
The Modbus slave address of this transmitter. Factory default is `32`.
The module fixed addresses used in multi-drop wiring are 1–6.

```
Read slave address:   01 03 00 1E 00 01 E4 0C
Set slave address=1:  01 06 00 1E 00 01 28 0C
```

#### `0x001F` — Software version (Int16, read-only)

#### `0x0022–0x0023` — Device serial number (Int32, read-only)

---

## Known issues and recommended settings for robotics

### Problem 1: Drift

**Cause:** Zero tracking (reg `0x0011`) silently moves the baseline during
quiet periods. Also power-on auto-zero (reg `0x000F`) can introduce an
unintended offset if the sensor is under load when the node restarts.

**Recommended fix — send these during the 30 s Modbus window at startup:**

```python
# in _try_hardware_tare(), after the tare command, for each slave:
ser.write(_modbus_write_single(slave, 0x0011, 0))   # disable zero tracking
time.sleep(0.05); ser.read(8)
ser.write(_modbus_write_single(slave, 0x000F, 0))   # disable power-on auto-zero
time.sleep(0.05); ser.read(8)
ser.write(_modbus_write_single(slave, 0x0012, 0))   # disable stability display
time.sleep(0.05); ser.read(8)
ser.write(_modbus_write_single(slave, 0x0060, 5))   # 200 Hz sampling
time.sleep(0.05); ser.read(8)
ser.write(_modbus_write_single(slave, 0x0013, 1))   # light filter only
time.sleep(0.05); ser.read(8)
```

### Problem 2: Cross-axis coupling

**Cause:** The node currently uses scalar `force_scale` / `torque_scale`
(one gain per axis, no off-diagonal terms). Physical FT sensors have
mechanical cross-talk: a pure Fz load produces small Fx, Fy, Tx, Ty
signals. Without a 6×6 calibration matrix this looks like coupling.

**Fix:** Record raw counts from `/ft_sensor/raw` under known single-axis
loads, compute a 6×6 matrix $C$ and store it in config/cal_matrix.yaml:

$$\mathbf{F} = C \cdot \mathbf{r}_\text{raw}$$

Minimum viable: 6×6 diagonal (individual scale per channel).
Full decoupling: 6×6 dense matrix (off-diagonal terms remove cross-talk).

### Problem 3: Tare only works within 30 s of power-on

The sensor only accepts Modbus commands during the first ~30 s after
power-up. After that it is in continuous-output mode at 460 800 baud.
The node handles this with a two-tier strategy:
1. Hardware Modbus tare (preferred, stores offset in sensor, survives restart)
2. Software tare fallback (subtracts current reading in the node, lost on restart)

**To always get a hardware tare:** use `auto_tare:=true` (default) and
power-cycle the sensor before starting the node. The node sends the tare
command 0.5 s after startup, well within the window.

---

## Calibration procedure

```
1. Mount sensor, power-cycle
2. Launch node within 30 s (auto_tare=true handles zeroing)
3. Apply known load along each axis independently
4. Record /ft_sensor/raw for each known load
5. Build 6×6 matrix: C = F_known @ pinv(R_raw)
6. Save to config/cal_matrix.yaml
7. Load matrix in _publish() instead of scalar force_scale/torque_scale
```
