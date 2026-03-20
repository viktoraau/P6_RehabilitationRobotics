# MD FDCAN Register Cheat‑Sheet (per MD v2.x docs)

## Frames
- **WRITE_REGISTER**: `ID=0x40`, drive-id 10–999, payload pairs of `<regID:uint16><value>` (little‑endian), max 64 B total. Success → default response.
- **READ_REGISTER**: `ID=0x41`, payload pairs of `<regID:uint16><0x00 uint16>`, max 64 B. Success → response frame `ID=0x41` with `<regID><value>` pairs.
- **Default response** (on successful write): `0x0A | frame_id | quick_status | motor_temp[°C] | main_pos[rad] | main_vel[rad/s] | motor_torque[Nm] | out_pos[rad] | out_vel[rad/s]`.

## Register Map (most-used)
| Name | Addr | R/W | Type / Limits | Notes |
| --- | --- | --- | --- | --- |
| `canId` | 0x001 | RW | u32 [10–2000] | FDCAN node ID |
| `canBaudrate` | 0x002 | RW | u32 {1e6,2e6,5e6,8e6} | Data rate |
| `canWatchdog` | 0x003 | RW | u16 [0–2500] ms | Bus watchdog |
| `canTermination` | 0x004 | RW | u8 {0,1} | SW termination (if supported) |
| `motorName` | 0x010 | RW | char[24] | |
| `motorPolePairs` | 0x011 | RW | u32 [2–225] | |
| `motorKt` | 0x012 | RW | float >0 | Torque constant |
| `motorKt_a/b/c` | 0x013–0x015 | RW | float >0 | Phase‑specific optional |
| `motorIMax` | 0x016 | RW | float [1 – peak] | Phase current limit |
| `motorGearRatio` | 0x017 | RW | float | <1 reducer (e.g. 0.5 for 2:1); >1 multiplier |
| `motorTorqueBandwidth` | 0x018 | RW | u16 [50–2500] Hz | |
| `motorKV` | 0x01D | RW | u16 | KV rating RPM/V |
| `motorCalibrationMode` | 0x01E | RW | u8 {0 FULL, 1 NOPPDET} | |
| `outputEncoder` | 0x020 | RW | u8 {0 NONE,1 ME_AS_CENTER,2 ME_AS_OFFAXIS,3 MB053SFA17BENT00,4 CM_OFFAXIS,5 M24B_CENTER,6 M24B_OFFAXIS} | |
| `outputEncoderDir` | 0x021 | RW | u8 | Direction (reserved in doc) |
| `outputEncoderDefaultBaud` | 0x022 | RW | u32 (e.g. 115200) | |
| `outputEncoderVelocity` | 0x023 | RO | float | rad/s (5 kHz loop) |
| `outputEncoderPosition` | 0x024 | RO | float | rad (5 kHz loop) |
| `outputEncoderMode` | 0x025 | RW | u8 {0 NONE,1 STARTUP,2 MOTION,3 REPORT,4 MAIN} | |
| `outputEncoderCalibrationMode` | 0x026 | RW | u8 {0 FULL,1 DIRONLY} | |
| `motorPosPidKp/Ki/Kd/Windup` | 0x030/0x031/0x032/0x034 | RW | float | Position PID |
| `motorVelPidKp/Ki/Kd/Windup` | 0x040/0x041/0x042/0x044 | RW | float | Velocity PID |
| `motorImpPidKp/Kd` | 0x050/0x051 | RW | float | Impedance PD |
| `mainEncoderVelocity` | 0x062 | RO | float | rad/s (40 kHz) |
| `mainEncoderPosition` | 0x063 | RO | float | rad (40 kHz) |
| `motorTorque` | 0x064 | RO | float | Nm (40 kHz) |
| `runSaveCmd` | 0x080 | WO | u8≠0 | Save to flash |
| `runCalibrateCmd` | 0x083 | WO | u8≠0 | Main calibration |
| `runCalibrateOutputEncoderCmd` | 0x084 | WO | u8≠0 | Output encoder calib |
| `runCalibratePiGains` | 0x085 | WO | u8≠0 | Current PI auto‑cal |
| `runRestoreFactoryConfig` | 0x087 | WO | u8≠0 | Factory reset |
| `runReset` | 0x088 | WO | u8≠0 | Soft reset |
| `runClearWarnings` | 0x089 | WO | u8≠0 | Clear warnings |
| `runClearErrors` | 0x08A | WO | u8≠0 | Clear errors |
| `runZero` | 0x08C | WO | u8≠0 | Set new zero |
| `runCanReinit` | 0x08D | WO | u8≠0 | Reinit CAN |
| `positionLimitMax/Min` | 0x110/0x111 | RW | float | rad limits |
| `maxTorque` | 0x112 | RW | float | Nm |
| `maxVelocity` | 0x113 | RW | float | rad/s |
| `maxAcceleration` | 0x114 | RW | float | rad/s² |
| `maxDeceleration` | 0x115 | RW | float | rad/s² |
| `profileVelocity` | 0x120 | RW | float | rad/s |
| `profileAcceleration` | 0x121 | RW | float | rad/s² |
| `profileDeceleration` | 0x122 | RW | float | rad/s² |
| `quickStopDeceleration` | 0x123 | RW | float | rad/s² |
| `positionWindow` | 0x124 | RW | float | goal tolerance |
| `velocityWindow` | 0x125 | RW | float | goal tolerance |
| `motionModeCommand` | 0x140 | WO | u8 {0 IDLE,1 POSITION_PID,2 VELOCITY_PID,3 RAW_TORQUE,4 IMPEDANCE,7 POSITION_PROFILE,8 VELOCITY_PROFILE} | Set mode |
| `motionModeStatus` | 0x141 | RO | u8 | Current mode |
| `state` | 0x142 | RW | u16 | Internal state machine |
| `targetPosition/Velocity/Torque` | 0x150/0x151/0x152 | RW | float | rad, rad/s, Nm |
| `userGpioConfiguration` | 0x160 | RW | u8 {0 OFF,1 AUTO-BRAKE,2 GPIO IN} | |
| `userGpioState` | 0x161 | RO | u16 | |
| `reverseDirection` | 0x600 | RW | u8 | Main encoder dir; recalibrate after change |
| `shuntResistance` | 0x700 | RO | float [0.001–0.01] | Warning: hardware specific |
| `buildDate` | 0x800 | RO | u32 | |
| `commitHash` | 0x801 | RO | char[8] | |
| `firmwareVersion` | 0x802 | RO | u32 | |
| `hardwareVersion` | 0x803 | RO | u8 | |
| `bridgeType` | 0x804 | RO | u8 | MOSFET driver type |
| `quickStatus` | 0x805 | RO | u16 | Status vector |
| `mosfetTemperature` | 0x806 | RO | float | °C |
| `motorTemperature` | 0x807 | RO | float | °C |
| `motorShutdownTemp` | 0x808 | RW | u8 | °C trip to IDLE |
| `mainEncoderErrors` | 0x809 | RO | u32 | |
| `outputEncoderErrors` | 0x80A | RO | u32 | |
| `calibrationErrors` | 0x80B | RO | u32 | |
| `bridgeErrors` | 0x80C | RO | u32 | |
| `hardwareErrors` | 0x80D | RO | u32 | |
| `communicationErrors` | 0x80E | RO | u32 | |
| `motionErrors` | 0x810 | RO | u32 | |

## Quick usage notes
- Max frame payload is 64 bytes; batch multiple `<reg,value>` pairs up to that limit.
- All fields little‑endian; floats are 32‑bit IEEE754.
- For gearboxes: `motorGearRatio = joint_gear_ratio` (e.g., 0.1 for 10:1 reducer).
- After tuning/calibration writes, call `runSaveCmd` to persist to flash.
- To clear latched faults: `runClearWarnings`, `runClearErrors`, then re‑enable/mode set.
