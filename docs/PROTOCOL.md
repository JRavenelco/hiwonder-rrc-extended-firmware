# Host protocol

The extended firmware keeps the vendor's serial protocol unchanged and adds one
function to it. Everything on this page can be tried with `tools/motor_ext.py`.

## Transport

- USB-C port labelled **UART2** on the board. Its USB bridge (QinHeng, `1a86:55d4`)
  shows up on Linux as `/dev/ttyACM*` and under
  `/dev/serial/by-id/usb-1a86_USB_Single_Serial_<serial>-if00`.
- Inside the MCU it is USART3 (PD8/PD9) at **1,000,000 baud**, 8N1.
- The firmware never sends an answer on its own to a position command: the host
  polls. Battery voltage and IMU frames are pushed periodically.

## Frame

```
0xAA 0x55 | function | length | payload[length] | crc
```

`crc` is CRC-8/MAXIM (polynomial `0x31` reflected, `0x8C`; initial value 0)
computed over `function`, `length` and the payload. Frames with a function number
of `0x0C` or more, or with a wrong CRC, are dropped silently.

```python
def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0x8C if c & 1 else c >> 1
    return c
```

All multi-byte integers and `float32` values are little-endian.

## Functions

| Function | Name | Notes |
|---|---|---|
| `0x00` | SYS | vendor; pushes the battery voltage (`sub = 0x04`, `uint16` millivolts) |
| `0x01` | LED | vendor |
| `0x02` | BUZZER | vendor |
| `0x03` | MOTOR | vendor speed commands, **hardened**, see below |
| `0x04` | PWM_SERVO | vendor |
| `0x05` | BUS_SERVO | vendor |
| `0x06` | KEY | vendor |
| `0x07` | IMU | vendor frame, **now actually transmitted**, see below |
| `0x08` | GAMEPAD | vendor |
| `0x09` | SBUS | vendor |
| `0x0B` | **MOTOR_EXT** | **new: position control** |

## `0x0B` MOTOR_EXT

The first payload byte is the sub-command. `motor_id` is 0 for M1 up to 3 for M4
and must be the position motor of the firmware (3 in the released image).

### `0x00` CAPABILITIES

Request: 1 byte, `00`. Reply: 8 bytes.

| Offset | Type | Field | Released image |
|---|---|---|---|
| 0 | u8 | `cmd` = 0 | |
| 1 | u8 | protocol version | 3 |
| 2 | u8 | `motor_mask`, bit *n* set = position control on motor *n* | `0x08` (M4) |
| 3 | u8 | reserved | 0 |
| 4 | u32 | encoder ticks per turn of the output shaft | 1320 |

Ask for this first. It tells a host whether it is talking to the extended
firmware at all (the vendor firmware does not answer), which motor it may command
and how to convert degrees: `ticks = round(degrees * ticks_per_turn / 360)`.

### `0x01` MOVE_RELATIVE

Request: 14 bytes.

| Offset | Type | Field | Accepted range |
|---|---|---|---|
| 0 | u8 | `cmd` = 1 | |
| 1 | u8 | `motor_id` | the position motor |
| 2 | i32 | `delta_ticks`, relative to the current position | not 0 |
| 6 | f32 | `max_rps`, speed limit in rev/s | 0 < value <= 5.7 |
| 10 | u16 | `tolerance_ticks` | >= 1 |
| 12 | u16 | `timeout_ms` | 100 .. 60000 |

The move is `reached` when the error has stayed inside the tolerance with the
shaft at rest for five consecutive control cycles (50 ms). If the timeout expires
first, the status is `timeout` and the speed set point goes to zero. A new
MOVE_RELATIVE replaces a move or a HOLD in progress.

### `0x02` CANCEL, `0x03` ZERO, `0x04` STATUS

Request: 2 bytes, `cmd, motor_id`.

- **CANCEL** leaves the position mode (status `cancelled`) and asks for zero speed.
- **ZERO** makes the current position the origin of the reported positions. The
  hardware counter is not touched.
- **STATUS** only reports.

### `0x05` HOLD (protocol version 3)

Request: 10 bytes.

| Offset | Type | Field | Accepted range |
|---|---|---|---|
| 0 | u8 | `cmd` = 5 | |
| 1 | u8 | `motor_id` | the position motor |
| 2 | f32 | `max_rps`, speed limit while coming back | 0 < value <= 5.7 |
| 6 | u16 | `tolerance_ticks` | >= 1 |
| 8 | u16 | `max_pulse`, PWM cap while holding | 300 .. 1000 (30 % .. 100 %) |

Holds the position the shaft has when the command arrives, with no timeout. If
the shaft is pushed out of the tolerance it is driven back with the same control
law as a move. If the motor pushes at `max_pulse` without moving for 3 s, the HOLD
is released with zero PWM and status `stalled`. CANCEL, MOVE_RELATIVE, a speed
command for that motor or a stop also end it.

### The status report

MOVE_RELATIVE, CANCEL, ZERO, STATUS and HOLD are all answered with the same
20 bytes:

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `cmd`, the sub-command being answered |
| 1 | u8 | `motor_id` |
| 2 | u8 | `status`, see the table below |
| 3 | u8 | `active`: 1 while a move or a HOLD is running |
| 4 | i32 | position in ticks, relative to the origin set by ZERO |
| 8 | i32 | target in ticks, same origin |
| 12 | i32 | error = target − position |
| 16 | f32 | measured speed in rev/s |

| `status` | Meaning |
|---|---|
| 0 | `idle`: no position command since reset |
| 1 | `moving` |
| 2 | `reached` |
| 3 | `cancelled`: by CANCEL, by a stop or by a speed command |
| 4 | `timeout` |
| 5 | `invalid`: wrong `motor_id`, parameter out of range or unknown sub-command; the motor was not touched |
| 6 | `holding` |
| 7 | `stalled`: the HOLD let go because the shaft was blocked |

A MOVE_RELATIVE or HOLD frame with the wrong length, and any other command
shorter than two bytes, is dropped without an answer.

### Example frames

```
CAPABILITIES            AA 55 0B 01 00 05
  reply (v3, M4, 1320)  AA 55 0B 08 00 03 08 00 28 05 00 00 8F
STATUS of M4            AA 55 0B 02 04 03 02
ZERO on M4              AA 55 0B 02 03 03 6C
CANCEL on M4            AA 55 0B 02 02 03 A8
MOVE_RELATIVE M4 +110 ticks (30 degrees), 0.25 rev/s, tolerance 15, timeout 5000 ms
                        AA 55 0B 0E 01 03 6E 00 00 00 00 00 80 3E 0F 00 88 13 AE
HOLD M4, 1.0 rev/s, tolerance 8, PWM cap 400
                        AA 55 0B 0A 05 03 00 00 80 3F 08 00 90 01 36
```

## `0x03` MOTOR: what changed

The vendor's speed commands work as before, with three differences.

- Sub-commands 0 (one motor) and 1 (several motors) now ignore a `motor_id` above
  3 and frames shorter than the number of motors they announce. The original code
  used both values to index memory without checking them.
- Sub-commands 2 (stop one motor) and 3 (stop by mask) now perform a **hard
  stop**: the accumulated PWM is forced to zero. The original only wrote a zero
  set point, which does not stop a motor whose encoder is not counting.
- A speed command addressed to the position motor cancels a move or a HOLD in
  progress, so existing host software keeps full control.

```
Stop all four motors    AA 55 03 02 03 0F D3
```

## `0x07` IMU

The firmware sends the vendor's raw IMU frame at most every 50 ms (20 Hz):
24 bytes, six `float32` values.

| Offset | Field | Unit |
|---|---|---|
| 0, 4, 8 | acceleration x, y, z | m/s² |
| 12, 16, 20 | angular rate x, y, z | **rad/s** |

A host that treats the angular rate as degrees per second integrates a heading
57 times too small. Calibrate the gyroscope bias with the robot at rest before
integrating: on the bench board it was about (−0.07, +0.18, −0.01) rad/s.

## A first session with the tool

```bash
python3 tools/motor_ext.py --caps
python3 tools/motor_ext.py --status
python3 tools/motor_ext.py --imu 5
# from here on the motor moves: 12 V connected, wheel free, motor switch at hand
python3 tools/motor_ext.py --deg 30 --i-know-it-moves
python3 tools/motor_ext.py --hold --i-know-it-moves
python3 tools/motor_ext.py --cancel
python3 tools/motor_ext.py --stop
```

The tool asks for CAPABILITIES before moving, refuses a motor the firmware does
not control, and sends a hard stop if the shaft does not move or moves away from
the target.
