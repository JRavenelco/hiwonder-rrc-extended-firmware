# Extended firmware for the Hiwonder ROS Robot Controller V1.2

Firmware for the **Hiwonder ROS Robot Controller V1.2** (STM32F407VET6) that keeps
everything the vendor's firmware does and adds what a robot needs in order to use
the board as a self-contained motion controller:

| Addition | What it gives you |
|---|---|
| **Position control** (`MOTOR_EXT`, function `0x0B`) | Move a motor by a number of encoder ticks, with speed limit, tolerance, timeout and status reports. Closed in the STM32 at 100 Hz, not in the host. |
| **Position HOLD** | Keep the shaft where it is, with a limited PWM, and let go if it is blocked for 3 s. |
| **Hard stop** | A stop command really stops the motor, even with a dead encoder. |
| **IMU telemetry** | Accelerometer and gyroscope sent to the host at 20 Hz. The vendor source ships with this commented out. |
| **Independent watchdog** | A hang of the interrupts or of the RTOS resets the board in about a quarter of a second instead of leaving the PWM frozen. |
| **Input validation** | The vendor's motor commands no longer index past the motor table. |
| **Bug fix** | The M4 encoder timer is now started with the right handle. |

Position control works on **one motor port, M4 by default** (one `#define` moves it).
The reference setup is a JGB37-520R30-12 gear motor (1:30, 1320 ticks per turn).

This repository contains the complete, buildable source as an **STM32CubeIDE
project**, a script that rebuilds the validated image **bit for bit**, the
documentation of how it works and of how it was made, and a small host tool.

## Status: what has been tested, and how

The image called `pos-v8` was flashed on a real board and exercised on the bench
on 2026-10-04. Its SHA-256 is

```
3a4d9e885ac408b11da56e7263b9e94c843f1b7bbbbf399f21cc93ab4a23f1e8  RosRobotControllerM4-extended-m4-pos-v8.hex
```

| Item | Evidence |
|---|---|
| Relative moves on M4 | +30° ended at +27.8° in 1.2 s and −30° at −29.5° in 1.7 s, both `reached`; a 30000° run at 2.88 rev/s ended `reached` in 29.6 s |
| Top speed | 5.76 rev/s (345 rpm) measured with no load at 12 V and 100 % PWM |
| HOLD | 20 s without drift; pushed 90° away (by moving the target over SWD) it came back and held; with the encoder frozen the PWM stayed at its cap and the HOLD released after 3 s |
| Watchdog | A forced NMI resets the board by itself; 60 s at about 1500 requests per second gave no false reset |
| IMU telemetry | 19.7 Hz, every frame with a valid CRC |
| Reproducible build | The script in `tools/` reproduces that exact image on x86-64 and on aarch64 |
| STM32CubeIDE project | Builds with 0 errors in STM32CubeIDE 2.3.0. Same sources and options, another compiler version and the IDE's stub system calls: **its bytes differ from `pos-v8` and that build has not been run on hardware** |

Not tested: turning the wheel by hand against the HOLD, any load on the wheel, and
every motor port other than M1 and M4.

## Quick start

### 1. Flash the validated image

Take `RosRobotControllerM4-extended-m4-pos-v8.hex` from the
[Releases](../../releases) page (or rebuild it, see below), check its SHA-256 and
flash it over SWD with an ST-LINK. Three wires on header H1 are enough.
[docs/FLASHING.md](docs/FLASHING.md) has the pinout, the commands for
STM32CubeProgrammer and OpenOCD, how to back up the factory firmware first and how
to go back to it.

Flash with the 12 V supply disconnected.

### 2. Check that it answers

With the USB-C port labelled **UART2** connected to a Linux computer:

```bash
python3 tools/motor_ext.py --caps
```

```
MOTOR_EXT v3: position control on M4, 1320 ticks per turn
```

`--status` and `--imu 5` only read. Commands that move the motor need the 12 V
supply and are described in [docs/PROTOCOL.md](docs/PROTOCOL.md).

### 3. Build it yourself

*Bit-exact, with a stock Arm GNU Toolchain 15.2.Rel1 in `PATH`:*

```bash
python3 tools/build_reproducible.py
```

```
result   : IDENTICAL to the bench-validated image (pos-v8)
```

*With STM32CubeIDE:* `File > Import... > General > Existing Projects into
Workspace`, select the `firmware` folder of this repository and press the hammer.
[docs/BUILDING.md](docs/BUILDING.md) also shows how the project is created from an
empty one, option by option.

## How it works, in one paragraph

A 100 Hz timer interrupt samples the four quadrature encoders, runs the new
**outer position loop** for the position motor (a proportional controller that
outputs a speed set point), then the vendor's **inner speed loop** for the four
motors, and finally refreshes the watchdog, but only if the RTOS has given signs of
life in the last half second. Commands arrive as `AA 55 | function | length |
payload | CRC-8` frames at 1 Mbaud; the new function `0x0B` carries the position
commands and answers each one with a 20-byte status report.
[docs/HOW_IT_WORKS.md](docs/HOW_IT_WORKS.md) walks through the code.

## Repository layout

```
firmware/        the STM32CubeIDE project (.project, .cproject) and all its sources
  Core/          CubeMX-generated start-up code, interrupt handlers, peripherals
  Hiwonder/      the vendor's application: motors, IMU, protocol, UI   <- the changes are here
  Drivers/ Middlewares/ Third_Party/ USB_HOST/     HAL, FreeRTOS, LVGL, U8g2, ...
  compat/        two forwarding headers so the tree also builds on case-sensitive file systems
  STM32F407VETX_FLASH.ld      linker script of the validated image
  RosRobotControllerM4.ioc    the vendor's CubeMX file, kept as the pin and clock reference
tools/
  build_reproducible.py       rebuilds the validated image and compares its SHA-256
  link-order.txt  include-dirs.txt     the source list, link order and include order it uses
  motor_ext.py                host tool for the MOTOR_EXT protocol (Linux)
patches/         the same changes as one patch against the vendor's official V1.2 source
docs/            building, flashing, protocol, how it works, how it was made
```

`firmware/` holds the 276 source files that make the image, their headers, the
license files of the third-party components and two stub files that only the IDE
build compiles (see [docs/BUILDING.md](docs/BUILDING.md)). The history of this
repository starts with the vendor's unmodified files, so
`git diff` between the first two commits is the whole extension.

## Safety notes

- Flash and debug with the **12 V supply disconnected**. If the CPU is halted while
  a motor is powered, the timers keep their last PWM value and the motor keeps
  turning; the motor switch does not help, because it is read by the halted code.
- The motor switch on the board (pin PD3) is read by firmware. It is not a power
  cut.
- The watchdog leaves a window of about 0.25 s between a hang and the reset.
- The low-battery alarm of the vendor firmware is disabled in its configuration
  and this firmware does not change that: there is no discharge protection.
- Do not change option bytes or read-out protection. Nothing here needs it.

## Known limitations

- Position control exists for one motor only.
- Short moves overshoot by about 10 %; a 90° correction in HOLD overshot by 22°.
  There is no acceleration profile.
- The vendor's speed loop behaves as an integral plus derivative controller, the
  PWM runs at 200 Hz and values under 25 % are forced to zero. Below about
  3 rev/s with no load the speed oscillates around the dead band.
- Function number `0x0B` is also used for RGB LEDs in the numbering of newer
  Hiwonder software. Renumbering `MOTOR_EXT` is pending.
- The IMU frame carries acceleration in m/s² and angular rate in rad/s.

## Licensing

This repository redistributes source code from several owners:

- **Hiwonder** wrote the application in `firmware/Hiwonder/` and the project it
  belongs to. It is published by Hiwonder for this board **without a license
  file**; no license for it is granted here.
- The third-party components (STM32 HAL and CMSIS, FreeRTOS, LVGL, U8g2, Fusion,
  SEGGER RTT, LwMEM/LwRB/LwPKT/LwSHELL) keep their own licenses. See
  [THIRD_PARTY.md](THIRD_PARTY.md).
- The extension itself is the difference between the first two commits, also
  available as `patches/extended-firmware.patch`.

This project is not affiliated with or endorsed by Hiwonder or STMicroelectronics.
