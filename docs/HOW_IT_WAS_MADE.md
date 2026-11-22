# How it was made

A record of the path from the vendor's source to this repository: what was done,
in which order, what went wrong and how each result was checked. It is here so
that the next person does not have to rediscover it.

## 1. Starting point

The base is Hiwonder's official source for the ROS Robot Controller V1.2, a Keil
MDK (Arm Compiler) project generated with STM32CubeMX: HAL drivers, FreeRTOS,
LVGL and U8g2 for the display, and Hiwonder's application on top. It was kept
unmodified in a separate tree throughout; every change was made in a copy, so
that the whole extension is always available as one diff
(`patches/extended-firmware.patch`, and the first two commits of this
repository).

## 2. Building it with GCC

The extension was built with the GNU toolchain, not with Keil. The Keil project
file was treated as the source of truth: a script reads from it the exact list of
source files and include paths and compiles them with the Arm GNU Toolchain,
swapping the three files that are compiler-specific:

| Keil build | GCC build |
|---|---|
| `MDK-ARM/startup_stm32f407xx.s` | `Core/Startup/startup_stm32f407vetx.s` |
| FreeRTOS port `portable/RVDS/ARM_CM4F` | `portable/GCC/ARM_CM4F` |
| `Hiwonder/Misc/syscall.c` (Arm C library retargeting) | newlib-nano with `nosys.specs` |

Options: `-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Os`,
`-ffunction-sections -fdata-sections` with `--gc-sections`, plus
`-fno-strict-aliasing`, because the code reads byte buffers through struct
pointers the way Arm Compiler allows, and `-fcommon`, kept from the first GCC
build (the image links without it, but its bytes change).

Two details mattered more than expected:

- **Link order.** The first GCC builds were made on Windows and their object
  order came from a culture-aware sort that Python does not reproduce. The order
  was therefore frozen as data (`tools/link-order.txt`). With it, the Linux build
  reproduced the Windows-built image bit for bit, and every image since then
  comes out identical on an x86-64 machine and on an aarch64 Raspberry Pi.
- **Case sensitivity.** The tree comes from a case-insensitive file system. Two
  includes (`lvgl/lvgl.h` and `SBus.h`) do not match the names on disk; two
  forwarding headers in `firmware/compat/` solve it without touching vendor
  files.

One vendor file needed a change just to compile: `usbh_hiwonder_hid.c` passes
`NULL` where a `uint32_t` timeout is expected, which GCC 14 and later reject.

## 3. The image that seemed dead

The first GCC image was flashed through the ROM bootloader and the board stopped
answering. The image was rejected, the factory backup restored, and a static
comparison of the two binaries started. The cause turned out not to be in the
firmware at all: the helper used to enter the bootloader drove the DTR and RTS
lines of the USB bridge with inverted polarity, and the serial driver moves those
lines again whenever a port is opened or closed. The board was simply parked in
reset or in the bootloader, and any firmware would have looked mute.

Consequences that became rules:

- flash and reset only over SWD, with the serial ports closed;
- take a full flash backup before the first write and prove it can be restored;
- before blaming an image, reset the MCU and listen.

## 4. Working on a live board without stopping it

Halting a motor controller under power is dangerous (the timers keep their last
PWM), so nearly all observations were made by **reading RAM over SWD while the
firmware runs**: the motor objects, the tick counter, timer registers, the fault
status registers. The addresses come from the linker map of the exact image on
the board, which is one more reason for reproducible builds.

Every step on hardware followed the same gate: identify the file and its
SHA-256, flash with the 12 V supply disconnected, verify by reading back, check
communication, and only then power the motor with the wheel free and the motor
switch at hand.

## 5. Position control, step by step

1. **IMU telemetry first.** The first image validated on the board differed from
   the base only in the rate-limited IMU report: something harmless that proves
   the whole chain of build, flash, protocol and host.
2. **`MOTOR_EXT` with position control on M1.** The first tuning (gain 0.004,
   minimum speed 0.20 rev/s) overshot a 110-tick move by 100 %. Logging position
   against time showed why: below about 50 ticks of error the proportional speed
   fell under the minimum, so the end of every move was driven at constant speed.
   Raising the gain to 0.006 and halving the minimum brought the overshoot to
   about 10 %.
3. **Ticks per turn confirmed physically**: with a mark on the shaft, 1320 ticks
   brought it back to the same place, repeatably within 8 ticks. The profile the
   firmware applied by default assumed 3960.
4. **The stop that did not stop.** One motor turned out to have a dead encoder,
   and with it the vendor's stop command left the motor turning. Reading the code
   explained it (the speed loop is incremental) and `encoder_motor_stop()` was
   written.
5. **The configuration that did nothing.** Motor parameters set during
   initialisation had no effect: `set_chassis_type()` overwrote them later. They
   are now applied after it.
6. **Moving to M4.** The position motor became a `#define`. Reading TIM3's
   registers on the board showed its capture channels disabled: the vendor code
   started the wrong timer handle for the M4 encoder.
7. **Using all the speed.** The limit inherited from the vendor profile was
   3.0 rev/s. The top speed was measured (5.76 rev/s with no load at 12 V) and
   the limit raised to 5.7.

## 6. The hang that justified a watchdog

At the end of one bench run the board went silent with the motor loop dead. It
was examined over SWD without resetting it: the core was inside the NMI handler,
the tick counter was frozen, and the clock registers showed the external
oscillator and the PLL off. The vendor firmware enables the clock security
system, and its NMI handler is an endless loop, so a momentary failure of the
crystal froze everything with the PWM outputs holding their last value.

The independent watchdog was added in response and tested the same way it
failed: an NMI was forced by writing the pending bit over SWD, and the board
reset by itself with the watchdog flag set. A minute of heavy traffic produced no
false reset.

## 7. HOLD

Requested so that the robot could brake like a LEGO SPIKE motor does. It reuses
the position loop, adds a PWM cap and a stall timer. It was tested without
touching the wheel: the target was moved 90° by writing it over SWD, which is
equivalent to pushing the shaft, and a blocked shaft was simulated by stopping
the encoder timer. The fields it needs were appended at the end of the motor
struct so that the offsets used by the RAM-reading tools did not move.

## 8. This repository

- The comments of the extension were rewritten in English. Since only comments
  changed, the check was absolute: the rebuilt image had to be, and is, identical
  byte for byte to the one validated on the bench.
- The tree was reduced to the 276 sources that make the image, their headers,
  the license files and the two stub files the IDE build needs. The reproducible
  build still yields the same SHA-256 from it.
- The STM32CubeIDE project started from the IDE's own empty-project wizard for
  the STM32F407VETx; its include paths, symbols and options were then set to
  those of the validated build. The IDE's compiler is a different version, so its
  output is not bit-identical; it was compared with the validated image instead:
  the same program symbols apart from the stubs, and the same interrupt vector
  table by handler name.
- One difference between the two builds had to be accepted. The validated image
  takes the C library's stub system calls from the toolchain (`nosys.specs`). In
  the IDE that makes the linker print six notes, which STM32CubeIDE's linker
  error parser turns into six errors and a *Build Failed* on a build that worked.
  Unticking that parser in the project properties has no effect, because the
  linker tool contributes it again. The IDE project therefore compiles the two
  stub files that the vendor's tree already had (`syscalls.c`, `sysmem.c`), which
  is what every STM32CubeIDE project does; the reproducible build keeps ignoring
  them. An earlier draft of this project claimed those files were unsafe because
  `_write` calls an undefined weak function. That was wrong: on Arm, a call to an
  undefined weak function is linked as a no-op.

## Image history

| Image | Date | What changed |
|---|---|---|
| `pos-v4` | 2026-08-22 | position control on M1, IMU at 20 Hz, hard stop; first image validated with a motor |
| `pos-v5` | 2026-10-04 | position control on M4, M4 encoder fix, bounds checks in the vendor motor commands |
| `pos-v6` | 2026-10-04 | speed limit 5.7 rev/s |
| `pos-v7` | 2026-10-04 | independent watchdog |
| `pos-v8` | 2026-10-04 | position HOLD with stall protection; the image in this repository |
