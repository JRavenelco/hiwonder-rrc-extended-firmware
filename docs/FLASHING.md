# Flashing the board

The board is flashed over **SWD** with an ST-LINK. This is the only method used
and validated in this project; the reasons are at the end of this page.

## What you need

- An ST-LINK/V2 or compatible probe, and three jumper wires.
- A USB-C cable in the port labelled **UART2**, to power the logic side of the
  board from the computer.
- STM32CubeProgrammer (x86-64 hosts) or OpenOCD (any host, including a
  Raspberry Pi).
- The image: `RosRobotControllerM4-extended-m4-pos-v8.hex`, SHA-256
  `3a4d9e885ac408b11da56e7263b9e94c843f1b7bbbbf399f21cc93ab4a23f1e8`.
  Check it before flashing:

  ```bash
  sha256sum RosRobotControllerM4-extended-m4-pos-v8.hex
  ```

**Disconnect the 12 V supply and leave it disconnected until the new firmware has
answered.** With only USB power the battery telemetry reads about 4.1 V, which is
an easy way to confirm it.

## Wiring: header H1

H1 is the 2x13 pin header. Only three pins are needed when the board is powered
over USB:

| H1 pin | Signal | Connect to the ST-LINK |
|---|---|---|
| 3 | PA14 / SWCLK | SWCLK |
| 5 | GND | GND |
| 7 | PA13 / SWDIO | SWDIO |
| 1 | 3V3 | only if your probe needs a target-voltage reference; never feed the board from here |

NRST is not on H1. The tools below reset the MCU through SWD (SYSRESETREQ), so it
is not needed. Never connect the 5 V pin of the probe.

## 1. Back up what is on the board

The flash is 512 KiB starting at `0x08000000`. Read all of it and keep the file:
it is your way back to the factory firmware.

```bash
# STM32CubeProgrammer
STM32_Programmer_CLI -c port=SWD -u 0x08000000 0x80000 factory-backup.bin

# OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c init -c "dump_image factory-backup.bin 0x08000000 0x80000" -c exit
```

```bash
sha256sum factory-backup.bin
```

## 2. Write the image

Close every program that has the board's serial ports open, then:

```bash
# STM32CubeProgrammer: write, verify, reset
STM32_Programmer_CLI -c port=SWD -w RosRobotControllerM4-extended-m4-pos-v8.hex -v -rst

# OpenOCD: write, verify, reset
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program RosRobotControllerM4-extended-m4-pos-v8.hex verify reset exit"
```

The OpenOCD line is the one used on the bench (OpenOCD 0.12 on a Raspberry Pi 5,
ST-LINK/V2 firmware V2J37S7); it ends with `** Verified OK **`.

### Checking the flash contents afterwards

If you read the flash back and compare it with the image, convert the HEX with
`0xFF` as filler first:

```bash
arm-none-eabi-objcopy -I ihex -O binary --gap-fill 0xff \
    RosRobotControllerM4-extended-m4-pos-v8.hex image.bin
```

The image has an 8-byte alignment gap at `0x08000188..0x0800018F`. Erased flash
reads `0xFF` there, while `objcopy` fills gaps with `0x00` by default, so a naive
comparison reports a difference that is not real.

## 3. Check that the firmware answers

Still without 12 V:

```bash
python3 tools/motor_ext.py --caps      # MOTOR_EXT v3: position control on M4, 1320 ticks per turn
python3 tools/motor_ext.py --imu 5     # about 20 frames per second
python3 tools/motor_ext.py --status    # state idle, speed 0
```

Only then connect the 12 V supply, with the wheel free to turn and the motor
switch within reach, and try a small move (see [PROTOCOL.md](PROTOCOL.md)).

After a test you can ask the MCU why it last reset: bit 29 of `RCC_CSR`
(`0x40023874`) is set after a watchdog reset.

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c init -c "mdw 0x40023874" -c exit
```

## Going back

Write the backup to the same address:

```bash
STM32_Programmer_CLI -c port=SWD -w factory-backup.bin 0x08000000 -v -rst

openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program factory-backup.bin 0x08000000 verify reset exit"
```

## If the board stops answering

Before blaming the image, reset the MCU without rewriting anything:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c init -c "reset run" -c exit
```

Two things looked like "dead firmware" during this project and were not:

1. **The board parked in the ROM bootloader.** The other USB-C port (UART1) has
   an auto-boot circuit driven by the DTR and RTS lines of its USB bridge. Opening
   or closing that port with the wrong line states leaves the MCU in reset or in
   the bootloader, and then every image seems mute. Use the UART2 port for the
   protocol and SWD for flashing, and the circuit never comes into play.
2. **A clock failure.** The vendor code enables the clock security system and its
   NMI handler is an endless loop, so a glitch of the crystal froze the board with
   the PWM outputs stuck. This is what the independent watchdog of the extended
   firmware is for.

## Why SWD only

- It does not depend on the DTR/RTS circuit described above.
- It can read memory while the firmware runs, which is how most of the bench
  measurements in this project were taken.
- Nothing here needs option bytes, read-out protection or the boot pins. Leave
  them alone.

## Debugging with the motors powered

Do not halt the CPU (breakpoints, single stepping) while the 12 V supply is
connected. The control loop stops but the timers keep their last PWM value, and
M4 is driven by two timers that can stay high. The motor switch is read by the
halted code, so it does not protect you. With a powered motor, observe by reading
RAM without halting. The independent watchdog is configured to freeze while a
debugger halts the core, so debugging without 12 V works normally.
