# How the firmware works

This page follows a command from the USB cable to the motor and back, and points
at the file where each step lives. Paths are relative to `firmware/`. The parts
added by the extended firmware carry comments that start with
`Extended firmware` or `EXTENDED FIRMWARE`, so `grep -ri "extended firmware"`
lists all of them.

## The big picture

```
 host  --USB-C "UART2"-->  USART3, 1 Mbaud  --DMA-->  ring buffer  -->  packet_rx_task
                                                                            |
                                                             frame parser (AA 55, CRC-8)
                                                                            |
                                      handlers in Hiwonder/System/packet_handle.c
                                         0x03 MOTOR       speed set points, stops
                                         0x0B MOTOR_EXT   position commands      <- new
                                                                            |
                                                          fields of the motor object
                                                                            |
 TIM7 interrupt, 100 Hz  (Core/Src/stm32f4xx_it.c)                          v
   1. encoder_update()                  x4   read the quadrature counters, compute rev/s
   2. encoder_motor_position_control()  x1   position error -> speed set point  <- new
   3. encoder_motor_control()           x4   speed loop -> PWM
   4. watchdog_isr_kick()                    refresh the IWDG if the RTOS is alive  <- new
```

The MCU is an STM32F407VET6 at 168 MHz (16 MHz crystal) running FreeRTOS through
the CMSIS-RTOS2 layer. The RTOS tasks handle communication, the display, the IMU
and the gamepad; **motor control does not run in a task**. It runs in the TIM7
interrupt, so its timing does not depend on what the tasks are doing.

## Encoders and the vendor's speed loop

Each motor port has a timer in quadrature-encoder mode (M1 to M4: TIM5, TIM2,
TIM4, TIM3). The counters count every edge of both channels and wrap at 60000;
an update interrupt keeps track of the wraps, so `counter` in the motor object is
a 64-bit position in ticks.

`encoder_update()` (in `Hiwonder/Peripherals/encoder_motor.c`) turns the change
of that position over 10 ms into a speed, lightly filtered, in turns per second.

`encoder_motor_control()` is the vendor's speed loop and it has three properties
that shape everything built on top of it:

- It is **incremental**: each cycle the controller output is *added* to the
  previous PWM value (`pulse = current_pulse + output`). If the speed feedback is
  missing, a zero set point produces zero error, zero output and therefore an
  unchanged PWM: the motor keeps turning.
- PWM values under 250 out of 1000 are sent as zero, because the motor only hums
  there. This **dead band** is why very low speeds oscillate.
- It only runs while the motor switch (pin PD3) is on. With the switch off the
  PWM is zero.

The PWM frequency is 200 Hz with 1000 steps. M4 is driven by TIM10 and TIM11.

## Position control (new)

Position control is a **cascade**: a new outer loop decides the speed, and the
vendor's speed loop, untouched, produces the PWM.

```
position error [ticks] --x KP--> speed set point [rev/s] --speed loop--> PWM
```

`encoder_motor_position_control()` runs once per cycle for the position motor,
between the encoder sampling and the speed loop:

```c
error = self->position_target - self->counter;
...
speed = position_speed_command(self, error);   /* KP * error, limited, with a minimum */
self->pid_controller.set_point = speed;
```

`position_speed_command()` is a proportional law with two limits: never faster
than the `max_rps` of the command, and never slower than 0.10 rev/s, because a
smaller request would not get the PWM out of the dead band and the shaft would
stop short of the target.

The gain and that minimum were tuned on the bench. With a lower gain (0.004) and
a higher minimum (0.20) the last 50 ticks of every move were driven at constant
speed and a 110-tick move overshot by 100 %. With the present values (0.006 and
0.10) the same move overshoots by about 10 %.

### The life of a move

`MOVE_RELATIVE` calls `encoder_motor_move_relative()`. After checking every
parameter, it latches the target (`counter + delta`), the speed limit, the
tolerance and the timeout inside one critical section and sets the status to
`MOVING`. From then on the interrupt does the work:

- **inside the tolerance**: zero speed is requested, and once the shaft has also
  been at rest for 5 cycles the status becomes `REACHED`;
- **timeout**: the status becomes `TIMEOUT` and zero speed is requested;
- **otherwise**: keep driving towards the target.

The host follows the move by polling `STATUS`.

### HOLD

`encoder_motor_hold()` uses the same loop with the current position as the
target, no timeout and no `REACHED`. Two things are different:

- the PWM is capped at the `max_pulse` of the command, in the speed loop itself,
  so that a HOLD can never push with full force;
- **stall protection**: outside the tolerance, with the PWM at the cap and the
  shaft at rest for 300 cycles (3 s), the HOLD lets go: PWM to zero, status
  `STALLED`. A blocked wheel does not cook the motor or the driver.

### Stopping, for real

`encoder_motor_stop()` leaves any position mode and **zeroes the accumulated
PWM** and the controller state, instead of asking the speed loop to slow down.
It is what the vendor's stop commands now call. Given the incremental loop
described above, this is the only stop that works when the encoder is not
counting.

### Sharing data with an interrupt

The command functions are called from `packet_rx_task`; the control functions
run in the TIM7 interrupt. Every command therefore writes its fields with
interrupts disabled for a few instructions, saving and restoring `PRIMASK`.
`encoder_motor_position_snapshot()` reads a consistent copy the same way for the
status report.

### Which motor

`POSITION_MOTOR_INDEX` in `Hiwonder/System/global_conf.h` selects the motor
(3 = M4). The interrupt, the protocol handler and the motor profile all use it.

## The motor profile, and a trap

`apply_project_motor_config()` in `Hiwonder/Portings/motor_porting.c` sets
ticks per turn (1320), the speed limit (5.7 rev/s) and the speed-loop gains
(21, 0.9, 0.8) for M1 and for the position motor.

It has to be called **after** `set_chassis_type()`. That function, called at the
end of start-up, reconfigures all four motors with the profile of the selected
chassis and silently overwrites anything set earlier. Values written only in
`motors_init()` were dead code.

The same file contains a one-line fix of the vendor code: the encoder timer of M4
was started with the handle of M3's timer.

## The protocol handler (new)

`packet_motor_ext_handle()` in `Hiwonder/System/packet_handle.c` is registered
for function `0x0B`. The request and reply structs are packed, so their layout
is the wire format; [PROTOCOL.md](PROTOCOL.md) lists every byte. The handler
refuses anything it does not fully understand: a wrong length is dropped, a wrong
motor or an out-of-range value is answered with status 5 and changes nothing.

`Hiwonder/Misc/packet.h` gives the new function its number. `PACKET_FUNC_NONE`
is both the size of the handler table and the limit the parser checks, so it
moved up with it.

## IMU telemetry

`imu_task` in `Hiwonder/Portings/imu_porting.c` waits for the data-ready
interrupt of the QMI8658, reads acceleration and angular rate and now sends them
to the host with `packet_transmit()`, at most once every 50 ms. The task already
loops at about 20 Hz; the explicit guard keeps the limit true if that ever
changes, so the 1 Mbaud link cannot be flooded.

## The watchdog (new)

The board has no external watchdog, and the vendor firmware has two ways of
hanging with the PWM outputs frozen: its fault and NMI handlers are endless
loops. `Hiwonder/System/app.c` starts the STM32's **independent watchdog**, which
runs from its own internal oscillator and therefore survives a failure of the
main clock.

It is refreshed from two places that have to agree:

```c
/* app task, every 100 ms */               /* TIM7 interrupt, every 10 ms */
watchdog_heartbeat_age = 0;                if (watchdog_heartbeat_age < 50) {
                                               ++watchdog_heartbeat_age;
                                               IWDG->KR = 0xAAAA;   /* refresh */
                                           }
```

- If the **interrupts** stop (a fault handler, the NMI), nobody refreshes and the
  watchdog resets the MCU within about 250 ms.
- If the **RTOS** stops scheduling, the age reaches 50 (half a second), the
  interrupt stops refreshing, and the reset follows.

The reset switches the PWM outputs off. The watchdog is frozen while a debugger
halts the core.

## Where each change is

| File | Change |
|---|---|
| `Core/Src/stm32f4xx_it.c` | position loop and watchdog refresh in the TIM7 interrupt |
| `Hiwonder/Peripherals/encoder_motor.c`, `.h` | position loop, HOLD, hard stop, status snapshot |
| `Hiwonder/System/packet_handle.c` | `MOTOR_EXT` handler; bounds checks and hard stop in `MOTOR` |
| `Hiwonder/Misc/packet.h` | function number `0x0B` |
| `Hiwonder/System/global_conf.h` | `POSITION_MOTOR_INDEX` |
| `Hiwonder/Portings/motor_porting.c`, `motors_param.h` | motor profile, speed limit, M4 encoder fix |
| `Hiwonder/System/app.c`, `global.h` | watchdog; motor profile applied after the chassis type |
| `Hiwonder/Portings/imu_porting.c` | IMU telemetry at 20 Hz |
| `Hiwonder/USB_HOST/usbh_hiwonder_hid.c` | a `NULL` passed as a timeout, rejected by current GCC |
