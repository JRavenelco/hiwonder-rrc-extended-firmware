/**
 * @file encoder_motor.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 编码器电机硬件无关代码
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */


#include "encoder_motor.h"
#include "gpio.h"

/* ---------------------------------------------------------------------------
 * EXTENDED FIRMWARE - position control on top of the vendor's speed control
 *
 * The vendor firmware only offers closed-loop SPEED control. This file adds a
 * closed-loop POSITION mode as a cascade:
 *
 *   position error [ticks] --P--> speed set point [rev/s] --vendor PID--> PWM
 *
 *   - Outer loop: encoder_motor_position_control(), a proportional controller
 *     called from the TIM7 interrupt at 100 Hz, right after the encoders are
 *     sampled. Its output is pid_controller.set_point and nothing else.
 *   - Inner loop: the vendor's encoder_motor_control(), untouched except for
 *     the PWM cap used by HOLD. It is an INCREMENTAL controller
 *     (pulse += PID output), which is why a plain "set point = 0" does not
 *     stop a motor that has no valid feedback; see encoder_motor_stop().
 *
 * Two position modes share the outer loop:
 *   MOVE_RELATIVE  go to counter + delta; report REACHED once the error stays
 *                  inside the tolerance with the shaft at rest, or TIMEOUT.
 *   HOLD           keep the current counter indefinitely with a limited PWM,
 *                  and let go (STALLED) if the shaft is blocked for 3 s.
 *
 * Concurrency: the control functions run inside the TIM7 ISR, while the
 * commands arrive from the packet task. Every command therefore updates the
 * shared fields inside a short critical section (PRIMASK is saved and
 * restored, so the functions are also safe with interrupts already off).
 * ------------------------------------------------------------------------- */
#define POSITION_LOOP_HZ             100u   /* rate of the TIM7 control tick */
/* Outer position loop (P) that outputs a speed set point.
   Bench result, 2026-08-01, with KP=0.004 / MIN=0.20: 100 % overshoot
   (target 110 ticks, peak 219). Cause: with a low KP the proportional speed
   falls below the minimum once the error is under ~50 ticks, so the last
   stretch is driven at CONSTANT speed, with no braking at all.
   The minimum exists to get past the PWM dead band (250/1000), but applied
   close to the target it guarantees an overshoot. With the values below
   (bench, 2026-08-22) the same 110-tick move overshoots by about 10 %. */
#define POSITION_DEFAULT_KP          0.006f /* rev/s commanded per tick of error */
#define POSITION_MIN_COMMAND_RPS     0.10f  /* smallest non-zero speed command */
#define POSITION_STOP_RPS            0.05f  /* below this the shaft counts as at rest */
#define POSITION_REQUIRED_STABLE     5u     /* cycles in tolerance and at rest -> REACHED */
/* HOLD: the PWM cap must be above the dead band (250) or it could never push back. */
#define HOLD_MIN_PULSE               300
#define HOLD_STALL_LIMIT_CYCLES      300u  /* 3 s at the cap without moving -> release */

/* Saturates a 64-bit tick count to the int32 range used on the wire. */
static int64_t clamp_i64_to_i32(int64_t value)
{
    if(value > 2147483647LL) return 2147483647LL;
    if(value < -2147483647LL - 1LL) return -2147483647LL - 1LL;
    return value;
}

/**
 * @brief 编码器电机速度测量更新
 * @detials
 * @param self 编码器电机对象指针
 * @param period 当前更新距离上次更新的时间间隔(更新周期), 单位 sec
 * @param counter 编码器当前计数值
 * @retval None.
*/
void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t counter)
{
    counter = counter + self->overflow_num * self->ticks_overflow; /* 总的计数值, 60000 根据实际设置的定时器溢出值 */
    int delta_count = counter - self->counter;
    self->counter = counter; /* 存储新的计数值 */
    self->tps = (float)delta_count / period * 0.9f + self->tps * 0.1f; /* 计算脉冲频率 */
    self->rps = self->tps / self->ticks_per_circle; /* 计算转速 单位rps, 转每秒 */
}

/**
 * @brief 编码器电机速度控制任务
 * @detials 编码器电机速度PID控制任务,需要定时指定以完成PID控制更新
 * @param self 编码器电机对象指针
 * @param period 当前更新距离上次更新的时间间隔(更新周期), 单位 sec
 * @retval None.
*/
void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period)
{
    float pulse = 0;
    if(!HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin)) {
    pid_controller_update(&self->pid_controller, self->rps, period);   /* 更新 PID控制器 */
        pulse = self->current_pulse + self->pid_controller.output; /* 计算新的 PWM 值 */

        /* 对输出的 PWM 值进行限幅, 限幅根据定时器的设置确定，本示例定时器设置的占空比 0-100 对应 0-1000 */
        pulse = pulse > 1000 ?  1000 : pulse;
        pulse = pulse < -1000 ? -1000 : pulse;
        if(self->position_hold) {   /* HOLD: its own, lower PWM cap */
            pulse = pulse > self->hold_max_pulse ? self->hold_max_pulse : pulse;
            pulse = pulse < -self->hold_max_pulse ? -self->hold_max_pulse : pulse;
        }
    }
    self->set_pulse(self, pulse > -250 && pulse < 250 ? 0 : pulse); /* 设置新的PWM值且限制 PWM 的最小值, PWM过小电机只会发出嗡嗡声而不动 */
    self->current_pulse = pulse; /* 记录新的 PWM 值 */
}
/**
 * @brief 编码器电机设置PID控制目标速度
 * @param self self 编码器电机对象指针
 * @param rps 目标速度， 单位转每秒
 * @retval None.
 */
void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps)
{
    /* Extended firmware: a plain speed command takes the motor out of any
       position mode (MOVE_RELATIVE or HOLD). */
    self->position_hold = 0;
    self->position_active = 0;
    self->position_status = ENCODER_POSITION_CANCELLED;
    rps = rps > self->rps_limit ? self->rps_limit : (rps < -self->rps_limit ? -self->rps_limit : rps); /* 对速度进行限幅 */
    self->pid_controller.set_point = rps; /* 设置 PID 控制器目标 */
}

/* Empties the incremental loop and forces the PWM to zero. Without this the
   PWM stays stuck at its last value whenever there is no valid feedback.
   Call it with interrupts disabled or from the TIM7 ISR. */
static void encoder_motor_zero_output(EncoderMotorObjectTypeDef *self)
{
    self->pid_controller.set_point = 0.0f;
    self->pid_controller.previous_0_err = 0.0f;
    self->pid_controller.previous_1_err = 0.0f;
    self->pid_controller.output = 0.0f;
    self->current_pulse = 0;
    if(NULL != self->set_pulse) {
        self->set_pulse(self, 0);
    }
}

/* Hard stop: leaves any position mode and zeroes the PWM at once. The header
   explains why a zero set point alone is not enough. */
void encoder_motor_stop(EncoderMotorObjectTypeDef *self)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    self->position_hold = 0;
    self->position_active = 0;
    self->position_status = ENCODER_POSITION_CANCELLED;
    encoder_motor_zero_output(self);
    if(!primask) __enable_irq();
}

/* Starts a relative move. The parameters are validated first and a rejected
   command changes nothing. The target is absolute (counter + delta) and is
   latched together with its limits inside one critical section, so the ISR
   never sees a half-written command. */
int encoder_motor_move_relative(EncoderMotorObjectTypeDef *self,
                                int32_t delta_ticks,
                                float max_rps,
                                int32_t tolerance_ticks,
                                uint32_t timeout_ms)
{
    uint32_t primask;
    if((delta_ticks == 0) || (max_rps <= 0.0f) ||
       (max_rps > self->rps_limit) || (tolerance_ticks < 1) ||
       (timeout_ms < 100u) || (timeout_ms > 60000u)) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    self->position_hold = 0;   /* a new move replaces an active HOLD */
    self->position_target = self->counter + delta_ticks;
    self->position_max_rps = max_rps;
    self->position_tolerance_ticks = tolerance_ticks;
    self->position_elapsed_cycles = 0;
    self->position_timeout_cycles = (timeout_ms * POSITION_LOOP_HZ + 999u) / 1000u; /* ms -> control ticks, rounded up */
    self->position_stable_cycles = 0;
    self->position_status = ENCODER_POSITION_MOVING;
    self->position_active = 1;
    if(!primask) __enable_irq();
    return 0;
}

/* Starts a HOLD at the current counter value. max_pulse caps the PWM while
   holding; it must be above the dead band (HOLD_MIN_PULSE) or the motor could
   not push back at all. */
int encoder_motor_hold(EncoderMotorObjectTypeDef *self,
                       float max_rps,
                       int32_t tolerance_ticks,
                       int max_pulse)
{
    uint32_t primask;
    if((max_rps <= 0.0f) || (max_rps > self->rps_limit) || (tolerance_ticks < 1) ||
       (max_pulse < HOLD_MIN_PULSE) || (max_pulse > 1000)) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    self->position_target = self->counter;
    self->position_max_rps = max_rps;
    self->position_tolerance_ticks = tolerance_ticks;
    self->position_stable_cycles = 0;
    self->position_elapsed_cycles = 0;
    self->hold_max_pulse = (int16_t)max_pulse;
    self->hold_stall_cycles = 0;
    self->position_hold = 1;
    self->position_status = ENCODER_POSITION_HOLDING;
    self->position_active = 1;
    if(!primask) __enable_irq();
    return 0;
}

/* The control law shared by MOVE_RELATIVE and HOLD: proportional to the
   error, limited to +-position_max_rps and never slower than
   POSITION_MIN_COMMAND_RPS, so that the request always clears the dead band. */
static float position_speed_command(EncoderMotorObjectTypeDef *self, int64_t error)
{
    float speed = (float)error * self->position_kp;
    if(speed > self->position_max_rps) speed = self->position_max_rps;
    if(speed < -self->position_max_rps) speed = -self->position_max_rps;
    if((speed > 0.0f) && (speed < POSITION_MIN_COMMAND_RPS)) speed = POSITION_MIN_COMMAND_RPS;
    if((speed < 0.0f) && (speed > -POSITION_MIN_COMMAND_RPS)) speed = -POSITION_MIN_COMMAND_RPS;
    return speed;
}

/* Outer position loop. Runs in the TIM7 ISR at 100 Hz, after the encoders
   have been sampled and before the speed loop. It never drives the PWM itself
   (except to release a stalled HOLD): its output is the speed set point of
   the vendor's PID. */
void encoder_motor_position_control(EncoderMotorObjectTypeDef *self)
{
    int64_t error;
    float speed;

    if(!self->position_active) return;

    error = self->position_target - self->counter;

    if(self->position_hold) {
        /* HOLD: no timeout and no "reached". Blocked (PWM at the cap, shaft at
           rest and still outside the tolerance) for 3 s -> release, PWM to zero. */
        const int outside = (error > self->position_tolerance_ticks) ||
                            (error < -self->position_tolerance_ticks);
        const int at_cap = (self->current_pulse >= self->hold_max_pulse) ||
                           (self->current_pulse <= -self->hold_max_pulse);
        if(outside && at_cap &&
           (self->rps <= POSITION_STOP_RPS) && (self->rps >= -POSITION_STOP_RPS)) {
            if(++self->hold_stall_cycles >= HOLD_STALL_LIMIT_CYCLES) {
                self->position_hold = 0;
                self->position_active = 0;
                self->position_status = ENCODER_POSITION_STALLED;
                encoder_motor_zero_output(self);
                return;
            }
        } else {
            self->hold_stall_cycles = 0;
        }
        self->pid_controller.set_point = outside ? position_speed_command(self, error) : 0.0f;
        return;
    }

    /* MOVE_RELATIVE. Inside the tolerance: ask for zero speed and report
       REACHED only after the shaft has also been at rest for
       POSITION_REQUIRED_STABLE consecutive cycles (50 ms). */
    if((error <= self->position_tolerance_ticks) &&
       (error >= -self->position_tolerance_ticks)) {
        self->pid_controller.set_point = 0.0f;
        if((self->rps <= POSITION_STOP_RPS) && (self->rps >= -POSITION_STOP_RPS)) {
            if(++self->position_stable_cycles >= POSITION_REQUIRED_STABLE) {
                self->position_active = 0;
                self->position_status = ENCODER_POSITION_REACHED;
                return;
            }
        } else {
            self->position_stable_cycles = 0;
        }
    } else {
        self->position_stable_cycles = 0;
    }

    /* Give up when the timeout requested by the host expires. */
    if(++self->position_elapsed_cycles >= self->position_timeout_cycles) {
        self->position_active = 0;
        self->position_status = ENCODER_POSITION_TIMEOUT;
        self->pid_controller.set_point = 0.0f;
        return;
    }

    if((error <= self->position_tolerance_ticks) &&
       (error >= -self->position_tolerance_ticks)) return;

    /* Still outside the tolerance: keep driving towards the target. */
    speed = position_speed_command(self, error);
    self->pid_controller.set_point = speed;
}

/* Leaves the position mode; the speed loop then brings the motor to rest.
   Use encoder_motor_stop() instead when the feedback cannot be trusted. */
void encoder_motor_position_cancel(EncoderMotorObjectTypeDef *self)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    self->position_hold = 0;
    self->position_active = 0;
    self->position_status = ENCODER_POSITION_CANCELLED;
    self->pid_controller.set_point = 0.0f;
    if(!primask) __enable_irq();
}

/* The current position becomes the origin of the positions reported to the
   host. The control itself always works on the raw counter. */
void encoder_motor_position_zero(EncoderMotorObjectTypeDef *self)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    self->position_zero = self->counter;
    if(!primask) __enable_irq();
}

/* Coherent copy of the state for a MOTOR_EXT report, taken with interrupts
   disabled. Positions are relative to the origin and saturate at int32. */
void encoder_motor_position_snapshot(EncoderMotorObjectTypeDef *self,
                                     int32_t *position_ticks,
                                     int32_t *target_ticks,
                                     int32_t *error_ticks,
                                     float *rps,
                                     uint8_t *active,
                                     uint8_t *status)
{
    uint32_t primask = __get_PRIMASK();
    int64_t position;
    int64_t target;
    __disable_irq();
    position = self->counter - self->position_zero;
    target = self->position_target - self->position_zero;
    *position_ticks = (int32_t)clamp_i64_to_i32(position);
    *target_ticks = (int32_t)clamp_i64_to_i32(target);
    *error_ticks = (int32_t)clamp_i64_to_i32(self->position_target - self->counter);
    *rps = self->rps;
    *active = self->position_active;
    *status = self->position_status;
    if(!primask) __enable_irq();
}


/**
 * @breif 编码器电机对象初始化
 * @param self 编码器电机对象指针
 * @retval None.
*/
void encoder_motor_object_init(EncoderMotorObjectTypeDef *self)
{
    self->counter = 0;
    self->overflow_num = 0;
    self->tps = 0;
    self->rps = 0;
    self->current_pulse = 0;
    self->ticks_overflow = 0;
    /* Extended firmware: position mode off, with safe defaults. */
    self->position_active = 0;
    self->position_status = ENCODER_POSITION_IDLE;
    self->position_stable_cycles = 0;
    self->position_zero = 0;
    self->position_target = 0;
    self->position_max_rps = 1.0f;
    self->position_kp = POSITION_DEFAULT_KP;
    self->position_tolerance_ticks = 8;
    self->position_elapsed_cycles = 0;
    self->position_timeout_cycles = POSITION_LOOP_HZ;
    self->position_hold = 0;
    self->hold_max_pulse = 1000;
    self->hold_stall_cycles = 0;
    self->ticks_per_circle = 9999; /* 电机输出轴旋转一圈产生的计数个数, 根据电机实际情况填写 */
    pid_controller_init(&self->pid_controller, 0, 0, 0);
}
