/**
 * @file encoder_motor.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 编码器电机控制头文件
 * @version 0.1
 * @date 2023-05-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __ENCODER_MOTOR_H_
#define __ENCODER_MOTOR_H_

#include <stdint.h>
#include "pid.h"

typedef struct EncoderMotorObject EncoderMotorObjectTypeDef;

/* Extended firmware: state of the position mode. These numbers travel to the
   host unchanged, in the "status" byte of every MOTOR_EXT report. */
typedef enum {
    ENCODER_POSITION_IDLE = 0,      /* no position command since reset */
    ENCODER_POSITION_MOVING = 1,    /* MOVE_RELATIVE in progress */
    ENCODER_POSITION_REACHED = 2,   /* target reached and shaft at rest */
    ENCODER_POSITION_CANCELLED = 3, /* cancelled, stopped or replaced by a speed command */
    ENCODER_POSITION_TIMEOUT = 4,   /* the move did not finish within its timeout */
    /* 5 = invalid command: set by packet_handle.c in its reply, never by the motor */
    ENCODER_POSITION_HOLDING = 6,   /* HOLD active: keeping the position */
    ENCODER_POSITION_STALLED = 7,   /* HOLD released because the shaft was blocked (protection) */
} EncoderPositionStatusEnum;

/**
 * @brief 编码器电机对象结构体
*/
struct EncoderMotorObject{
    int64_t counter;        /**< @brief 总计数值, 64bit 认为不会溢出 */
    int64_t overflow_num;   /**< @brief 溢出计数 */
    int32_t ticks_overflow; /**< @brief 计数溢出值 */
    float tps;              /**< @brief ticks per second 计数器频率 */
    float rps;              /**< @brief revolutions per second 输出轴转速 转每秒 */
    int current_pulse;      /**< @brief 当前输出的PWM值, 有符号对应正反转 */
    PID_ControllerTypeDef pid_controller; /**< @brief PID 控制器 */

    /* Optional outer position loop. The velocity PID remains the inner loop.
       Written by the packet task inside critical sections, used by the ISR. */
    volatile uint8_t position_active;   /* 1 while MOVE_RELATIVE or HOLD is running */
    volatile uint8_t position_status;   /* EncoderPositionStatusEnum */
    uint8_t position_stable_cycles;     /* consecutive cycles in tolerance and at rest */
    int64_t position_zero;              /* counter value reported to the host as 0 */
    int64_t position_target;            /* absolute target, in raw counter ticks */
    float position_max_rps;             /* speed limit of the current command, rev/s */
    float position_kp;                  /* P gain: rev/s per tick of error */
    int32_t position_tolerance_ticks;   /* +- window around the target */
    uint32_t position_elapsed_cycles;   /* 100 Hz ticks since the move started */
    uint32_t position_timeout_cycles;   /* timeout of the move, in 100 Hz ticks */

    /** porting 可移植硬件接口 **/
    int32_t ticks_per_circle; /**< @brief 电机输出轴旋转一圈产生的计数个数, 根据电机实际情况填写 */
	float rps_limit;  /**< @brief 电机地转速极限，一般会取比100% PWM占空比时地转速稍小的值以确保PID控制器对速度的控制 */
    /**
      * @brief 设置电机速度  -1000 ~ 1000
      * @param self 编码器电机对象
      * @param pulse 新的PWM值, -1000~1000, 对应正反转0~100%转速
      * @retval None.
      */
    void (*set_pulse)(EncoderMotorObjectTypeDef *self, int pulse);

    /* Position HOLD (MOTOR_EXT 0x05). Kept at the end of the struct so that the
       offsets of the older fields, which the SWD diagnostic tools read straight
       from RAM, do not move. */
    volatile uint8_t position_hold;
    int16_t hold_max_pulse;     /* PWM cap while holding (300..1000) */
    uint16_t hold_stall_cycles; /* consecutive cycles at the cap without moving */
};


/**
 * @breif 编码器电机对象初始化
 * @param self 编码器电机对象指针
 * @retval None.
*/
void encoder_motor_object_init(EncoderMotorObjectTypeDef *self);


/**
 * @brief 编码器电机速度测量更新
 * @detials
 * @param self 编码器电机对象
 * @param period 本次调用与上次调用的时间间隔，单位为秒
 * @param counter 编码器当前计数值
 * @retval None.
*/
void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t new_counter);


/**
 * @brief 编码器电机速度控制任务
 * @detials 编码器电机速度PID控制任务,需要定时指定以完成PID控制更新
 * @param self 编码器电机对象
 * @param period 当前更新距离上次更新的时间间隔(更新周期), 单位 sec
 * @retval None.
*/
void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period);


/**
 * @brief 编码器电机设置PID控制目标速度
 * @param rps 目标速度， 单位转每秒
 * @retval None.
*/
void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps);

/**
 * @brief Immediate, safe stop of the motor.
 *
 * Forces the accumulated PWM to zero instead of leaving it to the PID. This is
 * needed because the loop is incremental (pulse += output): if the feedback is
 * missing or frozen, a zero set point gives zero error, zero output and a PWM
 * STUCK at its last value, with the motor spinning indefinitely.
 */
void encoder_motor_stop(EncoderMotorObjectTypeDef *self);

/**
 * @brief Starts a relative move of delta_ticks from the current position.
 *
 * @param delta_ticks     signed distance in encoder ticks (not zero)
 * @param max_rps         speed limit of the move, in (0, rps_limit] rev/s
 * @param tolerance_ticks the move is REACHED inside +-tolerance (at least 1)
 * @param timeout_ms      100..60000 ms; when it expires the status is TIMEOUT
 * @return 0 if accepted; -1 if any parameter is out of range (nothing changes).
 */
int encoder_motor_move_relative(EncoderMotorObjectTypeDef *self,
                                int32_t delta_ticks,
                                float max_rps,
                                int32_t tolerance_ticks,
                                uint32_t timeout_ms);
/**
 * @brief Position HOLD, like motor.HOLD on a LEGO SPIKE hub: keeps the current
 *        counter with no timeout and, if the shaft is pushed away, drives it
 *        back with the same control law as MOVE_RELATIVE.
 *
 * The PWM is limited to max_pulse. If the motor pushes at that cap without
 * moving for 3 s (blocked shaft), the HOLD is released with status STALLED and
 * zero PWM, so that neither the motor nor the driver overheats. A speed
 * command, MOVE_RELATIVE, CANCEL or a stop all end the HOLD.
 * @return 0 if accepted; -1 if any parameter is out of range.
 */
int encoder_motor_hold(EncoderMotorObjectTypeDef *self,
                       float max_rps,
                       int32_t tolerance_ticks,
                       int max_pulse);
/** @brief Outer position loop. Call once per 100 Hz control tick (TIM7 ISR),
 *         after encoder_update() and before encoder_motor_control(). */
void encoder_motor_position_control(EncoderMotorObjectTypeDef *self);
/** @brief Leaves the position mode (status CANCELLED) and asks for zero speed. */
void encoder_motor_position_cancel(EncoderMotorObjectTypeDef *self);
/** @brief Makes the current position the origin of the reported positions. */
void encoder_motor_position_zero(EncoderMotorObjectTypeDef *self);
/** @brief Coherent copy of position, target, error, speed and state. Positions
 *         are relative to the origin set by encoder_motor_position_zero(). */
void encoder_motor_position_snapshot(EncoderMotorObjectTypeDef *self,
                                     int32_t *position_ticks,
                                     int32_t *target_ticks,
                                     int32_t *error_ticks,
                                     float *rps,
                                     uint8_t *active,
                                     uint8_t *status);

#endif
