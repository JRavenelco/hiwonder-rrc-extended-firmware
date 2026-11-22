#ifndef __MOTORS_PARAM_H
#define __MOTORS_PARAM_H
#include "encoder_motor.h"

typedef enum {
	MOTOR_TYPE_JGB520,
	MOTOR_TYPE_JGB37,
	MOTOR_TYPE_JGA27,
	MOTOR_TYPE_JGB528,
}MotorTypeEnum;

/* 电机轴每转产生11个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为90.0。
 * 即电机输出轴每转一圈计数器计数值改变 11.0 * 4.0 * 90.0 = 3960
 */
#define MOTOR_JGB520_TICKS_PER_CIRCLE 3960.0f
#define MOTOR_JGB520_PID_KP  63.0f
#define MOTOR_JGB520_PID_KI  2.6f
#define MOTOR_JGB520_PID_KD  2.4f
#define MOTOR_JGB520_RPS_LIMIT 1.5f

/* 电机轴每转产生11个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为45.0:1。
 * 即电机输出轴每转一圈计数器计数值改变 11.0 * 4.0 * 45.0 = 1980
 */
#define MOTOR_JGB37_TICKS_PER_CIRCLE 1980.0f
#define MOTOR_JGB37_PID_KP  40.0f
#define MOTOR_JGB37_PID_KI  2.0f
#define MOTOR_JGB37_PID_KD  2.0f
#define MOTOR_JGB37_RPS_LIMIT 3.0f

/* 电机轴每转产生13个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为20.0:1。
 * 即电机输出轴每转一圈计数器计数值改变 13.0 * 4.0 * 20.0 = 1040
 */
#define MOTOR_JGA27_TICKS_PER_CIRCLE 1040.0f
#define MOTOR_JGA27_PID_KP  -36.0f
#define MOTOR_JGA27_PID_KI  -1.0f
#define MOTOR_JGA27_PID_KD  -1.0f
#define MOTOR_JGA27_RPS_LIMIT 6.0f

/* 电机轴每转产生11个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为131.0:1。
 * 即电机输出轴每转一圈计数器计数值改变 11.0 * 4.0 * 131.0 = 5764
 */
#define MOTOR_JGB528_TICKS_PER_CIRCLE 5764.0f
#define MOTOR_JGB528_PID_KP  300.0f
#define MOTOR_JGB528_PID_KI  2.0f
#define MOTOR_JGB528_PID_KD  12.0f
#define MOTOR_JGB528_RPS_LIMIT 1.1f

/* 电机轴每转产生11个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为90.0。
 * 即电机输出轴每转一圈计数器计数值改变 11.0 * 4.0 * 90.0 = 3960
 */
#define MOTOR_DEFAULT_TICKS_PER_CIRCLE 3960.0f
#define MOTOR_DEFAULT_PID_KP  63.0f
#define MOTOR_DEFAULT_PID_KI  2.6f
#define MOTOR_DEFAULT_PID_KD  2.4f
#define MOTOR_DEFAULT_RPS_LIMIT 1.35f

void set_motor_type(EncoderMotorObjectTypeDef *motor, MotorTypeEnum type);

/* Project motor: JGB37-520R30-12 (1:30), on M1 and on the position motor.
 * Top speed measured on M4 on 2026-10-04, no load, 11.96 V, 100 % PWM:
 * +5.76 / -5.74 rev/s (345 rpm). The limit sits just under that value so that
 * every rpm can be used: close to the top the loop works saturated (100 % PWM)
 * and the real speed depends on the battery and on the load. Before: 3.0 rev/s
 * (MOTOR_JGB37_RPS_LIMIT, the vendor's profile for the 1:45 gearbox). */
#define PROJECT_MOTOR_RPS_LIMIT 5.7f

/**
 * @brief Applies the project's motor configuration to M1 and to the position
 *        motor (POSITION_MOTOR_INDEX).
 *
 * It MUST be called AFTER set_chassis_type(): that function reconfigures all
 * four motors for the chassis (JETACKER -> JGB520, 3960 ticks, KP 63) and
 * would overwrite these values. Setting them only in motors_init() has no
 * effect.
 */
void apply_project_motor_config(void);

#endif
