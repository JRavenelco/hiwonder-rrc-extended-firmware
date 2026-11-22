#include "global.h"
#include "tim.h"
#include "lwmem_porting.h"
#include "motors_param.h"

/* 全局变量 */
EncoderMotorObjectTypeDef *motors[4];
/* static void packet_handler(struct PacketRawFrame *frame); */

static void motor1_set_pulse(EncoderMotorObjectTypeDef *self, int speed);
static void motor2_set_pulse(EncoderMotorObjectTypeDef *self, int speed);
static void motor3_set_pulse(EncoderMotorObjectTypeDef *self, int speed);
static void motor4_set_pulse(EncoderMotorObjectTypeDef *self, int speed);

void set_motor_param(EncoderMotorObjectTypeDef *motor, int32_t tpc, float rps_limit, float kp, float ki, float kd)
{
    motor->ticks_per_circle = tpc;
    motor->rps_limit = rps_limit;
    motor->pid_controller.kp = kp;
    motor->pid_controller.ki = ki;
    motor->pid_controller.kd = kd;
}

void set_motor_type(EncoderMotorObjectTypeDef *motor, MotorTypeEnum type) {
	switch(type) {
		case MOTOR_TYPE_JGB520:
			set_motor_param(motor, MOTOR_JGB520_TICKS_PER_CIRCLE, MOTOR_JGB520_RPS_LIMIT, MOTOR_JGB520_PID_KP, MOTOR_JGB520_PID_KI, MOTOR_JGB520_PID_KD);
			break;
		case MOTOR_TYPE_JGB37:
			set_motor_param(motor, MOTOR_JGB37_TICKS_PER_CIRCLE, MOTOR_JGB37_RPS_LIMIT, MOTOR_JGB37_PID_KP, MOTOR_JGB37_PID_KI, MOTOR_JGB37_PID_KD);
			break;
		case MOTOR_TYPE_JGA27:
			set_motor_param(motor, MOTOR_JGA27_TICKS_PER_CIRCLE, MOTOR_JGA27_RPS_LIMIT, MOTOR_JGA27_PID_KP, MOTOR_JGA27_PID_KI, MOTOR_JGA27_PID_KD);
			break;
		case MOTOR_TYPE_JGB528:
			set_motor_param(motor, MOTOR_JGB528_TICKS_PER_CIRCLE, MOTOR_JGB528_RPS_LIMIT, MOTOR_JGB528_PID_KP, MOTOR_JGB528_PID_KI, MOTOR_JGB528_PID_KD);
			break;
		default:
			break;
	}
}

void apply_project_motor_config(void)
{
    /* JGB37-520R30-12 on M1 and on the position motor (M4 since 2026-10-04):
       11 encoder lines * x4 quadrature * 1:30 gearbox = 1320 ticks per turn of
       the output shaft (confirmed on the bench with a mark on the shaft).

       Gains: do NOT use the JGB520 profile that the JETACKER chassis imposes
       (3960 ticks, KP 63); it is tuned for a 1:90 gearbox. A 1:30 gearbox turns
       ~3x faster for the same PWM, so the open-loop gain is ~3x higher and the
       loop oscillates. The vendor itself lowers KP from 63 to 40 in its JGB37
       profile (1980 ticks). The values below were validated on the bench on
       M1 (2026-08-22). Arguments: ticks per turn, speed limit, KP, KI, KD. */
    set_motor_param(motors[0], 1320, PROJECT_MOTOR_RPS_LIMIT, 21.0f, 0.9f, 0.8f);
    set_motor_param(motors[POSITION_MOTOR_INDEX], 1320, PROJECT_MOTOR_RPS_LIMIT, 21.0f, 0.9f, 0.8f);
}

void motors_init(void)
{
    for(int i = 0; i < 4; ++i) {
        motors[i] = LWMEM_CCM_MALLOC(sizeof( EncoderMotorObjectTypeDef));
        encoder_motor_object_init(motors[i]);
		motors[i]->ticks_overflow = 60000;
        motors[i]->ticks_per_circle = MOTOR_DEFAULT_TICKS_PER_CIRCLE;
        motors[i]->rps_limit = MOTOR_DEFAULT_RPS_LIMIT;
        motors[i]->pid_controller.set_point = 0.0f;
        motors[i]->pid_controller.kp = MOTOR_DEFAULT_PID_KP;
        motors[i]->pid_controller.ki = MOTOR_DEFAULT_PID_KI;
        motors[i]->pid_controller.kd = MOTOR_DEFAULT_PID_KD;
    }

    /* 马达 1 */
    motors[0]->set_pulse = motor1_set_pulse;
    /* Project motor configuration: applied here and, more importantly, again
       after set_chassis_type() (see apply_project_motor_config). */
    apply_project_motor_config();
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* 编码器 */
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim5);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);


    /* 马达 2 */
    motors[1]->set_pulse = motor2_set_pulse;
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* 编码器 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim2);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    /* 马达 3 */
    motors[2]->set_pulse = motor3_set_pulse;
    __HAL_TIM_SET_COUNTER(&htim9, 0);
    __HAL_TIM_ENABLE(&htim9);
    __HAL_TIM_MOE_ENABLE(&htim9);

    /* 编码器 */
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim4);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* 马达 4 */
    motors[3]->set_pulse = motor4_set_pulse;
    __HAL_TIM_SET_COUNTER(&htim10, 0);
    __HAL_TIM_SET_COUNTER(&htim11, 0);
    __HAL_TIM_ENABLE(&htim10);
    __HAL_TIM_ENABLE(&htim11);
    __HAL_TIM_MOE_ENABLE(&htim10);
    __HAL_TIM_MOE_ENABLE(&htim11);

    /* 编码器 */
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim3);
    /* Vendor bug fixed here: the original line started &htim4 (the M3 encoder)
       a second time instead of &htim3, which left TIM3 without CC1E/CC2E
       (TIM3_CCER read back as 0 over SWD on the bench). */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);


    // 测速更新定时器
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim7);

    //packet_register_callback(&packet_controller, PACKET_FUNC_MOTOR, packet_handler);
}

static void motor1_set_pulse(EncoderMotorObjectTypeDef *self, int speed)
{
    if(speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
    } else if(speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -speed);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}


static void motor2_set_pulse(EncoderMotorObjectTypeDef *self, int speed)
{
    if(speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
    } else if(speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

static void motor3_set_pulse(EncoderMotorObjectTypeDef *self, int speed)
{
    HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
    if(speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed);
    } else if(speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, -speed);
    } else {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    }
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}

static void motor4_set_pulse(EncoderMotorObjectTypeDef *self, int speed)
{
    HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
    if(speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, speed);
    } else if(speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, -speed);
    } else {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
    }
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}
