#include "packet.h"
#include "global.h"
#include "led.h"
#include "buzzer.h"
#include "serial_servo.h"
#include "packet_reports.h"


#pragma pack(1)
typedef struct {
    uint8_t cmd;
    uint8_t motor_num;
    struct {
        uint8_t motor_id;
        float speed;
    } element[];
} MotorMutilCtrlCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint8_t motor_id;
    float speed;
} MotorSingalCtrlCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint8_t motor_id;
} MotorSingalStopCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint8_t motor_mask;
} MotorMultiStopCommandTypeDef;

/* ---------------------------------------------------------------------------
 * EXTENDED FIRMWARE - MOTOR_EXT (function 0x0B): position control of one motor
 *
 * The frames keep the vendor's format on USART3 (1 Mbaud, the USB-C port
 * labelled UART2):
 *   0xAA 0x55 | function | length | payload[length] | CRC-8/MAXIM
 * with the CRC computed over function, length and payload. The first payload
 * byte is one of the sub-commands below. Multi-byte fields are little-endian
 * and the structs are packed (#pragma pack(1) above), so these struct layouts
 * ARE the wire format.
 *
 * Every well-formed command is answered with one MotorExtReport (20 bytes)
 * that repeats the sub-command; CAPABILITIES is answered with
 * MotorExtCapabilities (8 bytes) instead. Nothing is sent spontaneously: the
 * host polls STATUS to follow a move.
 * ------------------------------------------------------------------------- */
enum MotorExtSubCommand {
    MOTOR_EXT_CAPABILITIES = 0x00,  /* protocol version, motor mask, ticks per turn */
    MOTOR_EXT_MOVE_RELATIVE = 0x01, /* move by a signed number of ticks */
    MOTOR_EXT_CANCEL = 0x02,        /* leave the position mode, zero speed */
    MOTOR_EXT_ZERO = 0x03,          /* the current position becomes the origin */
    MOTOR_EXT_STATUS = 0x04,        /* report only */
    MOTOR_EXT_HOLD = 0x05,          /* hold the current position (protocol v3) */
};

typedef struct {                /* MOVE_RELATIVE request, 14 bytes */
    uint8_t cmd;
    uint8_t motor_id;           /* must be POSITION_MOTOR_INDEX */
    int32_t delta_ticks;        /* signed distance, 1320 ticks per turn here */
    float max_rps;              /* speed limit, rev/s */
    uint16_t tolerance_ticks;
    uint16_t timeout_ms;        /* 100..60000 */
} MotorExtMoveRelativeTypeDef;

typedef struct {                /* CANCEL, ZERO and STATUS requests, 2 bytes */
    uint8_t cmd;
    uint8_t motor_id;
} MotorExtSimpleCommandTypeDef;

typedef struct {                /* HOLD request, 10 bytes */
    uint8_t cmd;
    uint8_t motor_id;
    float max_rps;              /* speed limit while coming back, rev/s */
    uint16_t tolerance_ticks;
    uint16_t max_pulse;         /* PWM cap while holding, 300..1000 */
} MotorExtHoldTypeDef;

typedef struct {                /* reply to every command, 20 bytes */
    uint8_t cmd;                /* the sub-command being answered */
    uint8_t motor_id;
    uint8_t status;             /* EncoderPositionStatusEnum, or 5 = invalid command */
    uint8_t active;             /* 1 while a move or a HOLD is running */
    int32_t position_ticks;     /* relative to the origin set by ZERO */
    int32_t target_ticks;
    int32_t error_ticks;        /* target - position */
    float rps;                  /* measured speed, rev/s */
} MotorExtReportTypeDef;

/* v1: position control fixed on M1. v2: the position motor is the bit set in
   motor_mask (POSITION_MOTOR_INDEX) and the ticks per turn are read from the
   motor object; same frame layout. v3: HOLD (0x05) and the statuses
   6 (holding) and 7 (stalled). */
#define MOTOR_EXT_PROTOCOL_VERSION 3u

typedef struct {
    uint8_t cmd;
    uint8_t version;
    uint8_t motor_mask;
    uint8_t reserved;
    uint32_t ticks_per_revolution;
} MotorExtCapabilitiesTypeDef;

/* 串口舵机 */
typedef struct {
    uint8_t cmd;
    uint8_t servo_id;
    uint8_t args[];
} SerialServoCommandTypeDef;

/* 串口舵机 */
typedef struct {
    uint8_t cmd;
    uint8_t servo_num;
	uint8_t args[];
} SerialServoMultiCommandTypeDef;
/* 串口舵机 */
typedef struct {
    uint8_t cmd;
    uint16_t duration;
    uint8_t servo_num;
    struct {
        uint8_t servo_id;
        uint16_t position;
    } elements[];
} SerialServoSetPositionCommandTypeDef;

/* PWM 舵机 */
typedef struct {
    uint8_t cmd;
    uint8_t servo_id;
    uint8_t args[];
} PWM_ServoCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint16_t duration;
    uint8_t servo_id;
    uint16_t pulse;
} PWM_ServoSetPositionCommandTypeDef;

typedef struct {
    uint8_t cmd;
    uint16_t duration;
    uint8_t servo_num;
    struct {
        uint8_t servo_id;
        uint16_t pulse;
    } elements[];
} PWMServoSetMultiPositionCommandTypeDef;
/* LED */

typedef struct {
    uint8_t led_id;
    uint16_t on_time;
    uint16_t off_time;
    uint16_t repeat;
} LedCommandTypeDef;


typedef struct {
    uint16_t freq;
    uint16_t on_time;
    uint16_t off_time;
    uint16_t repeat;
} BuzzerCommandTypeDef;

#pragma pack()

/**
* @brief 串口命令回调处理
* @param frame 数据帧
* @retval void
*/
static void packet_led_handle(struct PacketRawFrame *frame)
{
    LedCommandTypeDef *cmd = (LedCommandTypeDef*)frame->data_and_checksum;
    uint8_t led_id = cmd->led_id - 1;
    if(led_id < 1) { /* ID 都是从 1 开始 */
        led_flash(leds[led_id], cmd->on_time, cmd->off_time, cmd->repeat);
    }
}

/**
* @brief 串口命令回调处理
* @param frame 数据帧
* @retval void
*/
static void packet_buzzer_handle(struct PacketRawFrame *frame)
{
    BuzzerCommandTypeDef *cmd = (BuzzerCommandTypeDef*)frame->data_and_checksum;
    buzzer_didi(buzzers[0], cmd->freq, cmd->on_time, cmd->off_time, cmd->repeat);
}


static void packet_serial_servo_report_init(PacketReportSerialServoTypeDef * report, uint8_t servo_id, uint8_t cmd, int success)
{
    report->servo_id = servo_id;
    report->sub_command = cmd;
    report->success = (uint8_t)((int8_t)success);
}

static void packet_serial_servo_handle(struct PacketRawFrame *frame)
{
    PacketReportSerialServoTypeDef report;
    switch(frame->data_and_checksum[0]) {
        case 0x01: { /* 舵机控制 */
            SerialServoSetPositionCommandTypeDef *cmd = (SerialServoSetPositionCommandTypeDef *)frame->data_and_checksum;
            for(int i = 0; i < cmd->servo_num; i++) {
                serial_servo_set_position(&serial_servo_controller, cmd->elements[i].servo_id, cmd->elements[i].position, cmd->duration);
            }
            break;
        }
        case 0x03: { /* 停止舵机 */
            SerialServoMultiCommandTypeDef *cmd = (SerialServoMultiCommandTypeDef *)frame->data_and_checksum;
			for(int i = 0; i < cmd->servo_num; i++) {
				serial_servo_stop(&serial_servo_controller, cmd->args[i]);
			}
            break;
        }
        case 0x05: { /* 位置读取 */
            int16_t position = 0;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd,  serial_servo_read_position(&serial_servo_controller, cmd->servo_id, &position));
            memcpy(report.args, &position, 2);
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 5);
            break;
        }
        case 0x07: { /* 输入电压读取 */
            uint16_t vin = 0;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_vin(&serial_servo_controller, cmd->servo_id, &vin));
            memcpy(report.args, &vin, 2);
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 5);
            break;
        }
        case 0x09: { /* 温度读取 */
            uint8_t temp = 0;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd,  serial_servo_read_temp(&serial_servo_controller, cmd->servo_id, &temp));
            report.args[0] = temp;
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 4);
            break;
        }
        case 0x0B: { /* 卸载动力 */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_load_unload(&serial_servo_controller, cmd->servo_id, 0);
            break;
        }
        case 0x0C: { /* 加载动力 */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_load_unload(&serial_servo_controller, cmd->servo_id, 1);
            break;
        }
		case 0x0D: { /* 动力状态读取 */
            uint8_t load_unload;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_load_unload(&serial_servo_controller, cmd->servo_id, &load_unload));
            report.args[0] = load_unload;
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 4);
            break;
		}
        case 0x10: { /* ID 写入 */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_set_id(&serial_servo_controller, cmd->servo_id, cmd->args[0]);
            break;
        }
        case 0x12: { /* ID 读取 */
            uint8_t servo_id;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_id(&serial_servo_controller, cmd->servo_id, &servo_id));
            report.args[0] = servo_id;
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 4);
            break;
        }
        case 0x20: { /* 偏差调整 */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_set_deviation(&serial_servo_controller, cmd->servo_id, cmd->args[0]);
            break;
        }
        case 0x22: { /* 偏差读取 */
            int8_t dev = 0;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_deviation(&serial_servo_controller, cmd->servo_id, &dev));
            report.args[0] = (uint8_t)dev;
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 4);
            break;
        }
        case 0x24: { /* 偏差保存 */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_save_deviation(&serial_servo_controller, cmd->servo_id);
            break;
        }
        case 0x30: { /* 位置限制设置 */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_set_angle_limit(&serial_servo_controller, cmd->servo_id, *((uint16_t*)(&cmd->args[0])), *((uint16_t*)(&cmd->args[2])));
            break;
        }
        case 0x32: { /* 位置限制读取 */
            uint16_t limit[2] = {0};
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_angle_limit(&serial_servo_controller, cmd->servo_id, limit));
            memcpy(&report.args, limit, 4);
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 7);
            break;
        }
        case 0x34: { /* 电压限制设置 */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_set_vin_limit(&serial_servo_controller, cmd->servo_id, *((uint16_t*)(&cmd->args[0])), *((uint16_t*)(&cmd->args[2])));
            break;
        }
        case 0x36: { /* 电压限制读取 */
            uint16_t limit[2] = {0};
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_vin_limit(&serial_servo_controller, cmd->servo_id, limit));
            memcpy(&report.args, limit, 4);
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 7);
            break;
        }
        case 0x38: { /* 温度限制设置 */
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            serial_servo_set_temp_limit(&serial_servo_controller, cmd->servo_id, cmd->args[0]);
            break;
        }
        case 0x3A: { /* 温度限制读取 */
            uint8_t limit = 0;
            SerialServoCommandTypeDef *cmd = (SerialServoCommandTypeDef *)frame->data_and_checksum;
            packet_serial_servo_report_init(&report, cmd->servo_id, cmd->cmd, serial_servo_read_temp_limit(&serial_servo_controller, cmd->servo_id, &limit));
            report.args[0] = limit;
            packet_transmit(&packet_controller, PACKET_FUNC_BUS_SERVO, &report, 4);
            break;
        }

        default:
            break;
    }
}
/**
* @brief PWM舵机串口命令回调处理
* @param frame 数据帧
* @retval void
*/
static void packet_pwm_servo_handle(struct PacketRawFrame *frame)
{
    switch(frame->data_and_checksum[0]) {
        case 0x01: {    //多个舵机控制
            PWMServoSetMultiPositionCommandTypeDef *cmd = (PWMServoSetMultiPositionCommandTypeDef *)frame->data_and_checksum;
            for(int i = 0; i < cmd->servo_num; ++i) {
                if(cmd->elements[i].servo_id <= 4) {
                    pwm_servo_set_position( pwm_servos[cmd->elements[i].servo_id - 1], cmd->elements[i].pulse, cmd->elements[i].pulse );
                }
            }
            break;
        }
        case 0x03: {    //单个舵机控制
            PWM_ServoSetPositionCommandTypeDef *cmd = (PWM_ServoSetPositionCommandTypeDef *)frame->data_and_checksum;
            //上位机从1号舵机开始
            if(cmd->servo_id <= 4) {
                pwm_servo_set_position( pwm_servos[cmd->servo_id - 1], cmd->pulse, cmd->duration );
            }
            break;
        }
        case 0x05: { // 读取舵机当前位置
            PWM_ServoCommandTypeDef *cmd = (PWM_ServoCommandTypeDef*)frame->data_and_checksum;
            if(cmd->servo_id <= 4) {
                uint16_t pulse = pwm_servos[cmd->servo_id - 1]->current_duty;
                PacketReportPWMServoTypeDef report;
                report.servo_id = cmd->servo_id;
                report.sub_command = cmd->cmd;
                memcpy(report.args, &pulse, 2);
                packet_transmit(&packet_controller, PACKET_FUNC_PWM_SERVO, &report, 4);
            }
            break;
        }
        case 0x07: { // 设置舵机偏差
            PWM_ServoCommandTypeDef *cmd = (PWM_ServoCommandTypeDef*)frame->data_and_checksum;
            if(cmd->servo_id <= 4) {
                pwm_servo_set_offset(pwm_servos[cmd->servo_id - 1], ((int)((int8_t)cmd->args[0])));
            }
            break;
        }
        case 0x09: { // 读取舵机偏差
            PWM_ServoCommandTypeDef *cmd = (PWM_ServoCommandTypeDef*)frame->data_and_checksum;
            if(cmd->servo_id <= 4) {
                int offset = pwm_servos[cmd->servo_id - 1]->offset;
                PacketReportPWMServoTypeDef report;
                report.servo_id = cmd->servo_id;
                report.sub_command = cmd->cmd;
                report.args[0] = (uint8_t)((int8_t)offset);
                packet_transmit(&packet_controller, PACKET_FUNC_PWM_SERVO, &report, 3);
            }
            break;
        }
        default:
            break;
    }
}

/**
* @brief 马达控制串口回调处理
* @param frame 数据帧
* @retval void
*/

static void packet_motor_handle(struct PacketRawFrame *frame)
{

    switch(frame->data_and_checksum[0]) {
        case 0: {
            MotorSingalCtrlCommandTypeDef *mscc = (MotorSingalCtrlCommandTypeDef *)frame->data_and_checksum;
            if(frame->data_length < sizeof(MotorSingalCtrlCommandTypeDef)) break;
            if(mscc->motor_id > 3u) break; /* the original indexed motors[] without a bounds check */
            if(mscc->motor_id == POSITION_MOTOR_INDEX) encoder_motor_position_cancel(motors[POSITION_MOTOR_INDEX]);
            motors[mscc->motor_id]->pid_controller.set_point = mscc->speed;
            break;
        }
        case 1: {
            MotorMutilCtrlCommandTypeDef *mmcc = NULL;
            mmcc = (MotorMutilCtrlCommandTypeDef *)frame->data_and_checksum;
            if(frame->data_length < 2u) break;
            /* a motor_num larger than what the frame carries would read past the buffer */
            if(frame->data_length < 2u + mmcc->motor_num * sizeof(mmcc->element[0])) break;
            for(int i = 0; i < mmcc->motor_num; ++i) {
                if(mmcc->element[i].motor_id > 3u) continue;
                if(mmcc->element[i].motor_id == POSITION_MOTOR_INDEX) encoder_motor_position_cancel(motors[POSITION_MOTOR_INDEX]);
                motors[mmcc->element[i].motor_id]->pid_controller.set_point = mmcc->element[i].speed;
            }
            break;
        }
        case 2:  {
            MotorSingalStopCommandTypeDef *mssc = (MotorSingalStopCommandTypeDef *)frame->data_and_checksum;
            if(mssc->motor_id > 3u) break;
            /* Hard stop: empties the accumulator of the incremental loop. A zero
               set point alone does not stop the motor if the feedback is missing. */
            encoder_motor_stop(motors[mssc->motor_id]);
            break;
        }
        case 3: {
            MotorMultiStopCommandTypeDef *mmsc = (MotorMultiStopCommandTypeDef *)frame->data_and_checksum;
            for(int i = 0; i < 4; ++i) {
                if(mmsc->motor_mask & (0x01 << i)) {
                    encoder_motor_stop(motors[i]);
                }
            }
            break;
        }
        default:
            break;
    }
}

/* Sends one MotorExtReport. status_override = 0xFF reports the real status
   of the motor; any other value replaces it (5 = command rejected). */
static void packet_motor_ext_report(uint8_t cmd, uint8_t status_override)
{
    MotorExtReportTypeDef report;
    encoder_motor_position_snapshot(motors[POSITION_MOTOR_INDEX], &report.position_ticks,
                                    &report.target_ticks, &report.error_ticks,
                                    &report.rps, &report.active, &report.status);
    report.cmd = cmd;
    report.motor_id = POSITION_MOTOR_INDEX;
    if(status_override != 0xFFu) report.status = status_override;
    packet_transmit(&packet_controller, PACKET_FUNC_MOTOR_EXT, &report, sizeof(report));
}

/* MOTOR_EXT dispatcher. Frames with the wrong length are dropped without an
   answer. A motor_id other than POSITION_MOTOR_INDEX, an out-of-range
   parameter or an unknown sub-command is answered with status 5 (invalid) and
   leaves the motor untouched. */
static void packet_motor_ext_handle(struct PacketRawFrame *frame)
{
    uint8_t cmd;
    if(frame->data_length < 1u) return;
    cmd = frame->data_and_checksum[0];

    if(cmd == MOTOR_EXT_CAPABILITIES) {
        MotorExtCapabilitiesTypeDef report = {
            MOTOR_EXT_CAPABILITIES, MOTOR_EXT_PROTOCOL_VERSION,
            (uint8_t)(1u << POSITION_MOTOR_INDEX), 0u,
            (uint32_t)motors[POSITION_MOTOR_INDEX]->ticks_per_circle};
        packet_transmit(&packet_controller, PACKET_FUNC_MOTOR_EXT, &report, sizeof(report));
        return;
    }

    if(frame->data_length < sizeof(MotorExtSimpleCommandTypeDef)) return;
    if(frame->data_and_checksum[1] != POSITION_MOTOR_INDEX) {
        packet_motor_ext_report(cmd, 5u);
        return;
    }

    switch(cmd) {
        case MOTOR_EXT_MOVE_RELATIVE: {
            MotorExtMoveRelativeTypeDef *move;
            int result;
            if(frame->data_length != sizeof(MotorExtMoveRelativeTypeDef)) return;
            move = (MotorExtMoveRelativeTypeDef *)frame->data_and_checksum;
            result = encoder_motor_move_relative(motors[POSITION_MOTOR_INDEX], move->delta_ticks,
                                                 move->max_rps, move->tolerance_ticks,
                                                 move->timeout_ms);
            packet_motor_ext_report(cmd, result == 0 ? 0xFFu : 5u);
            break;
        }
        case MOTOR_EXT_CANCEL:
            encoder_motor_position_cancel(motors[POSITION_MOTOR_INDEX]);
            packet_motor_ext_report(cmd, 0xFFu);
            break;
        case MOTOR_EXT_ZERO:
            encoder_motor_position_zero(motors[POSITION_MOTOR_INDEX]);
            packet_motor_ext_report(cmd, 0xFFu);
            break;
        case MOTOR_EXT_STATUS:
            packet_motor_ext_report(cmd, 0xFFu);
            break;
        case MOTOR_EXT_HOLD: {
            MotorExtHoldTypeDef *hold;
            int result;
            if(frame->data_length != sizeof(MotorExtHoldTypeDef)) return;
            hold = (MotorExtHoldTypeDef *)frame->data_and_checksum;
            result = encoder_motor_hold(motors[POSITION_MOTOR_INDEX], hold->max_rps,
                                        hold->tolerance_ticks, hold->max_pulse);
            packet_motor_ext_report(cmd, result == 0 ? 0xFFu : 5u);
            break;
        }
        default:
            packet_motor_ext_report(cmd, 5u);
            break;
    }
}


void packet_handle_init(void)
{
    packet_register_callback(&packet_controller, PACKET_FUNC_LED, packet_led_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_BUZZER, packet_buzzer_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_MOTOR, packet_motor_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_BUS_SERVO, packet_serial_servo_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_PWM_SERVO, packet_pwm_servo_handle);
    packet_register_callback(&packet_controller, PACKET_FUNC_MOTOR_EXT, packet_motor_ext_handle);
}
