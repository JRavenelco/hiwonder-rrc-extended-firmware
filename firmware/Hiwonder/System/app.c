/**
 * @file app.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 主应用逻辑
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "cmsis_os2.h"
#include "led.h"
#include "lwmem_porting.h"
#include "global.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "global.h"
#include "adc.h"
#include "u8g2_porting.h"
#include "packet_reports.h"
#include "packet_handle.h"
#include "motors_param.h"
#include "serial_servo.h"


void buzzers_init(void);
void buttons_init(void);
void leds_init(void);
void motors_init(void);
void serial_servo_init(void);
void pwm_servos_init(void);
void sbus_init(void);
void chassis_init(void);

/* EXTENDED FIRMWARE - independent watchdog (IWDG).
   The board has no external watchdog, and the IWDG runs from the LSI
   (~32 kHz), so it keeps counting even if the crystal fails. It is refreshed
   by the TIM7 ISR (100 Hz, the motor loop), but only while this task has shown
   signs of life within the last 500 ms. A hang of the interrupts (the CSS NMI,
   HardFault, Error_Handler, configASSERT) or of the RTOS therefore resets the
   board, and the reset switches the PWM outputs off.
   Why it was added: on 2026-10-04 a clock-security (CSS) NMI left the board
   mute with the PWM frozen; the vendor's NMI handler ends in while(1). */
#define WATCHDOG_PRESCALER_DIV32     3u   /* IWDG_PR: LSI/32, ~1 kHz */
#define WATCHDOG_RELOAD              250u /* ~250 ms (LSI 17-47 kHz: 170-470 ms) */
#define WATCHDOG_HEARTBEAT_MAX_TICKS 50u  /* TIM7 cycles without a heartbeat: 500 ms */
#define WATCHDOG_HEARTBEAT_PERIOD_MS 100u /* how often the app task reports in */

static volatile uint32_t watchdog_heartbeat_age = 0;
static volatile uint8_t watchdog_running = 0;

static void watchdog_start(void)
{
    __HAL_DBGMCU_FREEZE_IWDG();   /* frozen while a debugger halts the CPU */
    IWDG->KR = 0xCCCCu;           /* start; it cannot be stopped again until a reset */
    IWDG->KR = 0x5555u;           /* unlock PR and RLR */
    IWDG->PR = WATCHDOG_PRESCALER_DIV32;
    IWDG->RLR = WATCHDOG_RELOAD;
    /* Wait (bounded) until the new prescaler and reload values are taken. */
    for(uint32_t i = 0; (IWDG->SR != 0u) && (i < 1000000u); ++i) {
    }
    IWDG->KR = 0xAAAAu;           /* first refresh, with the new reload value */
    watchdog_heartbeat_age = 0;
    watchdog_running = 1;
}

/* Called from the TIM7 ISR, 100 times per second. Each call ages the
   heartbeat by one; the app task puts the age back to zero every 100 ms.
   If the task stops running, the age reaches 50 (500 ms), the refresh stops
   and the IWDG resets the MCU about 250 ms later. If the interrupts stop,
   nobody refreshes at all and the reset comes within those 250 ms. */
void watchdog_isr_kick(void)
{
    if(!watchdog_running) return;
    if(watchdog_heartbeat_age < WATCHDOG_HEARTBEAT_MAX_TICKS) {
        ++watchdog_heartbeat_age;
        IWDG->KR = 0xAAAAu;
    }
}


void button_event_callback(ButtonObjectTypeDef *button,  ButtonEventIDEnum event)
{
    PacketReportKeyEventTypeDef report = {
        .key_id = button->id,
        .event = (uint8_t)(int)event,
    };
    packet_transmit(&packet_controller, PACKET_FUNC_KEY, &report, sizeof(PacketReportKeyEventTypeDef));
	if(event == BUTTON_EVENT_CLICK) {
		buzzer_didi(buzzers[0], 2000, 50, 50, 1);
	}
}

void app_task_entry(void *argument)
{
    extern osTimerId_t led_timerHandle;
    extern osTimerId_t buzzer_timerHandle;
    extern osTimerId_t button_timerHandle;
    extern osTimerId_t battery_check_timerHandle;
    extern osMessageQueueId_t moving_ctrl_queueHandle;
    extern osMessageQueueId_t bluetooth_tx_queueHandle;

    motors_init();
    pwm_servos_init();
	serial_servo_init();
    leds_init();
    buzzers_init();
    buttons_init();

    button_register_callback(buttons[0], button_event_callback);
    button_register_callback(buttons[1], button_event_callback);

    osTimerStart(led_timerHandle, LED_TASK_PERIOD);
    osTimerStart(buzzer_timerHandle, BUZZER_TASK_PERIOD);
    osTimerStart(button_timerHandle, BUTTON_TASK_PERIOD);
    osTimerStart(battery_check_timerHandle, BATTERY_TASK_PERIOD);
    packet_handle_init();

//    char msg = '\0';
//    uint8_t msg_prio;
//    osMessageQueueReset(moving_ctrl_queueHandle);

    chassis_init();
    set_chassis_type(CHASSIS_TYPE_JETACKER);
    /* set_chassis_type() reconfigures all four motors with the chassis
       profile; re-apply the real configuration of M1 and of the position
       motor after it. */
    apply_project_motor_config();

    /* Everything is initialised: from here on, a hang resets the board. */
    watchdog_start();
    for(;;) {
        watchdog_heartbeat_age = 0;   /* heartbeat: the RTOS is alive */
        osDelay(WATCHDOG_HEARTBEAT_PERIOD_MS);
    }
}
