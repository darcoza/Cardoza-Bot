#include <uni.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "robot_message.h"

// TODO: Make "A" the "FORWARD" pin for both motors!
#define L_PIN_A 3
#define L_PIN_B 2 
#define R_PIN_A 1
#define R_PIN_B 0

#define LA_CHAN LEDC_CHANNEL_0
#define LB_CHAN LEDC_CHANNEL_1
#define RA_CHAN LEDC_CHANNEL_2
#define RB_CHAN LEDC_CHANNEL_3

static QueueHandle_t q_motion = NULL;

void set_motor(int, int);

void init_motors() {
    logi("Initializing Motion Subsystem...");

    // Configure the LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE, // high speed timers
        .duty_resolution  = LEDC_TIMER_9_BIT, // up to 512
        .freq_hz          = 5000, // hertz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer.timer_num = LEDC_TIMER_0;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_timer.timer_num = LEDC_TIMER_1;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_timer.timer_num = LEDC_TIMER_2;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_timer.timer_num = LEDC_TIMER_3;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure the LEDC channel
    ledc_channel_config_t l_pin_a_channel = {
        .gpio_num       = L_PIN_A,
        .speed_mode     = ledc_timer.speed_mode,
        .channel        = LA_CHAN,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 4000, // Set duty cycle (0 ~ 2^duty_resolution - 1)
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&l_pin_a_channel));

    ledc_channel_config_t l_pin_b_channel = {
        .gpio_num       = L_PIN_B,
        .speed_mode     = ledc_timer.speed_mode,
        .channel        = LB_CHAN,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_1,
        .duty           = 4000, // Set duty cycle (0 ~ 2^duty_resolution - 1)
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&l_pin_b_channel));

    ledc_channel_config_t r_pin_a_channel = {
        .gpio_num       = R_PIN_A,
        .speed_mode     = ledc_timer.speed_mode,
        .channel        = RA_CHAN,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_2,
        .duty           = 4000, // Set duty cycle (0 ~ 2^duty_resolution - 1)
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&r_pin_a_channel));

    ledc_channel_config_t r_pin_b_channel = {
        .gpio_num       = R_PIN_B,
        .speed_mode     = ledc_timer.speed_mode,
        .channel        = RB_CHAN,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_3,
        .duty           = 4000, // Set duty cycle (0 ~ 2^duty_resolution - 1)
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&r_pin_b_channel));

    // Init motors to zero speed
    set_motor(0, 0);
    set_motor(1, 0);
}

void set_motor(int side, int speed){
    // speed: -128 - 127
    // TODO: implement what the motors do.
    logi("MOTOR SET %d %d\n", side, speed);

    if (side != 0 && side != 1) return;
    if (abs(speed) > 128) return;

    int duty = (int) (abs(speed) * 512.0 / 128.0);
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, side ? RA_CHAN : LA_CHAN, 
        speed > 0 ? duty : 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, side ? RA_CHAN : LA_CHAN));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, side ? RB_CHAN : LB_CHAN, 
        speed < 0 ? duty : 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, side ? RB_CHAN : LB_CHAN));
}

static void motion_subsystem_task(){
    RobotMessage_t msg = {0};
    for (;;){
        if (!xQueueReceive(q_motion, &msg, portMAX_DELAY)) continue;

        if (msg.type != MOTION_REQUEST) continue;

        uint8_t motor = (msg.value >> 11) & 1;
        uint8_t accel = (msg.value >> 10) & 1;
        int8_t  speed = msg.value & 0xFF;

        set_motor(motor, speed);
    }
}

void start_motion_subsystem(QueueHandle_t q_motion_addr){
    q_motion = q_motion_addr;
    init_motors();
    xTaskCreate(motion_subsystem_task, "motion_subsystem", 4096, NULL, 10, NULL);
}