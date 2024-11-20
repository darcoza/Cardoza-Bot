#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <uni.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "robot_io.h"
#include "motion_subsystem.h"
#include "navigation_subsystem.h"
#include "encoder_subsystem.h"
#include "robot_message.h"

#define INCLUDE_vTaskDelete     1
#define LED_PIN                 4

static QueueHandle_t q_io = NULL;
static QueueHandle_t q_motion = NULL;
static TaskHandle_t led_task = NULL;
bool motion_enabled = false;

uint32_t l_power = 0;
uint32_t r_power = 0;

RobotMessage_t my_message = {0,0};

static void led_slow_blink_task() {

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << LED_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // ESP_ERROR_CHECK(gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT));    
    // set up LED for output via PWM
    for(;;){
        // Turn ON
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(750));

        // Turn OFF
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(750));
    }

}

static void robot_controller_task() {
    logi("Initializing CARDOZABOT controller...\n");

    RobotMessage_t msg = {0};
    for (;;){
        // Check the IO Q
        if (!xQueueReceive(q_io, &msg, portMAX_DELAY)) continue;

        // Interpret inbound message≥
        if (msg.type == ALERT && msg.value == IO_DISCONNECTED){
            logi("<Controller>: IO Disconnected.\n");
            xTaskCreate(led_slow_blink_task, "led_slow_blink", 4096, NULL, 10, &led_task);
            continue;
        }

        if (msg.type == ALERT && msg.value == IO_CONNECTED){
            logi("<Controller>: IO Connected.\n");
            if (led_task != NULL) vTaskDelete(led_task);
            gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
            gpio_set_level(LED_PIN, 1);
            continue;
        }

        if (msg.type == INTENT){
            uint16_t cmd = (msg.value >> 8) & 0xF;
            uint16_t val = msg.value & 0xFF;
            // logi("INTENT RCVD: cmd: %d, val: %d\n", cmd, val);

            // TODO: Interpret intent in current context

            // We make new instances of messages to make sure we don't
            // overwrite previous commands.
            if (cmd == STOP){
                RobotMessage_t l_motion_msg = {0};
                RobotMessage_t r_motion_msg = {0};
                l_power = 0; // update our memory
                r_power = 0;
                l_motion_msg = encode_message(MOTION_REQUEST, 0, 0, 0);
                r_motion_msg = encode_message(MOTION_REQUEST, 1, 0, 0);
                xQueueSend(q_motion, &l_motion_msg, 100);
                xQueueSend(q_motion, &r_motion_msg, 100);
                continue;
            }

            if (cmd == TANK_DRIVE_LEFT){
                RobotMessage_t l_motion_msg = {0};
                l_power = val; // update our memory
                l_motion_msg = encode_message(MOTION_REQUEST, 0, 0, l_power);
                xQueueSend(q_motion, &l_motion_msg, 100);
                continue;
            }

            if (cmd == TANK_DRIVE_RIGHT){
                RobotMessage_t r_motion_msg = {0};
                r_power = val; // update our memory
                r_motion_msg = encode_message(MOTION_REQUEST, 1, 0, r_power);
                xQueueSend(q_motion, &r_motion_msg, 100);
                continue;
            }

            // TODO: Decide what action to take with motors

            // TODO: Execute action to take with motors
            // motion_msg = encode_message(MOTION_REQUEST, 0, 1023);
            // xQueueSend(q_motion, &motion_msg, 100);
            // motion_msg = encode_message(MOTION_REQUEST, 1, 1023);
            // xQueueSend(q_motion, &motion_msg, 100);

            continue;
        }

        // Default:
        logi("MSG received by controller, but this functionality is not implemented.");
    }
}

void start_robot_controller(){
    xTaskCreate(robot_controller_task, "robot_controller", 4096, NULL, 10, NULL);
}

void start_robot(){
    q_io = xQueueCreate(16, sizeof(RobotMessage_t));
    q_motion = xQueueCreate(16, sizeof(RobotMessage_t));

    start_robot_controller(); // Creates task, returns
    start_motion_subsystem(q_motion); // Creates task, returns
    // start_navigation_subsystem(); // Creates task, returns
    // start_encoder_subsystem(); // Creates task, returns
    start_robot_io(q_io); // IO management task, does not return
}

