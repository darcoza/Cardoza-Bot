#include <string.h>

#include <uni.h>
#include "driver/gpio.h"

#include <stdlib.h>
#include <inttypes.h>


#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <uni.h>

#include "sdkconfig.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "robot_controller.h"
#include "robot_message.h"

// #include "RobotController.h"

#define L_DEADZONE 60
#define R_DEADZONE 60

#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

// Custom "instance"
typedef struct robot_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;  // which "seat" is being used
} robot_platform_instance_t;

// Declarations
static robot_platform_instance_t* get_robot_platform_instance(uni_hid_device_t* d);

// For Communcations
static QueueHandle_t q_to_controller = NULL;

static void robot_on_io_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    // Tell the controller there's no device yet.
    RobotMessage_t msg = {0};
    msg = encode_message(ALERT, IO_DISCONNECTED);
    xQueueSend(q_to_controller, &msg, 100);
}

static void robot_on_init_complete(void) {
    // Safe to call "unsafe" functions since they are called from BT thread
    // Start scanning
    uni_bt_enable_new_connections_unsafe(true);

    // Based on runtime condition, you can delete or list the stored BT keys.
    if (1)
        uni_bt_del_keys_unsafe();
    else
        uni_bt_list_keys_unsafe();
}

static uni_error_t robot_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    // You can filter discovered devices here.
    // Just return any value different from UNI_ERROR_SUCCESS;
    // @param addr: the Bluetooth address
    // @param name: could be NULL, could be zero-length, or might contain the name.
    // @param cod: Class of Device. See "uni_bt_defines.h" for possible values.
    // @param rssi: Received Signal Strength Indicator (RSSI) measured in dBms. The higher (255) the better.

    // As an example, if you want to filter out keyboards, do:
    if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD) {
        return UNI_ERROR_IGNORE_DEVICE;
    }
    return UNI_ERROR_SUCCESS;
}


static void robot_on_io_device_connected(uni_hid_device_t* d) {
    logi("IO device connected: %p\n", d);
    RobotMessage_t msg = {0};
    msg = encode_message(ALERT, IO_CONNECTED);
    xQueueSend(q_to_controller, &msg, 100);
}

static void robot_on_io_device_disconnected(uni_hid_device_t* d) {
    logi("IO device disconnected: %p\n", d);
    RobotMessage_t msg = {0};
    msg = encode_message(ALERT, IO_DISCONNECTED);
    xQueueSend(q_to_controller, &msg, 100);
}

static uni_error_t robot_on_io_ready(uni_hid_device_t* d) {
    robot_platform_instance_t* ins = get_robot_platform_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;
    return UNI_ERROR_SUCCESS;
}

static void robot_on_oob_event(uni_platform_oob_event_t event, void* data) {
    ARG_UNUSED(event);
    ARG_UNUSED(data);
}

static void robot_on_io_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    static uni_controller_t prev = {0};
    uni_gamepad_t* gp;

    // Optimization to avoid processing the previous data so that the console
    // does not get spammed with a lot of logs, but remove it from your project.
    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
        return;
    }

    prev = *ctl;

    if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) return;

    gp = &ctl->gamepad;

    // TANK DRIVE
    int8_t l_drive = 0, r_drive = 0; // (inverted... negative Y is in the UP direction)
                                        // we'll handle but not letting axis values to get past 127.0 in either direction
    if (abs(gp->axis_y) > L_DEADZONE) l_drive = (int8_t)((-1.0 * gp->axis_y) * 127.0 / 512.0);
    if (abs(gp->axis_ry) > R_DEADZONE) r_drive = (int8_t)((-1.0 * gp->axis_ry) * 127.0 / 512.0);

    RobotMessage_t l_drive_msg = encode_message(INTENT, TANK_DRIVE_LEFT, l_drive);
    RobotMessage_t r_drive_msg = encode_message(INTENT, TANK_DRIVE_RIGHT, r_drive);

    xQueueSend(q_to_controller, &l_drive_msg, 100);
    xQueueSend(q_to_controller, &r_drive_msg, 100);
}

static const uni_property_t* robot_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}


/* Helpers */
static robot_platform_instance_t* get_robot_platform_instance(uni_hid_device_t* d) {
    return (robot_platform_instance_t*)&d->platform_data[0];
}

/* Entry Point */
struct uni_platform* get_robot_platform(void) {
    static struct uni_platform plat = {
        .name = "CARDOZABOT",
        .init = robot_on_io_init,
        .on_init_complete = robot_on_init_complete,
        .on_device_discovered = robot_on_device_discovered,
        .on_device_connected = robot_on_io_device_connected,
        .on_device_disconnected = robot_on_io_device_disconnected,
        .on_device_ready = robot_on_io_ready,
        .on_oob_event = robot_on_oob_event,
        .on_controller_data = robot_on_io_data,
        .get_property = robot_get_property,
    };

    return &plat;
}

void start_robot_io(QueueHandle_t q_intent){
    q_to_controller = q_intent;

    btstack_init();

    // Must be called before uni_init()
    uni_platform_set_custom(get_robot_platform());

    // Init Bluepad32.
    uni_init(0 /* argc */, NULL /* argv */);

    // Does not return.
    btstack_run_loop_execute();
}