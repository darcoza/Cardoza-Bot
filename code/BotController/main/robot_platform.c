
#include <string.h>

#include <uni.h>
#include "driver/gpio.h"
// #include "RobotController.h"

// Custom "instance"
typedef struct robot_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;  // which "seat" is being used
} robot_platform_instance_t;

// Declarations
static void trigger_event_on_gamepad(uni_hid_device_t* d);
static robot_platform_instance_t* get_robot_platform_instance(uni_hid_device_t* d);

static void robot_on_controller_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    logi("custom: init()\n");
}

static void robot_on_init_complete(void) {
    logi("custom: on_init_complete()\n");

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
        logi("Ignoring keyboard\n");
        return UNI_ERROR_IGNORE_DEVICE;
    }

    return UNI_ERROR_SUCCESS;
}


static void robot_on_controller_device_connected(uni_hid_device_t* d) {
    logi("Controller connected: %p\n", d);
}

static void robot_on_controller_disconnected(uni_hid_device_t* d) {
    logi("Controller disconnected: %p\n", d);
}


static uni_error_t robot_on_controller_ready(uni_hid_device_t* d) {
    logi("custom: device ready: %p\n", d);
    robot_platform_instance_t* ins = get_robot_platform_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;

    trigger_event_on_gamepad(d);
    return UNI_ERROR_SUCCESS;
}

// Not sure what an OOB event is... out of band?
static void robot_on_oob_event(uni_platform_oob_event_t event, void* data) {
    ARG_UNUSED(event);
    ARG_UNUSED(data);
    // switch (event) {
    //     case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON: {
    //         uni_hid_device_t* d = data;

    //         if (d == NULL) {
    //             loge("ERROR: my_platform_on_oob_event: Invalid NULL device\n");
    //             return;
    //         }
    //         logi("custom: on_device_oob_event(): %d\n", event);

    //         robot_platform_instance_t* ins = get_robot_platform_instance(d);
    //         ins->gamepad_seat = ins->gamepad_seat == GAMEPAD_SEAT_A ? GAMEPAD_SEAT_B : GAMEPAD_SEAT_A;

    //         trigger_event_on_gamepad(d);
    //         break;
    //     }

    //     case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
    //         logi("custom: Bluetooth enabled: %d\n", (bool)(data));
    //         break;

    //     default:
    //         logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
    //         break;
    // }
}

static void do_nothing(void){
    return;
}

static void robot_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    static uint8_t leds = 0;
    static uint8_t enabled = true;
    static uni_controller_t prev = {0};
    uni_gamepad_t* gp;

    // Optimization to avoid processing the previous data so that the console
    // does not get spammed with a lot of logs, but remove it from your project.
    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
        return;
    }
    prev = *ctl;
    // Print device Id before dumping gamepad.
    // This could be very CPU intensive and might crash the ESP32.
    // Remove these 2 lines in production code.
    //    logi("(%p), id=%d, \n", d, uni_hid_device_get_idx_for_instance(d));
    //    uni_controller_dump(ctl);

    switch (ctl->klass) {
        case UNI_CONTROLLER_CLASS_GAMEPAD:
            gp = &ctl->gamepad;
            logi("Buttons pressed: %d:%d, %d:%d, %d, %d, %d\n", gp->axis_x, gp->axis_y, gp->axis_rx, gp->axis_ry, gp->buttons, gp->throttle, gp->brake);
            
            // // Debugging
            // // Axis ry: control rumble
            // if ((gp->buttons & BUTTON_A) && d->report_parser.play_dual_rumble != NULL) {
            //     d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 250 /* duration ms */,
            //                                       255 /* weak magnitude */, 0 /* strong magnitude */);
            // }
            // // Buttons: Control LEDs On/Off
            // if ((gp->buttons & BUTTON_B) && d->report_parser.set_player_leds != NULL) {
            //     d->report_parser.set_player_leds(d, leds++ & 0x0f);
            // }
            // // Axis: control RGB color
            // if ((gp->buttons & BUTTON_X) && d->report_parser.set_lightbar_color != NULL) {
            //     uint8_t r = (gp->axis_x * 256) / 512;
            //     uint8_t g = (gp->axis_y * 256) / 512;
            //     uint8_t b = (gp->axis_rx * 256) / 512;
            //     d->report_parser.set_lightbar_color(d, r, g, b);
            // }

            // // Toggle Bluetooth connections
            // if ((gp->buttons & BUTTON_SHOULDER_L) && enabled) {
            //     logi("*** Disabling Bluetooth connections\n");
            //     uni_bt_enable_new_connections_safe(false);
            //     enabled = false;
            // }
            // if ((gp->buttons & BUTTON_SHOULDER_R) && !enabled) {
            //     logi("*** Enabling Bluetooth connections\n");
            //     uni_bt_enable_new_connections_safe(true);
            //     enabled = true;
            // }
            break;
        default:
            break;
    }
}

static const uni_property_t* robot_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}


/* Helpers */
static robot_platform_instance_t* get_robot_platform_instance(uni_hid_device_t* d) {
    return (robot_platform_instance_t*)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t* d) {
    robot_platform_instance_t* ins = get_robot_platform_instance(d);

    // if (d->report_parser.play_dual_rumble != NULL) {
    //     d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 150 /* duration ms */, 128 /* weak magnitude */,
    //                                       40 /* strong magnitude */);
    // }

    // if (d->report_parser.set_player_leds != NULL) {
    //     d->report_parser.set_player_leds(d, ins->gamepad_seat);
    // }

    // if (d->report_parser.set_lightbar_color != NULL) {
    //     uint8_t red = (ins->gamepad_seat & 0x01) ? 0xff : 0;
    //     uint8_t green = (ins->gamepad_seat & 0x02) ? 0xff : 0;
    //     uint8_t blue = (ins->gamepad_seat & 0x04) ? 0xff : 0;
    //     d->report_parser.set_lightbar_color(d, red, green, blue);
    // }
}

/* Entry Point */
struct uni_platform* get_robot_platform(void) {
    static struct uni_platform plat = {
        .name = "CardozaBot",
        .init = robot_on_controller_init,
        .on_init_complete = robot_on_init_complete,
        .on_device_discovered = robot_on_device_discovered,
        .on_device_connected = robot_on_controller_device_connected,
        .on_device_disconnected = robot_on_controller_disconnected,
        .on_device_ready = robot_on_controller_ready,
        .on_oob_event = robot_on_oob_event,
        .on_controller_data = robot_on_controller_data,
        .get_property = robot_get_property,
    };

    return &plat;
}



/* General conceptual flow... 
- Robot should remain completely OFF until we have connection with a controller
- Once connection takes place, we will INITIALIZE all robot tasks
- Robot tasks include:
-- process command received from Xbox Controller
-- map processed command to motor activity, (FUTURE) leveraging current positional state
-- read encoders and update current positional state
-- (FUTURE) read accelerometer and update current positional state
*/