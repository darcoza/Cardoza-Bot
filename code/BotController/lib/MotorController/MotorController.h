// Motors should have a command Q
// Command: motor + speed (0-255)

// Motor encoders should have an interrupt Q
// Changes the global state of the encoders?
// Tracks how far we've gone? Heading? Approximates speed?

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// QUEUE for cross-task communication
static QueueHandle_t motor_cmd_queue;

static QueueHandle_t encoder_state_queue;
