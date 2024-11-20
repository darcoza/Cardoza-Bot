
#include <uni.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static void encoder_subsystem_task(){
    for (;;){
        logi("ENCODER\n");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void start_encoder_subsystem(){
    xTaskCreate(encoder_subsystem_task, "encoder_subsystem", 4096, NULL, 10, NULL);
}