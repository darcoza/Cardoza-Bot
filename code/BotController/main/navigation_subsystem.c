#include <uni.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static void navigation_subsystem_task(){
    for (;;){
        logi("NAVIGATION\n");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void start_navigation_subsystem(){
    xTaskCreate(navigation_subsystem_task, "navigation_subsystem", 4096, NULL, 10, NULL);
}