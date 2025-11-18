/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "ld2410.h"

const char *TAG = "APP";
bool prevPresence = false;
LD2410_device_t *ld2410;

static void ld2410_task(void *arg)
{
    while (1)
    {
        ld2410_check(ld2410);
        bool newPresence = ld2410_presence_detected(ld2410);
        if (prevPresence != newPresence)
        {
            prevPresence = newPresence;
            ESP_LOGI(TAG, "Presence changed: %s", newPresence ? "PRESENT" : "EMPTY");
        }
        vTaskDelay(10); // Let FreeRTOS breathe
    }
}

void app_main(void)
{
    ld2410 = ld2410_new();
    if (!ld2410_begin(ld2410))
    {
        ESP_LOGE(TAG, "Failed to communicate with the sensor.");
        while (true)
        {
        }
    }
    else
    {
        ESP_LOGI(TAG, "Setup completed");
    }
    xTaskCreate(ld2410_task, "ld2410_task", CONFIG_LD2410_TASK_STACK_SIZE, NULL, 10, NULL);
}
