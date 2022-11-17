/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    while (true)
    {
        uint32_t free = esp_get_free_heap_size();
        uint32_t free_min = esp_get_minimum_free_heap_size();
        ESP_LOGI(TAG, "Minimum / current free heap size->%u / %u", free_min, free);
        vTaskDelay(pdMS_TO_TICKS(30 * 1000));
    }
}
