/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "app_fsk.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    fsk_init();

    xTaskCreatePinnedToCore(fsk_task, "fsk_task", 4096, NULL, 8, NULL, 1);
}
