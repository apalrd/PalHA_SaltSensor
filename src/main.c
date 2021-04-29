/* Ethernet Basic Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* All of the definitions related to the Ethernet driver */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "main";

void app_main(void)
{
    /* Call Ethernet function */
    void eth_begin();
    eth_begin();


    while(1)
    {
        ESP_LOGI(TAG,"Continuing to do things");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
