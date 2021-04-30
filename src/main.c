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
#include "maxbotix.h"

static const char *TAG = "main";

void app_main(void)
{
    /* Call Ethernet function */
    void eth_begin();
    eth_begin();

    /* Call Maxbotix function */
    maxbotix_init();

    /* Reduce logging from maxbotix */
    esp_log_level_set("maxbotix", ESP_LOG_WARN);


    while(1)
    {
        uint16_t sample = maxbotix_get_latest();
        int16_t count = 0;
        ESP_LOGI(TAG,"Continuing to do things, latest sample is %d",sample);
        float result = maxbotix_get_median(0.6f,8,64,&count);
        ESP_LOGI(TAG,"Median sample returned %f, sample count %d",(double)result,count);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
