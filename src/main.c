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
#include "ethernet.h"
#include "mqtt.h"

static const char *TAG = "main";


static void app_task(void *pvParameters)
{
    /* Generate topic ID from clid */
    static char topic[128];
    static char element[128];
    static char msg[1024];
    sprintf(topic,"raw/%s/ultra",mqtt_clid);
    ESP_LOGI(TAG,"Publish topic is %s",topic);

    while(1)
    {
        /* Publish to MQTT every 1 minute */
        int16_t count = 0;
        uint16_t sample = maxbotix_get_latest();
        int32_t age = maxbotix_get_age();
        float result = maxbotix_get_median(0.6f,8,64,&count);
        ESP_LOGI(TAG,"Median sample returned %f, sample count %d",(double)result,count);
        
        /* Start with curly brace */
        strcpy(msg,"{");
        /* Always add latest reading */
        sprintf(element,"\"Latest\": %d,",sample);
        strcat(msg,element);

        /* Always add age */
        sprintf(element,"\"Age\": %d",age);
        strcat(msg,element);

        /* Include result and count if not negative */
        if(result >= 0)
        {
            sprintf(element,", \"Dist\": %f,",result);
            strcat(msg,element);
            sprintf(element," \"Count\": %d",count);
            strcat(msg,element);
        }
        /* Add trailing curly brace */
        strcat(msg,"}");
        ESP_LOGI(TAG,"Publish topic is %s",topic);
        ESP_LOGI(TAG,"Payload to publish is %s",msg);

        /* Actually publish the message */
        int id = esp_mqtt_client_publish(mqtt_client, topic, msg, 0, 0, 0);
        ESP_LOGI(TAG,"Published mesage with ID %d",id);

        /* Wait delay for 1 minute interval */
        vTaskDelay(pdMS_TO_TICKS(1000*60));
    }
}

void app_main(void)
{
    /* Call Ethernet function */
    eth_init();

    /* Call MQTT function */
    mqtt_init();

    /* Call Maxbotix function */
    maxbotix_init();

    /* Reduce logging from maxbotix */
    esp_log_level_set("maxbotix", ESP_LOG_WARN);

    /* Start main task at lower priority */
    xTaskCreatePinnedToCore(app_task, "app_task", 2048, NULL, 1, NULL,1);
}
