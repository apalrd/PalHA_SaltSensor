/* Ethernet code, derived from Ethernet basic driver */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_eth_phy.h"
#include "esp_eth_mac.h"
#include "esp_eth_com.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "lwip/err.h"
#include "lwip/dns.h"

static const char *TAG = "ethernet";

char eth_mac[6];

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 eth_mac[0], eth_mac[1], eth_mac[2], eth_mac[3], eth_mac[4], eth_mac[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

void eth_init(void)
{
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    // Set default handlers to process TCP/IP stuffs
    ESP_ERROR_CHECK(esp_eth_set_default_handlers(eth_netif));
    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    /* Initialize and configure struct for MAC */
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    //MDC and MDIO are 23 and 18 respectively
    mac_config.smi_mdc_gpio_num = 23;
    mac_config.smi_mdio_gpio_num = 18;
    //Lengthen timeout period
    mac_config.sw_reset_timeout_ms = 1000;
    //Internal ESP32 MAC
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&mac_config);

    if(mac == NULL){
        ESP_LOGE(TAG,"esp_eth_mac_new_esp32 failed");
        abort();
    }
    ESP_LOGI(TAG,"esp_eth_mac_new complete");

    /* Initialize and configure struct for PHY */
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    //PHY is at address 0
    phy_config.phy_addr = 0;
    //Reset is the power control, GPIO 12
    //phy_config.reset_gpio_num = 12;
    phy_config.reset_gpio_num = -1;
    //LAN8270 is used as the PHY
    esp_eth_phy_t *phy = esp_eth_phy_new_lan8720(&phy_config);

    if(phy == NULL){
        ESP_LOGE(TAG,"esp_eth_phy_new failed");
        abort();
    }
    ESP_LOGI(TAG,"esp_eth_phy_new complete");

    /* Configure GPIO for PHY power */
    gpio_pad_select_gpio(12);
    gpio_set_direction(12,GPIO_MODE_OUTPUT);
    gpio_set_level(12, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG,"Configure PHY power complete");

    /* Configure Ethernet module with the MAC and PHY */
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_LOGI(TAG,"Created default Ethernet config");
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

    /* attach Ethernet driver to TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    /* Get MAC Addr so the rest of the code can use it */
    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, eth_mac);
    ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 eth_mac[0], eth_mac[1], eth_mac[2], eth_mac[3], eth_mac[4], eth_mac[5]);

    ESP_LOGI(TAG,"Completed Ethernet initialization");
}
