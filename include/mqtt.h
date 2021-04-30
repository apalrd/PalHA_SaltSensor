#include "mqtt_client.h"
#pragma once

/* Configure MQTT for the project */
#define MQTT_BROKER_URL "mqtt://telstar.palnet.net:1883"

/* Client ID */
extern char mqtt_clid[16];

/* Client object */
extern esp_mqtt_client_handle_t mqtt_client;

/* Functions */
void mqtt_init(void);