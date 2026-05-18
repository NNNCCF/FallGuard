#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    const char *host;               /* MQTT broker host */
    uint16_t    port;               /* MQTT broker port, 0 = 8883 */
    const char *device_id;          /* Device ID, also used as MQTT client ID */
    const char *username;           /* MQTT username */
    const char *password;           /* MQTT password fallback; prefer NVS */
    const char *server_cert_pem;    /* Broker CA/server certificate in PEM format */
    uint32_t    interval_ms;        /* Publish interval in ms, 0 = 1000 */
} mqtt_reporter_config_t;

esp_err_t mqtt_reporter_init(const mqtt_reporter_config_t *config);
void      mqtt_reporter_set_fall(bool is_fall);
void      mqtt_reporter_set_presence(bool is_present);
void      mqtt_reporter_deinit(void);
