#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

typedef enum {
    WIFI_STATE_IDLE = 0,
    WIFI_STATE_AP_MODE,
    WIFI_STATE_STA_CONNECTING,
    WIFI_STATE_STA_CONNECTED,
    WIFI_STATE_STA_DISCONNECTED,
    WIFI_STATE_STA_RECONNECTING,
    WIFI_STATE_STA_FAILED,
} wifi_state_t;

#define WIFI_MANAGER_EVENT_IDLE             BIT0
#define WIFI_MANAGER_EVENT_AP_MODE          BIT1
#define WIFI_MANAGER_EVENT_STA_CONNECTING   BIT2
#define WIFI_MANAGER_EVENT_STA_CONNECTED    BIT3
#define WIFI_MANAGER_EVENT_STA_DISCONNECTED BIT4
#define WIFI_MANAGER_EVENT_STA_RECONNECTING BIT5
#define WIFI_MANAGER_EVENT_STA_FAILED       BIT6
#define WIFI_MANAGER_ALL_STATE_BITS         (WIFI_MANAGER_EVENT_IDLE             \
                                             | WIFI_MANAGER_EVENT_AP_MODE        \
                                             | WIFI_MANAGER_EVENT_STA_CONNECTING \
                                             | WIFI_MANAGER_EVENT_STA_CONNECTED  \
                                             | WIFI_MANAGER_EVENT_STA_DISCONNECTED \
                                             | WIFI_MANAGER_EVENT_STA_RECONNECTING \
                                             | WIFI_MANAGER_EVENT_STA_FAILED)

typedef struct {
    wifi_state_t state;
    bool has_credentials;
    char ssid[33];
    char ip[16];
    int rssi;
    char message[64];
} wifi_status_t;

typedef struct {
    EventGroupHandle_t app_event_group;
} wifi_manager_config_t;

esp_err_t wifi_manager_init(const wifi_manager_config_t *config);
esp_err_t wifi_manager_deinit(void);
esp_err_t wifi_manager_start(void);
esp_err_t wifi_manager_clear_credentials(void);
esp_err_t wifi_manager_get_status(wifi_status_t *status);
const char *wifi_manager_state_to_string(wifi_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H */
