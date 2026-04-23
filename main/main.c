#include <stdbool.h>
#include <stdio.h>

#include "button_manager.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ld6002c.h"
#include "led_indicator.h"
#include "nvs_flash.h"
#include "oled_display.h"
#include "wifi_manager.h"

#define FALLGUARD_BUTTON_GPIO GPIO_NUM_0
#define FALLGUARD_LED_GPIO    GPIO_NUM_2
#define FALLGUARD_OLED_SDA_GPIO GPIO_NUM_8
#define FALLGUARD_OLED_SCL_GPIO GPIO_NUM_9
#define FALLGUARD_OLED_I2C_ADDR 0x3C
#define FALLGUARD_OLED_WIDTH    128
#define FALLGUARD_OLED_HEIGHT   64
#define BUTTON_EVENT_QUEUE_LEN 8

static const char *TAG = "fallguard";

static EventGroupHandle_t s_app_event_group;
static QueueHandle_t s_button_event_queue;
static bool s_radar_initialized;
static bool s_led_error_latched;

static wifi_state_t app_wifi_state_from_bits(EventBits_t bits)
{
    if ((bits & WIFI_MANAGER_EVENT_AP_MODE) != 0U) {
        return WIFI_STATE_AP_MODE;
    }
    if ((bits & WIFI_MANAGER_EVENT_STA_CONNECTING) != 0U) {
        return WIFI_STATE_STA_CONNECTING;
    }
    if ((bits & WIFI_MANAGER_EVENT_STA_CONNECTED) != 0U) {
        return WIFI_STATE_STA_CONNECTED;
    }
    if ((bits & WIFI_MANAGER_EVENT_STA_DISCONNECTED) != 0U) {
        return WIFI_STATE_STA_DISCONNECTED;
    }
    if ((bits & WIFI_MANAGER_EVENT_STA_RECONNECTING) != 0U) {
        return WIFI_STATE_STA_RECONNECTING;
    }
    if ((bits & WIFI_MANAGER_EVENT_STA_FAILED) != 0U) {
        return WIFI_STATE_STA_FAILED;
    }
    return WIFI_STATE_IDLE;
}

static void app_apply_led_mode(wifi_state_t state)
{
    led_mode_t led_mode = LED_MODE_AP;

    if (s_led_error_latched) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(led_indicator_set_mode(LED_MODE_ERROR));
        return;
    }

    switch (state) {
    case WIFI_STATE_STA_CONNECTING:
    case WIFI_STATE_STA_RECONNECTING:
    case WIFI_STATE_STA_DISCONNECTED:
        led_mode = LED_MODE_STA_CONNECTING;
        break;
    case WIFI_STATE_STA_CONNECTED:
        led_mode = LED_MODE_ONLINE;
        break;
    case WIFI_STATE_IDLE:
    case WIFI_STATE_AP_MODE:
    case WIFI_STATE_STA_FAILED:
    default:
        led_mode = LED_MODE_AP;
        break;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(led_indicator_set_mode(led_mode));
}

static void app_log_status_snapshot(const char *reason)
{
    wifi_status_t status;
    esp_err_t err = wifi_manager_get_status(&status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Status snapshot failed (%s): %s", reason, esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG,
             "Status snapshot [%s]: state=%s ip=%s has_credentials=%s rssi=%d radar_ready=%s",
             reason,
             wifi_manager_state_to_string(status.state),
             status.ip[0] != '\0' ? status.ip : "-",
             status.has_credentials ? "true" : "false",
             status.rssi,
             s_radar_initialized ? "true" : "false");
    if (status.message[0] != '\0') {
        ESP_LOGI(TAG, "Status detail [%s]: %s", reason, status.message);
    }
}

static void app_handle_short_press(void)
{
    ESP_LOGI(TAG, "Short press detected");
    app_log_status_snapshot("short_press");
}

static void app_handle_long_press(void)
{
    esp_err_t err;

    ESP_LOGW(TAG, "Long press detected, clearing Wi-Fi credentials");
    err = wifi_manager_clear_credentials();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear Wi-Fi credentials: %s", esp_err_to_name(err));
        return;
    }

    err = wifi_manager_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart Wi-Fi provisioning: %s", esp_err_to_name(err));
    }
}

static void app_factory_reset(void)
{
    ESP_LOGW(TAG, "Factory reset requested");

    ld6002c_deinit();
    oled_display_deinit();
    wifi_manager_deinit();
    button_manager_deinit();
    led_indicator_deinit();

    nvs_flash_deinit();
    ESP_ERROR_CHECK(nvs_flash_erase());
    esp_restart();
}

static void app_handle_button_event(button_event_t event)
{
    switch (event) {
    case BUTTON_EVENT_SHORT_PRESS:
        app_handle_short_press();
        break;
    case BUTTON_EVENT_LONG_PRESS:
        app_handle_long_press();
        break;
    case BUTTON_EVENT_FACTORY_RESET:
        app_factory_reset();
        break;
    default:
        break;
    }
}

static void app_on_fall_status(const ld6002c_fall_status_t *status)
{
    if (status == NULL) {
        return;
    }

    ESP_LOGI(TAG, "Radar fall status: is_fall=%s", status->is_fall ? "true" : "false");
    ESP_ERROR_CHECK_WITHOUT_ABORT(oled_display_update_fall_status(status));
}

static void app_on_fw_status(const ld6002c_fw_status_t *status)
{
    if (status == NULL) {
        return;
    }

    ESP_LOGI(TAG, "Radar firmware: project=%d version=%u.%u.%u",
             (int)status->project, status->major, status->sub, status->modified);
    ESP_ERROR_CHECK_WITHOUT_ABORT(oled_display_update_fw_status(status));
}

static void app_on_params(const ld6002c_params_t *params)
{
    if (params == NULL) {
        ESP_LOGW(TAG, "Radar parameters query failed");
        return;
    }

    ESP_LOGI(TAG,
             "Radar params: height=%.2f threshold=%.2f sensitivity=%lu zone=[%.2f %.2f %.2f %.2f]",
             params->high,
             params->threshold,
             (unsigned long)params->sensitivity,
             params->rect_XL,
             params->rect_XR,
             params->rect_ZF,
             params->rect_ZB);
    ESP_ERROR_CHECK_WITHOUT_ABORT(oled_display_update_params(params));
}

static void app_on_height_upload(const ld6002c_height_upload_t *result)
{
    if (result == NULL) {
        return;
    }

    ESP_LOGI(TAG, "Radar height upload: value=%lu", (unsigned long)result->value);
    ESP_ERROR_CHECK_WITHOUT_ABORT(oled_display_update_height_upload(result));
}

static void app_on_3d_cloud(const ld6002c_3d_cloud_t *cloud)
{
    if (cloud == NULL) {
        return;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(oled_display_update_3d_cloud(cloud));
}

static void app_on_human_status(const ld6002c_human_status_t *status)
{
    if (status == NULL) {
        return;
    }

    ESP_LOGI(TAG, "Radar human status: is_human=%s", status->is_human ? "true" : "false");
    ESP_ERROR_CHECK_WITHOUT_ABORT(oled_display_update_human_status(status));
}

static void app_init_radar(void)
{
    ld6002c_config_t radar_config = {
        .uart_port = LD6002C_UART_PORT,
        .tx_pin = LD6002C_DEFAULT_TX_PIN,
        .rx_pin = LD6002C_DEFAULT_RX_PIN,
        .callbacks = {
            .on_fall_status = app_on_fall_status,
            .on_fw_status = app_on_fw_status,
            .on_params = app_on_params,
            .on_set_height = NULL,
            .on_set_threshold = NULL,
            .on_set_sensitivity = NULL,
            .on_set_alarm_zone = NULL,
            .on_height_upload = app_on_height_upload,
            .on_3d_cloud = app_on_3d_cloud,
            .on_human_status = app_on_human_status,
        },
    };

    esp_err_t err = ld6002c_init(&radar_config);
    if (err != ESP_OK) {
        s_radar_initialized = false;
        s_led_error_latched = true;
        ESP_LOGE(TAG, "Radar init failed: %s", esp_err_to_name(err));
        ESP_ERROR_CHECK_WITHOUT_ABORT(oled_display_set_radar_ready(false));
        ESP_ERROR_CHECK_WITHOUT_ABORT(led_indicator_set_mode(LED_MODE_ERROR));
        return;
    }

    s_radar_initialized = true;
    ESP_ERROR_CHECK_WITHOUT_ABORT(oled_display_set_radar_ready(true));
    ESP_LOGI(TAG, "Radar init complete");

    err = ld6002c_query_fw_status();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Radar firmware query failed: %s", esp_err_to_name(err));
    }

    err = ld6002c_get_params();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Radar parameter query failed: %s", esp_err_to_name(err));
    }
}

static void app_init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    wifi_state_t last_wifi_state = WIFI_STATE_IDLE;
    esp_err_t oled_err;

    app_init_nvs();

    s_app_event_group = xEventGroupCreate();
    s_button_event_queue = xQueueCreate(BUTTON_EVENT_QUEUE_LEN, sizeof(button_event_t));
    if (s_app_event_group == NULL || s_button_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to allocate application resources");
        abort();
    }

    ESP_ERROR_CHECK(led_indicator_init(&(led_indicator_config_t){
        .gpio_num = FALLGUARD_LED_GPIO,
    }));

    ESP_ERROR_CHECK(button_manager_init(&(button_manager_config_t){
        .gpio_num = FALLGUARD_BUTTON_GPIO,
        .event_queue = s_button_event_queue,
    }));

    ESP_ERROR_CHECK(wifi_manager_init(&(wifi_manager_config_t){
        .app_event_group = s_app_event_group,
    }));
    ESP_ERROR_CHECK(wifi_manager_start());

    oled_err = oled_display_init(&(oled_display_config_t){
        .i2c_port = I2C_NUM_0,
        .sda_gpio_num = FALLGUARD_OLED_SDA_GPIO,
        .scl_gpio_num = FALLGUARD_OLED_SCL_GPIO,
        .i2c_addr = FALLGUARD_OLED_I2C_ADDR,
        .width = FALLGUARD_OLED_WIDTH,
        .height = FALLGUARD_OLED_HEIGHT,
    });
    if (oled_err != ESP_OK) {
        ESP_LOGE(TAG, "OLED init failed: %s", esp_err_to_name(oled_err));
    } else {
        ESP_ERROR_CHECK_WITHOUT_ABORT(oled_display_set_radar_ready(false));
    }

    app_init_radar();
    ESP_LOGI(TAG, "FallGuard boot ready");

    while (true) {
        button_event_t button_event;
        if (xQueueReceive(s_button_event_queue, &button_event, pdMS_TO_TICKS(200)) == pdPASS) {
            app_handle_button_event(button_event);
        }

        wifi_state_t current_state = app_wifi_state_from_bits(xEventGroupGetBits(s_app_event_group));
        if (current_state != last_wifi_state) {
            last_wifi_state = current_state;
            ESP_LOGI(TAG, "Wi-Fi state changed: %s", wifi_manager_state_to_string(current_state));
            app_apply_led_mode(current_state);
        }
    }
}
