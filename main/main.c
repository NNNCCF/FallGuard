#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "button_manager.h"
#include "buzzer_alarm.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ld6002c.h"
#include "led_indicator.h"
#include "mqtt_reporter.h"
#include "nvs_flash.h"
#include "wifi_manager.h"

#define MQTT_BROKER_HOST  "116.204.127.178"
#define MQTT_BROKER_PORT  1883
#define MQTT_DEVICE_ID    "DEV-MINI-001"
#define MQTT_USERNAME     "DEV-MINI-001"
#define MQTT_PASSWORD     "HvBABsArDz7L5XMqyMAXCD0sUzdZSu6m"

#define FALLGUARD_BUTTON_GPIO GPIO_NUM_0
#define FALLGUARD_LED_GPIO    GPIO_NUM_2
#define FALLGUARD_BUZZER_GPIO GPIO_NUM_39
#define FALLGUARD_BOOT_BEEP_MS 35
#define FALLGUARD_PARAM_QUERY_START_DELAY_MS 1000
#define FALLGUARD_PARAM_QUERY_RETRY_MS 3000
#define FALLGUARD_PARAM_QUERY_MAX_ATTEMPTS 5
#define FALLGUARD_RADAR_PROFILE_ACK_TIMEOUT_MS 3000
#define FALLGUARD_3D_ENABLE_DELAY_MS 10000
#define FALLGUARD_3D_DIAG_TIMEOUT_MS 5000
#define BUTTON_EVENT_QUEUE_LEN 8
#define FALLGUARD_RADAR_HEIGHT_M 2.7f
#define FALLGUARD_RADAR_THRESHOLD_M 0.5f
#define FALLGUARD_RADAR_SENSITIVITY 30U
#define FALLGUARD_RADAR_ZONE_BOUNDARY_M 1.5f
#define FALLGUARD_ENABLE_USER_LOG 0
#define FALLGUARD_ENABLE_3D_CLOUD 0

static const char *TAG = "fallguard";

typedef enum {
    RADAR_PROFILE_STEP_HEIGHT = 0,
    RADAR_PROFILE_STEP_THRESHOLD,
    RADAR_PROFILE_STEP_SENSITIVITY,
    RADAR_PROFILE_STEP_ALARM_ZONE,
    RADAR_PROFILE_STEP_DONE,
    RADAR_PROFILE_STEP_FAILED,
} radar_profile_step_t;

static EventGroupHandle_t s_app_event_group;
static QueueHandle_t s_button_event_queue;
static bool s_radar_initialized;
static bool s_led_error_latched;
static bool s_3d_cloud_enabled;
static TickType_t s_boot_tick;
static bool s_3d_cloud_request_sent;
static TickType_t s_3d_cloud_enable_tick;
static bool s_3d_cloud_seen;
static bool s_3d_cloud_nonzero_logged;
static bool s_3d_cloud_diag_logged;
static bool s_fw_status_received;
static bool s_params_received;
static bool s_params_unavailable;
static bool s_radar_stream_seen;
static uint8_t s_param_query_attempts;
static TickType_t s_param_query_last_tick;
static TickType_t s_radar_stream_first_tick;
static ld6002c_project_t s_radar_project;
static radar_profile_step_t s_radar_profile_step;
static bool s_radar_profile_command_in_flight;
static TickType_t s_radar_profile_command_tick;

static const char *app_radar_project_to_string(ld6002c_project_t project)
{
    switch (project) {
    case PROJECT_PRESENCE:
        return "PRESENCE";
    case PROJECT_BREATH:
        return "BREATH";
    case PROJECT_GESTURE:
        return "GESTURE";
    case PROJECT_RANGING:
        return "RANGING";
    case PROJECT_PEOPLE_CNT:
        return "PEOPLE_CNT";
    case PROJECT_3D_CLOUD:
        return "3D_CLOUD";
    default:
        return "UNKNOWN";
    }
}

static void app_mark_radar_stream_seen(void)
{
    if (!s_radar_stream_seen) {
        s_radar_stream_seen = true;
        s_radar_stream_first_tick = xTaskGetTickCount();
        ESP_LOGI(TAG, "Radar stream detected; parameter query will start after %d ms",
                 FALLGUARD_PARAM_QUERY_START_DELAY_MS);
    }
}

static const char *app_radar_profile_step_to_string(radar_profile_step_t step)
{
    switch (step) {
    case RADAR_PROFILE_STEP_HEIGHT:
        return "height";
    case RADAR_PROFILE_STEP_THRESHOLD:
        return "threshold";
    case RADAR_PROFILE_STEP_SENSITIVITY:
        return "sensitivity";
    case RADAR_PROFILE_STEP_ALARM_ZONE:
        return "alarm_zone";
    case RADAR_PROFILE_STEP_DONE:
        return "done";
    case RADAR_PROFILE_STEP_FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

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

    mqtt_reporter_deinit();
    ld6002c_deinit();
    buzzer_alarm_deinit();
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

    app_mark_radar_stream_seen();
    mqtt_reporter_set_fall(status->is_fall);
    ESP_ERROR_CHECK_WITHOUT_ABORT(buzzer_alarm_set_active(status->is_fall));
}

static void app_on_fw_status(const ld6002c_fw_status_t *status)
{
    if (status == NULL) {
        return;
    }

    app_mark_radar_stream_seen();
    s_fw_status_received = true;
    s_radar_project = status->project;
    ESP_LOGI(TAG, "Radar firmware: project=%s(%d) version=%u.%u.%u",
             app_radar_project_to_string(status->project),
             (int)status->project,
             status->major,
             status->sub,
             status->modified);
}

static void app_on_params(const ld6002c_params_t *params)
{
    if (params == NULL) {
        ESP_LOGW(TAG, "Radar parameters query failed");
        return;
    }

    app_mark_radar_stream_seen();
    s_params_received = true;
    s_params_unavailable = false;
    ESP_LOGI(TAG,
             "Radar params: height=%.2f threshold=%.2f sensitivity=%lu zone=[%.2f %.2f %.2f %.2f]",
             params->high,
             params->threshold,
             (unsigned long)params->sensitivity,
             params->rect_XL,
             params->rect_XR,
             params->rect_ZF,
             params->rect_ZB);
}

static void app_handle_radar_profile_result(radar_profile_step_t expected_step, uint8_t result)
{
    if (!s_radar_profile_command_in_flight || s_radar_profile_step != expected_step) {
        ESP_LOGW(TAG, "Unexpected radar profile ack for %s (in_flight=%s current_step=%s result=%u)",
                 app_radar_profile_step_to_string(expected_step),
                 s_radar_profile_command_in_flight ? "true" : "false",
                 app_radar_profile_step_to_string(s_radar_profile_step),
                 (unsigned int)result);
        return;
    }

    s_radar_profile_command_in_flight = false;
    s_radar_profile_command_tick = 0;

    if (result != 1U) {
        s_radar_profile_step = RADAR_PROFILE_STEP_FAILED;
        ESP_LOGE(TAG, "Radar profile step failed: %s", app_radar_profile_step_to_string(expected_step));
        return;
    }

    ESP_LOGI(TAG, "Radar profile step applied: %s", app_radar_profile_step_to_string(expected_step));
    switch (expected_step) {
    case RADAR_PROFILE_STEP_HEIGHT:
        s_radar_profile_step = RADAR_PROFILE_STEP_THRESHOLD;
        break;
    case RADAR_PROFILE_STEP_THRESHOLD:
        s_radar_profile_step = RADAR_PROFILE_STEP_SENSITIVITY;
        break;
    case RADAR_PROFILE_STEP_SENSITIVITY:
        s_radar_profile_step = RADAR_PROFILE_STEP_ALARM_ZONE;
        break;
    case RADAR_PROFILE_STEP_ALARM_ZONE: {
        s_radar_profile_step = RADAR_PROFILE_STEP_DONE;
        ESP_LOGI(TAG,
                 "Radar profile applied: height=%.2f threshold=%.2f sensitivity=%lu zone=[%.2f %.2f %.2f %.2f]",
                 FALLGUARD_RADAR_HEIGHT_M,
                 FALLGUARD_RADAR_THRESHOLD_M,
                 (unsigned long)FALLGUARD_RADAR_SENSITIVITY,
                 FALLGUARD_RADAR_ZONE_BOUNDARY_M,
                 FALLGUARD_RADAR_ZONE_BOUNDARY_M,
                 FALLGUARD_RADAR_ZONE_BOUNDARY_M,
                 FALLGUARD_RADAR_ZONE_BOUNDARY_M);
        printf("OK\r\n");

        esp_err_t err = ld6002c_get_params();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Radar parameter query after profile apply failed: %s", esp_err_to_name(err));
        }
        break;
    }
    case RADAR_PROFILE_STEP_DONE:
    case RADAR_PROFILE_STEP_FAILED:
    default:
        break;
    }
}

static void app_on_set_height(uint8_t result)
{
    app_mark_radar_stream_seen();
    app_handle_radar_profile_result(RADAR_PROFILE_STEP_HEIGHT, result);
}

static void app_on_set_threshold(uint8_t result)
{
    app_mark_radar_stream_seen();
    app_handle_radar_profile_result(RADAR_PROFILE_STEP_THRESHOLD, result);
}

static void app_on_set_sensitivity(uint8_t result)
{
    app_mark_radar_stream_seen();
    app_handle_radar_profile_result(RADAR_PROFILE_STEP_SENSITIVITY, result);
}

static void app_on_set_alarm_zone(uint8_t result)
{
    app_mark_radar_stream_seen();
    app_handle_radar_profile_result(RADAR_PROFILE_STEP_ALARM_ZONE, result);
}

static void app_try_query_params(const char *reason)
{
    if (s_params_received || s_params_unavailable) {
        return;
    }

    if (s_param_query_attempts >= FALLGUARD_PARAM_QUERY_MAX_ATTEMPTS) {
        s_params_unavailable = true;
        ESP_LOGW(TAG,
                 "Radar params unavailable after %u attempts; leaving parameter page in N/A state",
                 (unsigned int)s_param_query_attempts);
        return;
    }

    esp_err_t err = ld6002c_get_params();
    s_param_query_last_tick = xTaskGetTickCount();
    s_param_query_attempts++;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Radar parameter query attempt %u failed (%s): %s",
                 (unsigned int)s_param_query_attempts,
                 reason,
                 esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Radar parameter query attempt %u sent (%s)",
             (unsigned int)s_param_query_attempts, reason);
}

static void app_on_height_upload(const ld6002c_height_upload_t *result)
{
    if (result == NULL) {
        return;
    }

    app_mark_radar_stream_seen();
}

static void app_on_3d_cloud(const ld6002c_3d_cloud_t *cloud)
{
    if (cloud == NULL) {
        return;
    }

    app_mark_radar_stream_seen();
    if (!s_3d_cloud_enabled) {
        return;
    }

    if (!s_3d_cloud_seen) {
        ESP_LOGI(TAG, "Received first 3D cloud frame: targets=%ld", (long)cloud->target_num);
    }
    s_3d_cloud_seen = true;
    if (cloud->target_num > 0 && !s_3d_cloud_nonzero_logged) {
        ESP_LOGI(TAG, "Received populated 3D cloud frame: targets=%ld", (long)cloud->target_num);
        s_3d_cloud_nonzero_logged = true;
    }
}

static void app_maybe_enable_3d_cloud(void)
{
#if FALLGUARD_ENABLE_3D_CLOUD && FALLGUARD_ENABLE_USER_LOG
    if (s_3d_cloud_enabled || !s_radar_initialized) {
        return;
    }

    if ((xTaskGetTickCount() - s_boot_tick) < pdMS_TO_TICKS(FALLGUARD_3D_ENABLE_DELAY_MS)) {
        return;
    }

    if (s_3d_cloud_request_sent) {
        return;
    }

    s_3d_cloud_request_sent = true;
    if (s_fw_status_received) {
        ESP_LOGI(TAG, "Requesting 3D cloud stream, radar project=%s(%d)",
                 app_radar_project_to_string(s_radar_project),
                 (int)s_radar_project);
        if (s_radar_project != PROJECT_3D_CLOUD) {
            ESP_LOGW(TAG,
                     "Current radar project is %s(%d); the official LD6002C protocol only documents auto-uploaded 0x0A08 cloud frames for project 5 after User log is enabled",
                     app_radar_project_to_string(s_radar_project),
                     (int)s_radar_project);
        }
    } else {
        ESP_LOGW(TAG, "Requesting 3D cloud stream before firmware project is reported");
    }

    esp_err_t err = ld6002c_set_user_log(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable 3D cloud stream: %s", esp_err_to_name(err));
        return;
    }

    s_3d_cloud_enable_tick = xTaskGetTickCount();
    s_3d_cloud_enabled = true;
    ESP_LOGI(TAG, "3D cloud stream enabled after %d ms", FALLGUARD_3D_ENABLE_DELAY_MS);
#else
    return;
#endif
}

static void app_maybe_log_3d_diagnostic(void)
{
#if FALLGUARD_ENABLE_3D_CLOUD && FALLGUARD_ENABLE_USER_LOG
    if (!s_3d_cloud_enabled || s_3d_cloud_seen || s_3d_cloud_diag_logged) {
        return;
    }

    if ((xTaskGetTickCount() - s_3d_cloud_enable_tick) < pdMS_TO_TICKS(FALLGUARD_3D_DIAG_TIMEOUT_MS)) {
        return;
    }

    s_3d_cloud_diag_logged = true;
    if (s_fw_status_received) {
        ESP_LOGW(TAG,
                 "No documented 0x0A08 3D cloud frame received within %d ms after enabling User log; current radar project=%s(%d)",
                 FALLGUARD_3D_DIAG_TIMEOUT_MS,
                 app_radar_project_to_string(s_radar_project),
                 (int)s_radar_project);
    } else {
        ESP_LOGW(TAG,
                 "No documented 0x0A08 3D cloud frame received within %d ms after enabling User log, and radar firmware project is still unknown",
                 FALLGUARD_3D_DIAG_TIMEOUT_MS);
    }
#else
    return;
#endif
}

static void app_maybe_retry_params(void)
{
    if (s_params_received || s_params_unavailable || !s_radar_initialized) {
        return;
    }

    if (s_radar_profile_step != RADAR_PROFILE_STEP_DONE &&
        s_radar_profile_step != RADAR_PROFILE_STEP_FAILED) {
        return;
    }

    if (!s_radar_stream_seen) {
        return;
    }

    if ((xTaskGetTickCount() - s_radar_stream_first_tick)
        < pdMS_TO_TICKS(FALLGUARD_PARAM_QUERY_START_DELAY_MS)) {
        return;
    }

    if (s_param_query_attempts == 0) {
        app_try_query_params("after_stream_detected");
        return;
    }

    if ((xTaskGetTickCount() - s_param_query_last_tick) < pdMS_TO_TICKS(FALLGUARD_PARAM_QUERY_RETRY_MS)) {
        return;
    }

    app_try_query_params("retry");
}

static void app_maybe_apply_radar_profile(void)
{
    esp_err_t err = ESP_OK;

    if (!s_radar_initialized || !s_radar_stream_seen || s_radar_profile_command_in_flight) {
        return;
    }

    switch (s_radar_profile_step) {
    case RADAR_PROFILE_STEP_HEIGHT:
        err = ld6002c_set_height(FALLGUARD_RADAR_HEIGHT_M);
        break;
    case RADAR_PROFILE_STEP_THRESHOLD:
        err = ld6002c_set_threshold(FALLGUARD_RADAR_THRESHOLD_M);
        break;
    case RADAR_PROFILE_STEP_SENSITIVITY:
        err = ld6002c_set_sensitivity(FALLGUARD_RADAR_SENSITIVITY);
        break;
    case RADAR_PROFILE_STEP_ALARM_ZONE:
        err = ld6002c_set_alarm_zone(FALLGUARD_RADAR_ZONE_BOUNDARY_M,
                                     FALLGUARD_RADAR_ZONE_BOUNDARY_M,
                                     FALLGUARD_RADAR_ZONE_BOUNDARY_M,
                                     FALLGUARD_RADAR_ZONE_BOUNDARY_M);
        break;
    case RADAR_PROFILE_STEP_DONE:
    case RADAR_PROFILE_STEP_FAILED:
    default:
        return;
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send radar profile step %s: %s",
                 app_radar_profile_step_to_string(s_radar_profile_step),
                 esp_err_to_name(err));
        s_radar_profile_step = RADAR_PROFILE_STEP_FAILED;
        return;
    }

    s_radar_profile_command_in_flight = true;
    s_radar_profile_command_tick = xTaskGetTickCount();
    ESP_LOGI(TAG, "Radar profile command sent: %s",
             app_radar_profile_step_to_string(s_radar_profile_step));
}

static void app_maybe_timeout_radar_profile(void)
{
    if (!s_radar_profile_command_in_flight) {
        return;
    }

    if ((xTaskGetTickCount() - s_radar_profile_command_tick)
        < pdMS_TO_TICKS(FALLGUARD_RADAR_PROFILE_ACK_TIMEOUT_MS)) {
        return;
    }

    ESP_LOGE(TAG, "Timed out waiting for radar profile ack: %s",
             app_radar_profile_step_to_string(s_radar_profile_step));
    s_radar_profile_command_in_flight = false;
    s_radar_profile_command_tick = 0;
    s_radar_profile_step = RADAR_PROFILE_STEP_FAILED;
}

static void app_on_human_status(const ld6002c_human_status_t *status)
{
    if (status == NULL) {
        return;
    }

    app_mark_radar_stream_seen();
    mqtt_reporter_set_presence(status->is_human);
    ESP_LOGI(TAG, "Radar human status: is_human=%s", status->is_human ? "true" : "false");
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
            .on_set_height = app_on_set_height,
            .on_set_threshold = app_on_set_threshold,
            .on_set_sensitivity = app_on_set_sensitivity,
            .on_set_alarm_zone = app_on_set_alarm_zone,
            .on_height_upload = app_on_height_upload,
            .on_user_log = NULL,
            .on_3d_cloud = app_on_3d_cloud,
            .on_human_status = app_on_human_status,
        },
    };

    esp_err_t err = ld6002c_init(&radar_config);
    if (err != ESP_OK) {
        s_radar_initialized = false;
        s_led_error_latched = true;
        ESP_LOGE(TAG, "Radar init failed: %s", esp_err_to_name(err));
        ESP_ERROR_CHECK_WITHOUT_ABORT(led_indicator_set_mode(LED_MODE_ERROR));
        return;
    }

    s_radar_initialized = true;
    ESP_LOGI(TAG, "Radar init complete");

#if !FALLGUARD_ENABLE_USER_LOG
    err = ld6002c_set_user_log(false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Radar User log disable failed: %s", esp_err_to_name(err));
    }
#endif

    err = ld6002c_query_fw_status();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Radar firmware query failed: %s", esp_err_to_name(err));
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

    app_init_nvs();
    s_boot_tick = xTaskGetTickCount();
    s_3d_cloud_enabled = false;
    s_3d_cloud_request_sent = false;
    s_3d_cloud_enable_tick = 0;
    s_3d_cloud_seen = false;
    s_3d_cloud_nonzero_logged = false;
    s_3d_cloud_diag_logged = false;
    s_fw_status_received = false;
    s_params_received = false;
    s_params_unavailable = false;
    s_radar_stream_seen = false;
    s_param_query_attempts = 0;
    s_param_query_last_tick = 0;
    s_radar_stream_first_tick = 0;
    s_radar_project = PROJECT_PRESENCE;
    s_radar_profile_step = RADAR_PROFILE_STEP_HEIGHT;
    s_radar_profile_command_in_flight = false;
    s_radar_profile_command_tick = 0;

    s_app_event_group = xEventGroupCreate();
    s_button_event_queue = xQueueCreate(BUTTON_EVENT_QUEUE_LEN, sizeof(button_event_t));
    if (s_app_event_group == NULL || s_button_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to allocate application resources");
        abort();
    }

    ESP_ERROR_CHECK(led_indicator_init(&(led_indicator_config_t){
        .gpio_num = FALLGUARD_LED_GPIO,
    }));

    ESP_ERROR_CHECK(buzzer_alarm_init(&(buzzer_alarm_config_t){
        .gpio_num = FALLGUARD_BUZZER_GPIO,
    }));
    ESP_ERROR_CHECK(buzzer_alarm_beep_once(FALLGUARD_BOOT_BEEP_MS));

    ESP_ERROR_CHECK(button_manager_init(&(button_manager_config_t){
        .gpio_num = FALLGUARD_BUTTON_GPIO,
        .event_queue = s_button_event_queue,
    }));

    ESP_ERROR_CHECK(wifi_manager_init(&(wifi_manager_config_t){
        .app_event_group = s_app_event_group,
    }));
    ESP_ERROR_CHECK(wifi_manager_start());

    app_init_radar();

    ESP_ERROR_CHECK(mqtt_reporter_init(&(mqtt_reporter_config_t){
        .host        = MQTT_BROKER_HOST,
        .port        = MQTT_BROKER_PORT,
        .device_id   = MQTT_DEVICE_ID,
        .username    = MQTT_USERNAME,
        .password    = MQTT_PASSWORD,
        .interval_ms = 1000,
    }));

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

        app_maybe_enable_3d_cloud();
        app_maybe_log_3d_diagnostic();
        app_maybe_timeout_radar_profile();
        app_maybe_apply_radar_profile();
        app_maybe_retry_params();
    }
}
