#include "app_radar.h"

#include <stdint.h>
#include <stdio.h>

#include "app_config.h"
#include "buzzer_alarm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ld6002c.h"
#include "mqtt_reporter.h"

static const char *TAG = "app_radar";

#define FALLGUARD_RADAR_STATUS_LOG_INTERVAL_MS 1000

typedef enum {
    RADAR_PROFILE_STEP_HEIGHT = 0,
    RADAR_PROFILE_STEP_THRESHOLD,
    RADAR_PROFILE_STEP_SENSITIVITY,
    RADAR_PROFILE_STEP_ALARM_ZONE,
    RADAR_PROFILE_STEP_DONE,
    RADAR_PROFILE_STEP_FAILED,
} radar_profile_step_t;

static bool s_radar_initialized;
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
static TickType_t s_status_log_last_tick;
static bool s_latest_is_human;
static bool s_latest_is_human_valid;
static bool s_latest_is_fall;
static bool s_latest_is_fall_valid;
static float s_latest_target_height_m;
static bool s_latest_target_height_valid;

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

static const char *app_bool_status_to_cn(bool value, bool valid)
{
    if (!valid) {
        return "未知";
    }

    return value ? "是" : "否";
}

static void app_log_periodic_radar_status(void)
{
    const TickType_t now = xTaskGetTickCount();
    char height_text[16];

    if ((now - s_status_log_last_tick) < pdMS_TO_TICKS(FALLGUARD_RADAR_STATUS_LOG_INTERVAL_MS)) {
        return;
    }

    if (s_latest_target_height_valid) {
        snprintf(height_text, sizeof(height_text), "%.2f m", (double)s_latest_target_height_m);
    } else {
        snprintf(height_text, sizeof(height_text), "%s", "未知");
    }

    s_status_log_last_tick = now;
    ESP_LOGI(TAG, "是否有人: %s 是否跌倒: %s 目标高度: %s",
             app_bool_status_to_cn(s_latest_is_human, s_latest_is_human_valid),
             app_bool_status_to_cn(s_latest_is_fall, s_latest_is_fall_valid),
             height_text);
}

static void app_on_fall_status(const ld6002c_fall_status_t *status)
{
    if (status == NULL) {
        return;
    }

    app_mark_radar_stream_seen();
    s_latest_is_fall = status->is_fall;
    s_latest_is_fall_valid = true;
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
    s_latest_target_height_m = result->value;
    s_latest_target_height_valid = true;
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

static void app_on_human_status(const ld6002c_human_status_t *status)
{
    if (status == NULL) {
        return;
    }

    app_mark_radar_stream_seen();
    s_latest_is_human = status->is_human;
    s_latest_is_human_valid = true;
    mqtt_reporter_set_presence(status->is_human);
    ESP_LOGI(TAG, "Radar human status: is_human=%s", status->is_human ? "true" : "false");
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

static void app_radar_reset_state(void)
{
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
    s_status_log_last_tick = 0;
    s_latest_is_human = false;
    s_latest_is_human_valid = false;
    s_latest_is_fall = false;
    s_latest_is_fall_valid = false;
    s_latest_target_height_m = 0.0f;
    s_latest_target_height_valid = false;
}

esp_err_t app_radar_init(void)
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

    app_radar_reset_state();

    esp_err_t err = ld6002c_init(&radar_config);
    if (err != ESP_OK) {
        s_radar_initialized = false;
        ESP_LOGE(TAG, "Radar init failed: %s", esp_err_to_name(err));
        return err;
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

    return ESP_OK;
}

void app_radar_deinit(void)
{
    ld6002c_deinit();
    s_radar_initialized = false;
}

void app_radar_process(void)
{
    app_log_periodic_radar_status();
    app_maybe_enable_3d_cloud();
    app_maybe_log_3d_diagnostic();
    app_maybe_timeout_radar_profile();
    app_maybe_apply_radar_profile();
    app_maybe_retry_params();
}

bool app_radar_is_initialized(void)
{
    return s_radar_initialized;
}
