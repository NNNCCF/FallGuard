#include "mqtt_reporter.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <stdio.h>
#include <string.h>

#define TAG                  "mqtt_reporter"
#define TOPIC_BUF_LEN        64
#define PAYLOAD_BUF_LEN      256
#define TASK_STACK_SIZE      4096
#define TASK_PRIORITY        5
#define DEFAULT_MQTT_TLS_PORT 8883
#define MQTT_NVS_NAMESPACE   "mqtt"
#define MQTT_NVS_KEY_PASSWORD "password"
#define MQTT_PASSWORD_MAX_LEN 128

extern const uint8_t mqtt_broker_ca_pem_start[] asm("_binary_mqtt_broker_ca_pem_start");
extern const uint8_t mqtt_broker_ca_pem_end[] asm("_binary_mqtt_broker_ca_pem_end");

static esp_mqtt_client_handle_t s_client     = NULL;
static TaskHandle_t             s_task       = NULL;
static volatile bool            s_connected  = false;
static volatile bool            s_is_fall    = false;
static volatile bool            s_is_present = false;
static char                     s_device_id[48];
static char                     s_topic[TOPIC_BUF_LEN];
static uint32_t                 s_interval_ms;
static char                     s_password[MQTT_PASSWORD_MAX_LEN];

static esp_err_t load_mqtt_password_from_nvs(char *out_password, size_t out_size)
{
    if (out_password == NULL || out_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(MQTT_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t required_size = out_size;
    err = nvs_get_str(handle, MQTT_NVS_KEY_PASSWORD, out_password, &required_size);
    nvs_close(handle);
    return err;
}

static const char *resolve_mqtt_password(const char *fallback_password)
{
    memset(s_password, 0, sizeof(s_password));

    esp_err_t err = load_mqtt_password_from_nvs(s_password, sizeof(s_password));
    if (err == ESP_OK && s_password[0] != '\0') {
        ESP_LOGI(TAG, "Using MQTT password from NVS namespace '%s'", MQTT_NVS_NAMESPACE);
        return s_password;
    }

    if (fallback_password != NULL && fallback_password[0] != '\0') {
        size_t len = strnlen(fallback_password, sizeof(s_password) - 1);
        memcpy(s_password, fallback_password, len);
        s_password[len] = '\0';
        ESP_LOGW(TAG, "Using MQTT password from Kconfig fallback; prefer NVS in production");
        return s_password;
    }

    ESP_LOGW(TAG, "MQTT password not configured in NVS and fallback is empty");
    return NULL;
}

static void clear_sensitive_state(void)
{
    memset(s_password, 0, sizeof(s_password));
}

static void publish_task(void *arg)
{
    char payload[PAYLOAD_BUF_LEN];

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(s_interval_ms));

        if (!s_connected) {
            continue;
        }

        bool is_fall    = s_is_fall;
        bool is_present = s_is_present;
        int  hr         = 60 + (int)(esp_random() % 41); /* 60~100 bpm */
        int  br         = 12 + (int)(esp_random() % 9);  /* 12~20 bpm */

        int len = snprintf(payload, sizeof(payload),
                           "{"
                           "\"device_id\":\"%s\","
                           "\"heart_rate_per_min\":%d,"
                           "\"breath_rate_per_min\":%d,"
                           "\"is_fall\":%s,"
                           "\"is_person_present\":%s"
                           "}",
                           s_device_id, hr, br,
                           is_fall    ? "true" : "false",
                           is_present ? "true" : "false");

        if (len > 0 && len < (int)sizeof(payload)) {
            int msg_id = esp_mqtt_client_publish(s_client, s_topic, payload, len, 1, 0);
            if (msg_id < 0) {
                ESP_LOGW(TAG, "Publish failed (not connected?)");
            } else {
                ESP_LOGD(TAG, "Published [%s]: %s", s_topic, payload);
            }
        }
    }
}

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    (void)arg;
    (void)base;
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to broker over TLS");
        s_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected from broker");
        s_connected = false;
        break;
    case MQTT_EVENT_ERROR:
        if (event->error_handle) {
            ESP_LOGW(TAG, "Error type=%d", event->error_handle->error_type);
        }
        break;
    default:
        break;
    }
}

esp_err_t mqtt_reporter_init(const mqtt_reporter_config_t *config)
{
    if (!config || !config->host || !config->device_id || !config->server_cert_pem) {
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(s_device_id, config->device_id, sizeof(s_device_id) - 1);
    s_device_id[sizeof(s_device_id) - 1] = '\0';

    s_interval_ms = (config->interval_ms > 0) ? config->interval_ms : 1000;
    s_is_fall     = false;
    s_is_present  = false;
    s_connected   = false;

    snprintf(s_topic, sizeof(s_topic), "/device/%s/data", s_device_id);

    const char *password = resolve_mqtt_password(config->password);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.hostname             = config->host,
        .broker.address.port                 = config->port ? config->port : DEFAULT_MQTT_TLS_PORT,
        .broker.address.transport            = MQTT_TRANSPORT_OVER_SSL,
        .broker.verification.certificate     = config->server_cert_pem,
        .credentials.username                = config->username,
        .credentials.authentication.password = password,
        .credentials.client_id               = config->device_id,
        .session.keepalive                   = 60,
        .network.reconnect_timeout_ms        = 5000,
    };

    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "esp_mqtt_client_init failed");
        clear_sensitive_state();
        return ESP_FAIL;
    }

    esp_err_t err = esp_mqtt_client_register_event(
        s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        clear_sensitive_state();
        return err;
    }

    err = esp_mqtt_client_start(s_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_mqtt_client_start failed: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        clear_sensitive_state();
        return err;
    }

    BaseType_t ret = xTaskCreate(publish_task, "mqtt_pub",
                                 TASK_STACK_SIZE, NULL, TASK_PRIORITY, &s_task);
    if (ret != pdPASS) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        clear_sensitive_state();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Init ok: broker=%s:%u transport=mqtts device=%s topic=%s interval=%lums",
             config->host,
             (unsigned)(config->port ? config->port : DEFAULT_MQTT_TLS_PORT),
             s_device_id, s_topic, (unsigned long)s_interval_ms);

    return ESP_OK;
}

void mqtt_reporter_set_fall(bool is_fall)
{
    s_is_fall = is_fall;
}

void mqtt_reporter_set_presence(bool is_present)
{
    s_is_present = is_present;
}

void mqtt_reporter_deinit(void)
{
    if (s_task) {
        vTaskDelete(s_task);
        s_task = NULL;
    }
    if (s_client) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }
    s_connected = false;
    clear_sensitive_state();
}
