#include "wifi_manager.h"

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "nvs.h"

#define WIFI_NAMESPACE             "fallguard"
#define WIFI_KEY_SSID              "wifi_ssid"
#define WIFI_KEY_PASSWORD          "wifi_pass"
#define WIFI_AP_PASSWORD           "12345678"
#define WIFI_AP_CHANNEL            6
#define WIFI_AP_MAX_CONNECTIONS    1
#define WIFI_COMMAND_QUEUE_LENGTH  4
#define WIFI_STATUS_POLL_BODY_MAX  256
#define WIFI_CONNECT_TIMEOUT_MS    10000

#define WIFI_INTERNAL_EVENT_GOT_IP       BIT0
#define WIFI_INTERNAL_EVENT_DISCONNECTED BIT1
#define WIFI_INTERNAL_EVENT_CANCEL       BIT2

typedef enum {
    WIFI_CMD_START = 0,
    WIFI_CMD_RECONNECT,
    WIFI_CMD_SHUTDOWN,
} wifi_manager_cmd_t;

typedef struct {
    bool initialized;
    bool wifi_started;
    bool ap_active;
    bool transitioning;
    EventGroupHandle_t app_event_group;
    EventGroupHandle_t internal_event_group;
    QueueHandle_t command_queue;
    SemaphoreHandle_t lock;
    TaskHandle_t worker_task;
    httpd_handle_t http_server;
    esp_netif_t *sta_netif;
    esp_netif_t *ap_netif;
    esp_event_handler_instance_t wifi_event_instance;
    esp_event_handler_instance_t ip_event_instance;
    wifi_status_t status;
} wifi_manager_ctx_t;

static const char *TAG = "wifi_manager";

static const char *s_provision_page =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>FallGuard Setup</title>"
    "<style>"
    ":root{color-scheme:light;font-family:'Segoe UI',sans-serif;"
    "--bg:#eef3ff;--panel:#ffffff;--ink:#14213d;--muted:#5c6b8a;"
    "--accent:#ff6b35;--accent2:#2ec4b6;}"
    "body{margin:0;min-height:100vh;display:grid;place-items:center;"
    "background:radial-gradient(circle at top,#fff8e8 0,#eef3ff 48%,#dfe8ff 100%);"
    "color:var(--ink)}"
    ".card{width:min(92vw,440px);background:rgba(255,255,255,.92);backdrop-filter:blur(10px);"
    "border-radius:24px;padding:28px;box-shadow:0 18px 50px rgba(20,33,61,.18)}"
    "h1{margin:0 0 10px;font-size:28px;letter-spacing:.02em}"
    "p{margin:0 0 18px;color:var(--muted);line-height:1.5}"
    "label{display:block;margin:14px 0 8px;font-weight:600}"
    "input{width:100%;box-sizing:border-box;border:1px solid #ccd6f6;border-radius:14px;"
    "padding:14px 16px;font-size:16px;background:#f9fbff}"
    "button{margin-top:18px;width:100%;border:0;border-radius:999px;padding:14px 18px;"
    "font-size:16px;font-weight:700;color:#fff;background:linear-gradient(135deg,var(--accent),#ff8c5a);"
    "box-shadow:0 12px 30px rgba(255,107,53,.25)}"
    ".status{margin-top:20px;padding:16px;border-radius:16px;background:#f4f7ff;border:1px solid #dce5ff}"
    ".status strong{display:block;margin-bottom:6px}"
    ".pill{display:inline-block;padding:6px 10px;border-radius:999px;background:#e6fff8;color:#0d7368;"
    "font-size:13px;font-weight:700;margin-bottom:10px}"
    "</style></head><body><div class='card'><div class='pill'>FallGuard v1</div>"
    "<h1>Wi-Fi Provisioning</h1><p>Connect your gateway to the local network."
    " The page refreshes status automatically while the device is joining Wi-Fi.</p>"
    "<form id='wifi-form'><label for='ssid'>SSID</label><input id='ssid' maxlength='32' required>"
    "<label for='password'>Password</label><input id='password' type='password' maxlength='63'>"
    "<button type='submit'>Connect</button></form>"
    "<div class='status'><strong id='state'>Loading...</strong><span id='message'>Waiting for device status.</span>"
    "<div id='ip' style='margin-top:8px;color:#5c6b8a'></div></div></div>"
    "<script>"
    "const stateEl=document.getElementById('state');"
    "const messageEl=document.getElementById('message');"
    "const ipEl=document.getElementById('ip');"
    "async function refresh(){"
    " try{const res=await fetch('/status');const data=await res.json();"
    "  stateEl.textContent='State: '+data.state;"
    "  messageEl.textContent=data.message||'No status message yet.';"
    "  ipEl.textContent=data.ip?('IP: '+data.ip):'';"
    " }catch(err){messageEl.textContent='Unable to read device status.';}"
    "}"
    "document.getElementById('wifi-form').addEventListener('submit',async(event)=>{"
    " event.preventDefault();"
    " const payload={ssid:document.getElementById('ssid').value.trim(),"
    " password:document.getElementById('password').value};"
    " if(!payload.ssid){messageEl.textContent='SSID is required.';return;}"
    " const res=await fetch('/connect',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});"
    " const data=await res.json();"
    " stateEl.textContent='State: '+(data.state||'STA_CONNECTING');"
    " messageEl.textContent=data.message||'Credentials received. Connecting...';"
    "});"
    "refresh();setInterval(refresh,2000);"
    "</script></body></html>";

static wifi_manager_ctx_t s_ctx = {0};

static void copy_string(char *dest, size_t dest_size, const char *src)
{
    if (dest == NULL || dest_size == 0) {
        return;
    }

    if (src == NULL) {
        src = "";
    }

    snprintf(dest, dest_size, "%s", src);
}

static EventBits_t wifi_manager_state_to_bit(wifi_state_t state)
{
    switch (state) {
    case WIFI_STATE_IDLE:
        return WIFI_MANAGER_EVENT_IDLE;
    case WIFI_STATE_AP_MODE:
        return WIFI_MANAGER_EVENT_AP_MODE;
    case WIFI_STATE_STA_CONNECTING:
        return WIFI_MANAGER_EVENT_STA_CONNECTING;
    case WIFI_STATE_STA_CONNECTED:
        return WIFI_MANAGER_EVENT_STA_CONNECTED;
    case WIFI_STATE_STA_DISCONNECTED:
        return WIFI_MANAGER_EVENT_STA_DISCONNECTED;
    case WIFI_STATE_STA_RECONNECTING:
        return WIFI_MANAGER_EVENT_STA_RECONNECTING;
    case WIFI_STATE_STA_FAILED:
        return WIFI_MANAGER_EVENT_STA_FAILED;
    default:
        return WIFI_MANAGER_EVENT_IDLE;
    }
}

const char *wifi_manager_state_to_string(wifi_state_t state)
{
    switch (state) {
    case WIFI_STATE_IDLE:
        return "IDLE";
    case WIFI_STATE_AP_MODE:
        return "AP_MODE";
    case WIFI_STATE_STA_CONNECTING:
        return "STA_CONNECTING";
    case WIFI_STATE_STA_CONNECTED:
        return "STA_CONNECTED";
    case WIFI_STATE_STA_DISCONNECTED:
        return "STA_DISCONNECTED";
    case WIFI_STATE_STA_RECONNECTING:
        return "STA_RECONNECTING";
    case WIFI_STATE_STA_FAILED:
        return "STA_FAILED";
    default:
        return "UNKNOWN";
    }
}

static void wifi_manager_publish_state(wifi_state_t state)
{
    if (s_ctx.app_event_group == NULL) {
        return;
    }

    xEventGroupClearBits(s_ctx.app_event_group, WIFI_MANAGER_ALL_STATE_BITS);
    xEventGroupSetBits(s_ctx.app_event_group, wifi_manager_state_to_bit(state));
}

static void wifi_manager_set_state(wifi_state_t state, const char *fmt, ...)
{
    char message[sizeof(s_ctx.status.message)] = {0};

    if (fmt != NULL) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(message, sizeof(message), fmt, args);
        va_end(args);
    }

    xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
    s_ctx.status.state = state;
    if (fmt != NULL) {
        copy_string(s_ctx.status.message, sizeof(s_ctx.status.message), message);
    }
    xSemaphoreGive(s_ctx.lock);

    wifi_manager_publish_state(state);
}

static void wifi_manager_clear_ip_info(void)
{
    xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
    s_ctx.status.ip[0] = '\0';
    s_ctx.status.rssi = 0;
    xSemaphoreGive(s_ctx.lock);
}

static void wifi_manager_set_has_credentials(bool has_credentials)
{
    xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
    s_ctx.status.has_credentials = has_credentials;
    xSemaphoreGive(s_ctx.lock);
}

static esp_err_t wifi_manager_open_storage(nvs_handle_t *handle)
{
    return nvs_open(WIFI_NAMESPACE, NVS_READWRITE, handle);
}

static esp_err_t wifi_manager_save_credentials(const char *ssid, const char *password)
{
    nvs_handle_t handle;
    esp_err_t err = wifi_manager_open_storage(&handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_str(handle, WIFI_KEY_SSID, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(handle, WIFI_KEY_PASSWORD, password != NULL ? password : "");
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        wifi_manager_set_has_credentials(true);
    }

    return err;
}

static esp_err_t wifi_manager_load_credentials(char *ssid, size_t ssid_size,
                                               char *password, size_t password_size)
{
    size_t required_size = 0;
    nvs_handle_t handle;
    esp_err_t err = wifi_manager_open_storage(&handle);
    if (err != ESP_OK) {
        return err;
    }

    required_size = ssid_size;
    err = nvs_get_str(handle, WIFI_KEY_SSID, ssid, &required_size);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    required_size = password_size;
    err = nvs_get_str(handle, WIFI_KEY_PASSWORD, password, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        password[0] = '\0';
        err = ESP_OK;
    }

    nvs_close(handle);

    if (err != ESP_OK) {
        return err;
    }

    if (ssid[0] == '\0') {
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

static esp_err_t wifi_manager_clear_credentials_internal(void)
{
    nvs_handle_t handle;
    esp_err_t err = wifi_manager_open_storage(&handle);
    if (err != ESP_OK) {
        return err;
    }

    esp_err_t erase_ssid_err = nvs_erase_key(handle, WIFI_KEY_SSID);
    esp_err_t erase_pass_err = nvs_erase_key(handle, WIFI_KEY_PASSWORD);
    if (erase_ssid_err != ESP_OK && erase_ssid_err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return erase_ssid_err;
    }
    if (erase_pass_err != ESP_OK && erase_pass_err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return erase_pass_err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        wifi_manager_set_has_credentials(false);
        wifi_manager_clear_ip_info();
        xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
        s_ctx.status.ssid[0] = '\0';
        xSemaphoreGive(s_ctx.lock);
    }

    return err;
}

esp_err_t wifi_manager_clear_credentials(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xEventGroupSetBits(s_ctx.internal_event_group, WIFI_INTERNAL_EVENT_CANCEL);
    return wifi_manager_clear_credentials_internal();
}

static void wifi_manager_update_rssi(void)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
        return;
    }

    xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
    s_ctx.status.rssi = ap_info.rssi;
    xSemaphoreGive(s_ctx.lock);
}

static char *wifi_manager_status_json_string(cJSON **json_out)
{
    wifi_status_t status;
    char *json_string = NULL;

    if (wifi_manager_get_status(&status) != ESP_OK) {
        return NULL;
    }

    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        return NULL;
    }

    cJSON_AddStringToObject(json, "state", wifi_manager_state_to_string(status.state));
    cJSON_AddStringToObject(json, "ssid", status.ssid);
    cJSON_AddStringToObject(json, "ip", status.ip);
    cJSON_AddStringToObject(json, "message", status.message);
    cJSON_AddBoolToObject(json, "has_credentials", status.has_credentials);
    cJSON_AddNumberToObject(json, "rssi", status.rssi);

    json_string = cJSON_PrintUnformatted(json);
    if (json_out != NULL) {
        *json_out = json;
    } else {
        cJSON_Delete(json);
    }

    return json_string;
}

static esp_err_t wifi_manager_send_json(httpd_req_t *req, const char *status_code, cJSON *json)
{
    char *payload = NULL;
    esp_err_t err = ESP_FAIL;

    if (json == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    payload = cJSON_PrintUnformatted(json);
    if (payload == NULL) {
        return ESP_ERR_NO_MEM;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, status_code);
    err = httpd_resp_sendstr(req, payload);
    cJSON_free(payload);
    return err;
}

static esp_err_t wifi_manager_handle_root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, s_provision_page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t wifi_manager_handle_status_get(httpd_req_t *req)
{
    cJSON *json = NULL;
    char *payload = wifi_manager_status_json_string(&json);
    esp_err_t err;

    if (payload == NULL || json == NULL) {
        if (json != NULL) {
            cJSON_Delete(json);
        }
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "status unavailable");
    }

    httpd_resp_set_type(req, "application/json");
    err = httpd_resp_sendstr(req, payload);

    cJSON_free(payload);
    cJSON_Delete(json);
    return err;
}

static esp_err_t wifi_manager_handle_connect_post(httpd_req_t *req)
{
    char *body = NULL;
    cJSON *request_json = NULL;
    cJSON *response_json = NULL;
    esp_err_t err = ESP_FAIL;
    const cJSON *ssid_json = NULL;
    const cJSON *password_json = NULL;

    if (req->content_len <= 0 || req->content_len > WIFI_STATUS_POLL_BODY_MAX) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid payload length");
    }

    body = calloc((size_t)req->content_len + 1U, sizeof(char));
    if (body == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }

    int received = httpd_req_recv(req, body, req->content_len);
    if (received <= 0) {
        free(body);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "failed to read request");
    }
    body[received] = '\0';

    request_json = cJSON_Parse(body);
    free(body);
    if (request_json == NULL) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
    }

    ssid_json = cJSON_GetObjectItemCaseSensitive(request_json, "ssid");
    password_json = cJSON_GetObjectItemCaseSensitive(request_json, "password");
    if (!cJSON_IsString(ssid_json) || ssid_json->valuestring == NULL || ssid_json->valuestring[0] == '\0') {
        cJSON_Delete(request_json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid is required");
    }

    if (strlen(ssid_json->valuestring) >= sizeof(s_ctx.status.ssid)) {
        cJSON_Delete(request_json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid too long");
    }

    if (password_json != NULL && (!cJSON_IsString(password_json) || password_json->valuestring == NULL)) {
        cJSON_Delete(request_json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "password must be a string");
    }

    if (password_json != NULL && strlen(password_json->valuestring) > 63) {
        cJSON_Delete(request_json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "password too long");
    }

    err = wifi_manager_save_credentials(ssid_json->valuestring,
                                        password_json != NULL ? password_json->valuestring : "");
    if (err == ESP_OK) {
        err = wifi_manager_start();
    }
    cJSON_Delete(request_json);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to apply credentials: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "failed to start Wi-Fi");
    }

    response_json = cJSON_CreateObject();
    if (response_json == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }

    cJSON_AddBoolToObject(response_json, "ok", true);
    cJSON_AddStringToObject(response_json, "state", wifi_manager_state_to_string(WIFI_STATE_STA_CONNECTING));
    cJSON_AddStringToObject(response_json, "message", "Credentials received. Connecting...");

    err = wifi_manager_send_json(req, "202 Accepted", response_json);
    cJSON_Delete(response_json);
    return err;
}

static esp_err_t wifi_manager_start_http_server(void)
{
    static const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = wifi_manager_handle_root_get,
        .user_ctx = NULL,
    };
    static const httpd_uri_t connect_uri = {
        .uri = "/connect",
        .method = HTTP_POST,
        .handler = wifi_manager_handle_connect_post,
        .user_ctx = NULL,
    };
    static const httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = wifi_manager_handle_status_get,
        .user_ctx = NULL,
    };

    if (s_ctx.http_server != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 6;
    config.lru_purge_enable = true;

    esp_err_t err = httpd_start(&s_ctx.http_server, &config);
    if (err != ESP_OK) {
        return err;
    }

    err = httpd_register_uri_handler(s_ctx.http_server, &root_uri);
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_ctx.http_server, &connect_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(s_ctx.http_server, &status_uri);
    }
    if (err != ESP_OK) {
        httpd_stop(s_ctx.http_server);
        s_ctx.http_server = NULL;
    }

    return err;
}

static void wifi_manager_stop_http_server(void)
{
    if (s_ctx.http_server != NULL) {
        httpd_stop(s_ctx.http_server);
        s_ctx.http_server = NULL;
    }
}

static esp_err_t wifi_manager_get_ap_ssid(char *ssid, size_t ssid_size)
{
    uint8_t mac[6];
    esp_err_t err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        return err;
    }

    snprintf(ssid, ssid_size, "FallGuard-%02X%02X", mac[4], mac[5]);
    return ESP_OK;
}

static esp_err_t wifi_manager_start_wifi_if_needed(void)
{
    if (s_ctx.wifi_started) {
        return ESP_OK;
    }

    esp_err_t err = esp_wifi_start();
    if (err == ESP_OK) {
        s_ctx.wifi_started = true;
    }
    return err;
}

static esp_err_t wifi_manager_enter_ap_mode(const char *message)
{
    char ap_ssid[32];
    wifi_config_t ap_config = {0};
    esp_err_t err = wifi_manager_get_ap_ssid(ap_ssid, sizeof(ap_ssid));
    if (err != ESP_OK) {
        return err;
    }

    copy_string((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), ap_ssid);
    copy_string((char *)ap_config.ap.password, sizeof(ap_config.ap.password), WIFI_AP_PASSWORD);
    ap_config.ap.channel = WIFI_AP_CHANNEL;
    ap_config.ap.max_connection = WIFI_AP_MAX_CONNECTIONS;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    ap_config.ap.ssid_len = (uint8_t)strlen(ap_ssid);
    ap_config.ap.pmf_cfg.required = false;

    s_ctx.transitioning = true;
    (void)esp_wifi_disconnect();
    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    }
    if (err == ESP_OK) {
        err = wifi_manager_start_wifi_if_needed();
    }
    s_ctx.transitioning = false;
    if (err != ESP_OK) {
        return err;
    }

    err = wifi_manager_start_http_server();
    if (err != ESP_OK) {
        return err;
    }

    s_ctx.ap_active = true;
    wifi_manager_clear_ip_info();
    xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
    copy_string(s_ctx.status.ssid, sizeof(s_ctx.status.ssid), ap_ssid);
    xSemaphoreGive(s_ctx.lock);
    wifi_manager_set_state(WIFI_STATE_AP_MODE, "%s", message);
    ESP_LOGI(TAG, "SoftAP started: %s", ap_ssid);
    return ESP_OK;
}

static esp_err_t wifi_manager_begin_sta_attempt(const char *ssid, const char *password, bool keep_ap)
{
    wifi_config_t sta_config = {0};
    esp_err_t err;

    copy_string((char *)sta_config.sta.ssid, sizeof(sta_config.sta.ssid), ssid);
    copy_string((char *)sta_config.sta.password, sizeof(sta_config.sta.password), password);
    sta_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    sta_config.sta.failure_retry_cnt = 0;
    sta_config.sta.pmf_cfg.capable = true;
    sta_config.sta.pmf_cfg.required = false;

    s_ctx.transitioning = true;
    err = esp_wifi_set_mode(keep_ap ? WIFI_MODE_APSTA : WIFI_MODE_STA);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    }
    if (err == ESP_OK) {
        err = wifi_manager_start_wifi_if_needed();
    }
    s_ctx.transitioning = false;
    if (err != ESP_OK) {
        return err;
    }

    xEventGroupClearBits(s_ctx.internal_event_group,
                         WIFI_INTERNAL_EVENT_GOT_IP
                             | WIFI_INTERNAL_EVENT_DISCONNECTED
                             | WIFI_INTERNAL_EVENT_CANCEL);
    err = esp_wifi_connect();
    if (err == ESP_OK) {
        xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
        copy_string(s_ctx.status.ssid, sizeof(s_ctx.status.ssid), ssid);
        xSemaphoreGive(s_ctx.lock);
    }
    return err;
}

static esp_err_t wifi_manager_connect_with_retries(const char *ssid, const char *password, bool keep_ap)
{
    static const uint32_t retry_backoff_ms[] = {5000, 10000, 20000};

    for (size_t attempt = 0; attempt < 3; ++attempt) {
        wifi_state_t state = (attempt == 0) ? WIFI_STATE_STA_CONNECTING : WIFI_STATE_STA_RECONNECTING;
        esp_err_t err = wifi_manager_begin_sta_attempt(ssid, password, keep_ap);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start STA attempt %u: %s",
                     (unsigned)(attempt + 1U), esp_err_to_name(err));
        } else {
            wifi_manager_set_state(state, "%s", attempt == 0 ? "Connecting to Wi-Fi..."
                                                             : "Retrying Wi-Fi connection...");
            EventBits_t bits = xEventGroupWaitBits(
                s_ctx.internal_event_group,
                WIFI_INTERNAL_EVENT_GOT_IP | WIFI_INTERNAL_EVENT_DISCONNECTED | WIFI_INTERNAL_EVENT_CANCEL,
                pdTRUE,
                pdFALSE,
                pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

            if ((bits & WIFI_INTERNAL_EVENT_CANCEL) != 0U) {
                ESP_LOGI(TAG, "STA attempt cancelled");
                return ESP_ERR_INVALID_STATE;
            }

            if ((bits & WIFI_INTERNAL_EVENT_GOT_IP) != 0U) {
                wifi_manager_update_rssi();
                wifi_manager_set_state(WIFI_STATE_STA_CONNECTED, "Wi-Fi connected");

                if (keep_ap || s_ctx.ap_active) {
                    wifi_manager_stop_http_server();
                    s_ctx.transitioning = true;
                    err = esp_wifi_set_mode(WIFI_MODE_STA);
                    s_ctx.transitioning = false;
                    if (err != ESP_OK) {
                        ESP_LOGW(TAG, "Failed to disable AP after STA join: %s", esp_err_to_name(err));
                    } else {
                        s_ctx.ap_active = false;
                    }
                }

                return ESP_OK;
            }

            if ((bits & WIFI_INTERNAL_EVENT_DISCONNECTED) == 0U) {
                ESP_LOGW(TAG, "STA attempt %u timed out", (unsigned)(attempt + 1U));
                s_ctx.transitioning = true;
                (void)esp_wifi_disconnect();
                s_ctx.transitioning = false;
            }
        }

        if (attempt < 2U) {
            vTaskDelay(pdMS_TO_TICKS(retry_backoff_ms[attempt]));
        }
    }

    wifi_manager_set_state(WIFI_STATE_STA_FAILED, "Connection failed. Returning to AP mode.");
    ESP_LOGW(TAG, "Wi-Fi retries exhausted, clearing credentials");
    ESP_ERROR_CHECK_WITHOUT_ABORT(wifi_manager_clear_credentials_internal());

    return wifi_manager_enter_ap_mode("Connection failed. Please configure Wi-Fi again.");
}

static void wifi_manager_queue_reconnect(void)
{
    wifi_manager_cmd_t command = WIFI_CMD_RECONNECT;

    if (s_ctx.command_queue == NULL) {
        return;
    }

    if (xQueueSend(s_ctx.command_queue, &command, 0) != pdPASS) {
        ESP_LOGW(TAG, "Reconnect command queue is full");
    }
}

static void wifi_manager_wifi_event_handler(void *arg, esp_event_base_t event_base,
                                            int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_data;

    if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_manager_clear_ip_info();
        if (s_ctx.transitioning) {
            return;
        }

        wifi_state_t current_state = WIFI_STATE_IDLE;
        xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
        current_state = s_ctx.status.state;
        xSemaphoreGive(s_ctx.lock);

        if (current_state == WIFI_STATE_STA_CONNECTED) {
            wifi_manager_set_state(WIFI_STATE_STA_DISCONNECTED, "Wi-Fi disconnected");
            wifi_manager_queue_reconnect();
        } else if (current_state == WIFI_STATE_STA_CONNECTING || current_state == WIFI_STATE_STA_RECONNECTING) {
            xEventGroupSetBits(s_ctx.internal_event_group, WIFI_INTERNAL_EVENT_DISCONNECTED);
        }
    }
}

static void wifi_manager_ip_event_handler(void *arg, esp_event_base_t event_base,
                                          int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;

    if (event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        char ip_string[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_string, sizeof(ip_string));

        xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
        copy_string(s_ctx.status.ip, sizeof(s_ctx.status.ip), ip_string);
        xSemaphoreGive(s_ctx.lock);
        xEventGroupSetBits(s_ctx.internal_event_group, WIFI_INTERNAL_EVENT_GOT_IP);
    }
}

static void wifi_manager_worker_task(void *arg)
{
    (void)arg;

    while (true) {
        wifi_manager_cmd_t command = WIFI_CMD_START;
        if (xQueueReceive(s_ctx.command_queue, &command, portMAX_DELAY) != pdPASS) {
            continue;
        }

        if (command == WIFI_CMD_SHUTDOWN) {
            break;
        }

        char ssid[33] = {0};
        char password[64] = {0};
        esp_err_t err = wifi_manager_load_credentials(ssid, sizeof(ssid), password, sizeof(password));
        if (err == ESP_OK) {
            wifi_manager_set_has_credentials(true);
            ESP_LOGI(TAG, "Starting STA workflow using stored credentials");
            err = wifi_manager_connect_with_retries(ssid, password, s_ctx.ap_active);
            if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "STA workflow ended with %s", esp_err_to_name(err));
            }
        } else if (err == ESP_ERR_NVS_NOT_FOUND || err == ESP_ERR_NOT_FOUND) {
            wifi_manager_set_has_credentials(false);
            err = wifi_manager_enter_ap_mode("Waiting for Wi-Fi credentials");
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start AP mode: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "Failed to read Wi-Fi credentials: %s", esp_err_to_name(err));
        }
    }

    vTaskDelete(NULL);
}

esp_err_t wifi_manager_init(const wifi_manager_config_t *config)
{
    if (config == NULL || config->app_event_group == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx.initialized) {
        return ESP_OK;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.app_event_group = config->app_event_group;
    s_ctx.internal_event_group = xEventGroupCreate();
    s_ctx.command_queue = xQueueCreate(WIFI_COMMAND_QUEUE_LENGTH, sizeof(wifi_manager_cmd_t));
    s_ctx.lock = xSemaphoreCreateMutex();

    if (s_ctx.internal_event_group == NULL || s_ctx.command_queue == NULL || s_ctx.lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    s_ctx.sta_netif = esp_netif_create_default_wifi_sta();
    s_ctx.ap_netif = esp_netif_create_default_wifi_ap();
    if (s_ctx.sta_netif == NULL || s_ctx.ap_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_init_config);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_manager_wifi_event_handler,
                                              NULL, &s_ctx.wifi_event_instance);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &wifi_manager_ip_event_handler,
                                              NULL, &s_ctx.ip_event_instance);
    if (err != ESP_OK) {
        return err;
    }

    BaseType_t task_created = xTaskCreate(wifi_manager_worker_task, "wifi_manager",
                                          6144, NULL, 8, &s_ctx.worker_task);
    if (task_created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_ctx.status.state = WIFI_STATE_IDLE;
    s_ctx.status.rssi = 0;
    copy_string(s_ctx.status.message, sizeof(s_ctx.status.message), "Wi-Fi manager initialized");
    wifi_manager_publish_state(WIFI_STATE_IDLE);
    s_ctx.initialized = true;

    return ESP_OK;
}

esp_err_t wifi_manager_start(void)
{
    wifi_manager_cmd_t command = WIFI_CMD_START;

    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xEventGroupSetBits(s_ctx.internal_event_group, WIFI_INTERNAL_EVENT_CANCEL);
    if (xQueueSend(s_ctx.command_queue, &command, pdMS_TO_TICKS(100)) != pdPASS) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t wifi_manager_get_status(wifi_status_t *status)
{
    if (!s_ctx.initialized || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
    *status = s_ctx.status;
    xSemaphoreGive(s_ctx.lock);

    if (status->state == WIFI_STATE_STA_CONNECTED) {
        wifi_manager_update_rssi();
        xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
        status->rssi = s_ctx.status.rssi;
        xSemaphoreGive(s_ctx.lock);
    }

    return ESP_OK;
}

esp_err_t wifi_manager_deinit(void)
{
    if (!s_ctx.initialized) {
        return ESP_OK;
    }

    xEventGroupSetBits(s_ctx.internal_event_group, WIFI_INTERNAL_EVENT_CANCEL);
    if (s_ctx.command_queue != NULL) {
        wifi_manager_cmd_t command = WIFI_CMD_SHUTDOWN;
        (void)xQueueSend(s_ctx.command_queue, &command, 0);
    }

    if (s_ctx.worker_task != NULL) {
        vTaskDelete(s_ctx.worker_task);
        s_ctx.worker_task = NULL;
    }

    wifi_manager_stop_http_server();

    if (s_ctx.wifi_event_instance != NULL) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_ctx.wifi_event_instance);
    }
    if (s_ctx.ip_event_instance != NULL) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_ctx.ip_event_instance);
    }

    if (s_ctx.wifi_started) {
        (void)esp_wifi_stop();
    }
    (void)esp_wifi_deinit();

    if (s_ctx.command_queue != NULL) {
        vQueueDelete(s_ctx.command_queue);
    }
    if (s_ctx.internal_event_group != NULL) {
        vEventGroupDelete(s_ctx.internal_event_group);
    }
    if (s_ctx.lock != NULL) {
        vSemaphoreDelete(s_ctx.lock);
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    return ESP_OK;
}
