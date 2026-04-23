#include "oled_display.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define OLED_DISPLAY_I2C_SPEED_HZ     (400 * 1000)
#define OLED_DISPLAY_CMD_BITS         8
#define OLED_DISPLAY_PARAM_BITS       8
#define OLED_DISPLAY_REFRESH_MS       500
#define OLED_DISPLAY_PAGE_PERIOD_MS   2000
#define OLED_DISPLAY_CHAR_WIDTH       6
#define OLED_DISPLAY_CHAR_BITMAP_W    5
#define OLED_DISPLAY_CHAR_HEIGHT      7
#define OLED_DISPLAY_MAX_LINES        8

typedef struct {
    char character;
    uint8_t columns[OLED_DISPLAY_CHAR_BITMAP_W];
} oled_glyph_t;

typedef struct {
    bool radar_ready;
    bool has_fall_status;
    bool has_fw_status;
    bool has_params;
    bool has_height_upload;
    bool has_3d_cloud;
    bool has_human_status;
    ld6002c_fall_status_t fall_status;
    ld6002c_fw_status_t fw_status;
    ld6002c_params_t params;
    ld6002c_height_upload_t height_upload;
    ld6002c_3d_cloud_t cloud;
    ld6002c_human_status_t human_status;
} oled_display_snapshot_t;

typedef struct {
    bool initialized;
    bool running;
    uint8_t current_page;
    oled_display_config_t config;
    uint8_t *framebuffer;
    SemaphoreHandle_t lock;
    TaskHandle_t task_handle;
    i2c_master_bus_handle_t i2c_bus;
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_handle_t panel_handle;
    oled_display_snapshot_t snapshot;
} oled_display_ctx_t;

static const char *TAG = "oled_display";

static const oled_glyph_t s_oled_font[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x00, 0x03, 0x03, 0x00}},
    {'/', {0x03, 0x04, 0x08, 0x10, 0x60}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'0', {0x3E, 0x45, 0x49, 0x51, 0x3E}},
    {'1', {0x00, 0x21, 0x7F, 0x01, 0x00}},
    {'2', {0x21, 0x43, 0x45, 0x49, 0x31}},
    {'3', {0x42, 0x41, 0x51, 0x69, 0x46}},
    {'4', {0x0C, 0x14, 0x24, 0x7F, 0x04}},
    {'5', {0x72, 0x51, 0x51, 0x51, 0x4E}},
    {'6', {0x1E, 0x29, 0x49, 0x49, 0x06}},
    {'7', {0x40, 0x47, 0x48, 0x50, 0x60}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x30, 0x49, 0x49, 0x4A, 0x3C}},
    {'A', {0x3F, 0x48, 0x48, 0x48, 0x3F}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x48, 0x48, 0x48, 0x40}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x2E}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x41, 0x41, 0x7F, 0x41, 0x41}},
    {'J', {0x02, 0x01, 0x41, 0x7E, 0x40}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x01, 0x01, 0x01, 0x01}},
    {'M', {0x7F, 0x20, 0x10, 0x20, 0x7F}},
    {'N', {0x7F, 0x20, 0x10, 0x08, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x48, 0x48, 0x48, 0x30}},
    {'Q', {0x3E, 0x41, 0x45, 0x42, 0x3D}},
    {'R', {0x7F, 0x48, 0x4C, 0x4A, 0x31}},
    {'S', {0x31, 0x49, 0x49, 0x49, 0x46}},
    {'T', {0x40, 0x40, 0x7F, 0x40, 0x40}},
    {'U', {0x7E, 0x01, 0x01, 0x01, 0x7E}},
    {'V', {0x7C, 0x02, 0x01, 0x02, 0x7C}},
    {'W', {0x7E, 0x01, 0x0E, 0x01, 0x7E}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x70, 0x08, 0x07, 0x08, 0x70}},
    {'Z', {0x43, 0x45, 0x49, 0x51, 0x61}},
};

static oled_display_ctx_t s_ctx = {0};

static void oled_display_release_resources(bool stop_task)
{
    s_ctx.running = false;
    if (stop_task && s_ctx.task_handle != NULL) {
        vTaskDelete(s_ctx.task_handle);
        s_ctx.task_handle = NULL;
    }
    if (s_ctx.panel_handle != NULL) {
        esp_lcd_panel_disp_on_off(s_ctx.panel_handle, false);
        esp_lcd_panel_del(s_ctx.panel_handle);
        s_ctx.panel_handle = NULL;
    }
    if (s_ctx.io_handle != NULL) {
        esp_lcd_panel_io_del(s_ctx.io_handle);
        s_ctx.io_handle = NULL;
    }
    if (s_ctx.i2c_bus != NULL) {
        i2c_del_master_bus(s_ctx.i2c_bus);
        s_ctx.i2c_bus = NULL;
    }
    if (s_ctx.lock != NULL) {
        vSemaphoreDelete(s_ctx.lock);
        s_ctx.lock = NULL;
    }
    free(s_ctx.framebuffer);
    s_ctx.framebuffer = NULL;
}

static const uint8_t *oled_display_find_glyph(char c)
{
    size_t i;

    if (c >= 'a' && c <= 'z') {
        c = (char)(c - ('a' - 'A'));
    }

    for (i = 0; i < sizeof(s_oled_font) / sizeof(s_oled_font[0]); ++i) {
        if (s_oled_font[i].character == c) {
            return s_oled_font[i].columns;
        }
    }

    return s_oled_font[0].columns;
}

static void oled_display_set_pixel(uint8_t *buffer, uint8_t width, uint8_t height,
                                   uint8_t x, uint8_t y, bool on)
{
    size_t index;

    if (x >= width || y >= height) {
        return;
    }

    index = x + ((size_t)(y / 8U) * width);
    if (on) {
        buffer[index] |= (uint8_t)(1U << (y % 8U));
    } else {
        buffer[index] &= (uint8_t)~(1U << (y % 8U));
    }
}

static void oled_display_clear_buffer(uint8_t *buffer, size_t len)
{
    memset(buffer, 0, len);
}

static void oled_display_draw_text(uint8_t *buffer, uint8_t width, uint8_t height,
                                   uint8_t x, uint8_t y, const char *text)
{
    while (*text != '\0' && (x + OLED_DISPLAY_CHAR_BITMAP_W) < width) {
        const uint8_t *glyph = oled_display_find_glyph(*text);
        for (uint8_t col = 0; col < OLED_DISPLAY_CHAR_BITMAP_W; ++col) {
            for (uint8_t row = 0; row < OLED_DISPLAY_CHAR_HEIGHT; ++row) {
                if ((glyph[col] & (1U << row)) != 0U) {
                    oled_display_set_pixel(buffer, width, height,
                                           (uint8_t)(x + col), (uint8_t)(y + row), true);
                }
            }
        }
        x = (uint8_t)(x + OLED_DISPLAY_CHAR_WIDTH);
        ++text;
    }
}

static void oled_display_draw_line(uint8_t *buffer, const char *text, uint8_t line)
{
    oled_display_draw_text(buffer, s_ctx.config.width, s_ctx.config.height,
                           0, (uint8_t)(line * 8U), text);
}

static void oled_display_format_line(char *buffer, size_t buffer_size, const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    vsnprintf(buffer, buffer_size, fmt, args);
    va_end(args);
}

static const char *oled_display_bool_string(bool value)
{
    return value ? "YES" : "NO";
}

static void oled_display_render_page_overview(const oled_display_snapshot_t *snapshot)
{
    char line[24];

    oled_display_draw_line(s_ctx.framebuffer, "RADAR OVERVIEW", 0);
    oled_display_format_line(line, sizeof(line), "READY %s", oled_display_bool_string(snapshot->radar_ready));
    oled_display_draw_line(s_ctx.framebuffer, line, 1);

    if (snapshot->has_fall_status) {
        oled_display_format_line(line, sizeof(line), "FALL %s",
                                 oled_display_bool_string(snapshot->fall_status.is_fall));
    } else {
        oled_display_format_line(line, sizeof(line), "FALL WAIT");
    }
    oled_display_draw_line(s_ctx.framebuffer, line, 2);

    if (snapshot->has_human_status) {
        oled_display_format_line(line, sizeof(line), "HUMAN %s",
                                 oled_display_bool_string(snapshot->human_status.is_human));
    } else {
        oled_display_format_line(line, sizeof(line), "HUMAN WAIT");
    }
    oled_display_draw_line(s_ctx.framebuffer, line, 3);

    if (snapshot->has_3d_cloud) {
        oled_display_format_line(line, sizeof(line), "TARGETS %ld", (long)snapshot->cloud.target_num);
    } else {
        oled_display_format_line(line, sizeof(line), "TARGETS WAIT");
    }
    oled_display_draw_line(s_ctx.framebuffer, line, 4);

    if (snapshot->has_height_upload) {
        oled_display_format_line(line, sizeof(line), "HEIGHT %lu",
                                 (unsigned long)snapshot->height_upload.value);
    } else {
        oled_display_format_line(line, sizeof(line), "HEIGHT WAIT");
    }
    oled_display_draw_line(s_ctx.framebuffer, line, 5);

    if (snapshot->has_fw_status) {
        oled_display_format_line(line, sizeof(line), "FW %u.%u.%u",
                                 snapshot->fw_status.major,
                                 snapshot->fw_status.sub,
                                 snapshot->fw_status.modified);
    } else {
        oled_display_format_line(line, sizeof(line), "FW WAIT");
    }
    oled_display_draw_line(s_ctx.framebuffer, line, 6);
    oled_display_draw_line(s_ctx.framebuffer, "PAGE 1/4", 7);
}

static void oled_display_render_page_params(const oled_display_snapshot_t *snapshot)
{
    char line[24];

    oled_display_draw_line(s_ctx.framebuffer, "RADAR PARAMS", 0);
    if (!snapshot->has_params) {
        oled_display_draw_line(s_ctx.framebuffer, "PARAM WAIT", 2);
        oled_display_draw_line(s_ctx.framebuffer, "PAGE 2/4", 7);
        return;
    }

    oled_display_format_line(line, sizeof(line), "H %.2f TH %.2f",
                             snapshot->params.high, snapshot->params.threshold);
    oled_display_draw_line(s_ctx.framebuffer, line, 1);
    oled_display_format_line(line, sizeof(line), "SENS %lu",
                             (unsigned long)snapshot->params.sensitivity);
    oled_display_draw_line(s_ctx.framebuffer, line, 2);
    oled_display_format_line(line, sizeof(line), "XL %.2f XR %.2f",
                             snapshot->params.rect_XL, snapshot->params.rect_XR);
    oled_display_draw_line(s_ctx.framebuffer, line, 3);
    oled_display_format_line(line, sizeof(line), "ZF %.2f ZB %.2f",
                             snapshot->params.rect_ZF, snapshot->params.rect_ZB);
    oled_display_draw_line(s_ctx.framebuffer, line, 4);
    oled_display_draw_line(s_ctx.framebuffer, "PAGE 2/4", 7);
}

static void oled_display_render_page_meta(const oled_display_snapshot_t *snapshot)
{
    char line[24];

    oled_display_draw_line(s_ctx.framebuffer, "RADAR META", 0);
    if (snapshot->has_fw_status) {
        oled_display_format_line(line, sizeof(line), "PROJECT %d", (int)snapshot->fw_status.project);
        oled_display_draw_line(s_ctx.framebuffer, line, 1);
        oled_display_format_line(line, sizeof(line), "VER %u.%u.%u",
                                 snapshot->fw_status.major,
                                 snapshot->fw_status.sub,
                                 snapshot->fw_status.modified);
        oled_display_draw_line(s_ctx.framebuffer, line, 2);
    } else {
        oled_display_draw_line(s_ctx.framebuffer, "FW WAIT", 1);
    }

    oled_display_format_line(line, sizeof(line), "PARAM %s",
                             oled_display_bool_string(snapshot->has_params));
    oled_display_draw_line(s_ctx.framebuffer, line, 3);
    oled_display_format_line(line, sizeof(line), "CLOUD %s",
                             oled_display_bool_string(snapshot->has_3d_cloud));
    oled_display_draw_line(s_ctx.framebuffer, line, 4);
    oled_display_format_line(line, sizeof(line), "HEIGHT %s",
                             oled_display_bool_string(snapshot->has_height_upload));
    oled_display_draw_line(s_ctx.framebuffer, line, 5);
    oled_display_draw_line(s_ctx.framebuffer, "PAGE 3/4", 7);
}

static void oled_display_render_page_cloud(const oled_display_snapshot_t *snapshot)
{
    char line[24];

    oled_display_draw_line(s_ctx.framebuffer, "CLOUD DETAIL", 0);
    if (!snapshot->has_3d_cloud || snapshot->cloud.target_num <= 0) {
        oled_display_draw_line(s_ctx.framebuffer, "NO CLOUD DATA", 2);
        oled_display_draw_line(s_ctx.framebuffer, "PAGE 4/4", 7);
        return;
    }

    oled_display_format_line(line, sizeof(line), "TARGETS %ld", (long)snapshot->cloud.target_num);
    oled_display_draw_line(s_ctx.framebuffer, line, 1);
    oled_display_format_line(line, sizeof(line), "ID %ld",
                             (long)snapshot->cloud.points[0].cluster_index);
    oled_display_draw_line(s_ctx.framebuffer, line, 2);
    oled_display_format_line(line, sizeof(line), "X %.2f", snapshot->cloud.points[0].x);
    oled_display_draw_line(s_ctx.framebuffer, line, 3);
    oled_display_format_line(line, sizeof(line), "Y %.2f", snapshot->cloud.points[0].y);
    oled_display_draw_line(s_ctx.framebuffer, line, 4);
    oled_display_format_line(line, sizeof(line), "Z %.2f", snapshot->cloud.points[0].z);
    oled_display_draw_line(s_ctx.framebuffer, line, 5);
    oled_display_format_line(line, sizeof(line), "SPD %.2f", snapshot->cloud.points[0].speed);
    oled_display_draw_line(s_ctx.framebuffer, line, 6);
    oled_display_draw_line(s_ctx.framebuffer, "PAGE 4/4", 7);
}

static void oled_display_render_snapshot(const oled_display_snapshot_t *snapshot)
{
    size_t framebuffer_size = ((size_t)s_ctx.config.width * s_ctx.config.height) / 8U;

    oled_display_clear_buffer(s_ctx.framebuffer, framebuffer_size);
    switch (s_ctx.current_page) {
    case 0:
        oled_display_render_page_overview(snapshot);
        break;
    case 1:
        oled_display_render_page_params(snapshot);
        break;
    case 2:
        oled_display_render_page_meta(snapshot);
        break;
    case 3:
    default:
        oled_display_render_page_cloud(snapshot);
        break;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_panel_draw_bitmap(
        s_ctx.panel_handle,
        0,
        0,
        s_ctx.config.width,
        s_ctx.config.height,
        s_ctx.framebuffer));
}

static void oled_display_task(void *arg)
{
    TickType_t last_page_switch = xTaskGetTickCount();

    (void)arg;

    while (s_ctx.running) {
        oled_display_snapshot_t snapshot;

        if ((xTaskGetTickCount() - last_page_switch) >= pdMS_TO_TICKS(OLED_DISPLAY_PAGE_PERIOD_MS)) {
            s_ctx.current_page = (uint8_t)((s_ctx.current_page + 1U) % 4U);
            last_page_switch = xTaskGetTickCount();
        }

        xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
        snapshot = s_ctx.snapshot;
        xSemaphoreGive(s_ctx.lock);

        oled_display_render_snapshot(&snapshot);
        vTaskDelay(pdMS_TO_TICKS(OLED_DISPLAY_REFRESH_MS));
    }

    vTaskDelete(NULL);
}

static esp_err_t oled_display_update_snapshot(void (*update_fn)(oled_display_snapshot_t *snapshot, const void *arg),
                                              const void *arg)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_ctx.lock, portMAX_DELAY);
    update_fn(&s_ctx.snapshot, arg);
    xSemaphoreGive(s_ctx.lock);
    return ESP_OK;
}

static void oled_display_update_ready_impl(oled_display_snapshot_t *snapshot, const void *arg)
{
    snapshot->radar_ready = *(const bool *)arg;
}

static void oled_display_update_fall_impl(oled_display_snapshot_t *snapshot, const void *arg)
{
    snapshot->has_fall_status = true;
    snapshot->fall_status = *(const ld6002c_fall_status_t *)arg;
}

static void oled_display_update_fw_impl(oled_display_snapshot_t *snapshot, const void *arg)
{
    snapshot->has_fw_status = true;
    snapshot->fw_status = *(const ld6002c_fw_status_t *)arg;
}

static void oled_display_update_params_impl(oled_display_snapshot_t *snapshot, const void *arg)
{
    snapshot->has_params = true;
    snapshot->params = *(const ld6002c_params_t *)arg;
}

static void oled_display_update_height_impl(oled_display_snapshot_t *snapshot, const void *arg)
{
    snapshot->has_height_upload = true;
    snapshot->height_upload = *(const ld6002c_height_upload_t *)arg;
}

static void oled_display_update_cloud_impl(oled_display_snapshot_t *snapshot, const void *arg)
{
    snapshot->has_3d_cloud = true;
    snapshot->cloud = *(const ld6002c_3d_cloud_t *)arg;
}

static void oled_display_update_human_impl(oled_display_snapshot_t *snapshot, const void *arg)
{
    snapshot->has_human_status = true;
    snapshot->human_status = *(const ld6002c_human_status_t *)arg;
}

esp_err_t oled_display_init(const oled_display_config_t *config)
{
    size_t framebuffer_size;
    esp_lcd_panel_io_i2c_config_t io_config = {0};
    esp_lcd_panel_dev_config_t panel_config = {0};
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {0};
    i2c_master_bus_config_t bus_config = {0};

    if (config == NULL || config->width != 128 || (config->height != 64 && config->height != 32)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx.initialized) {
        return ESP_OK;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.config = *config;
    s_ctx.lock = xSemaphoreCreateMutex();
    framebuffer_size = ((size_t)config->width * config->height) / 8U;
    s_ctx.framebuffer = calloc(framebuffer_size, sizeof(uint8_t));
    if (s_ctx.lock == NULL || s_ctx.framebuffer == NULL) {
        oled_display_release_resources(false);
        return ESP_ERR_NO_MEM;
    }

    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.i2c_port = config->i2c_port;
    bus_config.sda_io_num = config->sda_gpio_num;
    bus_config.scl_io_num = config->scl_gpio_num;
    bus_config.flags.enable_internal_pullup = true;
    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_ctx.i2c_bus);
    if (ret != ESP_OK) {
        oled_display_release_resources(false);
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    io_config.dev_addr = config->i2c_addr;
    io_config.scl_speed_hz = OLED_DISPLAY_I2C_SPEED_HZ;
    io_config.control_phase_bytes = 1;
    io_config.dc_bit_offset = 6;
    io_config.lcd_cmd_bits = OLED_DISPLAY_CMD_BITS;
    io_config.lcd_param_bits = OLED_DISPLAY_PARAM_BITS;
    ret = esp_lcd_new_panel_io_i2c(s_ctx.i2c_bus, &io_config, &s_ctx.io_handle);
    if (ret != ESP_OK) {
        oled_display_release_resources(false);
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        return ret;
    }

    ssd1306_config.height = config->height;
    panel_config.bits_per_pixel = 1;
    panel_config.reset_gpio_num = -1;
    panel_config.vendor_config = &ssd1306_config;
    ret = esp_lcd_new_panel_ssd1306(s_ctx.io_handle, &panel_config, &s_ctx.panel_handle);
    if (ret != ESP_OK) {
        oled_display_release_resources(false);
        ESP_LOGE(TAG, "Failed to create SSD1306 panel: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = esp_lcd_panel_reset(s_ctx.panel_handle);
    if (ret != ESP_OK) {
        oled_display_release_resources(false);
        ESP_LOGE(TAG, "Failed to reset panel: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = esp_lcd_panel_init(s_ctx.panel_handle);
    if (ret != ESP_OK) {
        oled_display_release_resources(false);
        ESP_LOGE(TAG, "Failed to init panel: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = esp_lcd_panel_disp_on_off(s_ctx.panel_handle, true);
    if (ret != ESP_OK) {
        oled_display_release_resources(false);
        ESP_LOGE(TAG, "Failed to turn on panel: %s", esp_err_to_name(ret));
        return ret;
    }

    s_ctx.running = true;
    if (xTaskCreate(oled_display_task, "oled_display", 4096, NULL, 3, &s_ctx.task_handle) != pdPASS) {
        oled_display_release_resources(false);
        return ESP_ERR_NO_MEM;
    }

    s_ctx.initialized = true;
    ESP_LOGI(TAG, "OLED ready on I2C port %d SDA=%d SCL=%d addr=0x%02X",
             config->i2c_port, config->sda_gpio_num, config->scl_gpio_num, config->i2c_addr);
    return ESP_OK;
}

void oled_display_deinit(void)
{
    if (!s_ctx.initialized) {
        return;
    }

    s_ctx.running = false;
    oled_display_release_resources(true);
    memset(&s_ctx, 0, sizeof(s_ctx));
}

esp_err_t oled_display_set_radar_ready(bool ready)
{
    return oled_display_update_snapshot(oled_display_update_ready_impl, &ready);
}

esp_err_t oled_display_update_fall_status(const ld6002c_fall_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return oled_display_update_snapshot(oled_display_update_fall_impl, status);
}

esp_err_t oled_display_update_fw_status(const ld6002c_fw_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return oled_display_update_snapshot(oled_display_update_fw_impl, status);
}

esp_err_t oled_display_update_params(const ld6002c_params_t *params)
{
    if (params == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return oled_display_update_snapshot(oled_display_update_params_impl, params);
}

esp_err_t oled_display_update_height_upload(const ld6002c_height_upload_t *result)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return oled_display_update_snapshot(oled_display_update_height_impl, result);
}

esp_err_t oled_display_update_3d_cloud(const ld6002c_3d_cloud_t *cloud)
{
    if (cloud == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return oled_display_update_snapshot(oled_display_update_cloud_impl, cloud);
}

esp_err_t oled_display_update_human_status(const ld6002c_human_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return oled_display_update_snapshot(oled_display_update_human_impl, status);
}
