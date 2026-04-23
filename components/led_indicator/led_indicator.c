#include "led_indicator.h"

#include <stdbool.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

typedef struct {
    bool initialized;
    gpio_num_t gpio_num;
    led_mode_t current_mode;
    QueueHandle_t mode_queue;
    TaskHandle_t task_handle;
} led_indicator_ctx_t;

static const char *TAG = "led_indicator";
static led_indicator_ctx_t s_ctx = {0};

static TickType_t led_indicator_phase_delay(led_mode_t mode, bool led_is_on)
{
    switch (mode) {
    case LED_MODE_AP:
        return pdMS_TO_TICKS(500);
    case LED_MODE_STA_CONNECTING:
        return pdMS_TO_TICKS(125);
    case LED_MODE_ONLINE:
        return pdMS_TO_TICKS(led_is_on ? 100 : 2900);
    case LED_MODE_ERROR:
        return pdMS_TO_TICKS(250);
    default:
        return pdMS_TO_TICKS(500);
    }
}

static void led_indicator_task(void *arg)
{
    (void)arg;

    bool led_is_on = false;
    led_mode_t next_mode = LED_MODE_AP;

    while (true) {
        if (s_ctx.current_mode == LED_MODE_ERROR) {
            gpio_set_level(s_ctx.gpio_num, 1);
            if (xQueueReceive(s_ctx.mode_queue, &next_mode, led_indicator_phase_delay(LED_MODE_ERROR, true)) == pdPASS) {
                s_ctx.current_mode = next_mode;
                led_is_on = true;
            }
            continue;
        }

        gpio_set_level(s_ctx.gpio_num, led_is_on ? 1 : 0);
        if (xQueueReceive(s_ctx.mode_queue, &next_mode,
                          led_indicator_phase_delay(s_ctx.current_mode, led_is_on)) == pdPASS) {
            s_ctx.current_mode = next_mode;
            led_is_on = true;
            continue;
        }

        led_is_on = !led_is_on;
    }
}

esp_err_t led_indicator_init(const led_indicator_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx.initialized) {
        return ESP_OK;
    }

    s_ctx.mode_queue = xQueueCreate(1, sizeof(led_mode_t));
    if (s_ctx.mode_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t io_config = {
        .pin_bit_mask = 1ULL << config->gpio_num,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_config), TAG, "Failed to configure LED GPIO");
    ESP_RETURN_ON_ERROR(gpio_set_level(config->gpio_num, 0), TAG, "Failed to clear LED");

    s_ctx.gpio_num = config->gpio_num;
    s_ctx.current_mode = LED_MODE_AP;

    BaseType_t task_created = xTaskCreate(led_indicator_task, "led_indicator", 2048,
                                          NULL, 4, &s_ctx.task_handle);
    if (task_created != pdPASS) {
        vQueueDelete(s_ctx.mode_queue);
        s_ctx.mode_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_ctx.initialized = true;
    return ESP_OK;
}

esp_err_t led_indicator_set_mode(led_mode_t mode)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueOverwrite(s_ctx.mode_queue, &mode) != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void led_indicator_deinit(void)
{
    if (!s_ctx.initialized) {
        return;
    }

    if (s_ctx.task_handle != NULL) {
        vTaskDelete(s_ctx.task_handle);
    }
    if (s_ctx.mode_queue != NULL) {
        vQueueDelete(s_ctx.mode_queue);
    }

    gpio_set_level(s_ctx.gpio_num, 0);
    s_ctx = (led_indicator_ctx_t){0};
}
