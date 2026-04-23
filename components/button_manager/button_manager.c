#include "button_manager.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#define BUTTON_DEBOUNCE_MS          20
#define BUTTON_SHORT_PRESS_MAX_MS   2000
#define BUTTON_LONG_PRESS_MIN_MS    5000
#define BUTTON_FACTORY_RESET_MIN_MS 10000

typedef struct {
    bool initialized;
    bool is_pressed;
    int stable_level;
    gpio_num_t gpio_num;
    TickType_t pressed_at_ticks;
    QueueHandle_t event_queue;
    TimerHandle_t debounce_timer;
} button_manager_ctx_t;

static const char *TAG = "button_manager";
static button_manager_ctx_t s_ctx = {0};

static void button_manager_emit_event(button_event_t event)
{
    if (s_ctx.event_queue == NULL) {
        return;
    }

    if (xQueueSend(s_ctx.event_queue, &event, 0) != pdPASS) {
        ESP_LOGW(TAG, "Button event queue is full");
    }
}

static void button_manager_process_release(TickType_t released_at_ticks)
{
    TickType_t duration_ticks = released_at_ticks - s_ctx.pressed_at_ticks;
    uint32_t duration_ms = (uint32_t)(duration_ticks * portTICK_PERIOD_MS);

    if (duration_ms < BUTTON_SHORT_PRESS_MAX_MS) {
        button_manager_emit_event(BUTTON_EVENT_SHORT_PRESS);
    } else if (duration_ms >= BUTTON_FACTORY_RESET_MIN_MS) {
        button_manager_emit_event(BUTTON_EVENT_FACTORY_RESET);
    } else if (duration_ms >= BUTTON_LONG_PRESS_MIN_MS) {
        button_manager_emit_event(BUTTON_EVENT_LONG_PRESS);
    } else {
        ESP_LOGI(TAG, "Ignoring mid-length press: %lu ms", (unsigned long)duration_ms);
    }
}

static void button_manager_debounce_timer_cb(TimerHandle_t timer)
{
    (void)timer;

    int level = gpio_get_level(s_ctx.gpio_num);
    if (level == s_ctx.stable_level) {
        return;
    }

    s_ctx.stable_level = level;
    if (level == 0) {
        s_ctx.is_pressed = true;
        s_ctx.pressed_at_ticks = xTaskGetTickCount();
    } else if (s_ctx.is_pressed) {
        s_ctx.is_pressed = false;
        button_manager_process_release(xTaskGetTickCount());
    }
}

static void IRAM_ATTR button_manager_gpio_isr_handler(void *arg)
{
    (void)arg;

    BaseType_t higher_priority_task_woken = pdFALSE;
    xTimerResetFromISR(s_ctx.debounce_timer, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t button_manager_init(const button_manager_config_t *config)
{
    if (config == NULL || config->event_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx.initialized) {
        return ESP_OK;
    }

    s_ctx.gpio_num = config->gpio_num;
    s_ctx.event_queue = config->event_queue;
    s_ctx.stable_level = 1;
    s_ctx.debounce_timer = xTimerCreate("button_db", pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS),
                                        pdFALSE, NULL, button_manager_debounce_timer_cb);
    if (s_ctx.debounce_timer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t io_config = {
        .pin_bit_mask = 1ULL << config->gpio_num,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_config), TAG, "Failed to configure button GPIO");

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(config->gpio_num, button_manager_gpio_isr_handler, NULL),
                        TAG, "Failed to add button ISR");

    s_ctx.initialized = true;
    return ESP_OK;
}

void button_manager_deinit(void)
{
    if (!s_ctx.initialized) {
        return;
    }

    gpio_isr_handler_remove(s_ctx.gpio_num);
    if (s_ctx.debounce_timer != NULL) {
        xTimerDelete(s_ctx.debounce_timer, portMAX_DELAY);
    }

    s_ctx = (button_manager_ctx_t){0};
}
