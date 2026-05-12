#include "buzzer_alarm.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define BUZZER_ALARM_ON_LEVEL  0
#define BUZZER_ALARM_OFF_LEVEL 1

typedef enum {
    BUZZER_ALARM_COMMAND_SET_ACTIVE = 0,
    BUZZER_ALARM_COMMAND_BEEP_ONCE,
} buzzer_alarm_command_type_t;

typedef struct {
    buzzer_alarm_command_type_t type;
    bool active;
    uint32_t duration_ms;
} buzzer_alarm_command_t;

typedef struct {
    bool initialized;
    bool active;
    gpio_num_t gpio_num;
    QueueHandle_t command_queue;
    TaskHandle_t task_handle;
} buzzer_alarm_ctx_t;

static const char *TAG = "buzzer_alarm";
static buzzer_alarm_ctx_t s_ctx = {0};

static bool buzzer_alarm_wait_or_update(TickType_t delay_ticks)
{
    buzzer_alarm_command_t command = {0};

    if (xQueueReceive(s_ctx.command_queue, &command, delay_ticks) == pdPASS) {
        if (command.type == BUZZER_ALARM_COMMAND_BEEP_ONCE) {
            if (!s_ctx.active) {
                gpio_set_level(s_ctx.gpio_num, BUZZER_ALARM_ON_LEVEL);
                vTaskDelay(pdMS_TO_TICKS(command.duration_ms));
                gpio_set_level(s_ctx.gpio_num, BUZZER_ALARM_OFF_LEVEL);
            }
            return true;
        }

        s_ctx.active = command.active;
        if (!command.active) {
            gpio_set_level(s_ctx.gpio_num, BUZZER_ALARM_OFF_LEVEL);
        }
        return true;
    }

    return false;
}

static void buzzer_alarm_task(void *arg)
{
    (void)arg;

    while (true) {
        if (!s_ctx.active) {
            gpio_set_level(s_ctx.gpio_num, BUZZER_ALARM_OFF_LEVEL);
            (void)buzzer_alarm_wait_or_update(portMAX_DELAY);
            continue;
        }

        gpio_set_level(s_ctx.gpio_num, BUZZER_ALARM_ON_LEVEL);
        if (buzzer_alarm_wait_or_update(pdMS_TO_TICKS(120))) {
            continue;
        }

        gpio_set_level(s_ctx.gpio_num, BUZZER_ALARM_OFF_LEVEL);
        if (buzzer_alarm_wait_or_update(pdMS_TO_TICKS(120))) {
            continue;
        }

        gpio_set_level(s_ctx.gpio_num, BUZZER_ALARM_ON_LEVEL);
        if (buzzer_alarm_wait_or_update(pdMS_TO_TICKS(120))) {
            continue;
        }

        gpio_set_level(s_ctx.gpio_num, BUZZER_ALARM_OFF_LEVEL);
        (void)buzzer_alarm_wait_or_update(pdMS_TO_TICKS(640));
    }
}

esp_err_t buzzer_alarm_init(const buzzer_alarm_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx.initialized) {
        return ESP_OK;
    }

    s_ctx.command_queue = xQueueCreate(1, sizeof(buzzer_alarm_command_t));
    if (s_ctx.command_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t io_config = {
        .pin_bit_mask = 1ULL << config->gpio_num,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_config), TAG, "Failed to configure buzzer GPIO");
    ESP_RETURN_ON_ERROR(gpio_set_level(config->gpio_num, BUZZER_ALARM_OFF_LEVEL),
                        TAG, "Failed to clear buzzer GPIO");

    s_ctx.gpio_num = config->gpio_num;
    s_ctx.active = false;

    BaseType_t task_created = xTaskCreate(buzzer_alarm_task, "buzzer_alarm", 2048,
                                          NULL, 4, &s_ctx.task_handle);
    if (task_created != pdPASS) {
        vQueueDelete(s_ctx.command_queue);
        s_ctx.command_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_ctx.initialized = true;
    return ESP_OK;
}

esp_err_t buzzer_alarm_set_active(bool active)
{
    buzzer_alarm_command_t command = {
        .type = BUZZER_ALARM_COMMAND_SET_ACTIVE,
        .active = active,
    };

    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueOverwrite(s_ctx.command_queue, &command) != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t buzzer_alarm_beep_once(uint32_t duration_ms)
{
    buzzer_alarm_command_t command = {
        .type = BUZZER_ALARM_COMMAND_BEEP_ONCE,
        .duration_ms = duration_ms,
    };

    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (duration_ms == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xQueueOverwrite(s_ctx.command_queue, &command) != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void buzzer_alarm_deinit(void)
{
    if (!s_ctx.initialized) {
        return;
    }

    if (s_ctx.task_handle != NULL) {
        vTaskDelete(s_ctx.task_handle);
    }
    if (s_ctx.command_queue != NULL) {
        vQueueDelete(s_ctx.command_queue);
    }

    gpio_set_level(s_ctx.gpio_num, BUZZER_ALARM_OFF_LEVEL);
    s_ctx = (buzzer_alarm_ctx_t){0};
}
