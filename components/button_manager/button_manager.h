#ifndef BUTTON_MANAGER_H
#define BUTTON_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/queue.h"

typedef enum {
    BUTTON_EVENT_SHORT_PRESS = 0,
    BUTTON_EVENT_LONG_PRESS,
    BUTTON_EVENT_FACTORY_RESET,
} button_event_t;

typedef struct {
    gpio_num_t gpio_num;
    QueueHandle_t event_queue;
} button_manager_config_t;

esp_err_t button_manager_init(const button_manager_config_t *config);
void button_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_MANAGER_H */
