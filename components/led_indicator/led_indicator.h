#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include "esp_err.h"

typedef enum {
    LED_MODE_AP = 0,
    LED_MODE_STA_CONNECTING,
    LED_MODE_ONLINE,
    LED_MODE_ERROR,
} led_mode_t;

typedef struct {
    gpio_num_t gpio_num;
} led_indicator_config_t;

esp_err_t led_indicator_init(const led_indicator_config_t *config);
esp_err_t led_indicator_set_mode(led_mode_t mode);
void led_indicator_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_INDICATOR_H */
