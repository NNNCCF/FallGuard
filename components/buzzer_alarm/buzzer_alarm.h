#ifndef BUZZER_ALARM_H
#define BUZZER_ALARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t gpio_num;
} buzzer_alarm_config_t;

esp_err_t buzzer_alarm_init(const buzzer_alarm_config_t *config);
esp_err_t buzzer_alarm_beep_once(uint32_t duration_ms);
esp_err_t buzzer_alarm_set_active(bool active);
void buzzer_alarm_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_ALARM_H */
