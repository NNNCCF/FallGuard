#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_types.h"
#include "ld6002c.h"

typedef struct {
    i2c_port_num_t i2c_port;
    gpio_num_t sda_gpio_num;
    gpio_num_t scl_gpio_num;
    uint8_t i2c_addr;
    uint8_t width;
    uint8_t height;
} oled_display_config_t;

esp_err_t oled_display_init(const oled_display_config_t *config);
void oled_display_deinit(void);

esp_err_t oled_display_set_radar_ready(bool ready);
esp_err_t oled_display_update_fall_status(const ld6002c_fall_status_t *status);
esp_err_t oled_display_update_fw_status(const ld6002c_fw_status_t *status);
esp_err_t oled_display_update_params(const ld6002c_params_t *params);
esp_err_t oled_display_update_height_upload(const ld6002c_height_upload_t *result);
esp_err_t oled_display_update_3d_cloud(const ld6002c_3d_cloud_t *cloud);
esp_err_t oled_display_update_human_status(const ld6002c_human_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* OLED_DISPLAY_H */
