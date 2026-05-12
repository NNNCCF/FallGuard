#ifndef FALLGUARD_APP_RADAR_H
#define FALLGUARD_APP_RADAR_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t app_radar_init(void);
void app_radar_deinit(void);
void app_radar_process(void);
bool app_radar_is_initialized(void);

#endif /* FALLGUARD_APP_RADAR_H */
