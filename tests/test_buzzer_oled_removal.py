from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


class BuzzerOledRemovalTest(unittest.TestCase):
    def test_main_uses_buzzer_alarm_on_gpio39_for_fall_status(self):
        main_c = read("main/main.c")
        app_config_h = read("main/app_config.h")
        app_radar_c = read("main/app_radar.c")

        self.assertIn('#include "buzzer_alarm.h"', main_c)
        self.assertIn("#define FALLGUARD_BUZZER_GPIO GPIO_NUM_39", app_config_h)
        self.assertIn(".gpio_num = FALLGUARD_BUZZER_GPIO", main_c)
        self.assertIn("buzzer_alarm_set_active(status->is_fall)", app_radar_c)
        self.assertIn("buzzer_alarm_deinit();", main_c)

    def test_oled_display_is_not_referenced_by_main_or_component_dependencies(self):
        main_c = read("main/main.c")
        main_cmake = read("main/CMakeLists.txt")

        self.assertNotIn("oled_display", main_c)
        self.assertNotIn("OLED", main_c)
        self.assertNotIn("oled_display", main_cmake)
        self.assertIn("buzzer_alarm", main_cmake)

    def test_buzzer_component_is_registered_and_exposes_expected_api(self):
        header = ROOT / "components/buzzer_alarm/buzzer_alarm.h"
        source = ROOT / "components/buzzer_alarm/buzzer_alarm.c"
        cmake = ROOT / "components/buzzer_alarm/CMakeLists.txt"

        self.assertTrue(header.exists())
        self.assertTrue(source.exists())
        self.assertTrue(cmake.exists())

        header_text = header.read_text(encoding="utf-8")
        source_text = source.read_text(encoding="utf-8")
        cmake_text = cmake.read_text(encoding="utf-8")

        self.assertIn("typedef struct", header_text)
        self.assertIn("gpio_num_t gpio_num", header_text)
        self.assertIn("esp_err_t buzzer_alarm_init", header_text)
        self.assertIn("esp_err_t buzzer_alarm_set_active(bool active)", header_text)
        self.assertIn("esp_err_t buzzer_alarm_beep_once(uint32_t duration_ms)", header_text)
        self.assertIn("void buzzer_alarm_deinit(void)", header_text)
        self.assertIn("GPIO_MODE_OUTPUT", source_text)
        self.assertIn("gpio_set_level", source_text)
        self.assertIn("idf_component_register", cmake_text)

    def test_buzzer_gpio_is_active_low(self):
        source_text = read("components/buzzer_alarm/buzzer_alarm.c")

        self.assertIn("#define BUZZER_ALARM_ON_LEVEL  0", source_text)
        self.assertIn("#define BUZZER_ALARM_OFF_LEVEL 1", source_text)
        self.assertIn("gpio_set_level(config->gpio_num, BUZZER_ALARM_OFF_LEVEL)", source_text)
        self.assertIn("gpio_set_level(s_ctx.gpio_num, BUZZER_ALARM_OFF_LEVEL)", source_text)

    def test_startup_plays_one_short_buzzer_beep_after_buzzer_init(self):
        main_c = read("main/main.c")

        init_call = "buzzer_alarm_init(&(buzzer_alarm_config_t)"
        beep_call = "buzzer_alarm_beep_once(FALLGUARD_BOOT_BEEP_MS)"

        self.assertIn("#define FALLGUARD_BOOT_BEEP_MS 35", read("main/app_config.h"))
        self.assertIn(beep_call, main_c)
        self.assertLess(main_c.index(init_call), main_c.index(beep_call))

    def test_radar_logs_presence_and_fall_status_once_per_second(self):
        app_radar_c = read("main/app_radar.c")

        self.assertIn("#define FALLGUARD_RADAR_STATUS_LOG_INTERVAL_MS 1000", app_radar_c)
        self.assertIn("app_log_periodic_radar_status", app_radar_c)
        self.assertIn("是否有人: %s 是否跌倒: %s", app_radar_c)
        self.assertIn("s_latest_is_human", app_radar_c)
        self.assertIn("s_latest_is_fall", app_radar_c)
        self.assertIn("s_latest_target_height_m", app_radar_c)
        self.assertIn("s_latest_target_height_valid", app_radar_c)
        self.assertIn("snprintf(height_text", app_radar_c)
        self.assertIn("app_log_periodic_radar_status();", app_radar_c)

    def test_wifi_provisioning_page_is_chinese(self):
        wifi_manager_c = read("components/wifi_manager/wifi_manager.c")

        self.assertIn("<title>FallGuard 配网</title>", wifi_manager_c)
        self.assertIn("<h1>FallGuard 配网</h1>", wifi_manager_c)
        self.assertIn("连接设备到家庭 Wi-Fi", wifi_manager_c)
        self.assertIn("Wi-Fi 名称", wifi_manager_c)
        self.assertIn("Wi-Fi 密码", wifi_manager_c)
        self.assertIn("开始连接", wifi_manager_c)
        self.assertIn("附近的 Wi-Fi", wifi_manager_c)
        self.assertIn("刷新列表", wifi_manager_c)
        self.assertIn("正在扫描附近 Wi-Fi", wifi_manager_c)
        self.assertNotIn("Wi-Fi Provisioning", wifi_manager_c)
        self.assertNotIn("Nearby Wi-Fi", wifi_manager_c)
        self.assertNotIn("Refresh list", wifi_manager_c)

    def test_wifi_provisioning_saves_radar_install_height(self):
        wifi_manager_h = read("components/wifi_manager/wifi_manager.h")
        wifi_manager_c = read("components/wifi_manager/wifi_manager.c")
        app_radar_c = read("main/app_radar.c")

        self.assertIn("安装高度", wifi_manager_c)
        self.assertIn("id='height_m'", wifi_manager_c)
        self.assertIn("height_m:Number(heightEl.value)", wifi_manager_c)
        self.assertIn("WIFI_KEY_RADAR_HEIGHT_CM", wifi_manager_c)
        self.assertIn("nvs_set_u16(handle, WIFI_KEY_RADAR_HEIGHT_CM", wifi_manager_c)
        self.assertIn("esp_err_t wifi_manager_get_radar_height_m(float *height_m)", wifi_manager_h)
        self.assertIn("wifi_manager_get_radar_height_m(&s_configured_radar_height_m)", app_radar_c)
        self.assertIn("ld6002c_set_height(s_configured_radar_height_m)", app_radar_c)


if __name__ == "__main__":
    unittest.main()
