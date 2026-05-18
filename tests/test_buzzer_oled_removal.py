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
        self.assertIn("WIFI_KEY_RADAR_HEIGHT_VERSION", wifi_manager_c)
        self.assertIn("nvs_set_u8(handle, WIFI_KEY_RADAR_HEIGHT_VERSION", wifi_manager_c)
        self.assertIn("esp_err_t wifi_manager_get_radar_height_m(float *height_m)", wifi_manager_h)
        self.assertIn("wifi_manager_get_radar_height_m(&s_configured_radar_height_m)", app_radar_c)
        self.assertIn("ld6002c_set_height(s_configured_radar_height_m)", app_radar_c)

    def test_wifi_provisioning_ignores_legacy_saved_default_height(self):
        wifi_manager_c = read("components/wifi_manager/wifi_manager.c")

        self.assertIn("WIFI_RADAR_HEIGHT_STORAGE_VERSION", wifi_manager_c)
        self.assertIn("nvs_get_u8(handle, WIFI_KEY_RADAR_HEIGHT_VERSION", wifi_manager_c)
        self.assertIn("*height_m = WIFI_RADAR_HEIGHT_DEFAULT_M;", wifi_manager_c)
        self.assertIn("legacy radar height", wifi_manager_c)

    def test_radar_reloads_saved_install_height_after_provisioning(self):
        app_radar_c = read("main/app_radar.c")

        self.assertIn("app_maybe_reload_radar_height", app_radar_c)
        self.assertIn("s_last_height_reload_tick", app_radar_c)
        self.assertIn("RADAR_PROFILE_STEP_HEIGHT", app_radar_c)
        self.assertIn("Radar install height changed", app_radar_c)
        self.assertIn("app_maybe_reload_radar_height();", app_radar_c)

    def test_radar_default_profile_matches_boot_requirements(self):
        app_config_h = read("main/app_config.h")
        wifi_manager_c = read("components/wifi_manager/wifi_manager.c")

        self.assertIn("#define FALLGUARD_RADAR_HEIGHT_M 2.4f", app_config_h)
        self.assertIn("#define FALLGUARD_RADAR_THRESHOLD_M 0.5f", app_config_h)
        self.assertIn("#define FALLGUARD_RADAR_SENSITIVITY 20U", app_config_h)
        self.assertIn("#define FALLGUARD_RADAR_ZONE_BOUNDARY_M 1.5f", app_config_h)
        self.assertIn("#define WIFI_RADAR_HEIGHT_DEFAULT_M 2.4f", wifi_manager_c)
        self.assertIn("value='2.4'", wifi_manager_c)

    def test_radar_boot_profile_does_not_wait_for_stream_before_sending(self):
        app_radar_c = read("main/app_radar.c")

        self.assertIn("static void app_maybe_apply_radar_profile(void)", app_radar_c)
        self.assertIn("!s_radar_initialized || s_radar_profile_command_in_flight", app_radar_c)
        self.assertNotIn("!s_radar_initialized || !s_radar_stream_seen || s_radar_profile_command_in_flight", app_radar_c)

    def test_radar_init_sends_height_profile_before_auxiliary_commands(self):
        app_radar_c = read("main/app_radar.c")

        self.assertIn("app_maybe_apply_radar_profile();", app_radar_c)
        self.assertLess(app_radar_c.index("app_maybe_apply_radar_profile();"),
                        app_radar_c.index("ld6002c_set_user_log(false)"))
        self.assertLess(app_radar_c.index("app_maybe_apply_radar_profile();"),
                        app_radar_c.index("ld6002c_query_fw_status()"))

    def test_radar_profile_repeats_each_setting_like_vendor_tool(self):
        app_radar_c = read("main/app_radar.c")

        self.assertIn("#define FALLGUARD_RADAR_PROFILE_SEND_REPEATS 3", app_radar_c)
        self.assertIn("app_send_radar_profile_step", app_radar_c)
        self.assertIn("for (uint8_t attempt = 0; attempt < FALLGUARD_RADAR_PROFILE_SEND_REPEATS; ++attempt)", app_radar_c)
        self.assertIn("Duplicate radar profile ack ignored", app_radar_c)

    def test_radar_profile_does_not_block_later_settings_on_missing_ack(self):
        app_radar_c = read("main/app_radar.c")

        self.assertIn("static void app_send_full_radar_profile(void)", app_radar_c)
        self.assertIn("RADAR_PROFILE_STEP_HEIGHT,", app_radar_c)
        self.assertIn("RADAR_PROFILE_STEP_THRESHOLD,", app_radar_c)
        self.assertIn("RADAR_PROFILE_STEP_SENSITIVITY,", app_radar_c)
        self.assertIn("RADAR_PROFILE_STEP_ALARM_ZONE,", app_radar_c)
        self.assertIn("s_radar_profile_step = RADAR_PROFILE_STEP_DONE;", app_radar_c)
        self.assertIn("app_send_full_radar_profile();", app_radar_c)

    def test_ld6002c_commands_use_client_frame_id_range(self):
        ld6002c_c = read("components/ld6002c/ld6002c.c")

        self.assertIn("#define LD6002C_CLIENT_TX_ID_BASE 0x8000U", ld6002c_c)
        self.assertIn("s_ctx.tx_id = LD6002C_CLIENT_TX_ID_BASE;", ld6002c_c)

    def test_wifi_provisioning_uses_open_ap_and_captive_portal(self):
        wifi_manager_c = read("components/wifi_manager/wifi_manager.c")
        wifi_cmake = read("components/wifi_manager/CMakeLists.txt")

        self.assertIn("WIFI_AUTH_OPEN", wifi_manager_c)
        self.assertIn("wifi_manager_start_dns_server", wifi_manager_c)
        self.assertIn("wifi_manager_stop_dns_server", wifi_manager_c)
        self.assertIn("dns_server_task", wifi_manager_c)
        self.assertIn("lwip/sockets.h", wifi_manager_c)
        self.assertIn("lwip/def.h", wifi_manager_c)
        self.assertIn("/generate_204", wifi_manager_c)
        self.assertIn("/hotspot-detect.html", wifi_manager_c)
        self.assertIn("/connecttest.txt", wifi_manager_c)
        self.assertIn("PRIV_REQUIRES", wifi_cmake)
        self.assertIn("lwip", wifi_cmake)

    def test_wifi_disconnect_logs_driver_reason_for_diagnosis(self):
        wifi_manager_c = read("components/wifi_manager/wifi_manager.c")

        self.assertIn("wifi_event_sta_disconnected_t", wifi_manager_c)
        self.assertIn("wifi_manager_disconnect_reason_to_string", wifi_manager_c)
        self.assertIn("disconnected: reason=%u", wifi_manager_c)
        self.assertIn("WIFI_REASON_BEACON_TIMEOUT", wifi_manager_c)

    def test_wifi_transient_reconnect_failure_keeps_saved_credentials(self):
        wifi_manager_c = read("components/wifi_manager/wifi_manager.c")

        self.assertIn("clear_credentials_on_failure", wifi_manager_c)
        self.assertIn("command == WIFI_CMD_START", wifi_manager_c)
        self.assertIn("if (clear_credentials_on_failure) {", wifi_manager_c)


if __name__ == "__main__":
    unittest.main()
