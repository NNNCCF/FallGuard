from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


class BuzzerOledRemovalTest(unittest.TestCase):
    def test_main_uses_buzzer_alarm_on_gpio39_for_fall_status(self):
        main_c = read("main/main.c")

        self.assertIn('#include "buzzer_alarm.h"', main_c)
        self.assertIn("#define FALLGUARD_BUZZER_GPIO GPIO_NUM_39", main_c)
        self.assertIn(".gpio_num = FALLGUARD_BUZZER_GPIO", main_c)
        self.assertIn("buzzer_alarm_set_active(status->is_fall)", main_c)
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

        self.assertIn("#define FALLGUARD_BOOT_BEEP_MS 35", main_c)
        self.assertIn(beep_call, main_c)
        self.assertLess(main_c.index(init_call), main_c.index(beep_call))


if __name__ == "__main__":
    unittest.main()
