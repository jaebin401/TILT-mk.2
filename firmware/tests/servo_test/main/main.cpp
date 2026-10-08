#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "pwm_servo_test";

// Change this pin to match the servo signal wire.
constexpr gpio_num_t kServoPwmPin = GPIO_NUM_4;

constexpr std::uint32_t kPwmFrequencyHz = 50;
constexpr std::uint32_t kPwmPeriodUs = 1'000'000 / kPwmFrequencyHz;
constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kChannel = LEDC_CHANNEL_0;
constexpr std::uint32_t kDutyResolutionBits = 14;
constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_14_BIT;
constexpr std::uint32_t kMaxDuty = (1U << kDutyResolutionBits) - 1U;

// Electrical guard rails only. Approach the mechanical end stops slowly.
constexpr std::int32_t kMinimumPulseUs = 500;
constexpr std::int32_t kMaximumPulseUs = 2'500;
constexpr std::int32_t kCenterPulseUs = 1'500;

std::int32_t s_pulse_us = kCenterPulseUs;
bool s_output_enabled = false;

std::uint32_t pulseToDuty(std::int32_t pulse_us) {
    return static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(pulse_us) * kMaxDuty +
         kPwmPeriodUs / 2U) /
        kPwmPeriodUs);
}

esp_err_t applyPulse() {
    const std::uint32_t duty = pulseToDuty(s_pulse_us);
    esp_err_t result = ledc_set_duty(kSpeedMode, kChannel, duty);
    if (result != ESP_OK) {
        return result;
    }
    result = ledc_update_duty(kSpeedMode, kChannel);
    if (result == ESP_OK) {
        s_output_enabled = true;
    }
    return result;
}

void printStatus() {
    const float duty_percent =
        100.0f * static_cast<float>(s_pulse_us) /
        static_cast<float>(kPwmPeriodUs);
    std::printf("PWM=%s | pulse=%4ld us | frequency=%3lu Hz | duty=%5.2f %%\n",
                s_output_enabled ? "ON " : "OFF",
                static_cast<long>(s_pulse_us),
                static_cast<unsigned long>(kPwmFrequencyHz),
                duty_percent);
}

void printHelp() {
    std::printf(
        "\nTILT PWM servo calibration\n"
        "  GPIO : %d\n"
        "  Range: %ld..%ld us (electrical guard rails)\n\n"
        "  a / d : -/+   1 us\n"
        "  s / w : -/+  10 us\n"
        "  q / e : -/+ 100 us\n"
        "  1     : 1000 us preset\n"
        "  2     : 1500 us center\n"
        "  3     : 2000 us preset\n"
        "  o     : output ON\n"
        "  SPACE : output OFF immediately\n"
        "  ?     : print this help\n\n",
        static_cast<int>(kServoPwmPin),
        static_cast<long>(kMinimumPulseUs),
        static_cast<long>(kMaximumPulseUs));
    printStatus();
}

void disableOutput() {
    const esp_err_t result = ledc_stop(kSpeedMode, kChannel, 0);
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "failed to stop PWM: %s", esp_err_to_name(result));
        return;
    }
    s_output_enabled = false;
    printStatus();
}

void setPulse(std::int32_t requested_pulse_us) {
    const std::int32_t clamped_pulse_us =
        std::clamp(requested_pulse_us, kMinimumPulseUs, kMaximumPulseUs);
    if (clamped_pulse_us != requested_pulse_us) {
        std::printf("limit reached: requested %ld us, clamped to %ld us\n",
                    static_cast<long>(requested_pulse_us),
                    static_cast<long>(clamped_pulse_us));
    }
    s_pulse_us = clamped_pulse_us;

    const esp_err_t result = applyPulse();
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "failed to update PWM: %s", esp_err_to_name(result));
        return;
    }
    printStatus();
}

esp_err_t initializePwm() {
    const ledc_timer_config_t timer_config = {
        .speed_mode = kSpeedMode,
        .duty_resolution = kDutyResolution,
        .timer_num = kTimer,
        .freq_hz = kPwmFrequencyHz,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    esp_err_t result = ledc_timer_config(&timer_config);
    if (result != ESP_OK) {
        return result;
    }

    const ledc_channel_config_t channel_config = {
        .gpio_num = kServoPwmPin,
        .speed_mode = kSpeedMode,
        .channel = kChannel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = kTimer,
        .duty = pulseToDuty(kCenterPulseUs),
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {},
        .deconfigure = false,
    };
    result = ledc_channel_config(&channel_config);
    if (result != ESP_OK) {
        return result;
    }

    // Boot safely with no control pulses. Press 'o' when the mechanism is clear.
    return ledc_stop(kSpeedMode, kChannel, 0);
}

esp_err_t initializeConsoleInput() {
    usb_serial_jtag_driver_config_t config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    const esp_err_t result = usb_serial_jtag_driver_install(&config);
    return result == ESP_ERR_INVALID_STATE ? ESP_OK : result;
}

void handleInput(char key) {
    switch (key) {
        case 'a': setPulse(s_pulse_us - 1); break;
        case 'd': setPulse(s_pulse_us + 1); break;
        case 's': setPulse(s_pulse_us - 10); break;
        case 'w': setPulse(s_pulse_us + 10); break;
        case 'q': setPulse(s_pulse_us - 100); break;
        case 'e': setPulse(s_pulse_us + 100); break;
        case '1': setPulse(1'000); break;
        case '2': setPulse(kCenterPulseUs); break;
        case '3': setPulse(2'000); break;
        case 'o': setPulse(s_pulse_us); break;
        case ' ': disableOutput(); break;
        case '?': printHelp(); break;
        default: break;
    }
}

void consoleTask(void*) {
    printHelp();
    std::uint8_t input = 0;
    while (true) {
        const int count = usb_serial_jtag_read_bytes(
            &input, 1, pdMS_TO_TICKS(50));
        if (count > 0) {
            handleInput(static_cast<char>(std::tolower(input)));
        }
    }
}

}  // namespace

extern "C" void app_main() {
    setvbuf(stdout, nullptr, _IONBF, 0);

    esp_err_t result = initializePwm();
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "PWM initialization failed: %s",
                 esp_err_to_name(result));
        return;
    }

    result = initializeConsoleInput();
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "USB console initialization failed: %s",
                 esp_err_to_name(result));
        return;
    }

    const BaseType_t task_result = xTaskCreate(consoleTask,
                                                "pwm_servo_console",
                                                4096,
                                                nullptr,
                                                5,
                                                nullptr);
    if (task_result != pdPASS) {
        ESP_LOGE(kTag, "failed to create PWM servo console task");
        disableOutput();
    }
}
