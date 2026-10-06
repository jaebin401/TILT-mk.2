#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tilt/config/HardwareConfig.h"
#include "tilt/config/MotionConfig.h"
#include "tilt/config/RobotConfig.h"
#include "tilt/sts3215/Sts3215Bus.h"

namespace {

constexpr char kTag[] = "tilt_main";

constexpr const auto& kServoIds = tilt::config::robot::kServoIds;

constexpr std::array<std::uint32_t, 10> kBaudCandidates = {
    1'000'000, 500'000, 250'000, 128'000, 115'200,
    76'800, 57'600, 38'400, 19'200, 9'600,
};

struct KeyMap {
    char key;
    std::size_t servo_index;
    std::int8_t direction;
};

constexpr std::array<KeyMap, 12> kKeyMap = {{
    {'q', 0, +1}, {'a', 0, -1},
    {'w', 1, +1}, {'s', 1, -1},
    {'e', 2, +1}, {'d', 2, -1},
    {'r', 3, +1}, {'f', 3, -1},
    {'t', 4, +1}, {'g', 4, -1},
    {'y', 5, +1}, {'h', 5, -1},
}};

tilt::sts3215::BusConfig makeBusConfig() {
    tilt::sts3215::BusConfig config;
    config.uart_port = tilt::config::hardware::kServoUartPort;
    config.tx_pin = tilt::config::hardware::kServoUartTxPin;
    config.rx_pin = tilt::config::hardware::kServoUartRxPin;
    config.baud_rate = tilt::config::hardware::kServoUartBaudRate;
    config.response_timeout_ms =
        tilt::config::hardware::kServoResponseTimeoutMs;
    config.rx_buffer_size = tilt::config::hardware::kServoRxBufferSize;
    config.tx_buffer_size = tilt::config::hardware::kServoTxBufferSize;
    return config;
}

tilt::sts3215::Sts3215Bus s_bus(makeBusConfig());
std::array<std::uint16_t, kServoIds.size()> s_current_position{};
bool s_coarse_mode = false;
bool s_armed = false;

std::int32_t currentStep() {
    return s_coarse_mode ? tilt::config::motion::kCoarseStepTicks
                         : tilt::config::motion::kFineStepTicks;
}

std::uint16_t currentSpeed() {
    return s_coarse_mode ? tilt::config::motion::kCoarseServoSpeedRaw
                         : tilt::config::motion::kFineServoSpeedRaw;
}

const char* currentModeName() {
    return s_coarse_mode ? "COARSE" : "FINE";
}

esp_err_t initializeConsoleInput() {
    usb_serial_jtag_driver_config_t config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    const esp_err_t result = usb_serial_jtag_driver_install(&config);
    return result == ESP_ERR_INVALID_STATE ? ESP_OK : result;
}

void printHelp() {
    std::printf("\nTILT STS3215 incremental control\n");
    for (std::size_t index = 0; index < kServoIds.size(); ++index) {
        const auto joint_name = tilt::config::robot::kJointNames[index];
        std::printf("  %c/%c : %.*s (servo %u) +/-\n",
                    kKeyMap[index * 2].key,
                    kKeyMap[index * 2 + 1].key,
                    static_cast<int>(joint_name.size()),
                    joint_name.data(),
                    kServoIds[index]);
    }
    std::printf("  m   : FINE/COARSE mode\n");
    std::printf("  p   : scan servo IDs 0..253\n");
    std::printf("  b   : scan common baud rates using configured IDs\n");
    std::printf("  o   : re-read positions and torque ON\n");
    std::printf("  ?   : print this help\n");
    std::printf("  SPACE: emergency stop, torque OFF\n");
    std::printf("Mode=%s, step=%ld ticks (%.3f deg), armed=%s\n\n",
                currentModeName(),
                static_cast<long>(currentStep()),
                currentStep() * 360.0f / 4096.0f,
                s_armed ? "yes" : "no");
}

void refreshCurrentPositions() {
    for (std::size_t index = 0; index < kServoIds.size(); ++index) {
        std::uint16_t position =
            tilt::config::robot::kPositionReadFallbackTicks;
        const esp_err_t result = s_bus.readPosition(kServoIds[index], position);
        if (result == ESP_OK) {
            ESP_LOGI(kTag,
                     "servo %u present position: %u ticks (%.3f deg)",
                     kServoIds[index],
                     position,
                     tilt::sts3215::Sts3215Bus::ticksToDegrees(position));
        } else {
            ESP_LOGW(kTag,
                     "servo %u position read failed (%s); using %u locally",
                     kServoIds[index],
                     esp_err_to_name(result),
                     tilt::config::robot::kPositionReadFallbackTicks);
        }
        s_current_position[index] = position;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void armServos() {
    refreshCurrentPositions();
    const esp_err_t result = s_bus.setTorqueAll(kServoIds.data(),
                                                kServoIds.size(),
                                                true);
    s_armed = result == ESP_OK;
    if (s_armed) {
        ESP_LOGI(kTag, "all six servos armed");
    } else {
        ESP_LOGE(kTag, "failed to arm all servos: %s", esp_err_to_name(result));
    }
}

void emergencyStop() {
    s_armed = false;
    const esp_err_t result = s_bus.emergencyStop(kServoIds.data(),
                                                 kServoIds.size());
    if (result == ESP_OK) {
        std::printf("!! EMERGENCY STOP: all servo torque OFF !!\n");
    } else {
        std::printf("!! EMERGENCY STOP command error: %s !!\n",
                    esp_err_to_name(result));
    }
}

void toggleStepMode() {
    s_coarse_mode = !s_coarse_mode;
    std::printf("mode=%s, step=%ld ticks (%.3f deg)\n",
                currentModeName(),
                static_cast<long>(currentStep()),
                currentStep() * 360.0f / 4096.0f);
}

void scanServoIds() {
    std::array<std::uint8_t, 254> found_ids{};
    std::size_t found_count = 0;
    std::printf("\nScanning servo IDs 0..253 at %lu bps...\n",
                static_cast<unsigned long>(s_bus.baudRate()));
    const esp_err_t result = s_bus.scanIds(0,
                                           253,
                                           found_ids.data(),
                                           found_ids.size(),
                                           found_count);
    if (result != ESP_OK) {
        std::printf("scan failed: %s\n\n", esp_err_to_name(result));
        return;
    }
    for (std::size_t index = 0;
         index < found_count && index < found_ids.size();
         ++index) {
        std::printf("  found ID %u\n", found_ids[index]);
    }
    std::printf("scan complete: %u servo(s) found\n\n",
                static_cast<unsigned>(found_count));
}

bool anyConfiguredServoResponds() {
    for (const std::uint8_t id : kServoIds) {
        if (s_bus.ping(id) == ESP_OK) {
            return true;
        }
    }
    return false;
}

void scanBaudRates() {
    const std::uint32_t original_baud = s_bus.baudRate();
    std::printf("\nScanning common baud rates...\n");
    for (const std::uint32_t baud : kBaudCandidates) {
        if (s_bus.setBaudRate(baud) != ESP_OK) {
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        std::printf("  %lu bps: ", static_cast<unsigned long>(baud));
        if (anyConfiguredServoResponds()) {
            std::printf("response found; keeping this baud rate\n\n");
            return;
        }
        std::printf("no response\n");
    }
    s_bus.setBaudRate(original_baud);
    std::printf("No configured servo responded; restored %lu bps\n\n",
                static_cast<unsigned long>(original_baud));
}

void moveServo(const KeyMap& mapping) {
    if (!s_armed) {
        std::printf("servos are disarmed; press 'o' to re-arm\n");
        return;
    }

    const std::size_t index = mapping.servo_index;
    const std::int32_t requested =
        static_cast<std::int32_t>(s_current_position[index]) +
        static_cast<std::int32_t>(mapping.direction) * currentStep();
    const std::uint16_t position =
        tilt::sts3215::Sts3215Bus::clampPosition(requested);
    const esp_err_t result = s_bus.writePosition(kServoIds[index],
                                                 position,
                                                 currentSpeed());
    if (result != ESP_OK) {
        std::printf("servo %u write failed: %s\n",
                    kServoIds[index],
                    esp_err_to_name(result));
        return;
    }
    s_current_position[index] = position;
    std::printf("servo %u: position=%u ticks (%.3f deg) [%s]\n",
                kServoIds[index],
                position,
                tilt::sts3215::Sts3215Bus::ticksToDegrees(position),
                currentModeName());
}

void consoleTask(void*) {
    printHelp();
    std::uint8_t input = 0;
    while (true) {
        const int count = usb_serial_jtag_read_bytes(&input,
                                                     1,
                                                     pdMS_TO_TICKS(50));
        if (count <= 0) {
            continue;
        }
        if (input == ' ') {
            emergencyStop();
            continue;
        }

        const char key = static_cast<char>(std::tolower(input));
        if (key == 'm') {
            toggleStepMode();
        } else if (key == 'p') {
            scanServoIds();
        } else if (key == 'b') {
            scanBaudRates();
        } else if (key == 'o') {
            armServos();
        } else if (key == '?') {
            printHelp();
        } else {
            for (const KeyMap& mapping : kKeyMap) {
                if (mapping.key == key) {
                    moveServo(mapping);
                    break;
                }
            }
        }
    }
}

}  // namespace

extern "C" void app_main() {
    esp_err_t result = s_bus.initialize();
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "servo bus initialization failed: %s",
                 esp_err_to_name(result));
        return;
    }

    result = initializeConsoleInput();
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "USB console initialization failed: %s",
                 esp_err_to_name(result));
        return;
    }

    armServos();

    const BaseType_t task_result = xTaskCreate(consoleTask,
                                                "servo_console",
                                                4096,
                                                nullptr,
                                                5,
                                                nullptr);
    if (task_result != pdPASS) {
        ESP_LOGE(kTag, "failed to create servo console task");
        emergencyStop();
    }
}
