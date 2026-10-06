#include "tilt/sts3215/Sts3215Bus.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#include "freertos/task.h"

namespace tilt::sts3215 {
namespace {

constexpr std::uint8_t kHeader = 0xFF;
constexpr std::uint8_t kInstructionPing = 0x01;
constexpr std::uint8_t kInstructionRead = 0x02;
constexpr std::uint8_t kInstructionWrite = 0x03;
constexpr std::uint8_t kInstructionSyncWrite = 0x83;

constexpr std::uint8_t kAddressTorqueEnable = 40;
constexpr std::uint8_t kAddressAcceleration = 41;
constexpr std::uint8_t kAddressGoalPosition = 42;
constexpr std::uint8_t kAddressPresentPosition = 56;

constexpr std::size_t kMaxParameterCount = 253;
constexpr std::size_t kMaxPacketSize = kMaxParameterCount + 6;
constexpr std::size_t kReceiveBufferSize = kMaxPacketSize;

bool isServoId(std::uint8_t id) {
    return id < kBroadcastId;
}

bool isWritableId(std::uint8_t id) {
    return id <= kBroadcastId;
}

std::uint8_t lowByte(std::uint16_t value) {
    return static_cast<std::uint8_t>(value & 0xFFu);
}

std::uint8_t highByte(std::uint16_t value) {
    return static_cast<std::uint8_t>((value >> 8u) & 0xFFu);
}

std::uint8_t checksum(const std::uint8_t* bytes,
                      std::size_t first,
                      std::size_t last) {
    std::uint32_t sum = 0;
    for (std::size_t index = first; index < last; ++index) {
        sum += bytes[index];
    }
    return static_cast<std::uint8_t>(~sum);
}

esp_err_t makeInstructionPacket(std::uint8_t id,
                                std::uint8_t instruction,
                                const std::uint8_t* parameters,
                                std::size_t parameter_count,
                                std::uint8_t* packet,
                                std::size_t& packet_size) {
    if (!isWritableId(id) || packet == nullptr ||
        parameter_count > kMaxParameterCount ||
        (parameter_count > 0 && parameters == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    packet[0] = kHeader;
    packet[1] = kHeader;
    packet[2] = id;
    packet[3] = static_cast<std::uint8_t>(parameter_count + 2);
    packet[4] = instruction;
    if (parameter_count > 0) {
        std::memcpy(packet + 5, parameters, parameter_count);
    }
    packet_size = parameter_count + 6;
    packet[packet_size - 1] = checksum(packet, 2, packet_size - 1);
    return ESP_OK;
}

}  // namespace

Sts3215Bus::Sts3215Bus(const BusConfig& config) : config_(config) {}

Sts3215Bus::~Sts3215Bus() {
    deinitialize();
}

esp_err_t Sts3215Bus::initialize() {
    if (initialized_) {
        return ESP_OK;
    }
    if (config_.baud_rate == 0 || config_.response_timeout_ms == 0 ||
        config_.rx_buffer_size <= 0 || config_.tx_buffer_size < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (mutex_ == nullptr) {
        mutex_ = xSemaphoreCreateMutex();
        if (mutex_ == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    uart_config_t uart_config{};
    uart_config.baud_rate = static_cast<int>(config_.baud_rate);
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t result = uart_driver_install(config_.uart_port,
                                           config_.rx_buffer_size,
                                           config_.tx_buffer_size,
                                           0,
                                           nullptr,
                                           0);
    if (result == ESP_OK) {
        owns_uart_driver_ = true;
    } else if (result != ESP_ERR_INVALID_STATE) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
        return result;
    }

    result = uart_param_config(config_.uart_port, &uart_config);
    if (result == ESP_OK) {
        result = uart_set_pin(config_.uart_port,
                              config_.tx_pin,
                              config_.rx_pin,
                              UART_PIN_NO_CHANGE,
                              UART_PIN_NO_CHANGE);
    }
    if (result != ESP_OK) {
        if (owns_uart_driver_) {
            uart_driver_delete(config_.uart_port);
        }
        owns_uart_driver_ = false;
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
        return result;
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t Sts3215Bus::deinitialize() {
    esp_err_t result = ESP_OK;
    if (initialized_ && owns_uart_driver_) {
        result = uart_driver_delete(config_.uart_port);
    }
    initialized_ = false;
    owns_uart_driver_ = false;
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
    return result;
}

bool Sts3215Bus::isInitialized() const {
    return initialized_;
}

esp_err_t Sts3215Bus::lock() {
    if (!initialized_ || mutex_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const TickType_t wait_ticks = std::max<TickType_t>(
        1, pdMS_TO_TICKS(config_.response_timeout_ms + 20));
    return xSemaphoreTake(mutex_, wait_ticks) == pdTRUE ? ESP_OK
                                                        : ESP_ERR_TIMEOUT;
}

void Sts3215Bus::unlock() {
    xSemaphoreGive(mutex_);
}

esp_err_t Sts3215Bus::setBaudRate(std::uint32_t baud_rate) {
    if (baud_rate == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t result = lock();
    if (result != ESP_OK) {
        return result;
    }
    result = uart_set_baudrate(config_.uart_port, baud_rate);
    if (result == ESP_OK) {
        config_.baud_rate = baud_rate;
    }
    unlock();
    return result;
}

std::uint32_t Sts3215Bus::baudRate() const {
    return config_.baud_rate;
}

esp_err_t Sts3215Bus::sendInstructionUnlocked(
    std::uint8_t id,
    std::uint8_t instruction,
    const std::uint8_t* parameters,
    std::size_t parameter_count) {
    std::array<std::uint8_t, kMaxPacketSize> packet{};
    std::size_t packet_size = 0;
    esp_err_t result = makeInstructionPacket(id,
                                             instruction,
                                             parameters,
                                             parameter_count,
                                             packet.data(),
                                             packet_size);
    if (result != ESP_OK) {
        return result;
    }

    const int written = uart_write_bytes(config_.uart_port,
                                         packet.data(),
                                         packet_size);
    if (written != static_cast<int>(packet_size)) {
        return ESP_FAIL;
    }
    return uart_wait_tx_done(config_.uart_port,
                             pdMS_TO_TICKS(config_.response_timeout_ms));
}

esp_err_t Sts3215Bus::transactForStatusUnlocked(
    std::uint8_t id,
    std::uint8_t instruction,
    const std::uint8_t* parameters,
    std::size_t parameter_count,
    std::uint8_t* response_parameters,
    std::size_t expected_parameter_count,
    std::uint8_t* status_error) {
    if (!isServoId(id) || expected_parameter_count > kMaxParameterCount ||
        (expected_parameter_count > 0 && response_parameters == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    std::array<std::uint8_t, kMaxPacketSize> request{};
    std::size_t request_size = 0;
    esp_err_t result = makeInstructionPacket(id,
                                             instruction,
                                             parameters,
                                             parameter_count,
                                             request.data(),
                                             request_size);
    if (result != ESP_OK) {
        return result;
    }

    uart_flush_input(config_.uart_port);
    const int written = uart_write_bytes(config_.uart_port,
                                         request.data(),
                                         request_size);
    if (written != static_cast<int>(request_size)) {
        return ESP_FAIL;
    }
    result = uart_wait_tx_done(config_.uart_port,
                              pdMS_TO_TICKS(config_.response_timeout_ms));
    if (result != ESP_OK) {
        return result;
    }

    std::array<std::uint8_t, kReceiveBufferSize> received{};
    std::size_t received_size = 0;
    const TickType_t timeout_ticks = std::max<TickType_t>(
        1, pdMS_TO_TICKS(config_.response_timeout_ms));
    const TickType_t start = xTaskGetTickCount();

    while (received_size < received.size() &&
           xTaskGetTickCount() - start <= timeout_ticks) {
        const TickType_t elapsed = xTaskGetTickCount() - start;
        const TickType_t remaining = elapsed < timeout_ticks
                                         ? timeout_ticks - elapsed
                                         : 1;
        const int count = uart_read_bytes(config_.uart_port,
                                          received.data() + received_size,
                                          received.size() - received_size,
                                          remaining);
        if (count > 0) {
            received_size += static_cast<std::size_t>(count);
        }

        const std::size_t response_size = expected_parameter_count + 6;
        for (std::size_t offset = 0;
             offset + response_size <= received_size;
             ++offset) {
            const std::uint8_t* candidate = received.data() + offset;
            if (candidate[0] != kHeader || candidate[1] != kHeader ||
                candidate[2] != id ||
                candidate[3] != expected_parameter_count + 2 ||
                candidate[response_size - 1] !=
                    checksum(candidate, 2, response_size - 1)) {
                continue;
            }

            if (response_size == request_size &&
                std::memcmp(candidate, request.data(), request_size) == 0) {
                continue;
            }

            const std::uint8_t error = candidate[4];
            if (status_error != nullptr) {
                *status_error = error;
            }
            if (expected_parameter_count > 0) {
                std::memcpy(response_parameters,
                            candidate + 5,
                            expected_parameter_count);
            }
            return error == 0 ? ESP_OK : ESP_FAIL;
        }

        if (count <= 0) {
            break;
        }
    }
    return received_size == 0 ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t Sts3215Bus::ping(std::uint8_t id, std::uint8_t* status_error) {
    esp_err_t result = lock();
    if (result != ESP_OK) {
        return result;
    }
    result = transactForStatusUnlocked(id,
                                       kInstructionPing,
                                       nullptr,
                                       0,
                                       nullptr,
                                       0,
                                       status_error);
    unlock();
    return result;
}

esp_err_t Sts3215Bus::scanIds(std::uint8_t first_id,
                              std::uint8_t last_id,
                              std::uint8_t* found_ids,
                              std::size_t found_capacity,
                              std::size_t& found_count) {
    if (!isServoId(first_id) || !isServoId(last_id) ||
        first_id > last_id || (found_capacity > 0 && found_ids == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    found_count = 0;
    for (std::uint16_t id = first_id; id <= last_id; ++id) {
        if (ping(static_cast<std::uint8_t>(id)) == ESP_OK) {
            if (found_count < found_capacity) {
                found_ids[found_count] = static_cast<std::uint8_t>(id);
            }
            ++found_count;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_OK;
}

esp_err_t Sts3215Bus::readRegister(std::uint8_t id,
                                   std::uint8_t address,
                                   std::uint8_t* data,
                                   std::size_t data_length,
                                   std::uint8_t* status_error) {
    if (!isServoId(id) || data == nullptr || data_length == 0 ||
        data_length > 253) {
        return ESP_ERR_INVALID_ARG;
    }
    const std::uint8_t parameters[] = {
        address, static_cast<std::uint8_t>(data_length)};
    esp_err_t result = lock();
    if (result != ESP_OK) {
        return result;
    }
    result = transactForStatusUnlocked(id,
                                       kInstructionRead,
                                       parameters,
                                       sizeof(parameters),
                                       data,
                                       data_length,
                                       status_error);
    unlock();
    return result;
}

esp_err_t Sts3215Bus::writeRegister(std::uint8_t id,
                                    std::uint8_t address,
                                    const std::uint8_t* data,
                                    std::size_t data_length) {
    if (!isWritableId(id) || data == nullptr || data_length == 0 ||
        data_length > 252) {
        return ESP_ERR_INVALID_ARG;
    }
    std::array<std::uint8_t, kMaxParameterCount> parameters{};
    parameters[0] = address;
    std::memcpy(parameters.data() + 1, data, data_length);

    esp_err_t result = lock();
    if (result != ESP_OK) {
        return result;
    }
    result = sendInstructionUnlocked(id,
                                     kInstructionWrite,
                                     parameters.data(),
                                     data_length + 1);
    unlock();
    return result;
}

esp_err_t Sts3215Bus::setTorque(std::uint8_t id, bool enabled) {
    const std::uint8_t value = enabled ? 1 : 0;
    return writeRegister(id, kAddressTorqueEnable, &value, 1);
}

esp_err_t Sts3215Bus::setTorqueAll(const std::uint8_t* ids,
                                   std::size_t id_count,
                                   bool enabled) {
    if (ids == nullptr || id_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t first_error = ESP_OK;
    for (std::size_t index = 0; index < id_count; ++index) {
        const esp_err_t result = setTorque(ids[index], enabled);
        if (first_error == ESP_OK && result != ESP_OK) {
            first_error = result;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return first_error;
}

esp_err_t Sts3215Bus::emergencyStop(const std::uint8_t* ids,
                                    std::size_t id_count) {
    return setTorqueAll(ids, id_count, false);
}

esp_err_t Sts3215Bus::setAcceleration(std::uint8_t id,
                                      std::uint8_t acceleration) {
    return writeRegister(id, kAddressAcceleration, &acceleration, 1);
}

esp_err_t Sts3215Bus::readPosition(std::uint8_t id,
                                   std::uint16_t& position_ticks) {
    std::uint8_t data[2]{};
    const esp_err_t result = readRegister(id,
                                          kAddressPresentPosition,
                                          data,
                                          sizeof(data));
    if (result != ESP_OK) {
        return result;
    }
    const std::uint16_t position = static_cast<std::uint16_t>(data[0]) |
        (static_cast<std::uint16_t>(data[1]) << 8u);
    if (position > kPositionMax) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    position_ticks = position;
    return ESP_OK;
}

esp_err_t Sts3215Bus::writePosition(std::uint8_t id,
                                    std::uint16_t position_ticks,
                                    std::uint16_t speed,
                                    std::uint16_t move_time) {
    if (position_ticks > kPositionMax) {
        return ESP_ERR_INVALID_ARG;
    }
    const std::uint8_t data[] = {
        lowByte(position_ticks), highByte(position_ticks),
        lowByte(move_time), highByte(move_time),
        lowByte(speed), highByte(speed),
    };
    return writeRegister(id, kAddressGoalPosition, data, sizeof(data));
}

esp_err_t Sts3215Bus::syncWritePositions(
    const std::uint8_t* ids,
    const std::uint16_t* position_ticks,
    std::size_t servo_count,
    std::uint16_t speed,
    std::uint16_t move_time) {
    constexpr std::size_t kBytesPerServo = 7;
    constexpr std::size_t kMaxServoCount =
        (kMaxParameterCount - 2) / kBytesPerServo;
    if (ids == nullptr || position_ticks == nullptr || servo_count == 0 ||
        servo_count > kMaxServoCount) {
        return ESP_ERR_INVALID_ARG;
    }

    std::array<std::uint8_t, kMaxParameterCount> parameters{};
    std::size_t offset = 0;
    parameters[offset++] = kAddressGoalPosition;
    parameters[offset++] = 6;
    for (std::size_t index = 0; index < servo_count; ++index) {
        if (!isServoId(ids[index]) || position_ticks[index] > kPositionMax) {
            return ESP_ERR_INVALID_ARG;
        }
        parameters[offset++] = ids[index];
        parameters[offset++] = lowByte(position_ticks[index]);
        parameters[offset++] = highByte(position_ticks[index]);
        parameters[offset++] = lowByte(move_time);
        parameters[offset++] = highByte(move_time);
        parameters[offset++] = lowByte(speed);
        parameters[offset++] = highByte(speed);
    }

    esp_err_t result = lock();
    if (result != ESP_OK) {
        return result;
    }
    result = sendInstructionUnlocked(kBroadcastId,
                                     kInstructionSyncWrite,
                                     parameters.data(),
                                     offset);
    unlock();
    return result;
}

std::uint16_t Sts3215Bus::clampPosition(std::int32_t position_ticks) {
    if (position_ticks < static_cast<std::int32_t>(kPositionMin)) {
        return kPositionMin;
    }
    if (position_ticks > static_cast<std::int32_t>(kPositionMax)) {
        return kPositionMax;
    }
    return static_cast<std::uint16_t>(position_ticks);
}

float Sts3215Bus::ticksToDegrees(std::uint16_t position_ticks) {
    return static_cast<float>(std::min(position_ticks, kPositionMax)) *
           360.0f / 4096.0f;
}

std::uint16_t Sts3215Bus::degreesToTicks(float degrees) {
    if (!std::isfinite(degrees)) {
        return kPositionMin;
    }
    const float clamped = std::clamp(degrees, 0.0f, 360.0f);
    const auto ticks = static_cast<std::int32_t>(
        std::lround(clamped * 4096.0f / 360.0f));
    return clampPosition(ticks);
}

}  // namespace tilt::sts3215
