#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace tilt::sts3215 {

constexpr std::uint8_t kBroadcastId = 0xFE;
constexpr std::uint16_t kPositionMin = 0;
constexpr std::uint16_t kPositionMax = 4095;
constexpr std::uint32_t kDefaultBaudRate = 1'000'000;

struct BusConfig {
    uart_port_t uart_port = UART_NUM_1;
    gpio_num_t tx_pin = GPIO_NUM_17;
    gpio_num_t rx_pin = GPIO_NUM_18;
    std::uint32_t baud_rate = kDefaultBaudRate;
    std::uint32_t response_timeout_ms = 50;
    int rx_buffer_size = 512;
    int tx_buffer_size = 512;
};

class Sts3215Bus {
public:
    explicit Sts3215Bus(const BusConfig& config = {});
    ~Sts3215Bus();

    Sts3215Bus(const Sts3215Bus&) = delete;
    Sts3215Bus& operator=(const Sts3215Bus&) = delete;

    esp_err_t initialize();
    esp_err_t deinitialize();
    bool isInitialized() const;

    esp_err_t setBaudRate(std::uint32_t baud_rate);
    std::uint32_t baudRate() const;

    esp_err_t ping(std::uint8_t id, std::uint8_t* status_error = nullptr);
    esp_err_t scanIds(std::uint8_t first_id,
                      std::uint8_t last_id,
                      std::uint8_t* found_ids,
                      std::size_t found_capacity,
                      std::size_t& found_count);

    esp_err_t readRegister(std::uint8_t id,
                           std::uint8_t address,
                           std::uint8_t* data,
                           std::size_t data_length,
                           std::uint8_t* status_error = nullptr);
    esp_err_t writeRegister(std::uint8_t id,
                            std::uint8_t address,
                            const std::uint8_t* data,
                            std::size_t data_length);

    esp_err_t setTorque(std::uint8_t id, bool enabled);
    esp_err_t setTorqueAll(const std::uint8_t* ids,
                           std::size_t id_count,
                           bool enabled);
    esp_err_t emergencyStop(const std::uint8_t* ids, std::size_t id_count);
    esp_err_t setAcceleration(std::uint8_t id, std::uint8_t acceleration);

    esp_err_t readPosition(std::uint8_t id, std::uint16_t& position_ticks);
    esp_err_t writePosition(std::uint8_t id,
                            std::uint16_t position_ticks,
                            std::uint16_t speed = 0,
                            std::uint16_t move_time = 0);
    esp_err_t syncWritePositions(const std::uint8_t* ids,
                                 const std::uint16_t* position_ticks,
                                 std::size_t servo_count,
                                 std::uint16_t speed = 0,
                                 std::uint16_t move_time = 0);

    static std::uint16_t clampPosition(std::int32_t position_ticks);
    static float ticksToDegrees(std::uint16_t position_ticks);
    static std::uint16_t degreesToTicks(float degrees);

private:
    esp_err_t lock();
    void unlock();
    esp_err_t sendInstructionUnlocked(std::uint8_t id,
                                      std::uint8_t instruction,
                                      const std::uint8_t* parameters,
                                      std::size_t parameter_count);
    esp_err_t transactForStatusUnlocked(std::uint8_t id,
                                        std::uint8_t instruction,
                                        const std::uint8_t* parameters,
                                        std::size_t parameter_count,
                                        std::uint8_t* response_parameters,
                                        std::size_t expected_parameter_count,
                                        std::uint8_t* status_error);

    BusConfig config_;
    SemaphoreHandle_t mutex_ = nullptr;
    bool initialized_ = false;
    bool owns_uart_driver_ = false;
};

}  // namespace tilt::sts3215
