#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/uart.h"

namespace tilt::config::hardware {

inline constexpr uart_port_t kServoUartPort = UART_NUM_1;
inline constexpr gpio_num_t kServoUartTxPin = GPIO_NUM_17;
inline constexpr gpio_num_t kServoUartRxPin = GPIO_NUM_18;
inline constexpr std::uint32_t kServoUartBaudRate = 1'000'000;
inline constexpr std::uint32_t kServoResponseTimeoutMs = 30;
inline constexpr int kServoRxBufferSize = 512;
inline constexpr int kServoTxBufferSize = 512;

}  // namespace tilt::config::hardware
