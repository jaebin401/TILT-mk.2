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

// wheel_base / TB6612FNG. These defaults avoid the current IMU (GPIO8/9),
// STS3215 UART (GPIO17/18), native USB (GPIO19/20), strapping pins, and the
// ESP32-S3 flash/PSRAM pin ranges. Recheck them against the final carrier PCB.
inline constexpr gpio_num_t kWheelLeftPwmPin = GPIO_NUM_4;
inline constexpr gpio_num_t kWheelLeftIn1Pin = GPIO_NUM_5;
inline constexpr gpio_num_t kWheelLeftIn2Pin = GPIO_NUM_6;
inline constexpr gpio_num_t kWheelRightPwmPin = GPIO_NUM_7;
inline constexpr gpio_num_t kWheelRightIn1Pin = GPIO_NUM_10;
inline constexpr gpio_num_t kWheelRightIn2Pin = GPIO_NUM_11;
inline constexpr gpio_num_t kWheelStandbyPin = GPIO_NUM_12;
inline constexpr bool kWheelLeftInverted = false;
inline constexpr bool kWheelRightInverted = false;

}  // namespace tilt::config::hardware
