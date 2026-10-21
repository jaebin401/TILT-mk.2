#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"

namespace tilt::tb6612 {

struct MotorPins {
    gpio_num_t pwm = GPIO_NUM_NC;
    gpio_num_t in1 = GPIO_NUM_NC;
    gpio_num_t in2 = GPIO_NUM_NC;
    bool inverted = false;
};

struct Config {
    MotorPins left{};
    MotorPins right{};
    gpio_num_t standby = GPIO_NUM_NC;
    std::uint32_t pwm_frequency_hz = 20'000;
    std::uint32_t timer_resolution_hz = 10'000'000;
    float max_accel_per_second = 1.0f;
    float max_jerk_per_second2 = 3.0f;
};

class Tb6612Driver {
public:
    explicit Tb6612Driver(const Config& config);

    Tb6612Driver(const Tb6612Driver&) = delete;
    Tb6612Driver& operator=(const Tb6612Driver&) = delete;

    esp_err_t initialize();
    esp_err_t enable();

    // Signed normalized commands: -1 reverse, 0 stop, +1 forward.
    void setTarget(float left, float right);
    void stop();

    // Advances the jerk/acceleration-limited wheel state and writes outputs.
    esp_err_t update(float dt_seconds);

    // Immediate hardware-safe stop. Unlike stop(), this intentionally ignores
    // the motion ramp and pulls STBY low.
    esp_err_t emergencyStop();

    float leftTarget() const;
    float rightTarget() const;
    float leftOutput() const;
    float rightOutput() const;
    bool enabled() const { return enabled_; }

private:
    struct WheelState {
        float target = 0.0f;
        float speed = 0.0f;
        float accel = 0.0f;
        int applied_direction = 0;
    };

    esp_err_t configureGpio();
    esp_err_t configurePwm();
    esp_err_t setPwm(mcpwm_gen_handle_t generator,
                     mcpwm_cmpr_handle_t comparator, float duty);
    esp_err_t applyWheel(const MotorPins& pins, WheelState& state,
                         mcpwm_gen_handle_t generator,
                         mcpwm_cmpr_handle_t comparator);
    void advanceWheel(WheelState& state, float dt_seconds);

    Config config_;
    std::uint32_t period_ticks_ = 0;
    mcpwm_timer_handle_t timer_ = nullptr;
    mcpwm_oper_handle_t operator_ = nullptr;
    mcpwm_cmpr_handle_t left_comparator_ = nullptr;
    mcpwm_cmpr_handle_t right_comparator_ = nullptr;
    mcpwm_gen_handle_t left_generator_ = nullptr;
    mcpwm_gen_handle_t right_generator_ = nullptr;
    WheelState left_{};
    WheelState right_{};
    bool initialized_ = false;
    bool enabled_ = false;
};

}  // namespace tilt::tb6612
