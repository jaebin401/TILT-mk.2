#include "tilt/tb6612/Tb6612Driver.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>

namespace tilt::tb6612 {
namespace {

constexpr float kStoppedEpsilon = 0.0005f;

float clampCommand(float value) {
    if (!std::isfinite(value)) return 0.0f;
    return std::clamp(value, -1.0f, 1.0f);
}

float moveToward(float current, float target, float maximum_delta) {
    if (current < target) return std::min(current + maximum_delta, target);
    if (current > target) return std::max(current - maximum_delta, target);
    return current;
}

}  // namespace

Tb6612Driver::Tb6612Driver(const Config& config) : config_(config) {}

esp_err_t Tb6612Driver::configureGpio() {
    if (config_.standby == GPIO_NUM_NC ||
        config_.left.in1 == GPIO_NUM_NC || config_.left.in2 == GPIO_NUM_NC ||
        config_.right.in1 == GPIO_NUM_NC || config_.right.in2 == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_config_t output_config{};
    output_config.pin_bit_mask =
        (1ULL << config_.standby) |
        (1ULL << config_.left.in1) | (1ULL << config_.left.in2) |
        (1ULL << config_.right.in1) | (1ULL << config_.right.in2);
    output_config.mode = GPIO_MODE_OUTPUT;
    output_config.pull_up_en = GPIO_PULLUP_DISABLE;
    output_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    output_config.intr_type = GPIO_INTR_DISABLE;
    esp_err_t result = gpio_config(&output_config);
    if (result != ESP_OK) return result;

    gpio_set_level(config_.standby, 0);
    gpio_set_level(config_.left.in1, 0);
    gpio_set_level(config_.left.in2, 0);
    gpio_set_level(config_.right.in1, 0);
    gpio_set_level(config_.right.in2, 0);
    return ESP_OK;
}

esp_err_t Tb6612Driver::configurePwm() {
    if (config_.left.pwm == GPIO_NUM_NC || config_.right.pwm == GPIO_NUM_NC ||
        config_.pwm_frequency_hz == 0 || config_.timer_resolution_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    period_ticks_ = config_.timer_resolution_hz / config_.pwm_frequency_hz;
    if (period_ticks_ < 2) return ESP_ERR_INVALID_ARG;

    mcpwm_timer_config_t timer_config{};
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = config_.timer_resolution_hz;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = period_ticks_;
    esp_err_t result = mcpwm_new_timer(&timer_config, &timer_);
    if (result != ESP_OK) return result;

    mcpwm_operator_config_t operator_config{};
    operator_config.group_id = 0;
    result = mcpwm_new_operator(&operator_config, &operator_);
    if (result != ESP_OK) return result;
    result = mcpwm_operator_connect_timer(operator_, timer_);
    if (result != ESP_OK) return result;

    mcpwm_comparator_config_t comparator_config{};
    comparator_config.flags.update_cmp_on_tez = true;
    result = mcpwm_new_comparator(operator_, &comparator_config,
                                  &left_comparator_);
    if (result != ESP_OK) return result;
    result = mcpwm_new_comparator(operator_, &comparator_config,
                                  &right_comparator_);
    if (result != ESP_OK) return result;

    mcpwm_generator_config_t generator_config{};
    generator_config.gen_gpio_num = config_.left.pwm;
    result = mcpwm_new_generator(operator_, &generator_config,
                                 &left_generator_);
    if (result != ESP_OK) return result;
    generator_config.gen_gpio_num = config_.right.pwm;
    result = mcpwm_new_generator(operator_, &generator_config,
                                 &right_generator_);
    if (result != ESP_OK) return result;

    for (mcpwm_gen_handle_t generator : {left_generator_, right_generator_}) {
        mcpwm_gen_timer_event_action_t on_empty{};
        on_empty.direction = MCPWM_TIMER_DIRECTION_UP;
        on_empty.event = MCPWM_TIMER_EVENT_EMPTY;
        on_empty.action = MCPWM_GEN_ACTION_HIGH;
        result = mcpwm_generator_set_action_on_timer_event(generator, on_empty);
        if (result != ESP_OK) return result;
    }

    mcpwm_gen_compare_event_action_t left_compare{};
    left_compare.direction = MCPWM_TIMER_DIRECTION_UP;
    left_compare.comparator = left_comparator_;
    left_compare.action = MCPWM_GEN_ACTION_LOW;
    result = mcpwm_generator_set_action_on_compare_event(left_generator_,
                                                         left_compare);
    if (result != ESP_OK) return result;

    mcpwm_gen_compare_event_action_t right_compare{};
    right_compare.direction = MCPWM_TIMER_DIRECTION_UP;
    right_compare.comparator = right_comparator_;
    right_compare.action = MCPWM_GEN_ACTION_LOW;
    result = mcpwm_generator_set_action_on_compare_event(right_generator_,
                                                          right_compare);
    if (result != ESP_OK) return result;

    result = mcpwm_comparator_set_compare_value(left_comparator_, 0);
    if (result != ESP_OK) return result;
    result = mcpwm_comparator_set_compare_value(right_comparator_, 0);
    if (result != ESP_OK) return result;
    result = mcpwm_generator_set_force_level(left_generator_, 0, true);
    if (result != ESP_OK) return result;
    result = mcpwm_generator_set_force_level(right_generator_, 0, true);
    if (result != ESP_OK) return result;

    result = mcpwm_timer_enable(timer_);
    if (result != ESP_OK) return result;
    return mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP);
}

esp_err_t Tb6612Driver::initialize() {
    if (initialized_) return ESP_OK;
    esp_err_t result = configureGpio();
    if (result != ESP_OK) return result;
    result = configurePwm();
    if (result != ESP_OK) return result;
    initialized_ = true;
    return emergencyStop();
}

esp_err_t Tb6612Driver::enable() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    left_ = {};
    right_ = {};
    esp_err_t result = applyWheel(config_.left, left_, left_generator_,
                                  left_comparator_);
    if (result != ESP_OK) return result;
    result = applyWheel(config_.right, right_, right_generator_,
                        right_comparator_);
    if (result != ESP_OK) return result;
    result = gpio_set_level(config_.standby, 1);
    if (result == ESP_OK) enabled_ = true;
    return result;
}

void Tb6612Driver::setTarget(float left, float right) {
    left_.target = clampCommand(left);
    right_.target = clampCommand(right);
}

void Tb6612Driver::stop() {
    setTarget(0.0f, 0.0f);
}

void Tb6612Driver::advanceWheel(WheelState& state, float dt_seconds) {
    const float old_speed = state.speed;
    const float desired_accel = std::clamp(
        (state.target - state.speed) / dt_seconds,
        -config_.max_accel_per_second, config_.max_accel_per_second);
    state.accel = moveToward(
        state.accel, desired_accel,
        config_.max_jerk_per_second2 * dt_seconds);
    float next_speed = state.speed + state.accel * dt_seconds;

    // Do not cross a target or reverse an H-bridge while still carrying a
    // non-zero command from the previous direction.
    if ((state.target - old_speed) * (state.target - next_speed) <= 0.0f) {
        next_speed = state.target;
        state.accel = 0.0f;
    }
    if (old_speed * next_speed < 0.0f) {
        next_speed = 0.0f;
        state.accel = 0.0f;
    }
    if (std::fabs(next_speed) < kStoppedEpsilon &&
        std::fabs(state.target) < kStoppedEpsilon) {
        next_speed = 0.0f;
        state.accel = 0.0f;
    }
    state.speed = clampCommand(next_speed);
}

esp_err_t Tb6612Driver::setPwm(mcpwm_gen_handle_t generator,
                               mcpwm_cmpr_handle_t comparator, float duty) {
    duty = std::clamp(duty, 0.0f, 1.0f);
    if (duty <= kStoppedEpsilon) {
        esp_err_t result = mcpwm_comparator_set_compare_value(comparator, 0);
        if (result != ESP_OK) return result;
        return mcpwm_generator_set_force_level(generator, 0, true);
    }

    std::uint32_t compare = static_cast<std::uint32_t>(
        std::lround(duty * static_cast<float>(period_ticks_)));
    compare = std::min(compare, period_ticks_);
    esp_err_t result = mcpwm_comparator_set_compare_value(comparator, compare);
    if (result != ESP_OK) return result;
    return mcpwm_generator_set_force_level(generator, -1, true);
}

esp_err_t Tb6612Driver::applyWheel(const MotorPins& pins, WheelState& state,
                                   mcpwm_gen_handle_t generator,
                                   mcpwm_cmpr_handle_t comparator) {
    int direction = state.speed > kStoppedEpsilon
        ? 1 : (state.speed < -kStoppedEpsilon ? -1 : 0);
    if (pins.inverted) direction = -direction;

    if (direction != state.applied_direction) {
        esp_err_t result = setPwm(generator, comparator, 0.0f);
        if (result != ESP_OK) return result;
        gpio_set_level(pins.in1, direction > 0 ? 1 : 0);
        gpio_set_level(pins.in2, direction < 0 ? 1 : 0);
        state.applied_direction = direction;
    }
    return setPwm(generator, comparator, std::fabs(state.speed));
}

esp_err_t Tb6612Driver::update(float dt_seconds) {
    if (!initialized_ || !enabled_ || !std::isfinite(dt_seconds) ||
        dt_seconds <= 0.0f) {
        return ESP_ERR_INVALID_STATE;
    }
    advanceWheel(left_, dt_seconds);
    advanceWheel(right_, dt_seconds);
    esp_err_t result = applyWheel(config_.left, left_, left_generator_,
                                  left_comparator_);
    if (result != ESP_OK) return result;
    return applyWheel(config_.right, right_, right_generator_,
                      right_comparator_);
}

esp_err_t Tb6612Driver::emergencyStop() {
    left_ = {};
    right_ = {};
    enabled_ = false;
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = setPwm(left_generator_, left_comparator_, 0.0f);
    const esp_err_t right_result =
        setPwm(right_generator_, right_comparator_, 0.0f);
    if (first_error == ESP_OK) first_error = right_result;
    gpio_set_level(config_.left.in1, 0);
    gpio_set_level(config_.left.in2, 0);
    gpio_set_level(config_.right.in1, 0);
    gpio_set_level(config_.right.in2, 0);
    const esp_err_t standby_result = gpio_set_level(config_.standby, 0);
    if (first_error == ESP_OK) first_error = standby_result;
    return first_error;
}

float Tb6612Driver::leftTarget() const { return left_.target; }
float Tb6612Driver::rightTarget() const { return right_.target; }
float Tb6612Driver::leftOutput() const { return left_.speed; }
float Tb6612Driver::rightOutput() const { return right_.speed; }

}  // namespace tilt::tb6612
