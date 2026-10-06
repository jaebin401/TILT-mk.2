#include <cinttypes>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tilt_config.h"
#include "tilt_mpu6050.h"

namespace {

constexpr char kTag[] = "imu_test";
constexpr std::uint32_t kPrintEverySamples = 10;
constexpr float kRadiansToDegrees = 57.2957795130823208768f;

void printSample(const tilt::ImuRaw& raw, const tilt::Attitude& attitude) {
    std::printf(
        "raw a=(%d,%d,%d) g=(%d,%d,%d) | "
        "roll=%+.2f deg pitch=%+.2f deg yaw=N/A (6-axis IMU)\n",
        raw.ax,
        raw.ay,
        raw.az,
        raw.gx,
        raw.gy,
        raw.gz,
        attitude.roll_rad * kRadiansToDegrees,
        attitude.pitch_rad * kRadiansToDegrees);
}

}  // namespace

extern "C" void app_main() {
    ESP_LOGI(kTag,
             "MPU6050 demo: I2C%d SDA=%d SCL=%d addr=0x%02X, sample every %lu ms",
             tilt::IMU_I2C_PORT,
             tilt::IMU_SDA_PIN,
             tilt::IMU_SCL_PIN,
             tilt::IMU_I2C_ADDR,
             static_cast<unsigned long>(tilt::IMU_SAMPLE_PERIOD_MS));
    ESP_LOGI(kTag,
             "Yaw is intentionally unavailable: MPU6050 has no magnetometer.");

    if (!tilt::imu_init()) {
        ESP_LOGE(kTag,
                 "MPU6050 init failed. Check 3.3V, GND, SDA/SCL, pull-ups, and AD0=GND.");
        return;
    }

    tilt::ComplementaryFilter filter;
    std::int64_t previous_time_us = esp_timer_get_time();
    std::uint32_t sample_count = 0;

    while (true) {
        tilt::ImuRaw raw{};
        const std::int64_t now_us = esp_timer_get_time();
        const float dt_s = static_cast<float>(now_us - previous_time_us) / 1'000'000.0f;
        previous_time_us = now_us;

        if (tilt::imu_read_raw(raw)) {
            const tilt::Attitude attitude = filter.update(raw, dt_s);
            ++sample_count;
            if (sample_count % kPrintEverySamples == 0) {
                printSample(raw, attitude);
            }
        } else {
            ESP_LOGW(kTag, "MPU6050 read failed");
        }

        vTaskDelay(pdMS_TO_TICKS(tilt::IMU_SAMPLE_PERIOD_MS));
    }
}
