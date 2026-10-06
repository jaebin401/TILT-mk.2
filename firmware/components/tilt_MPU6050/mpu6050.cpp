#include "tilt_mpu6050.h"

#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "tilt_config.h"

namespace tilt {
namespace {

constexpr std::uint8_t kRegisterPowerManagement1 = 0x6B;
constexpr std::uint8_t kRegisterConfiguration = 0x1A;
constexpr std::uint8_t kRegisterGyroConfiguration = 0x1B;
constexpr std::uint8_t kRegisterAccelerometerConfiguration = 0x1C;
constexpr std::uint8_t kRegisterAccelerometerXoutHigh = 0x3B;
constexpr std::uint8_t kRegisterWhoAmI = 0x75;
constexpr std::uint8_t kExpectedWhoAmI = 0x68;
constexpr std::uint8_t kDlpf44Hz = 0x03;
constexpr std::uint8_t kFullScale4G = 0x08;
constexpr std::uint8_t kFullScale500DegreesPerSecond = 0x08;
constexpr int kI2cTransferTimeoutMs = 20;

// Sensor axis -> torso-frame axis. Change only these tables if the board is
// mounted differently from the nominal torso-top orientation.
constexpr int AXIS_MAP[3] = {0, 1, 2};
constexpr float AXIS_SIGN[3] = {+1.0f, +1.0f, +1.0f};

i2c_master_bus_handle_t s_i2c_bus = nullptr;
i2c_master_dev_handle_t s_mpu_device = nullptr;

std::int16_t decodeBigEndian(std::uint8_t high, std::uint8_t low) {
    const std::uint16_t packed =
        (static_cast<std::uint16_t>(high) << 8U) | static_cast<std::uint16_t>(low);
    return static_cast<std::int16_t>(packed);
}

std::int16_t mapAxis(const std::int16_t source[3], int torso_axis) {
    std::int32_t mapped = static_cast<std::int32_t>(source[AXIS_MAP[torso_axis]]) *
                          (AXIS_SIGN[torso_axis] < 0.0f ? -1 : +1);
    if (mapped > 32767) {
        mapped = 32767;
    } else if (mapped < -32768) {
        mapped = -32768;
    }
    return static_cast<std::int16_t>(mapped);
}

bool writeRegister(std::uint8_t address, std::uint8_t value) {
    const std::uint8_t payload[] = {address, value};
    return i2c_master_transmit(s_mpu_device,
                               payload,
                               sizeof(payload),
                               kI2cTransferTimeoutMs) == ESP_OK;
}

bool readRegisters(std::uint8_t address, std::uint8_t* data, std::size_t size) {
    return i2c_master_transmit_receive(s_mpu_device,
                                       &address,
                                       1,
                                       data,
                                       size,
                                       kI2cTransferTimeoutMs) == ESP_OK;
}

void releaseI2cHandles() {
    if (s_mpu_device != nullptr) {
        i2c_master_bus_rm_device(s_mpu_device);
        s_mpu_device = nullptr;
    }
    if (s_i2c_bus != nullptr) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = nullptr;
    }
}

}  // namespace

bool imu_init() {
    if (s_mpu_device != nullptr) {
        return true;
    }

    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = static_cast<i2c_port_num_t>(IMU_I2C_PORT);
    bus_config.sda_io_num = static_cast<gpio_num_t>(IMU_SDA_PIN);
    bus_config.scl_io_num = static_cast<gpio_num_t>(IMU_SCL_PIN);
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    if (i2c_new_master_bus(&bus_config, &s_i2c_bus) != ESP_OK) {
        releaseI2cHandles();
        return false;
    }

    i2c_device_config_t device_config{};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = IMU_I2C_ADDR;
    device_config.scl_speed_hz = 400'000;
    if (i2c_master_bus_add_device(s_i2c_bus, &device_config, &s_mpu_device) !=
        ESP_OK) {
        releaseI2cHandles();
        return false;
    }

    std::uint8_t who_am_i = 0;
    if (!readRegisters(kRegisterWhoAmI, &who_am_i, 1) ||
        who_am_i != kExpectedWhoAmI ||
        !writeRegister(kRegisterPowerManagement1, 0x00) ||
        !writeRegister(kRegisterConfiguration, kDlpf44Hz) ||
        !writeRegister(kRegisterGyroConfiguration, kFullScale500DegreesPerSecond) ||
        !writeRegister(kRegisterAccelerometerConfiguration, kFullScale4G)) {
        releaseI2cHandles();
        return false;
    }
    return true;
}

bool imu_read_raw(ImuRaw& out) {
    if (s_mpu_device == nullptr) {
        return false;
    }

    std::uint8_t data[14]{};
    if (!readRegisters(kRegisterAccelerometerXoutHigh, data, sizeof(data))) {
        return false;
    }

    const std::int16_t acceleration_sensor[3] = {
        decodeBigEndian(data[0], data[1]),
        decodeBigEndian(data[2], data[3]),
        decodeBigEndian(data[4], data[5]),
    };
    const std::int16_t gyro_sensor[3] = {
        decodeBigEndian(data[8], data[9]),
        decodeBigEndian(data[10], data[11]),
        decodeBigEndian(data[12], data[13]),
    };

    out.ax = mapAxis(acceleration_sensor, 0);
    out.ay = mapAxis(acceleration_sensor, 1);
    out.az = mapAxis(acceleration_sensor, 2);
    out.gx = mapAxis(gyro_sensor, 0);
    out.gy = mapAxis(gyro_sensor, 1);
    out.gz = mapAxis(gyro_sensor, 2);
    return true;
}

}  // namespace tilt
