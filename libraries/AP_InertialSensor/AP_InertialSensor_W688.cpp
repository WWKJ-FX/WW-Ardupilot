/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_W688.h"

enum W688Register : uint8_t {
    W688_REG_WHO_AM_I = 0x00,
    W688_REG_CTRL1 = 0x02,
    W688_REG_CTRL2 = 0x03,
    W688_REG_CTRL3 = 0x04,
    W688_REG_CTRL5 = 0x06,
    W688_REG_CTRL7 = 0x08,
    W688_REG_STATUSINT = 0x2D,
    W688_REG_TEMP_L = 0x33,
    W688_REG_AX_L = 0x35,
    W688_REG_RESET_DONE = 0x4D,
    W688_REG_RESET = 0x60,
};

static constexpr uint8_t W688_CTRL1_ADDR_AI_EN = 1U << 6;
static constexpr uint8_t W688_CTRL1_BE = 1U << 5;
static constexpr uint8_t W688_CTRL5_ACC_LPF_ENABLE = 1U << 0;
static constexpr uint8_t W688_CTRL5_ACC_LPF_MODE_3 = 0x03U << 1;
static constexpr uint8_t W688_CTRL5_GYR_LPF_ENABLE = 1U << 4;
static constexpr uint8_t W688_CTRL5_GYR_LPF_MODE_3 = 0x03U << 5;
static constexpr uint8_t W688_CTRL7_SYNCSAMPLE = 1U << 7;
static constexpr uint8_t W688_STATUSINT_AVAIL = 1U << 0;
static constexpr uint8_t W688_STATUSINT_LOCKED = 1U << 1;

static constexpr uint8_t W688_WHO_AM_I_VALUE = 0x05;
static constexpr uint8_t W688_SOFT_RESET_CMD = 0xB0;
static constexpr uint8_t W688_RESET_DONE_VALUE = 0x80;

static constexpr uint8_t W688_CTRL1_DEFAULT = W688_CTRL1_ADDR_AI_EN | W688_CTRL1_BE;
static constexpr uint8_t W688_CTRL2_DEFAULT = (0x03U << 4) | 0x03U; // 16g, 896Hz
static constexpr uint8_t W688_CTRL3_DEFAULT = (0x07U << 4) | 0x03U; // 2048dps, 896Hz
static constexpr uint8_t W688_CTRL5_DEFAULT = W688_CTRL5_ACC_LPF_ENABLE |
                                                   W688_CTRL5_ACC_LPF_MODE_3 |
                                                   W688_CTRL5_GYR_LPF_ENABLE |
                                                   W688_CTRL5_GYR_LPF_MODE_3;
static constexpr uint8_t W688_CTRL7_DISABLE_ALL = 0x00;
static constexpr uint8_t W688_CTRL7_DEFAULT = W688_CTRL7_SYNCSAMPLE | 0x03U;

static constexpr uint8_t W688_READ_FLAG = 0x80;
static constexpr uint8_t W688_IMU_BURST_LENGTH = 12;
static constexpr uint8_t W688_WRITE_VERIFY_RETRIES = 5;
static constexpr uint8_t W688_HARDWARE_INIT_MAX_TRIES = 5;
static constexpr uint16_t W688_BACKEND_SAMPLE_RATE = 896;
static constexpr uint32_t W688_BACKEND_PERIOD_US = 1116;
static constexpr uint8_t W688_DEVTYPE = AP_InertialSensor_Backend::DEVTYPE_INS_W688;
static constexpr float W688_ACCEL_SCALE = GRAVITY_MSS / 2048.0f;
static constexpr float W688_GYRO_SCALE = radians(1.0f / 16.0f);

extern const AP_HAL::HAL& hal;

static int16_t combine_signed(uint8_t msb, uint8_t lsb)
{
    return int16_t(uint16_t(lsb) | (uint16_t(msb) << 8));
}

AP_InertialSensor_W688::AP_InertialSensor_W688(AP_InertialSensor &imu,
                                                       AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                       enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
    , _temperature_counter(0)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_W688::probe(AP_InertialSensor &imu,
                                  AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                  enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto *sensor = new AP_InertialSensor_W688(imu, std::move(dev), rotation);
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_W688::start()
{
    _dev->get_semaphore()->take_blocking();
    const bool ok = configure();
    _dev->get_semaphore()->give();

    if (!ok) {
        return;
    }

    if (!_imu.register_accel(accel_instance, W688_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(W688_DEVTYPE)) ||
        !_imu.register_gyro(gyro_instance, W688_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(W688_DEVTYPE))) {
        return;
    }

    set_accel_orientation(accel_instance, _rotation);
    set_gyro_orientation(gyro_instance, _rotation);

    periodic_handle = _dev->register_periodic_callback(W688_BACKEND_PERIOD_US,
                                                       FUNCTOR_BIND_MEMBER(&AP_InertialSensor_W688::poll, void));
}

bool AP_InertialSensor_W688::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

bool AP_InertialSensor_W688::init()
{
    _dev->set_read_flag(W688_READ_FLAG);

    if (!hardware_init()) {
        DEV_PRINTF("W688: failed to init\n");
        return false;
    }

    return true;
}

bool AP_InertialSensor_W688::hardware_init()
{
    bool ok = false;

    hal.scheduler->delay(20);
    _dev->get_semaphore()->take_blocking();
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    for (uint8_t attempt = 0; attempt < W688_HARDWARE_INIT_MAX_TRIES; attempt++) {
        _dev->write_register(W688_REG_RESET, W688_SOFT_RESET_CMD);
        hal.scheduler->delay(20);

        for (uint8_t wait_count = 0; wait_count < 50; wait_count++) {
            uint8_t who_am_i = 0;
            uint8_t reset_done = 0;

            if (!read_registers(W688_REG_WHO_AM_I, &who_am_i, 1) ||
                !read_registers(W688_REG_RESET_DONE, &reset_done, 1)) {
                hal.scheduler->delay(10);
                continue;
            }

            if (who_am_i == W688_WHO_AM_I_VALUE && reset_done == W688_RESET_DONE_VALUE) {
                ok = true;
                break;
            }

            hal.scheduler->delay(10);
        }

        if (ok) {
            break;
        }
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->get_semaphore()->give();
    return ok;
}

bool AP_InertialSensor_W688::configure()
{
    if (!write_register_verified(W688_REG_CTRL7, W688_CTRL7_DISABLE_ALL)) {
        return false;
    }
    hal.scheduler->delay_microseconds(2000);

    return write_register_verified(W688_REG_CTRL1, W688_CTRL1_DEFAULT) &&
           write_register_verified(W688_REG_CTRL2, W688_CTRL2_DEFAULT) &&
           write_register_verified(W688_REG_CTRL3, W688_CTRL3_DEFAULT) &&
           write_register_verified(W688_REG_CTRL5, W688_CTRL5_DEFAULT) &&
           write_register_verified(W688_REG_CTRL7, W688_CTRL7_DEFAULT);
}

bool AP_InertialSensor_W688::read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    return _dev->read_registers(reg, data, len);
}

bool AP_InertialSensor_W688::write_register_verified(uint8_t reg, uint8_t value)
{
    for (uint8_t retry = 0; retry < W688_WRITE_VERIFY_RETRIES; retry++) {
        _dev->write_register(reg, value);

        uint8_t readback = 0;
        if (read_registers(reg, &readback, 1) && readback == value) {
            return true;
        }
    }

    return false;
}

void AP_InertialSensor_W688::read_temperature()
{
    uint8_t buf[2] {};
    if (!read_registers(W688_REG_TEMP_L, buf, sizeof(buf))) {
        return;
    }

    const int16_t raw_temperature = combine_signed(buf[1], buf[0]);
    _publish_temperature(accel_instance, raw_temperature / 256.0f);
}

void AP_InertialSensor_W688::poll()
{
    uint8_t statusint = 0;
    if (!read_registers(W688_REG_STATUSINT, &statusint, 1)) {
        return;
    }

    if (++_temperature_counter >= W688_BACKEND_SAMPLE_RATE) {
        _temperature_counter = 0;
        read_temperature();
    }

    if ((statusint & W688_STATUSINT_AVAIL) == 0) {
        return;
    }

    if ((statusint & W688_STATUSINT_LOCKED) == 0) {
        hal.scheduler->delay_microseconds(2);
    }

    uint8_t burst[W688_IMU_BURST_LENGTH] {};
    if (!read_registers(W688_REG_AX_L, burst, sizeof(burst))) {
        return;
    }

    Vector3f accel{
        float(combine_signed(burst[1], burst[0])),
        float(combine_signed(burst[3], burst[2])),
        float(combine_signed(burst[5], burst[4]))
    };

    Vector3f gyro{
        float(combine_signed(burst[7], burst[6])),
        float(combine_signed(burst[9], burst[8])),
        float(combine_signed(burst[11], burst[10]))
    };

    accel *= W688_ACCEL_SCALE;
    gyro *= W688_GYRO_SCALE;

    _rotate_and_correct_accel(accel_instance, accel);
    _rotate_and_correct_gyro(gyro_instance, gyro);

    const uint64_t sample_us = AP_HAL::micros64();
    _notify_new_accel_raw_sample(accel_instance, accel, sample_us);
    _notify_new_gyro_raw_sample(gyro_instance, gyro, sample_us);
}
