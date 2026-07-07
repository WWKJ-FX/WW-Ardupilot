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

#include "AP_InertialSensor_W689.h"

enum W689Register : uint8_t {
    W689_REG_WHO_AM_I = 0x01,
    W689_REG_COM_CFG = 0x04,
    W689_REG_INT1_OUT_SEL1 = 0x05,
    W689_REG_INT1_OUT_SEL2 = 0x06,
    W689_REG_INT2_OUT_SEL1 = 0x07,
    W689_REG_INT2_OUT_SEL2 = 0x08,
    W689_REG_STATUS = 0x0B,
    W689_REG_ACC_XH = 0x0C,
    W689_REG_FIFO_CONFIG = 0x1C,
    W689_REG_FIFO_MODE = 0x1D,
    W689_REG_TEMP_H = 0x22,
    W689_REG_TEMP_L = 0x23,
    W689_REG_AOI1_CTRL = 0x30,
    W689_REG_AOI1_VTH = 0x32,
    W689_REG_AOI1_TTH = 0x33,
    W689_REG_ACC_CONF = 0x40,
    W689_REG_ACC_RANGE = 0x41,
    W689_REG_GYR_CONF = 0x42,
    W689_REG_GYR_RANGE = 0x43,
    W689_REG_RESET = 0x4A,
    W689_REG_SPI_I2C_CFG = 0x6F,
    W689_REG_PWR_CTRL = 0x7D,
    W689_REG_BANK_SEL = 0x7F,
};

static constexpr uint8_t W689_READ_FLAG = 0x80;
static constexpr uint8_t W689_WHO_AM_I_VALUE = 0x6A;
static constexpr uint8_t W689_STATUS_ACC_DRDY = 1U << 0;
static constexpr uint8_t W689_STATUS_GYR_DRDY = 1U << 1;
static constexpr uint8_t W689_STATUS_TMP_DRDY = 1U << 2;
static constexpr uint8_t W689_STATUS_ACC_CONF_ERR = 1U << 4;
static constexpr uint8_t W689_STATUS_GYR_CONF_ERR = 1U << 5;
static constexpr uint8_t W689_SOFT_RESET_CMD = 0xA5;
static constexpr uint8_t W689_SPI_PREPARE_CMD = 0x66;
static constexpr uint8_t W689_SPI_PREPARE_CLEAR = 0x00;
static constexpr uint8_t W689_WRITE_VERIFY_RETRIES = 5;
static constexpr uint8_t W689_HARDWARE_INIT_MAX_TRIES = 3;
static constexpr uint16_t W689_BACKEND_SAMPLE_RATE = 1600;
static constexpr uint32_t W689_BACKEND_PERIOD_US = 1000000UL / W689_BACKEND_SAMPLE_RATE;
static constexpr uint8_t W689_IMU_BURST_LENGTH = 12;
static constexpr uint8_t W689_DEVTYPE = AP_InertialSensor_Backend::DEVTYPE_INS_W689;

static constexpr uint8_t W689_COM_CONF_BDU = 1U << 6;
static constexpr uint8_t W689_COM_CONF_ADDR_AUTO = 1U << 4;
static constexpr uint8_t W689_COM_CONF_BOOT = 1U << 7;
static constexpr uint8_t W689_DEFAULT_COM_CONF = W689_COM_CONF_BDU | W689_COM_CONF_ADDR_AUTO;
static constexpr uint8_t W689_PWR_CTRL_ACCGYR_ENABLE = (1U << 3) | (1U << 2) | (1U << 1);
static constexpr uint8_t W689_SEG_MAIN = 0x00;
static constexpr uint8_t W689_SEG_I2C_SPI = 0x83;
static constexpr uint8_t W689_SEG_DIG_CTRL = 0x8C;
static constexpr uint8_t W689_I2C_DISABLE = 0xB4;
static constexpr uint8_t W689_FIFO_MODE_BYPASS = 0x00;
static constexpr uint8_t W689_ACC_CONF_DEFAULT = (1U << 7) | (2U << 4) | 0x0C; // high performance, avg4, 1600Hz
static constexpr uint8_t W689_GYR_CONF_DEFAULT = (1U << 7) | 0x0C; // high performance, avg1, 1600Hz
static constexpr uint8_t W689_ACC_RANGE_16G = 0x03;
static constexpr uint8_t W689_GYR_RANGE_2000DPS = 0x00;

static constexpr float W689_ACCEL_SCALE = GRAVITY_MSS / 2048.0f;
static constexpr float W689_GYRO_SCALE = radians(2000.0f / 32768.0f);
static constexpr float W689_TEMPERATURE_SCALE = 1.0f / 512.0f;
static constexpr float W689_TEMPERATURE_OFFSET = 23.0f;

extern const AP_HAL::HAL& hal;

static int16_t combine_signed_be(uint8_t msb, uint8_t lsb)
{
    return int16_t((uint16_t(msb) << 8) | uint16_t(lsb));
}

AP_InertialSensor_W689::AP_InertialSensor_W689(AP_InertialSensor &imu,
                                               AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                               enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
    , _temperature_counter(0)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_W689::probe(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto *sensor = new AP_InertialSensor_W689(imu, std::move(dev), rotation);
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_W689::start()
{
    _dev->get_semaphore()->take_blocking();
    const bool ok = configure();
    _dev->get_semaphore()->give();

    if (!ok) {
        return;
    }

    if (!_imu.register_accel(accel_instance, W689_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(W689_DEVTYPE)) ||
        !_imu.register_gyro(gyro_instance, W689_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(W689_DEVTYPE))) {
        return;
    }

    set_accel_orientation(accel_instance, _rotation);
    set_gyro_orientation(gyro_instance, _rotation);

    periodic_handle = _dev->register_periodic_callback(W689_BACKEND_PERIOD_US,
                                                       FUNCTOR_BIND_MEMBER(&AP_InertialSensor_W689::poll, void));
}

bool AP_InertialSensor_W689::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

bool AP_InertialSensor_W689::init()
{
    _dev->set_read_flag(W689_READ_FLAG);

    if (!hardware_init()) {
        DEV_PRINTF("W689: failed to init\n");
        return false;
    }

    return true;
}

bool AP_InertialSensor_W689::hardware_init()
{
    bool ok = false;

    hal.scheduler->delay(20);
    _dev->get_semaphore()->take_blocking();
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    for (uint8_t attempt = 0; attempt < W689_HARDWARE_INIT_MAX_TRIES; attempt++) {
        prepare_spi_mode();

        uint8_t who_am_i = 0;
        if (!read_registers(W689_REG_WHO_AM_I, &who_am_i, 1) ||
            who_am_i != W689_WHO_AM_I_VALUE) {
            hal.scheduler->delay(10);
            continue;
        }

        if (!write_register_verified(W689_REG_PWR_CTRL, 0x00)) {
            continue;
        }
        hal.scheduler->delay(2);

        select_segment(W689_SEG_MAIN);
        write_register_blind(W689_REG_COM_CFG, W689_COM_CONF_BOOT);
        write_register_blind(W689_REG_RESET, W689_SOFT_RESET_CMD);
        write_register_blind(W689_REG_RESET, W689_SOFT_RESET_CMD);
        hal.scheduler->delay(200);

        prepare_spi_mode();
        who_am_i = 0;
        if (read_registers(W689_REG_WHO_AM_I, &who_am_i, 1) &&
            who_am_i == W689_WHO_AM_I_VALUE) {
            ok = true;
            break;
        }
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->get_semaphore()->give();
    return ok;
}

bool AP_InertialSensor_W689::configure()
{
    select_segment(W689_SEG_MAIN);

    write_register_blind(W689_REG_PWR_CTRL, W689_PWR_CTRL_ACCGYR_ENABLE);
    hal.scheduler->delay(5);
    write_register_blind(W689_REG_PWR_CTRL, W689_PWR_CTRL_ACCGYR_ENABLE);
    hal.scheduler->delay(250);

    write_register_blind(W689_REG_INT1_OUT_SEL1, 0x00);
    write_register_blind(W689_REG_INT1_OUT_SEL2, 0x00);
    write_register_blind(W689_REG_INT2_OUT_SEL1, 0x00);
    write_register_blind(W689_REG_INT2_OUT_SEL2, 0x00);
    write_register_blind(W689_REG_AOI1_CTRL, 0x00);
    write_register_blind(W689_REG_AOI1_VTH, 0xFF);
    write_register_blind(W689_REG_AOI1_TTH, 0xFF);
    write_register_blind(W689_REG_FIFO_CONFIG, 0x00);
    write_register_blind(W689_REG_FIFO_MODE, W689_FIFO_MODE_BYPASS);

    write_register_blind(W689_REG_ACC_CONF, W689_ACC_CONF_DEFAULT);
    hal.scheduler->delay(2);
    write_register_blind(W689_REG_ACC_RANGE, W689_ACC_RANGE_16G);

    write_register_blind(W689_REG_GYR_CONF, W689_GYR_CONF_DEFAULT);
    write_register_blind(W689_REG_GYR_CONF, W689_GYR_CONF_DEFAULT);
    hal.scheduler->delay(2);
    write_register_blind(W689_REG_GYR_RANGE, W689_GYR_RANGE_2000DPS);
    write_register_blind(W689_REG_GYR_RANGE, W689_GYR_RANGE_2000DPS);

    write_register_blind(W689_REG_COM_CFG, W689_DEFAULT_COM_CONF);
    hal.scheduler->delay(2);

    select_segment(W689_SEG_DIG_CTRL);
    uint8_t dig_ctrl = 0;
    if (read_registers(W689_REG_AOI1_CTRL, &dig_ctrl, 1)) {
        dig_ctrl &= ~0x03;
        write_register_blind(W689_REG_AOI1_CTRL, dig_ctrl);
    }
    select_segment(W689_SEG_MAIN);

    return true;
}

bool AP_InertialSensor_W689::read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    return _dev->read_registers(reg, data, len);
}

void AP_InertialSensor_W689::write_register_blind(uint8_t reg, uint8_t value)
{
    _dev->write_register(reg, value);
    hal.scheduler->delay_microseconds(50);
}

bool AP_InertialSensor_W689::write_register_verified(uint8_t reg, uint8_t value)
{
    for (uint8_t retry = 0; retry < W689_WRITE_VERIFY_RETRIES; retry++) {
        write_register_blind(reg, value);

        uint8_t readback = 0;
        if (read_registers(reg, &readback, 1) && readback == value) {
            return true;
        }
    }

    return false;
}

void AP_InertialSensor_W689::select_segment(uint8_t segment)
{
    select_segment_blind(segment);
    hal.scheduler->delay(2);
}

void AP_InertialSensor_W689::select_segment_blind(uint8_t segment)
{
    write_register_blind(W689_REG_BANK_SEL, segment);
}

void AP_InertialSensor_W689::prepare_spi_mode()
{
    select_segment(W689_SEG_MAIN);
    write_register_blind(W689_REG_RESET, W689_SPI_PREPARE_CMD);
    hal.scheduler->delay(2);

    select_segment(W689_SEG_I2C_SPI);
    write_register_blind(W689_REG_SPI_I2C_CFG, W689_I2C_DISABLE);
    write_register_blind(W689_REG_SPI_I2C_CFG, W689_I2C_DISABLE);
    hal.scheduler->delay(2);

    select_segment(W689_SEG_MAIN);
    write_register_blind(W689_REG_RESET, W689_SPI_PREPARE_CLEAR);
    hal.scheduler->delay(2);
}

void AP_InertialSensor_W689::publish_raw_sample(const int16_t raw_accel[3], const int16_t raw_gyro[3], uint64_t sample_us)
{
    Vector3f accel{
        float(raw_accel[0]),
        float(raw_accel[1]),
        float(raw_accel[2])
    };
    accel *= W689_ACCEL_SCALE;

    Vector3f gyro{
        float(raw_gyro[0]),
        float(raw_gyro[1]),
        float(raw_gyro[2])
    };
    gyro *= W689_GYRO_SCALE;

    _rotate_and_correct_accel(accel_instance, accel);
    _rotate_and_correct_gyro(gyro_instance, gyro);

    _notify_new_accel_raw_sample(accel_instance, accel, sample_us);
    _notify_new_gyro_raw_sample(gyro_instance, gyro, sample_us);
}

void AP_InertialSensor_W689::read_temperature()
{
    uint8_t buf[2] {};
    if (!read_registers(W689_REG_TEMP_H, buf, sizeof(buf))) {
        return;
    }

    const int16_t raw_temperature = combine_signed_be(buf[0], buf[1]);
    _publish_temperature(accel_instance, raw_temperature * W689_TEMPERATURE_SCALE + W689_TEMPERATURE_OFFSET);
}

void AP_InertialSensor_W689::poll()
{
    uint8_t status = 0;
    if (!read_registers(W689_REG_STATUS, &status, 1)) {
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return;
    }

    if ((status & (W689_STATUS_ACC_CONF_ERR | W689_STATUS_GYR_CONF_ERR)) != 0) {
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return;
    }

    if (++_temperature_counter >= W689_BACKEND_SAMPLE_RATE &&
        (status & W689_STATUS_TMP_DRDY) != 0) {
        _temperature_counter = 0;
        read_temperature();
    }

    if ((status & (W689_STATUS_ACC_DRDY | W689_STATUS_GYR_DRDY)) !=
        (W689_STATUS_ACC_DRDY | W689_STATUS_GYR_DRDY)) {
        return;
    }

    uint8_t burst[W689_IMU_BURST_LENGTH] {};
    if (!read_registers(W689_REG_ACC_XH, burst, sizeof(burst))) {
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return;
    }

    int16_t raw_accel[3] {};
    int16_t raw_gyro[3] {};
    raw_accel[0] = combine_signed_be(burst[0], burst[1]);
    raw_accel[1] = combine_signed_be(burst[2], burst[3]);
    raw_accel[2] = combine_signed_be(burst[4], burst[5]);
    raw_gyro[0] = combine_signed_be(burst[6], burst[7]);
    raw_gyro[1] = combine_signed_be(burst[8], burst[9]);
    raw_gyro[2] = combine_signed_be(burst[10], burst[11]);

    publish_raw_sample(raw_accel, raw_gyro, AP_HAL::micros64());
}
