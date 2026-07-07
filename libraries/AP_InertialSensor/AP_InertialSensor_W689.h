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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_W689 : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation=ROTATION_NONE);

    void start() override;
    bool update() override;

private:
    AP_InertialSensor_W689(AP_InertialSensor &imu,
                           AP_HAL::OwnPtr<AP_HAL::Device> dev,
                           enum Rotation rotation);

    bool init();
    bool hardware_init();
    bool configure();
    void prepare_spi_mode();
    void select_segment(uint8_t segment);
    void select_segment_blind(uint8_t segment);
    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);
    void write_register_blind(uint8_t reg, uint8_t value);
    bool write_register_verified(uint8_t reg, uint8_t value);
    void read_temperature();
    void publish_raw_sample(const int16_t raw_accel[3], const int16_t raw_gyro[3], uint64_t sample_us);
    void poll();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_HAL::Device::PeriodicHandle periodic_handle;
    enum Rotation _rotation;
    uint8_t gyro_instance;
    uint8_t accel_instance;
    uint16_t _temperature_counter;
};
