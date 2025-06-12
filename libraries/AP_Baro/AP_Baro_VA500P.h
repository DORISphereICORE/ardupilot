/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

 * Driver for Valeport VA500P pressure sensor
 * This sensor operates on serial interface and provides pressure readings in dbar
 * The measurement range is configurable through the sensor's setup mode
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_VA500P_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

class AP_Baro_VA500P : public AP_Baro_Backend
{
public:
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::UARTDriver* dev);

private:
    AP_Baro_VA500P(AP_Baro &baro, AP_HAL::UARTDriver* dev);

    bool _init();
    void _timer();
    bool _read();
    bool _send_command(const char *cmd);
    bool _enter_config_mode();
    bool _enter_stream_mode();
    bool _set_pressure_units();
    bool _set_range_units();
    bool _process_buffer();
    void _add_char(char data);

    AP_HAL::UARTDriver* _dev;

    /* Shared values between thread sampling the HW and main thread */
    struct {
        uint32_t sum_pressure;
        uint32_t sum_temperature;
        uint8_t num_samples;
    } _accum;

    uint8_t _instance;

    // Buffer management
    static const uint8_t INPUT_BUFLEN = 100;
    char _buffer_a[INPUT_BUFLEN];
    char _buffer_b[INPUT_BUFLEN];
    char *_active_buffer;
    char *_last_buffer;
    uint8_t _active_buffer_index;
    bool _new_message_received;

    // Sensor state
    enum class SensorMode {
        UNINIT,
        CONFIG,
        STREAMING
    } _mode;

    float _depth;        // Pressure in dbar
    float _altitude;     // Altitude in meters
    float _sound_speed;  // Sound speed in m/s
    float _tare;         // Tare value
};

#endif  // AP_BARO_VA500P_ENABLED 