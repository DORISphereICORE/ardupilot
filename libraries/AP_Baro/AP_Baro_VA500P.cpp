#include <AP_HAL/AP_HAL.h>
#include "AP_Baro_VA500P.h"
#include <stdio.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

// Commands for VA500P
#define ENTER_SETUP_MODE_CMD "$V"
#define ENTER_RUN_MODE_CMD "$R"
#define SET_PRESSURE_UNITS_CMD "$U"
#define SET_RANGE_UNITS_CMD "$D"
#define SET_SOUND_SPEED_CMD "$S"

// Default temperature to use since VA500P doesn't provide temperature
#define VA500P_DEFAULT_TEMPERATURE 20.0f

// Conversion from dbar to Pascal (1 dbar = 10000 Pa)
#define DBAR_TO_PASCAL 10000.0f

#define NEW_NOTHROW new (std::nothrow)
#define __AP_LINE__ __LINE__

AP_Baro_VA500P::AP_Baro_VA500P(AP_Baro &baro, AP_HAL::UARTDriver* dev)
    : AP_Baro_Backend(baro)
    , _dev(dev)
    , _active_buffer(_buffer_a)
    , _last_buffer(_buffer_b)
    , _active_buffer_index(0)
    , _new_message_received(false)
    , _mode(SensorMode::UNINIT)
    , _depth(0)
    , _altitude(0)
    , _sound_speed(1500.0f)
    , _tare(0)
{
    memset(_buffer_a, 0, INPUT_BUFLEN);
    memset(_buffer_b, 0, INPUT_BUFLEN);
}

AP_Baro_Backend *AP_Baro_VA500P::probe(AP_Baro &baro, AP_HAL::UARTDriver* dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_Baro_VA500P *sensor = NEW_NOTHROW AP_Baro_VA500P(baro, dev);
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_Baro_VA500P::_init()
{
    if (!_dev) {
        return false;
    }

    // _dev->set_speed(AP_HAL::Device::SPEED_HIGH); // Remove or replace if not available for UARTDriver

    // Enter configuration mode
    if (!_enter_config_mode()) {
        return false;
    }

    // Set pressure units to dbar
    if (!_set_pressure_units()) {
        return false;
    }

    // Set range units to meters
    if (!_set_range_units()) {
        return false;
    }

    // Enter streaming mode
    if (!_enter_stream_mode()) {
        return false;
    }

    // Register timer callback (implement if needed for UARTDriver)
    // _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_Baro_VA500P::_timer, void));

    return true;
}

void AP_Baro_VA500P::update()
{
    if (_new_message_received) {
        WITH_SEMAPHORE(_sem);
        if (_process_buffer()) {
            // Convert dbar to Pascal
            float pressure = _depth * DBAR_TO_PASCAL;
            _copy_to_frontend(_instance, pressure, VA500P_DEFAULT_TEMPERATURE);
        }
        _new_message_received = false;
    }
}

void AP_Baro_VA500P::_timer()
{
    uint8_t recv[1];
    if (_dev->read(recv, 1) == 1) {
        _add_char(recv[0]);
    }
}

bool AP_Baro_VA500P::_send_command(const char *cmd)
{
    if (!_dev) {
        return false;
    }
    return _dev->write((const uint8_t*)cmd, strlen(cmd)) == strlen(cmd);
}

bool AP_Baro_VA500P::_enter_config_mode()
{
    if (_mode == SensorMode::CONFIG) {
        return true;
    }

    if (!_send_command(ENTER_SETUP_MODE_CMD)) {
        return false;
    }

    // Wait for prompt
    uint32_t start = AP_HAL::millis();
    while (AP_HAL::millis() - start < 2000) {
        uint8_t recv[1];
        if (_dev->read(recv, 1) == 1) {
            if (recv[0] == '>') {
                _mode = SensorMode::CONFIG;
                return true;
            }
        }
        hal.scheduler->delay_microseconds(100);
    }
    return false;
}

bool AP_Baro_VA500P::_enter_stream_mode()
{
    if (_mode == SensorMode::STREAMING) {
        return true;
    }

    if (!_send_command(ENTER_RUN_MODE_CMD)) {
        return false;
    }

    _mode = SensorMode::STREAMING;
    return true;
}

bool AP_Baro_VA500P::_set_pressure_units()
{
    if (!_enter_config_mode()) {
        return false;
    }

    // Set to dbar units
    char cmd[10];
    snprintf(cmd, sizeof(cmd), "%s;DBAR\n", SET_PRESSURE_UNITS_CMD);
    if (!_send_command(cmd)) {
        return false;
    }

    hal.scheduler->delay(500);
    return true;
}

bool AP_Baro_VA500P::_set_range_units()
{
    if (!_enter_config_mode()) {
        return false;
    }

    // Set to meters
    char cmd[10];
    snprintf(cmd, sizeof(cmd), "%s;M\n", SET_RANGE_UNITS_CMD);
    if (!_send_command(cmd)) {
        return false;
    }

    hal.scheduler->delay(500);
    return true;
}

void AP_Baro_VA500P::_add_char(char data)
{
    if (data == '$' || data == '>') {
        _active_buffer_index = 0;
    }

    if (_active_buffer_index < (INPUT_BUFLEN-1)) {
        _active_buffer[_active_buffer_index] = data;
        _active_buffer_index++;
    }

    if (data == '\n') {
        // Switch buffers
        if (_active_buffer == _buffer_a) {
            _active_buffer = _buffer_b;
            _last_buffer = _buffer_a;
        } else {
            _active_buffer = _buffer_a;
            _last_buffer = _buffer_b;
        }
        _active_buffer_index = 0;
        _new_message_received = true;
    }
}

bool AP_Baro_VA500P::_process_buffer()
{
    // Check for NMEA start character
    char* idx = strchr(_last_buffer, '$');
    if (idx == nullptr) {
        return false;
    }

    // Check for PRVAT message
    if (strstr(_last_buffer, "PRVAT") == nullptr) {
        return false;
    }

    // Parse NMEA message
    char* token = strtok(_last_buffer, ",");
    if (token == nullptr) {
        return false;
    }

    // Skip message type
    token = strtok(nullptr, ",");
    if (token == nullptr) {
        return false;
    }

    // Get altitude
    _altitude = atof(token);

    // Skip altitude unit
    token = strtok(nullptr, ",");
    if (token == nullptr) {
        return false;
    }

    // Get depth (pressure in dbar)
    _depth = atof(token);

    return true;
} 