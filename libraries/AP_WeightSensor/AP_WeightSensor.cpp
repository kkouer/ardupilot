/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

#include "AP_WeightSensor.h"

#if AP_WEIGHT_SENSOR_ENABLED

#include <string.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define WGT_MODBUS_FUNC_READ_HOLDING  0x03
#define WGT_MODBUS_REG_WEIGHT         0x0000
#define WGT_MODBUS_REG_COUNT          2
#define WGT_RESPONSE_LEN              9
#define WGT_REQUEST_TIMEOUT_MS        200
#define WGT_HEALTHY_TIMEOUT_MS        1000

AP_WeightSensor *AP_WeightSensor::_singleton;

const AP_Param::GroupInfo AP_WeightSensor::var_info[] = {
    // @Param: ADDR
    // @DisplayName: Weight sensor Modbus address
    // @Description: Modbus-RTU slave address of the weighing indicator (RDD-DX default is 1)
    // @Range: 1 247
    // @User: Standard
    AP_GROUPINFO("ADDR", 1, AP_WeightSensor, _addr, 1),

    // @Param: SCALE
    // @DisplayName: Weight sensor scale
    // @Description: Multiplier applied to the 32-bit register count to convert to kilograms. The indicator does not include a decimal point; if a 100.00 kg calibration weight was written as 10000, set this to 0.01.
    // @Increment: 0.0001
    // @User: Standard
    AP_GROUPINFO("SCALE", 2, AP_WeightSensor, _scale, 1.0),

    // @Param: OFFSET
    // @DisplayName: Weight sensor offset
    // @Description: Offset in kilograms added after scaling (weight = raw * SCALE + OFFSET)
    // @Units: kg
    // @User: Standard
    AP_GROUPINFO("OFFSET", 3, AP_WeightSensor, _offset, 0.0),

    // @Param: RATE
    // @DisplayName: Weight sensor poll rate
    // @Description: How often the flight controller reads the indicator over Modbus. Keep this at or below the indicator sample rate. Above 40 Hz the indicator manual recommends 115200 baud; the TD301D485H-A adapter supports at most 115200.
    // @Range: 1 40
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("RATE", 4, AP_WeightSensor, _rate_hz, 10),

    AP_GROUPEND
};

AP_WeightSensor::AP_WeightSensor()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("WeightSensor must be singleton");
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

AP_WeightSensor *AP_WeightSensor::get_singleton()
{
    return _singleton;
}

void AP_WeightSensor::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_WeightSensor, 0);
    if (_uart == nullptr) {
        return;
    }

    const uint32_t baud = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_WeightSensor, 0);
    _uart->begin(baud, 32, 32);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

    _state = State::Idle;
    _rx_len = 0;
    _last_request_ms = 0;
    _request_ms = 0;
    _last_healthy_ms = 0;
    _last_report_ms = 0;
    _raw = 0;
    _weight_kg = 0;
}

bool AP_WeightSensor::healthy() const
{
    if (_uart == nullptr) {
        return false;
    }
    return (AP_HAL::millis() - _last_healthy_ms) < WGT_HEALTHY_TIMEOUT_MS;
}

void AP_WeightSensor::send_request()
{
    uint8_t req[8];
    const uint8_t addr = (uint8_t)constrain_int16(_addr.get(), 1, 247);

    req[0] = addr;
    req[1] = WGT_MODBUS_FUNC_READ_HOLDING;
    req[2] = (uint8_t)(WGT_MODBUS_REG_WEIGHT >> 8);
    req[3] = (uint8_t)(WGT_MODBUS_REG_WEIGHT & 0xFF);
    req[4] = (uint8_t)(WGT_MODBUS_REG_COUNT >> 8);
    req[5] = (uint8_t)(WGT_MODBUS_REG_COUNT & 0xFF);
    const uint16_t crc = calc_crc_modbus(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    // Drop any leftover bytes before a new poll
    while (_uart->available() > 0) {
        _uart->read();
    }
    _rx_len = 0;

    _uart->write(req, sizeof(req));
}

void AP_WeightSensor::consume_bytes()
{
    while (_uart->available() > 0 && _rx_len < sizeof(_rx_buf)) {
        const int16_t b = _uart->read();
        if (b < 0) {
            break;
        }
        _rx_buf[_rx_len++] = (uint8_t)b;
    }
}

bool AP_WeightSensor::parse_frame()
{
    if (_rx_len < WGT_RESPONSE_LEN) {
        return false;
    }

    const uint8_t addr = (uint8_t)constrain_int16(_addr.get(), 1, 247);
    if (_rx_buf[0] != addr ||
        _rx_buf[1] != WGT_MODBUS_FUNC_READ_HOLDING ||
        _rx_buf[2] != 4) {
        memmove(_rx_buf, _rx_buf + 1, _rx_len - 1);
        _rx_len--;
        return false;
    }

    const uint16_t crc_got = (uint16_t)_rx_buf[7] | ((uint16_t)_rx_buf[8] << 8);
    const uint16_t crc_exp = calc_crc_modbus(_rx_buf, 7);
    if (crc_got != crc_exp) {
        memmove(_rx_buf, _rx_buf + 1, _rx_len - 1);
        _rx_len--;
        return false;
    }

    // CDAB: low word then high word, big-endian within each word
    const uint16_t low = ((uint16_t)_rx_buf[3] << 8) | _rx_buf[4];
    const uint16_t high = ((uint16_t)_rx_buf[5] << 8) | _rx_buf[6];
    _raw = (int32_t)(((uint32_t)high << 16) | low);
    _weight_kg = (_raw * _scale.get()) + _offset.get();
    _last_healthy_ms = AP_HAL::millis();
    _rx_len = 0;
    _state = State::Idle;
    return true;
}

void AP_WeightSensor::update()
{
    if (_uart == nullptr) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const uint8_t rate = (uint8_t)constrain_int16(_rate_hz.get(), 1, 40);
    const uint32_t period_ms = 1000 / rate;

    if (_state == State::WaitResponse) {
        consume_bytes();
        parse_frame();
        if (_state == State::WaitResponse &&
            (now_ms - _request_ms) >= WGT_REQUEST_TIMEOUT_MS) {
            _state = State::Idle;
            _rx_len = 0;
        }
    }

    if (_state == State::Idle && (now_ms - _last_request_ms) >= period_ms) {
        send_request();
        _last_request_ms = now_ms;
        _request_ms = now_ms;
        _state = State::WaitResponse;
    }

    if ((now_ms - _last_report_ms) >= period_ms) {
        _last_report_ms = now_ms;
        const bool ok = healthy();

        // @LoggerMessage: WGT
        // @Description: Weight sensor (RDD-DX Modbus) reading
        // @Field: TimeUS: Time since system startup
        // @Field: Weight: Scaled weight
        // @Field: Raw: Raw 32-bit register count
        // @Field: Health: 1 if a valid frame was received recently
        AP::logger().WriteStreaming(
            "WGT",
            "TimeUS,Weight,Raw,Health",
            "s---",
            "F---",
            "Qfib",
            AP_HAL::micros64(),
            _weight_kg,
            _raw,
            (uint8_t)ok);

        if (ok) {
            gcs().send_named_float("WGT", _weight_kg);
        }
    }
}

AP_WeightSensor *AP::weight_sensor()
{
    return AP_WeightSensor::get_singleton();
}

#endif  // AP_WEIGHT_SENSOR_ENABLED
