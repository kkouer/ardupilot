/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
#pragma once

#include "AP_WeightSensor_config.h"

#if AP_WEIGHT_SENSOR_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>

/*
  Ruilide RDD-DX over a UART after an RS485-to-TTL adapter (TD301D485H-A).

  Set a free flight-controller UART to SERIALn_PROTOCOL = 51 (WeightSensor)
  and SERIALn_BAUD = 9 (9600).  The adapter already handles RS485 direction;
  the flight controller uses 3.3V TTL.

  WGT_PROTO = 0 (default): the meter continuously sends an ASCII frame
    =+00000.0@<CR> (or =-....) about every 73 ms.  The flight controller
    only listens; it does not poll.

  WGT_PROTO = 1: Modbus-RTU poll of holding registers 0x0000-0x0001,
    function 0x03, 32-bit signed, CDAB word order.

  Telemetry is sent as MAVLink WEIGHT_SENSOR (id 32768) on stream EXTRA1,
  and also as NAMED_VALUE_FLOAT with name WGT.
 */

class AP_WeightSensor {
public:
    AP_WeightSensor();

    CLASS_NO_COPY(AP_WeightSensor);

    static AP_WeightSensor *get_singleton();

    void init();
    void update();

    bool enabled() const { return _uart != nullptr; }
    bool healthy() const;
    float weight_kg() const { return _weight_kg; }
    int32_t raw() const { return _raw; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class Protocol : uint8_t {
        ASCII = 0,
        Modbus = 1,
    };

    void send_request();
    void consume_bytes();
    void parse_ascii();
    bool parse_ascii_frame(const uint8_t *buf, uint8_t len);
    bool parse_modbus_frame();
    Protocol protocol() const;

    static AP_WeightSensor *_singleton;

    AP_HAL::UARTDriver *_uart;

    AP_Int16 _addr;
    AP_Float _scale;
    AP_Float _offset;
    AP_Int8 _rate_hz;
    AP_Int8 _protocol;

    enum class State : uint8_t {
        Idle = 0,
        WaitResponse = 1,
    } _state;

    uint8_t _rx_buf[64];
    uint8_t _rx_len;
    uint32_t _last_request_ms;
    uint32_t _request_ms;
    uint32_t _last_healthy_ms;
    uint32_t _last_report_ms;

    int32_t _raw;
    float _weight_kg;
};

namespace AP {
    AP_WeightSensor *weight_sensor();
};

#endif  // AP_WEIGHT_SENSOR_ENABLED
