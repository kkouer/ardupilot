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
  Ruilide RDD-DX (and compatible single-channel 485 weighing modules)
  over Modbus-RTU, after an RS485-to-TTL adapter such as TD301D485H-A.

  Set a free flight-controller UART to SERIALn_PROTOCOL = 51 (WeightSensor)
  and SERIALn_BAUD = 9 (9600), which is the meter default.  The adapter
  already handles RS485 direction; the flight controller just uses 3.3V TTL.

  Real-time weight is holding registers 0x0000-0x0001, function 0x03,
  32-bit signed, CDAB word order.  The meter stores counts without a
  decimal point; set WGT_SCALE (for example 0.01 if a 100.00 kg weight
  was entered as 10000 during calibration).
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
    void send_request();
    void consume_bytes();
    bool parse_frame();

    static AP_WeightSensor *_singleton;

    AP_HAL::UARTDriver *_uart;

    AP_Int16 _addr;
    AP_Float _scale;
    AP_Float _offset;
    AP_Int8 _rate_hz;

    enum class State : uint8_t {
        Idle = 0,
        WaitResponse = 1,
    } _state;

    uint8_t _rx_buf[16];
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
