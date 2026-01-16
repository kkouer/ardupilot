#include "GCS_Mavlink.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>

// Sensor Specs:
// Baud: 9600
// Rate: 10Hz
// Protocol: Binary (9 bytes)
extern const AP_HAL::HAL &hal;

class AP_Scale_Driver {
public:
  AP_Scale_Driver() {}

  void init() {
    // Find the serial port allocated for Scripting (28)
    _uart = AP::serialmanager().find_serial(
        AP_SerialManager::SerialProtocol_Scripting, 0);
    if (_uart != nullptr) {
      // Set specific baudrate as requested by user
      _uart->begin(9600);

      // Send initialization command
    }
  }

  // Call this at high frequency (e.g. 100Hz or 400Hz) to drain the buffer
  void update() {
    if (_uart == nullptr) {
      return;
    }

    // Polling Mechanism: Request data every 500ms (2Hz)
    uint32_t now = AP_HAL::millis();
    if (now - _last_poll_ms >= 200) {
      send_poll_request();
      _last_poll_ms = now;
    }

    // Read all available bytes to ensure we process the latest data immediately
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
      uint8_t c = _uart->read();
      _decode(c);
    }
  }

  // Request measurement command: A1 00 A0 A2 A3
  void send_poll_request() {
    if (_uart == nullptr) {
      return;
    }
    const uint8_t cmd[] = {0xA3, 0x00, 0xA2, 0xA4, 0xA5};
    _uart->write(cmd, sizeof(cmd));
  }

  // Command to Zero/Tare the scale
  // Protocol: AB 00 AA AC AD
  void tare() {
    if (_uart == nullptr) {
      return;
    }
    const uint8_t cmd[] = {0xAB, 0x00, 0xAA, 0xAC, 0xAD};
    _uart->write(cmd, sizeof(cmd));

    // Optional: Send text to confirm action
    gcs().send_text(MAV_SEVERITY_INFO, "Scale: Tare Command Sent");
  }

  // Call this at lower frequency (e.g. 10Hz) for heartbeat/debugging and
  // failsafe updating
  void send_mavlink() {
    // Unconditionally send the latest weight so it appears in Inspector
    gcs().send_named_float("WEIGHT", _latest_weight);
  }

private:
  AP_HAL::UARTDriver *_uart = nullptr;
  float _latest_weight = 0.0f;
  bool _healthy = false;
  uint32_t _last_poll_ms = 0;
  uint32_t _valid_frame_count = 0;

  // Binary parsing state
  enum class State {
    WAIT_START,
    READ_DATA,
    WAIT_END
  } _state = State::WAIT_START;

  static const uint8_t FRAME_LEN = 10;
  uint8_t _buffer[FRAME_LEN];
  uint8_t _buffer_idx = 0;

  void _decode(uint8_t c) {
    switch (_state) {
    case State::WAIT_START:
      if (c == 0xAA) {
        _buffer_idx = 0;
        _buffer[_buffer_idx++] = c;
        _state = State::READ_DATA;
      }
      break;

    case State::READ_DATA:
      _buffer[_buffer_idx++] = c;
      if (_buffer_idx >= FRAME_LEN) {
        _parse_frame();
        _state = State::WAIT_START;
      }
      break;

    case State::WAIT_END:
      break;
    }
  }

  void _parse_frame() {
    // Header check
    if (_buffer[0] != 0xAA) {
      return;
    }
    // Tail check
    if (_buffer[9] != 0xFF) {
      gcs().send_text(MAV_SEVERITY_WARNING, "Scale: Bad Tail %x", _buffer[9]);
      return;
    }

    // Checksum Validation: Sum of bytes 2..7 (Index 1..6)
    // Equals (Byte8 << 8) + Byte9 (Index 7, 8)
    uint16_t sum = 0;
    for (uint8_t i = 1; i <= 6; i++) {
      sum += _buffer[i];
    }

    uint16_t checksum = ((uint16_t)_buffer[7] << 8) | _buffer[8];

    if (sum != checksum) {
      // Debug mismatch
      // gcs().send_text(MAV_SEVERITY_WARNING, "Scale: Bad Chk Recv:%04x
      // Calc:%04x", (unsigned)checksum, (unsigned)sum);
      return;
    }

    // AD Value: Bytes 4, 5, 6 (Indices 4, 5, 6 in 0-based array)
    uint32_t raw_ad = ((uint32_t)_buffer[4] << 16) |
                      ((uint32_t)_buffer[5] << 8) | (uint32_t)_buffer[6];

    _latest_weight = (float)raw_ad;
    _healthy = true;
    _valid_frame_count++;

    // Send immediately for low latency
    gcs().send_named_float("WEIGHT", _latest_weight);
  }
};

// Global instance to be used in UserCode.cpp
// Global instance to be used in UserCode.cpp and GCS
extern AP_Scale_Driver scale_driver;
