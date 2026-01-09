#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "GCS_Mavlink.h"
#include <ctype.h>
#include <stdlib.h>
#include <math.h>

// Sensor Specs:
// Baud: 9600
// Rate: 10Hz
// Protocol: Binary (9 bytes)
extern const AP_HAL::HAL& hal;

class AP_Scale_Driver {
public:
    AP_Scale_Driver() {}

    void init() {
        gcs().send_text(MAV_SEVERITY_INFO, "Scale: Init");
        // Find the serial port allocated for Scripting (28)
        _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Scripting, 0);
        if (_uart != nullptr) {
            // Set specific baudrate as requested by user
            _uart->begin(9600); 
            
            // Send initialization command
            send_trigger();
        }
    }

    // Call this at high frequency (e.g. 100Hz or 400Hz) to drain the buffer
    void update() {
        if (_uart == nullptr) {
            return;
        }

        // Read all available bytes to ensure we process the latest data immediately
        int16_t nbytes = _uart->available();
        while (nbytes-- > 0) {
            uint8_t c = _uart->read();
            _decode(c);
        }
    }

    void send_trigger() {
        if (_uart == nullptr) {
            return;
        }
        // Command: A4 00 A3 A5 A2
        const uint8_t cmd[] = { 0xA4, 0x00, 0xA3, 0xA5, 0xA2 };
        _uart->write(cmd, sizeof(cmd));
        _last_trigger_ms = AP_HAL::millis();
    }

    // Call this at lower frequency (e.g. 10Hz) for heartbeat/debugging and failsafe updating
    void send_mavlink() {
        // Unconditionally send the latest weight so it appears in Inspector
        gcs().send_named_float("WEIGHT", _latest_weight);
        
        // Debug: Show we are alive and counting frames
        static uint32_t last_stat_ms = 0;
        if (AP_HAL::millis() - last_stat_ms > 2000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Scale: ValidFrames %u, Val %.2f", (unsigned)_valid_frame_count, _latest_weight);
            last_stat_ms = AP_HAL::millis();
        }
    }

private:
    AP_HAL::UARTDriver *_uart = nullptr;
    float _latest_weight = 0.0f;
    bool _healthy = false;
    uint32_t _last_trigger_ms = 0;
    uint32_t _valid_frame_count = 0;
    
    // Binary parsing state
    enum class State {
        WAIT_START,
        READ_DATA,
        WAIT_END
    } _state = State::WAIT_START;

    static const uint8_t FRAME_LEN = 9; // Based on example: AA A4 00 00 AD AD AD CK FF
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
                if (_buffer_idx >= 8) { // Read up to index 7 (Wait for last byte potentially)
                     // If we strictly follow the count, we need 9 bytes total.
                     // The example: AA A4 00 00 00 00 00 A4 FF -> 9 bytes
                     // Index:      0  1  2  3  4  5  6  7  8
                     if (_buffer_idx >= FRAME_LEN) {
                         _parse_frame();
                         _state = State::WAIT_START;
                     }
                }
                break;
                
            case State::WAIT_END:
                // Not strictly used if we just count bytes, but good for resync if Frame ends with fixed FF
                // In this implementation I'll just count bytes to 9 for simplicity based on the example.
                break;
        }
    }

    void _parse_frame() {
        // Debug: Print the full 9-byte frame immediately to identify what we captured
        static uint32_t last_hex_ms = 0;
        uint32_t now = AP_HAL::millis();
        bool print_debug = (now - last_hex_ms > 1000);
        
        if (print_debug) {
            gcs().send_text(MAV_SEVERITY_INFO, "Fr: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                _buffer[0], _buffer[1], _buffer[2], _buffer[3], 
                _buffer[4], _buffer[5], _buffer[6], _buffer[7], _buffer[8]);
            last_hex_ms = now;
        }

        // Header check
        if (_buffer[0] != 0xAA) {
             return;
        }

        // Checksum Validation: Sum of bytes 1..7 should equal byte 8
        uint8_t sum = 0;
        for (uint8_t i = 1; i < 8; i++) {
            sum += _buffer[i];
        }
        
        if (sum != _buffer[8]) {
            if (print_debug) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Bad Checksum: Calc %02X != Recv %02X", sum, _buffer[8]);
            }
            return;
        }

        // AD Value: Bytes 4, 5, 6 (Indices 4, 5, 6 in 0-based array)
        uint32_t raw_ad = ((uint32_t)_buffer[4] << 16) | ((uint32_t)_buffer[5] << 8) | (uint32_t)_buffer[6];
        
        _latest_weight = (float)raw_ad;
        _healthy = true;
        _valid_frame_count++;

        // Send immediately for low latency
        gcs().send_named_float("WEIGHT", _latest_weight);
    }
};

// Global instance to be used in UserCode.cpp
static AP_Scale_Driver scale_driver;
