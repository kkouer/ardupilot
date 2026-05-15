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
 */

/*
 * ArduPilot 外设驱动：思岚 SLAMTEC RPLIDAR（以 A2 为基线，兼容 S1/S3 等）
 *
 * 协议来源：官方数据手册（详见 AP_Proximity_RPLidarA2.h 顶部链接与中文流程说明）
 *
 * Author: Steven Josefs, IAV GmbH
 * Based on the LightWare SF40C ArduPilot device driver from Randy Mackay
 *
 */

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_RPLIDARA2_ENABLED

#include "AP_Proximity_RPLidarA2.h"

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_RPLidarA2.h"
#include <AP_InternalError/AP_InternalError.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define RP_DEBUG_LEVEL 0

#include <GCS_MAVLink/GCS.h>
#if RP_DEBUG_LEVEL
  #define Debug(level, fmt, args ...)  do { if (level <= RP_DEBUG_LEVEL) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
#else
  #define Debug(level, fmt, args ...)
#endif

#define COMM_ACTIVITY_TIMEOUT_MS        200

/*
 * SLAMTEC RPLIDAR / S 系列串口协议命令与应答类型速查（摘自官方手册，便于对照固件）：
 * 无负载无应答：STOP 0x25；SCAN 0x20；FORCE_SCAN 0x21；RESET 0x40
 * 无负载有应答：GET_DEVICE_INFO 0x50；GET_DEVICE_HEALTH 0x52；GET_SAMPLERATE 0x59
 * 有负载有应答：EXPRESS_SCAN 0x82；HQ_SCAN 0x83；GET_LIDAR_CONF 0x84；SET_LIDAR_CONF 0x85
 * 其它：NEW_BAUDRATE_CONFIRM 0x90；HQ_MOTOR_SPEED_CTRL 0xA8；SET_MOTOR_PWM 0xF0；GET_ACC_BOARD_FLAG 0xFF
 * 应答类型：DEVINFO 0x04；DEVHEALTH 0x06；SAMPLE_RATE 0x15；GET_LIDAR_CONF 0x20；SET_LIDAR_CONF 0x21
 * 测距数据：STANDARD 0x81；CAPSULED 0x82；HQ 0x83；ULTRA_CAPSULED 0x84；DENSE_CAPSULED 0x85；ULTRA_DENSE 0x86
 * GET_LIDAR_CONF 配置项示例：SCAN_MODE_TYPICAL 0x0000007C；SCAN_MODE_COUNT 0x70；SCAN_MODE_US_PER_SAMPLE 0x71 等
 */

// 命令字（与 SLAMTEC 串口协议一致）
//-----------------------------------------

// 无负载、无应答
#define RPLIDAR_PREAMBLE               0xA5
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21
#define RPLIDAR_CMD_RESET              0x40

// 无负载、有应答描述符 + 数据
#define RPLIDAR_CMD_GET_DEVICE_INFO    0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH  0x52

// 有负载、有应答（本文件内用到的子集）
#define RPLIDAR_CMD_EXPRESS_SCAN       0x82
#define RPLIDAR_CMD_GET_SAMPLERATE     0x59
#define RPLIDAR_CMD_MOTOR_SPEED_CTRL   0xA8
#define RPLIDAR_CMD_GET_LIDAR_CONF     0x84

#define RPLIDAR_ANS_TYPE_GET_LIDAR_CONF            0x20
#define RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED 0x85

#define RPLIDAR_CONF_SCAN_MODE_TYPICAL   0x0000007CU
#define RPLIDAR_EXPRESS_FLAG_BOOST       0x0001U

#ifndef RPLIDAR_DENSE_CAPSULE_BYTES
#define RPLIDAR_DENSE_CAPSULE_BYTES      84U
#endif

// 10 Hz 扫描频率 ≈ 600 RPM（MOTOR_SPEED_CTRL 使用 RPM 单位）
#define RPLIDAR_MOTOR_RPM_10HZ         600U

extern const AP_HAL::HAL& hal;

// SLAMTEC 串口请求包校验：0 ⊕ A5 ⊕ 命令 ⊕ 负载长度 ⊕ 各负载字节
static uint8_t rplidar_serial_checksum(uint8_t cmd, uint8_t payload_len, const uint8_t *payload)
{
    uint8_t cs = 0;
    cs ^= RPLIDAR_PREAMBLE;
    cs ^= cmd;
    cs ^= payload_len;
    for (uint8_t i = 0; i < payload_len; i++) {
        cs ^= payload[i];
    }
    return cs;
}

static inline uint16_t rplidar_u16_le(uint8_t lo, uint8_t hi)
{
    return uint16_t(lo) | (uint16_t(hi) << 8);
}

void AP_Proximity_RPLidarA2::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // S 系列：发完电机转速后等待稳定，再进入 GET_LIDAR_CONF → EXPRESS Dense 高速链（或退回 SCAN）
    if (_motor_stable_deadline_ms != 0 && AP_HAL::millis() >= _motor_stable_deadline_ms) {
        _motor_stable_deadline_ms = 0;
        begin_scan_after_init();
    }

    // 复位后满 3 s 再请求设备信息（S1 复位后可能只发 9 字节，与 A1/A2 的 63 字节横幅不同）
    uint32_t now_ms = AP_HAL::millis();
    if ((_state == State::RESET) && (now_ms - _last_reset_ms > 3000)) {
        send_request_for_device_info();
        _state = State::AWAITING_RESPONSE;
        _byte_count = 0;
    }

    get_readings();

    // 长时间无测距则标为无数据，并周期性尝试硬件复位
    if (AP_HAL::millis() - _last_distance_received_ms > COMM_ACTIVITY_TIMEOUT_MS) {
        set_status(AP_Proximity::Status::NoData);
        Debug(1, "LIDAR NO DATA");
        if (AP_HAL::millis() - _last_reset_ms > 10000) {
            reset_rplidar();
        }
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// 传感器最大量程（米）
float AP_Proximity_RPLidarA2::distance_max_m() const
{
    switch (model) {
    case Model::UNKNOWN:
        return 0.0f;
    case Model::A1:
        return 8.0f;
    case Model::A2:
        return 16.0f;
    case Model::A2M12:
    case Model::C1:
        return 12.0f;
    case Model::S1:
    case Model::S3:
        return 40.0f;
    }
    return 0.0f;
}

// 传感器最近盲区（米）
float AP_Proximity_RPLidarA2::distance_min_m() const
{
    switch (model) {
    case Model::UNKNOWN:
        return 0.0f;
    case Model::A1:
    case Model::A2:
    case Model::A2M12:
    case Model::C1:
    case Model::S1:
    case Model::S3:
        return 0.2f;
    }
    return 0.0f;
}

void AP_Proximity_RPLidarA2::reset_rplidar()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_RESET};
    _uart->write(tx_buffer, 2);
    Debug(1, "LIDAR reset");
    // TODO：手册要求复位后等待约 8 ms（原文档笔误写为 8 m）
    _last_reset_ms =  AP_HAL::millis();
    reset();
}

// 进入传统 SCAN（0x20）连续输出模式
void AP_Proximity_RPLidarA2::send_scan_mode_request()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_SCAN};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent scan mode request");
}

// 停止测距（退出扫描态）
void AP_Proximity_RPLidarA2::send_stop()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_STOP};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent STOP");
}

// 查询设备健康
void AP_Proximity_RPLidarA2::send_request_for_health()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_HEALTH};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent health request");
}

// 查询设备信息（型号、固件等）
void AP_Proximity_RPLidarA2::send_request_for_device_info()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_INFO};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent device information request");
}

void AP_Proximity_RPLidarA2::send_request_for_samplerate()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_SAMPLERATE};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent GET_SAMPLERATE request");
}

void AP_Proximity_RPLidarA2::send_motor_speed_ctrl(uint16_t rpm_rplidar)
{
    uint8_t pl[2] {
        uint8_t(rpm_rplidar & 0xFF),
        uint8_t((rpm_rplidar >> 8) & 0xFF),
    };
    const uint8_t pkt[] {
        RPLIDAR_PREAMBLE,
        RPLIDAR_CMD_MOTOR_SPEED_CTRL,
        2,
        pl[0],
        pl[1],
        rplidar_serial_checksum(RPLIDAR_CMD_MOTOR_SPEED_CTRL, 2, pl),
    };
    _uart->write(pkt, sizeof(pkt));
    Debug(1, "Sent MOTOR_SPEED_CTRL rpm=%u", (unsigned)rpm_rplidar);
}

void AP_Proximity_RPLidarA2::send_get_lidar_conf_typical_scan_mode()
{
    const uint32_t typ = RPLIDAR_CONF_SCAN_MODE_TYPICAL;
    uint8_t pl[4] {
        uint8_t(typ & 0xFFU),
        uint8_t((typ >> 8U) & 0xFFU),
        uint8_t((typ >> 16U) & 0xFFU),
        uint8_t((typ >> 24U) & 0xFFU),
    };
    const uint8_t pkt[] {
        RPLIDAR_PREAMBLE,
        RPLIDAR_CMD_GET_LIDAR_CONF,
        4,
        pl[0], pl[1], pl[2], pl[3],
        rplidar_serial_checksum(RPLIDAR_CMD_GET_LIDAR_CONF, 4, pl),
    };
    _uart->write(pkt, sizeof(pkt));
    Debug(1, "Sent GET_LIDAR_CONF TYPICAL");
}

void AP_Proximity_RPLidarA2::send_express_scan_dense_high_speed()
{
    const uint8_t wm = uint8_t(_express_scan_mode & 0xFFU);
    const uint16_t flags = RPLIDAR_EXPRESS_FLAG_BOOST;
    const uint16_t param = 0;
    const uint8_t pl[5] {
        wm,
        uint8_t(flags & 0xFFU),
        uint8_t((flags >> 8U) & 0xFFU),
        uint8_t(param & 0xFFU),
        uint8_t((param >> 8U) & 0xFFU),
    };
    const uint8_t pkt[] {
        RPLIDAR_PREAMBLE,
        RPLIDAR_CMD_EXPRESS_SCAN,
        5,
        pl[0], pl[1], pl[2], pl[3], pl[4],
        rplidar_serial_checksum(RPLIDAR_CMD_EXPRESS_SCAN, 5, pl),
    };
    _uart->write(pkt, sizeof(pkt));
    _awaiting_express_descriptor = true;
    Debug(1, "Sent EXPRESS_SCAN mode=%u BOOST", (unsigned)wm);
}

void AP_Proximity_RPLidarA2::send_fallback_standard_scan()
{
    send_scan_mode_request();
    _state = State::AWAITING_RESPONSE;
}

void AP_Proximity_RPLidarA2::begin_scan_after_init()
{
    // S1/S3：先取典型扫描模式，再以 EXPRESS + Boost 进入 Dense 高速测距；其它型号用传统 SCAN
    if (model == Model::S1 || model == Model::S3) {
        _awaiting_lidar_conf_typical = true;
        send_get_lidar_conf_typical_scan_mode();
        _state = State::AWAITING_RESPONSE;
        return;
    }
    send_scan_mode_request();
    _state = State::AWAITING_RESPONSE;
}

void AP_Proximity_RPLidarA2::consume_bytes(uint16_t count)
{
    if (count > _byte_count) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        _byte_count = 0;
        return;
    }
    _byte_count -= count;
    if (_byte_count) {
        memmove((void*)&_payload[0], (void*)&_payload[count], _byte_count);
    }
}

void AP_Proximity_RPLidarA2::reset()
{
    _state = State::RESET;
    _byte_count = 0;
    _motor_stable_deadline_ms = 0;
    _pre_scan_health_pending = false;
    _awaiting_samplerate_response = false;
    _awaiting_lidar_conf_typical = false;
    _awaiting_express_descriptor = false;
    _pending_descriptor_payload_len = 0;
    _express_scan_mode = 0;
    _dense_have_prev = false;
    _dense_last_sync_bit = 0;
    _t_express_sample_us = 250;
    _mp_debug_summary_sent = false;
    _stashed_health_status = 255;
    _stashed_health_err = 0;
    _stashed_t_standard_us = 0;
    _stashed_fw_minor = 0;
    _stashed_fw_major = 0;
    _stashed_hw = 0;
}

bool AP_Proximity_RPLidarA2::make_first_byte_in_payload(uint8_t desired_byte)
{
    if (_byte_count == 0) {
        return false;
    }
    if (_payload[0] == desired_byte) {
        return true;
    }
    for (auto i=1; i<_byte_count; i++) {
        if (_payload[i] == desired_byte) {
            consume_bytes(i);
            return true;
        }
    }
    // 缓冲区内找不到目标字节则清空，避免卡死
    _byte_count = 0;
    return false;
}

void AP_Proximity_RPLidarA2::get_readings()
{
    Debug(2, "             CURRENT STATE: %u ", (unsigned)_state);
    const uint32_t nbytes = _uart->available();
    if (nbytes == 0) {
        return;
    }
    const uint32_t bytes_to_read = MIN(nbytes, sizeof(_payload)-_byte_count);
    if (bytes_to_read == 0) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        reset();
        return;
    }
    const uint32_t bytes_read = _uart->read(&_payload[_byte_count], bytes_to_read);
    if (bytes_read == 0) {
        // available() 说有数据但 read 为 0，异常
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        reset();
        return;
    }
    _byte_count += bytes_read;

    uint32_t previous_loop_byte_count = UINT32_MAX;
    while (_byte_count) {
        if (_byte_count >= previous_loop_byte_count) {
            // 严重错误：本轮未消耗任何字节，防止死循环
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            _uart = nullptr;
            return;
        }
        previous_loop_byte_count = _byte_count;

        switch(_state){
        case State::RESET: {
            // 复位横幅：首字节 'R'，其后固定 62 字节信息（与首字节共 63 字节）
            if (!make_first_byte_in_payload('R')) { // 'R' = RPLIDAR 启动信息
                return;
            }
            if (_byte_count < 63) {
                return;
            }
#if RP_DEBUG_LEVEL
            // 调试：可将 63 字节横幅经 MAVLink 打出
            Debug(1, "Got RPLidar Information");
            char xbuffer[64]{};
            memcpy((void*)xbuffer, (void*)&_payload.information, 63);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RPLidar: (%s)", xbuffer);
#endif
            // 丢弃 63 字节横幅，改向设备查询信息
            consume_bytes(63);
            send_request_for_device_info();
            _state = State::AWAITING_RESPONSE;
            continue;
        }
        case State::AWAITING_RESPONSE:
            if (_payload[0] != RPLIDAR_PREAMBLE) {
                // 协议错误：首字节应为 A5
                reset();
                return;
            }

            // 应答描述符固定 7 字节，其后为具体负载
            if (_byte_count < sizeof(_descriptor)) {
                return;
            }
            // 根据描述符判断紧随其后的数据类型
            static const _descriptor SCAN_DATA_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81 }
            };
            static const _descriptor HEALTH_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06 }
            };
            static const _descriptor DEVICE_INFO_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04 }
            };
            static const _descriptor SAMPLE_RATE_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x04, 0x00, 0x00, 0x00, 0x15 }
            };
            Debug(2,"LIDAR descriptor found");
            if (_awaiting_samplerate_response &&
                memcmp((void*)&_payload[0], SAMPLE_RATE_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _awaiting_samplerate_response = false;
                _state = State::AWAITING_SAMPLERATE_DATA;
            } else if (_awaiting_lidar_conf_typical &&
                       _payload[0] == RPLIDAR_PREAMBLE && _payload[1] == 0x5A &&
                       _payload[6] == RPLIDAR_ANS_TYPE_GET_LIDAR_CONF) {
                _pending_descriptor_payload_len = rplidar_u16_le(_payload[2], _payload[3]);
                _awaiting_lidar_conf_typical = false;
                _state = State::AWAITING_LIDAR_CONF_DATA;
            } else if (_awaiting_express_descriptor) {
                if (_payload[0] == RPLIDAR_PREAMBLE && _payload[1] == 0x5A &&
                    _payload[6] == RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED) {
                    _pending_descriptor_payload_len = rplidar_u16_le(_payload[2], _payload[3]);
                    _awaiting_express_descriptor = false;
                    _state = State::AWAITING_EXPRESS_DENSE;
                    _dense_have_prev = false;
                    _dense_last_sync_bit = 0;
                    if (!_mp_debug_summary_sent && (model == Model::S3)) {
                        _mp_debug_summary_sent = true;
                        send_mp_debug_summary(true);
                    }
                } else {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RPLidar: Express 非 Dense，退回 SCAN");
                    _awaiting_express_descriptor = false;
                    send_stop();
                    send_fallback_standard_scan();
                }
            } else if (memcmp((void*)&_payload[0], SCAN_DATA_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_SCAN_DATA;
                if (!_mp_debug_summary_sent && (model == Model::S3)) {
                    _mp_debug_summary_sent = true;
                    send_mp_debug_summary(false);
                }
            } else if (memcmp((void*)&_payload[0], DEVICE_INFO_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_DEVICE_INFO;
            } else if (memcmp((void*)&_payload[0], HEALTH_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_HEALTH;
            } else {
                // 未知描述符：丢弃 7 字节后继续同步
            }
            consume_bytes(sizeof(_descriptor));
            break;

        case State::AWAITING_DEVICE_INFO:
            if (_byte_count < sizeof(_payload.device_info)) {
                return;
            }
            parse_response_device_info();
            consume_bytes(sizeof(_payload.device_info));
            break;

        case State::AWAITING_SCAN_DATA:
            if (_byte_count < sizeof(_payload.sensor_scan)) {
                return;
            }
            parse_response_data();
            consume_bytes(sizeof(_payload.sensor_scan));
            break;

        case State::AWAITING_HEALTH:
            if (_byte_count < sizeof(_payload.sensor_health)) {
                return;
            }
            parse_response_health();
            consume_bytes(sizeof(_payload.sensor_health));
            break;

        case State::AWAITING_SAMPLERATE_DATA:
            if (_byte_count < sizeof(_payload.sample_rate)) {
                return;
            }
            parse_response_samplerate();
            consume_bytes(sizeof(_payload.sample_rate));
            break;

        case State::AWAITING_LIDAR_CONF_DATA:
            if (_byte_count < _pending_descriptor_payload_len) {
                return;
            }
            parse_response_lidar_conf_typical();
            consume_bytes(_pending_descriptor_payload_len);
            break;

        case State::AWAITING_EXPRESS_DENSE:
            if (_byte_count < sizeof(RPLidarDenseCapsule)) {
                return;
            }
            parse_response_dense_capsule();
            consume_bytes(sizeof(RPLidarDenseCapsule));
            break;
        }
    }
}

void AP_Proximity_RPLidarA2::parse_response_device_info()
{
    Debug(1, "Received DEVICE_INFO");
    const char *device_type = "UNKNOWN";
    switch (_payload.device_info.model) {
    case 0x18:
        model = Model::A1;
        device_type = "A1";
        break;
    case 0x28:
        model = Model::A2;
        device_type = "A2";
        break;
    case 0x2C:
        model = Model::A2M12;
        device_type = "A2M12";
        break;
    case 0x41:
        model=Model::C1;
        device_type="C1";
        break;
    case 0x61:
        model = Model::S1;
        device_type = "S1";
        break;
    case 0x81:
        model = Model::S3;
        device_type = "S3";
        break;
    default:
        Debug(1, "Unknown device (%u)", _payload.device_info.model);
    }
    _stashed_fw_minor = _payload.device_info.firmware_minor;
    _stashed_fw_major = _payload.device_info.firmware_major;
    _stashed_hw = _payload.device_info.hardware;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, u8"RPLidar识别:%s FW%u.%u HW%u", device_type,
                  (unsigned)_payload.device_info.firmware_major,
                  (unsigned)_payload.device_info.firmware_minor,
                  (unsigned)_payload.device_info.hardware);
    // SLAMTEC 建议：正式扫前先发 HEALTH；此处再分支 S 系列电机与采样率
    _pre_scan_health_pending = true;
    send_request_for_health();
    _state = State::AWAITING_RESPONSE;
}

void AP_Proximity_RPLidarA2::parse_response_data()
{
    if (_sync_error) {
        // 5 字节同步丢失：等待下一周起始特征重新对齐
        Debug(1, "       OUT OF SYNC");
        // 首字节低两位：新一周常为 0b01
        if ((_payload[0] & 0x03) == 0x01) {
            _sync_error = 0;
            Debug(1, "                  RESYNC");
        } else {
            return;
        }
    }
    Debug(2, "UART %02x %02x%02x %02x%02x", _payload[0], _payload[2], _payload[1], _payload[4], _payload[3]); // 调试：原始五字节
    // 校验 SCAN 包：起始两位互补，且校验位为 1
    if (!((_payload.sensor_scan.startbit == !_payload.sensor_scan.not_startbit) && _payload.sensor_scan.checkbit)) {
        Debug(1, "Invalid Payload");
        _sync_error++;
        return;
    }

    const float angle_sign = (params.orientation == 1) ? -1.0f : 1.0f;
    const float angle_deg = wrap_360(_payload.sensor_scan.angle_q6/64.0f * angle_sign + params.yaw_correction);
    const float distance_m = (_payload.sensor_scan.distance_q2/4000.0f);
#if RP_DEBUG_LEVEL >= 2
    const float quality = _payload.sensor_scan.quality;
    Debug(2, "   D%02.2f A%03.1f Q%0.2f", distance_m, angle_deg, quality);
#endif
    if (!ignore_reading(angle_deg, distance_m)) {
        apply_proximity_sample(angle_deg, distance_m);
    }
}

void AP_Proximity_RPLidarA2::apply_proximity_sample(float angle_deg, float distance_m)
{
    _last_distance_received_ms = AP_HAL::millis();

    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);

    if (face != _last_face) {
        if (_last_distance_valid) {
            frontend.boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m, state.instance);
        } else {
            frontend.boundary.reset_face(face, state.instance);
        }

        _last_face = face;
        _last_distance_valid = false;
    }
    if (distance_m > distance_min_m()) {
        if (!_last_distance_valid || (distance_m < _last_distance_m)) {
            _last_distance_m = distance_m;
            _last_distance_valid = true;
            _last_angle_deg = angle_deg;
        }
        database_push(_last_angle_deg, _last_distance_m);
    }
}

void AP_Proximity_RPLidarA2::parse_response_health()
{
    if (_pre_scan_health_pending) {
        _stashed_health_status = _payload.sensor_health.status;
        _stashed_health_err = _payload.sensor_health.error_code;
        _pre_scan_health_pending = false;
        if (_payload.sensor_health.status == 3) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RPLidar: health HW error code=%u",
                          (unsigned)_payload.sensor_health.error_code);
        }
        // S 系列：先 GET_SAMPLERATE，再 MOTOR 10 Hz，延时后进入典型模式 + EXPRESS 高速测距
        if (model == Model::S1 || model == Model::S3) {
            _awaiting_samplerate_response = true;
            send_request_for_samplerate();
            _state = State::AWAITING_RESPONSE;
            return;
        }
        begin_scan_after_init();
        return;
    }

    // 非“扫前健康”路径下的健康应答（当前逻辑仅打日志）
    // status==3 表示硬件类错误
    if (_payload.sensor_health.status == 3) {
        Debug(1, "LIDAR Error");
    }
    Debug(1, "LIDAR Healthy");
}

void AP_Proximity_RPLidarA2::parse_response_samplerate()
{
    _stashed_t_standard_us = _payload.sample_rate.t_standard_us;
    const uint16_t te = _payload.sample_rate.t_express_us;
    if (te > 0) {
        _t_express_sample_us = te;
    }
    send_motor_speed_ctrl(RPLIDAR_MOTOR_RPM_10HZ);
    _motor_stable_deadline_ms = AP_HAL::millis() + 500;
    _state = State::AWAITING_RESPONSE;
}

void AP_Proximity_RPLidarA2::parse_response_lidar_conf_typical()
{
    if (_pending_descriptor_payload_len < 6) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RPLidar: GET_LIDAR_CONF 应答过短，退回 SCAN");
        send_fallback_standard_scan();
        return;
    }
    uint32_t rtyp;
    uint16_t mode;
    memcpy(&rtyp, &_payload[0], sizeof(rtyp));
    memcpy(&mode, &_payload[4], sizeof(mode));
    if (rtyp != uint32_t(RPLIDAR_CONF_SCAN_MODE_TYPICAL)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RPLidar: GET_LIDAR_CONF 类型异常 0x%08lx", (unsigned long)rtyp);
        send_fallback_standard_scan();
        return;
    }
    if (mode == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RPLidar: 典型扫描模式 ID=0，退回 SCAN");
        send_fallback_standard_scan();
        return;
    }
    _express_scan_mode = mode;
    send_express_scan_dense_high_speed();
    _state = State::AWAITING_RESPONSE;
}

void AP_Proximity_RPLidarA2::send_mp_debug_summary(bool express_dense_boost_mode)
{
    if (_uart == nullptr) {
        return;
    }
    const uint32_t baud = _uart->get_baud_rate();
    const char *mname = "S3";
    const MAV_SEVERITY sev = (_stashed_health_status == 3 || _stashed_health_status == 2)
        ? MAV_SEVERITY_WARNING
        : MAV_SEVERITY_INFO;

    const char *hzh = u8"(未读健康)";
    if (_stashed_health_status != 255) {
        switch (_stashed_health_status) {
        case 0: hzh = u8"正常"; break;
        case 1: hzh = u8"警告"; break;
        case 2: hzh = u8"错误"; break;
        case 3: hzh = u8"硬件故障"; break;
        default: hzh = u8"未知"; break;
        }
    }

    GCS_SEND_TEXT(sev, u8"RPLidar %s 链路就绪", mname);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, u8"型号:%s FW%u.%u HW%u", mname,
                  (unsigned)_stashed_fw_major, (unsigned)_stashed_fw_minor, (unsigned)_stashed_hw);
    if (_stashed_health_status == 255) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, u8"健康:未缓存");
    } else {
        GCS_SEND_TEXT(sev, u8"健康:%s err=%u st=%u", hzh,
                      (unsigned)_stashed_health_err, (unsigned)_stashed_health_status);
    }

    if (baud > 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, u8"串口波特率:%lu", (unsigned long)baud);
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, u8"串口波特率:无效/非UART");
    }

    if (express_dense_boost_mode) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, u8"扫描:EXPRESS Dense+Boost typ=%u",
                      (unsigned)_express_scan_mode);
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, u8"扫描:标准SCAN 0x20");
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, u8"采样周期us STD=%u EXPR=%u",
                  (unsigned)_stashed_t_standard_us, (unsigned)_t_express_sample_us);
    if (_stashed_t_standard_us > 0) {
        const float kstd = 1000000.0f / float(_stashed_t_standard_us) / 1000.0f;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, u8"STD约测频kHz:%.2f", double(kstd));
    }
    if (_t_express_sample_us > 0) {
        const float kex = 1000000.0f / float(_t_express_sample_us) / 1000.0f;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, u8"EXPR约测频kHz:%.2f", double(kex));
    }

    const unsigned rpm = (unsigned)RPLIDAR_MOTOR_RPM_10HZ;
    const unsigned hz_i = rpm / 60U;
    const unsigned hz_f = ((rpm * 10U) / 60U) % 10U;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, u8"MOTOR:%uRPM 指令扫描:%u.%uHz",
                  rpm, hz_i, hz_f);
}

void AP_Proximity_RPLidarA2::parse_response_dense_capsule()
{
    // 角度插值与同步位逻辑参考 Slamtec 开源 rplidar_sdk（handler_capsules.cpp，BSD 许可）
    RPLidarDenseCapsule curr;
    memcpy(&curr, &_payload[0], sizeof(curr));

    if (!_dense_have_prev) {
        _dense_prev = curr;
        _dense_have_prev = true;
        return;
    }

    RPLidarDenseCapsule &prev = _dense_prev;

    const int currentStartAngle_q8 = int((curr.start_angle_sync_q6 & 0x7FFFU)) << 2;
    const int prevStartAngle_q8 = int((prev.start_angle_sync_q6 & 0x7FFFU)) << 2;

    int diffAngle_q8 = currentStartAngle_q8 - prevStartAngle_q8;
    if (prevStartAngle_q8 > currentStartAngle_q8) {
        diffAngle_q8 += (360 << 8);
    }

    uint32_t dur_us = _t_express_sample_us;
    if (dur_us < 50U) {
        dur_us = 50U;
    }
    const int maxDiffAngleThreshold_q8 = int((360ULL * 100ULL * 40ULL / (1000000ULL / uint64_t(dur_us))) << 8ULL);

    if (diffAngle_q8 > maxDiffAngleThreshold_q8) {
        _dense_prev = curr;
        return;
    }

    const int angleInc_q16 = (diffAngle_q8 << 8) / 40;
    int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);

    int lastNodeSyncBit = _dense_last_sync_bit;
    const float angle_sign = (params.orientation == 1) ? -1.0f : 1.0f;

    for (int pos = 0; pos < 40; ++pos) {
        const uint16_t dist_mm = prev.cabin_dist_mm[pos];
        int angle_q6 = (currentAngle_raw_q16 >> 10);

        int syncBit = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < (angleInc_q16 << 1)) ? 1 : 0;
        syncBit = (syncBit ^ lastNodeSyncBit) & syncBit;
        lastNodeSyncBit = syncBit;

        currentAngle_raw_q16 += angleInc_q16;

        if (angle_q6 < 0) {
            angle_q6 += (360 << 6);
        }
        if (angle_q6 >= (360 << 6)) {
            angle_q6 -= (360 << 6);
        }

        const float angle_deg_raw = float(angle_q6) * (1.0f / 64.0f);
        const float angle_deg = wrap_360(angle_deg_raw * angle_sign + params.yaw_correction);
        const float distance_m = (dist_mm > 0) ? (dist_mm * 0.001f) : 0.0f;

        if (dist_mm > 0) {
            if (!ignore_reading(angle_deg, distance_m)) {
                apply_proximity_sample(angle_deg, distance_m);
            }
        }
    }

    _dense_last_sync_bit = lastNodeSyncBit;
    _dense_prev = curr;
}

#endif // AP_PROXIMITY_RPLIDARA2_ENABLED
