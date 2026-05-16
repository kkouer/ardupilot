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
 * ArduPilot 外设驱动：思岚 SLAMTEC RPLIDAR（以 A2 16 m 版为基线，兼容多型号含 S3）
 *
 * 协议说明来自官方数据手册 / S 系列协议：
 * https://www.slamtec.com/en/Lidar
 * http://bucket.download.slamtec.com/63ac3f0d8c859d3a10e51c6b3285fcce25a47357/LR001_SLAMTEC_rplidar_protocol_v1.0_en.pdf
 * S 系列（含 S3）中文协议：
 * https://bucket-download.slamtec.com/411e6a5ec1c43c2b9720ece9f3f79a3e180c5721/LR001_SLAMTEC_rplidar_S%20series_protocol_v1.0_cn.pdf
 *
 * RPLIDAR S3 与本驱动的主机侧工作流程（按 SLAMTEC 建议整理，与本代码路径对应）：
 * 1) 上电 / RESET：雷达启动；若有 UART 启动信息则等待接收完毕。
 * 2) GET_DEVICE_INFO：识别型号（S3 在 DEVICE_INFO 中型号字节为 0x81）。
 * 3) GET_DEVICE_HEALTH：正式测距前先查健康，避免保护停机 / 严重故障仍强启。
 * 4) GET_SAMPLERATE（S1/S3 路径）：读取标准/Express 下单次采样时间 Tstandard、Texpress（µs），便于转速相关调试。
 * 5) MOTOR_SPEED_CTRL（0xA8）：设置电机目标转速；本驱动固定 600 RPM，对应扫描频率约 10 Hz。
 * 6) 等待电机闭环稳定（约数百毫秒）；转速稳定后雷达才开始可靠输出测距。
 * 7) 启动测距：
 *    - S1/S3：GET_LIDAR_CONF(典型扫描模式) → EXPRESS_SCAN（扩展模式 + Boost 高速标志）→ 解析 Dense 胶囊包（应答类型 0x85）。
 *    - 其它型号：传统 SCAN（0x20），每点 5 字节。
 *    - 若高速描述符非 Dense 或典型模式获取失败，则回退 SCAN。
 * 8) 数据流：Dense 胶囊按 Slamtec SDK 校验前两字节（低/高各 4bit 拼成 8bit）与后续载荷异或；
 *    连续校验失败则 STOP 并退回标准 SCAN，减轻串口误码导致的“全向假障碍”。
 *    S1/S3：退回 SCAN 后经过 RPLIDAR_DENSE_SCAN_FALLBACK_RETRY_MS 毫秒自动排空 RX 并重走
 *    GET_SAMPLERATE→电机稳定→GET_LIDAR_CONF→EXPRESS Dense。
 *    同步位 S=1 仍用于每周转对齐角度；亦可据此估算当前 RPM。
 * 9) 结束：发送 STOP（0x25）；S 系列将 MOTOR_SPEED_CTRL 的 RPM 置 0 可进入空闲。
 *
 * Author: Steven Josefs, IAV GmbH
 * Based on the LightWare SF40C ArduPilot device driver from Randy Mackay
 *
 */

/*

# 真机接 SITL 串口示例（Proximity + RPLidar）：
./Tools/autotest/sim_vehicle.py -v Rover --gdb --debug -A --serial5=uart:/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:115200
param set SERIAL5_PROTOCOL 11
param set SERIAL5_BAUD 115200
param set PRX1_TYPE 5
reboot

# 部分型号需短接 JST 外侧两线使电机转动（视硬件而定）

*/


#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_RPLIDARA2_ENABLED

#include "AP_Proximity_Backend_Serial.h"

class AP_Proximity_RPLidarA2 : public AP_Proximity_Backend_Serial
{

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // 周期更新：读串口、推进状态机、超时看门狗
    void update(void) override;

    // 传感器最大/最小有效距离（米）
    float distance_max_m() const override;
    float distance_min_m() const override;

private:

    // SLAMTEC Dense 胶囊包：84 字节（与官方 SDK dense capsule 布局一致）
    struct PACKED RPLidarDenseCapsule {
        uint8_t s_checksum_1;
        uint8_t s_checksum_2;
        uint16_t start_angle_sync_q6;       ///< bit15 为每周转同步标志；低 15 位为起始角 q6（度×64）
        uint16_t cabin_dist_mm[40];         ///< 各采样距离，单位 mm；0 表示无效点
    };
    static_assert(sizeof(RPLidarDenseCapsule) == 84, "Dense 胶囊包应为 84 字节");

    // 串口解析状态：复位横幅 → 等应答描述符 → 收具体负载
    enum class State {
        RESET = 56,
        AWAITING_RESPONSE,
        AWAITING_SCAN_DATA,
        AWAITING_HEALTH,
        AWAITING_DEVICE_INFO,
        AWAITING_SAMPLERATE_DATA,
        AWAITING_LIDAR_CONF_DATA,
        AWAITING_EXPRESS_DENSE,
    } _state = State::RESET;

    // 向雷达发送各类请求
    void send_request_for_health();
    void send_scan_mode_request();
    void send_stop();
    void send_request_for_device_info();
    void send_request_for_samplerate();
    void send_motor_speed_ctrl(uint16_t rpm_rplidar);
    void send_get_lidar_conf_typical_scan_mode();
    void send_express_scan_dense_high_speed();
    void send_fallback_standard_scan();
    void begin_scan_after_init();
    void parse_response_samplerate();

    void parse_response_data();
    void parse_response_health();
    void parse_response_device_info();
    void parse_response_lidar_conf_typical();
    void parse_response_dense_capsule();

    void apply_proximity_sample(float angle_deg, float distance_m);

    // Mission Planner 消息窗口：S 系列链路就绪后的中文调试摘要
    void send_mp_debug_summary(bool express_dense_boost_mode);

    void get_readings();
    void reset_rplidar();
    void reset();

    // 从接收缓冲丢弃已处理字节（前移剩余数据）
    void consume_bytes(uint16_t count);

    /// 丢弃 UART 中未读字节，避免 STOP/切模式后残留测距流破坏 A5 描述符同步
    void drain_uart_rx();
    /// 因 Dense 校验失败退回 SCAN 后，到期则重试 Dense 链路（仅 S1/S3，且须在标准 SCAN 态）
    void maybe_retry_dense_after_fallback_scan();

    uint8_t _sync_error;
    uint16_t _byte_count;

    // 请求与超时相关
    uint32_t  _last_distance_received_ms;     ///< 最近一次收到测距样本的系统时间（ms）
    uint32_t  _last_reset_ms;
    uint32_t  _motor_stable_deadline_ms {};      ///< S 系列：发 MOTOR_SPEED_CTRL 后到此时刻再发 SCAN

    bool _pre_scan_health_pending {};          ///< 正在等待“扫前”那次 HEALTH 应答
    bool _awaiting_samplerate_response {};      ///< 已发 GET_SAMPLERATE，等待其应答描述符
    bool _awaiting_lidar_conf_typical {};      ///< 已发 GET_LIDAR_CONF(典型模式)，等待描述符
    bool _awaiting_express_descriptor {};      ///< 已发 EXPRESS_SCAN，等待 Dense 等测距描述符

    uint16_t _pending_descriptor_payload_len {}; ///< 当前单次应答负载长度（来自 7 字节描述符第 2~3 字节）
    uint16_t _express_scan_mode {};             ///< GET_LIDAR_CONF 返回的典型扫描模式 ID
    uint16_t _t_express_sample_us {250};        ///< GET_SAMPLERATE 的 Texpress，用于 Dense 角度门限（µs）

    RPLidarDenseCapsule _dense_prev {};
    bool _dense_have_prev {};
    int _dense_last_sync_bit {};                ///< Dense 解包同步位状态（Slamtec SDK 等价逻辑）
    uint8_t _dense_consecutive_checksum_fails {}; ///< Dense 胶囊校验连续失败计数（达阈值退回 SCAN）
    uint32_t _dense_auto_retry_at_ms {};         ///< 非 0：该时刻（ms）到达后从 SCAN 重试 Dense（仅校验失败退回时设置）

    bool _mp_debug_summary_sent {};             ///< 已向 GCS 发送本周期连接摘要（避免重复）
    uint8_t _stashed_health_status {255};       ///< 扫前 GET_HEALTH：status（255=未缓存）
    uint16_t _stashed_health_err {};            ///< 扫前健康错误码
    uint16_t _stashed_t_standard_us {};         ///< GET_SAMPLERATE：Tstandard（µs）
    uint8_t _stashed_fw_minor {};                ///< DEVICE_INFO 缓存供摘要
    uint8_t _stashed_fw_major {};
    uint8_t _stashed_hw {};

    // 与 Proximity 分区（面）相关的缓存
    AP_Proximity_Boundary_3D::Face _last_face;///< 当前正在累积的边界分区
    float _last_angle_deg;                    ///< 与 _last_distance_m 对应方位角（度）
    float _last_distance_m;                   ///< 该分区内当前最短距离（米）
    bool _last_distance_valid;                ///< _last_distance_m 是否有效

    struct PACKED _device_info {
        uint8_t model;
        uint8_t firmware_minor;
        uint8_t firmware_major;
        uint8_t hardware;
        uint8_t serial[16];
   };

    struct PACKED _sensor_scan {
        uint8_t startbit      : 1;            ///< 每周转首点常为 1，其余为 0
        uint8_t not_startbit  : 1;            ///< 与 startbit 互补
        uint8_t quality       : 6;            ///< 与回波强度相关的质量指示
        uint8_t checkbit      : 1;            ///< 固定为 1
        uint16_t angle_q6     : 15;           ///< 方位角 = angle_q6/64.0（度）
        uint16_t distance_q2  : 16;           ///< 距离（mm）= distance_q2/4.0
    };

    struct PACKED _sensor_health {
        uint8_t status;                       ///< 0 正常，1 警告，2 错误，3 硬件错误等（以手册为准）
        uint16_t error_code;                ///< 错误码
    };

    struct PACKED _sample_rate {
        uint16_t t_standard_us;               ///< SCAN 模式下单次采样时间（µs）
        uint16_t t_express_us;                ///< Express 模式下单次采样时间（µs）
    };

    struct PACKED _descriptor {
        uint8_t bytes[7];
    };

    // 复位后 UART 横幅信息（多数型号 63 字节）；不必长期保存，但用大缓冲便于一次 read
    struct PACKED _rpi_information {
        uint8_t bytes[63];
    };

    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        _sensor_scan sensor_scan;
        _sensor_health sensor_health;
        _sample_rate sample_rate;
        _descriptor descriptor;
        _rpi_information information;
        _device_info device_info;
        uint8_t forced_buffer_size[2048]; // 扩大容量以便 hal.serial 批量读取
    } _payload;
    static_assert(sizeof(_payload) >= 63, "至少容纳复位横幅 63 字节");

    enum class Model {
        UNKNOWN,
        A1,
        A2,
        A2M12,
        C1,
        S1,
        S3,
    } model = Model::UNKNOWN;

    bool make_first_byte_in_payload(uint8_t desired_byte);
};

#endif // AP_PROXIMITY_RPLIDARA2_ENABLED
