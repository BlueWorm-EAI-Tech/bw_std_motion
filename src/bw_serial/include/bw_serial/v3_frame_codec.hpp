#ifndef BW_SERIAL__V3_FRAME_CODEC_HPP_
#define BW_SERIAL__V3_FRAME_CODEC_HPP_

#include <cstdint>
#include <vector>

#include "protocol.hpp"

namespace bw_serial
{

// 按 V3 协议的 TYPE + LEN_L + LEN_H + PAYLOAD 计算 CRC-16/MODBUS。
uint16_t compute_v3_frame_crc(
    uint8_t frame_type,
    uint16_t payload_length,
    const uint8_t *payload,
    uint16_t payload_size);

// 封装一帧完整的 V3 线协议数据：
// 55 AA | TYPE | LEN_L | LEN_H | PAYLOAD | CRC_L | CRC_H
std::vector<uint8_t> encode_v3_frame(
    uint8_t frame_type,
    uint16_t payload_length,
    const uint8_t *payload);

// 将当前控制结构编码为一帧完整的 V3 控制报文。
std::vector<uint8_t> encode_v3_command_frame(const MantisCommandV3_t &command);

// 校验并解包一帧完整的 V3 反馈报文。
bool decode_v3_feedback_frame(
    const std::vector<uint8_t> &frame,
    MantisFeedbackV3_t *out_feedback);

}  // namespace bw_serial

#endif  // BW_SERIAL__V3_FRAME_CODEC_HPP_
