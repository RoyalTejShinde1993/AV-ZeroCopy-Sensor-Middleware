#pragma once

#include "abi/sensor_c_abi.h"
#include <cstdint>
#include <cstddef>

namespace av::core {

constexpr uint32_t AV_RECORD_MAGIC = 0x41564452; // "AVDR"

#pragma pack(push, 1)
struct BinaryPacketHeader {
    uint32_t magic{AV_RECORD_MAGIC};
    uint16_t version{1};
    uint16_t sensor_type{0};
    uint32_t sensor_id{0};
    uint64_t sequence_id{0};
    uint64_t hw_timestamp_ns{0};
    uint64_t ingest_timestamp_ns{0};
    uint32_t payload_length{0};
    uint32_t payload_crc32{0};
};
#pragma pack(pop)

inline uint32_t compute_crc32(const uint8_t* data, size_t length) noexcept {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k) {
            crc = (crc >> 1) ^ (0xEDB88320u & (-(static_cast<int32_t>(crc & 1))));
        }
    }
    return ~crc;
}

} // namespace av::core
