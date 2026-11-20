#ifndef AV_SENSOR_C_ABI_H
#define AV_SENSOR_C_ABI_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AV_SENSOR_CAMERA = 1,
    AV_SENSOR_LIDAR  = 2,
    AV_SENSOR_RADAR  = 3,
    AV_SENSOR_IMU    = 4,
    AV_SENSOR_CAN    = 5,
    AV_SENSOR_GPS    = 6
} av_sensor_type_t;

typedef enum {
    AV_STATUS_OK = 0,
    AV_STATUS_ERR_TIMEOUT = -1,
    AV_STATUS_ERR_POOL_EXHAUSTED = -2,
    AV_STATUS_ERR_HW_FAULT = -3
} av_status_t;

/* Cache-line aligned metadata descriptor passed via Zero-Copy Ring Buffer */
typedef struct __attribute__((aligned(64))) {
    uint64_t sequence_id;
    uint64_t hw_timestamp_ns;     /* Simulated PTP hardware clock */
    uint64_t ingest_timestamp_ns; /* Host arrival monotonic clock */
    uint32_t sensor_id;
    uint32_t sensor_type;
    uint32_t shm_slot_index;      /* Index into POSIX Shared Memory Pool */
    uint32_t payload_size_bytes;
    uint32_t checksum_crc32;
    uint32_t flags;
} av_frame_descriptor_t;

#ifdef __cplusplus
}
#endif

#endif /* AV_SENSOR_C_ABI_H */
