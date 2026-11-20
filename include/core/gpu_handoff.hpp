#pragma once

#include "abi/sensor_c_abi.h"
#include "core/time_sync.hpp"
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <sys/mman.h>

#ifdef AV_ENABLE_CUDA
#include <cuda_runtime.h>
#endif

namespace av::core {

struct GpuTensorDescriptor {
    uint64_t bundle_id;
    uint64_t sync_skew_ns;
    uintptr_t device_ptr_addr;
    size_t total_tensor_bytes;
    bool is_hardware_cuda;
};

class GpuZeroCopyHandoffEngine {
public:
    explicit GpuZeroCopyHandoffEngine(size_t staging_bytes = 2 * 1024 * 1024)
        : staging_bytes_(staging_bytes) {
#ifdef AV_ENABLE_CUDA
        if (cudaHostAlloc(&pinned_host_buf_, staging_bytes_, cudaHostAllocMapped) == cudaSuccess) {
            cudaHostGetDevicePointer(&device_mapped_ptr_, pinned_host_buf_, 0);
            cuda_active_ = true;
            return;
        }
#endif
        if (posix_memalign(&pinned_host_buf_, 4096, staging_bytes_) == 0 && pinned_host_buf_) {
            mlock(pinned_host_buf_, staging_bytes_);
            device_mapped_ptr_ = pinned_host_buf_;
        }
    }

    ~GpuZeroCopyHandoffEngine() {
        if (!pinned_host_buf_) return;
#ifdef AV_ENABLE_CUDA
        if (cuda_active_) {
            cudaFreeHost(pinned_host_buf_);
            return;
        }
#endif
        munlock(pinned_host_buf_, staging_bytes_);
        free(pinned_host_buf_);
    }

    GpuZeroCopyHandoffEngine(const GpuZeroCopyHandoffEngine&) = delete;
    GpuZeroCopyHandoffEngine& operator=(const GpuZeroCopyHandoffEngine&) = delete;

    GpuTensorDescriptor submit_synchronized_bundle(
        const SynchronizedSensorBundle& bundle,
        const uint8_t* cam_shm_ptr,
        const uint8_t* lidar_shm_ptr,
        const uint8_t* can_shm_ptr) noexcept {
        (void)cam_shm_ptr;
        (void)lidar_shm_ptr;
        (void)can_shm_ptr;

        uint64_t id = bundles_dispatched_.fetch_add(1, std::memory_order_relaxed);
        size_t total_bytes = bundle.camera_frame.payload_size_bytes +
                             bundle.lidar_frame.payload_size_bytes +
                             bundle.can_imu_frame.payload_size_bytes;

        GpuTensorDescriptor tensor_desc{};
        tensor_desc.bundle_id = id;
        tensor_desc.sync_skew_ns = bundle.max_skew_ns;
        tensor_desc.device_ptr_addr = reinterpret_cast<uintptr_t>(device_mapped_ptr_);
        tensor_desc.total_tensor_bytes = total_bytes;
        tensor_desc.is_hardware_cuda = cuda_active_;
        return tensor_desc;
    }

    uint64_t bundles_dispatched() const noexcept { return bundles_dispatched_.load(); }
    bool is_cuda_hardware_backed() const noexcept { return cuda_active_; }

private:
    size_t staging_bytes_;
    void* pinned_host_buf_{nullptr};
    void* device_mapped_ptr_{nullptr};
    bool cuda_active_{false};
    std::atomic<uint64_t> bundles_dispatched_{0};
};

} // namespace av::core
