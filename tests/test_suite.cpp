#include "core/gpu_handoff.hpp"
#include "core/lockfree_ring.hpp"
#include "core/serializer.hpp"
#include "core/shm_pool.hpp"
#include "core/time_sync.hpp"

#include <cassert>
#include <cstring>
#include <iostream>

int main() {
    std::cout << "[TEST] Running AV Middleware Verification Suite...\n";

    // 1. Test Lock-Free SPSC Ring Buffer
    av::core::LockFreeSpscRing<int, 4> ring;
    assert(ring.try_push(10));
    assert(ring.try_push(20));
    assert(ring.try_push(30));
    assert(!ring.try_push(40)); // Full at Capacity - 1
    assert(ring.try_pop().value() == 10);
    assert(ring.try_pop().value() == 20);
    assert(ring.try_pop().value() == 30);
    assert(!ring.try_pop().has_value());
    std::cout << "  [PASS] LockFreeSpscRing boundary & ordering\n";

    // 2. Test POSIX Zero-Copy SHM Pool Lifecycle
    av::core::ZeroCopyShmPool pool("/av_test_shm_pool", 4, 4096);
    int32_t s0 = pool.acquire_slot();
    int32_t s1 = pool.acquire_slot();
    assert(s0 >= 0 && s1 >= 0 && s0 != s1);
    uint8_t* ptr = pool.get_slot_ptr(static_cast<uint32_t>(s0));
    const char* msg = "NVIDIA_DRIVE_SHM_ZERO_COPY";
    std::memcpy(ptr, msg, std::strlen(msg));
    uint32_t crc = av::core::compute_crc32(ptr, std::strlen(msg));
    assert(crc != 0);
    pool.release_slot(static_cast<uint32_t>(s0));
    pool.release_slot(static_cast<uint32_t>(s1));
    std::cout << "  [PASS] POSIX SHM zero-copy allocation & CRC32 integrity\n";

    // 3. Test PTP Time Synchronizer & GPU Handoff
    av::core::PtpTimeSynchronizer sync(10'000'000ULL);
    av_frame_descriptor_t lidar_d{};
    lidar_d.hw_timestamp_ns = 100'000'000ULL;
    lidar_d.payload_size_bytes = 1024;
    sync.push_lidar(lidar_d);

    av_frame_descriptor_t can_d{};
    can_d.hw_timestamp_ns = 102'000'000ULL;
    can_d.payload_size_bytes = 256;
    sync.push_can_imu(can_d);

    av_frame_descriptor_t cam_d{};
    cam_d.hw_timestamp_ns = 104'000'000ULL;
    cam_d.payload_size_bytes = 4096;

    auto bundle = sync.try_align_with_camera(cam_d);
    assert(bundle.has_value());
    assert(bundle->max_skew_ns == 4'000'000ULL);

    av::core::GpuZeroCopyHandoffEngine gpu(65536);
    auto tensor = gpu.submit_synchronized_bundle(*bundle, nullptr, nullptr, nullptr);
    assert(tensor.total_tensor_bytes == (1024 + 256 + 4096));
    assert(gpu.bundles_dispatched() == 1);
    std::cout << "  [PASS] PTP nearest-neighbor alignment & GPU pinned handoff\n";

    std::cout << "[TEST] All assertions passed successfully!\n";
    return 0;
}
