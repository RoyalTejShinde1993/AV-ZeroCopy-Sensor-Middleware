#include "core/gpu_handoff.hpp"
#include "core/lockfree_ring.hpp"
#include "core/serializer.hpp"
#include "core/shm_pool.hpp"
#include "core/time_sync.hpp"
#include "runtime/service_manager.hpp"
#include "sal/sensors.hpp"

#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <thread>
#include <vector>

void pin_thread_to_core(int core_id) {
#ifdef __linux__
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
#else
    (void)core_id;
#endif
}

int main(int argc, char** argv) {
    int duration_sec = (argc > 1) ? std::stoi(argv[1]) : 5;

    constexpr size_t SHM_SLOTS = 256;
    constexpr size_t SHM_SLOT_SIZE = 1024 * 1024; // 1 MB per slot (256 MB Zero-Copy Pool)
    av::core::ZeroCopyShmPool shm_pool("/av_drive_sensor_pool", SHM_SLOTS, SHM_SLOT_SIZE);
    av::core::PtpTimeSynchronizer ptp_sync(25'000'000ULL); // 25ms max multi-sensor alignment skew
    av::core::GpuZeroCopyHandoffEngine gpu_handoff(4 * 1024 * 1024);

    av::sal::DescriptorQueue cam_queue;
    av::sal::DescriptorQueue lidar_queue;
    av::sal::DescriptorQueue can_queue;

    av::sal::FrontCameraSensor camera(101, shm_pool, cam_queue);
    av::sal::RoofLidarSensor lidar(201, shm_pool, lidar_queue);
    av::sal::VehicleCanImuSensor can_imu(301, shm_pool, can_queue);

    std::atomic<bool> pipeline_running{false};
    std::thread fusion_recorder_thread;

    uint64_t total_frames_processed = 0;
    uint64_t total_bytes_recorded = 0;
    uint64_t max_latency_ns = 0;
    uint64_t sum_latency_ns = 0;
    uint64_t sum_sync_skew_ns = 0;
    uint64_t crc_failures = 0;

    av::runtime::FastBootOrchestrator orchestrator;

    orchestrator.register_service({
        "ZeroCopy_SHM_MemoryPool", 0,
        [&]() { return shm_pool.slot_count() == SHM_SLOTS; },
        []() {},
        []() {}
    });

    orchestrator.register_service({
        "GPU_PinnedMemory_Handoff", 0,
        []() { return true; },
        []() {},
        []() {}
    });

    orchestrator.register_service({
        "PTP_Fusion_And_Recorder", 1,
        []() { return true; },
        [&]() {
            pipeline_running.store(true, std::memory_order_release);
            fusion_recorder_thread = std::thread([&]() {
                pin_thread_to_core(1);
                std::ofstream out_file("av_fleet_capture.bin", std::ios::binary | std::ios::trunc);

                auto record_and_release = [&](const av_frame_descriptor_t& desc, bool release_now) {
                    uint64_t now_ns = av::sal::now_monotonic_ns();
                    uint64_t latency_ns = (now_ns >= desc.hw_timestamp_ns)
                                              ? (now_ns - desc.hw_timestamp_ns)
                                              : 0;

                    const uint8_t* payload_ptr = shm_pool.get_slot_ptr(desc.shm_slot_index);
                    uint32_t verify_crc = av::core::compute_crc32(payload_ptr, desc.payload_size_bytes);
                    if (verify_crc != desc.checksum_crc32) {
                        ++crc_failures;
                    }

                    av::core::BinaryPacketHeader hdr{};
                    hdr.sensor_type = static_cast<uint16_t>(desc.sensor_type);
                    hdr.sensor_id = desc.sensor_id;
                    hdr.sequence_id = desc.sequence_id;
                    hdr.hw_timestamp_ns = desc.hw_timestamp_ns;
                    hdr.ingest_timestamp_ns = desc.ingest_timestamp_ns;
                    hdr.payload_length = desc.payload_size_bytes;
                    hdr.payload_crc32 = desc.checksum_crc32;

                    out_file.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
                    out_file.write(reinterpret_cast<const char*>(payload_ptr), desc.payload_size_bytes);

                    if (release_now) {
                        shm_pool.release_slot(desc.shm_slot_index);
                    }

                    ++total_frames_processed;
                    total_bytes_recorded += sizeof(hdr) + desc.payload_size_bytes;
                    sum_latency_ns += latency_ns;
                    if (latency_ns > max_latency_ns) {
                        max_latency_ns = latency_ns;
                    }
                };

                while (pipeline_running.load(std::memory_order_acquire) ||
                       cam_queue.size_approx() > 0 ||
                       lidar_queue.size_approx() > 0 ||
                       can_queue.size_approx() > 0) {
                    bool worked = false;

                    if (auto d = can_queue.try_pop()) {
                        ptp_sync.push_can_imu(*d);
                        record_and_release(*d, true);
                        worked = true;
                    }
                    if (auto d = lidar_queue.try_pop()) {
                        ptp_sync.push_lidar(*d);
                        record_and_release(*d, true);
                        worked = true;
                    }
                    if (auto d = cam_queue.try_pop()) {
                        if (auto bundle = ptp_sync.try_align_with_camera(*d)) {
                            gpu_handoff.submit_synchronized_bundle(
                                *bundle,
                                shm_pool.get_slot_ptr(bundle->camera_frame.shm_slot_index),
                                shm_pool.get_slot_ptr(bundle->lidar_frame.shm_slot_index),
                                shm_pool.get_slot_ptr(bundle->can_imu_frame.shm_slot_index));
                            sum_sync_skew_ns += bundle->max_skew_ns;
                        }
                        record_and_release(*d, true);
                        worked = true;
                    }

                    if (!worked) {
                        std::this_thread::sleep_for(std::chrono::microseconds(100));
                    }
                }
                out_file.flush();
            });
        },
        [&]() {
            pipeline_running.store(false, std::memory_order_release);
            if (fusion_recorder_thread.joinable()) {
                fusion_recorder_thread.join();
            }
        }
    });

    orchestrator.register_service({
        "SAL_FrontCamera_4K", 2,
        [&]() { return camera.init(); },
        [&]() { camera.start(); },
        [&]() { camera.stop(); }
    });

    orchestrator.register_service({
        "SAL_RoofLIDAR_128Ch", 2,
        [&]() { return lidar.init(); },
        [&]() { lidar.start(); },
        [&]() { lidar.stop(); }
    });

    orchestrator.register_service({
        "SAL_VehicleCAN_IMU", 2,
        [&]() { return can_imu.init(); },
        [&]() { can_imu.start(); },
        [&]() { can_imu.stop(); }
    });

    if (!orchestrator.boot_all()) {
        return 1;
    }

    std::cout << "[RUNTIME] Streaming, PTP-aligning & recording multi-sensor data for "
              << duration_sec << " seconds...\n";
    std::this_thread::sleep_for(std::chrono::seconds(duration_sec));

    orchestrator.shutdown_all();

    uint64_t gpu_bundles = gpu_handoff.bundles_dispatched();
    double avg_latency_us = total_frames_processed > 0
                                ? (static_cast<double>(sum_latency_ns) / total_frames_processed) / 1000.0
                                : 0.0;
    double max_latency_us = static_cast<double>(max_latency_ns) / 1000.0;
    double avg_skew_ms = gpu_bundles > 0
                             ? (static_cast<double>(sum_sync_skew_ns) / gpu_bundles) / 1'000'000.0
                             : 0.0;
    double mb_recorded = static_cast<double>(total_bytes_recorded) / (1024.0 * 1024.0);

    std::cout << "\n================ AV PLATFORM TELEMETRY SUMMARY ================\n";
    std::cout << "  POSIX SHM Segment       : " << shm_pool.name() << " (256 MB Zero-Copy)\n";
    std::cout << "  GPU Handoff Backend     : "
              << (gpu_handoff.is_cuda_hardware_backed() ? "CUDA cudaHostAllocMapped" : "Page-Locked mlock DMA Sim")
              << "\n";
    std::cout << "  Camera Frames Pub/Drop  : " << camera.frames_published() << " / " << camera.frames_dropped() << "\n";
    std::cout << "  LIDAR Packets Pub/Drop  : " << lidar.frames_published() << " / " << lidar.frames_dropped() << "\n";
    std::cout << "  CAN/IMU Msgs Pub/Drop   : " << can_imu.frames_published() << " / " << can_imu.frames_dropped() << "\n";
    std::cout << "  PTP Synced GPU Bundles  : " << gpu_bundles << " (Avg Skew: "
              << std::fixed << std::setprecision(2) << avg_skew_ms << " ms)\n";
    std::cout << "  Total Frames Processed  : " << total_frames_processed << "\n";
    std::cout << "  CRC32 Integrity Errors  : " << crc_failures << "\n";
    std::cout << "  Avg End-to-End Latency  : " << std::fixed << std::setprecision(2) << avg_latency_us << " us\n";
    std::cout << "  Max End-to-End Latency  : " << std::fixed << std::setprecision(2) << max_latency_us << " us\n";
    std::cout << "  Fleet Log Written       : av_fleet_capture.bin (" << std::setprecision(2) << mb_recorded << " MB)\n";
    std::cout << "===============================================================\n";

    return 0;
}
