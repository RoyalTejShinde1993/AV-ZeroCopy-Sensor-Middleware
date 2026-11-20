#pragma once

#include "abi/sensor_c_abi.h"
#include "core/lockfree_ring.hpp"
#include "core/serializer.hpp"
#include "core/shm_pool.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

namespace av::sal {

inline uint64_t now_monotonic_ns() noexcept {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

using DescriptorQueue = av::core::LockFreeSpscRing<av_frame_descriptor_t, 1024>;

class ISensor {
public:
    virtual ~ISensor() = default;
    virtual bool init() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual std::string name() const = 0;
    virtual uint64_t frames_published() const noexcept = 0;
    virtual uint64_t frames_dropped() const noexcept = 0;
};

class SimulatedSensorBase : public ISensor {
public:
    SimulatedSensorBase(std::string sensor_name,
                        uint32_t sensor_id,
                        av_sensor_type_t type,
                        size_t payload_bytes,
                        std::chrono::microseconds period,
                        av::core::ZeroCopyShmPool& shm_pool,
                        DescriptorQueue& output_queue)
        : name_(std::move(sensor_name)),
          sensor_id_(sensor_id),
          type_(type),
          payload_bytes_(payload_bytes),
          period_(period),
          shm_pool_(shm_pool),
          output_queue_(output_queue) {}

    ~SimulatedSensorBase() override { stop(); }

    bool init() override {
        return payload_bytes_ <= shm_pool_.slot_size();
    }

    void start() override {
        running_.store(true, std::memory_order_release);
        worker_ = std::thread(&SimulatedSensorBase::run_loop, this);
    }

    void stop() override {
        if (running_.exchange(false, std::memory_order_acq_rel)) {
            if (worker_.joinable()) {
                worker_.join();
            }
        }
    }

    std::string name() const override { return name_; }
    uint64_t frames_published() const noexcept override { return published_.load(); }
    uint64_t frames_dropped() const noexcept override { return dropped_.load(); }

protected:
    virtual void fill_sensor_payload(uint8_t* dst, size_t size, uint64_t seq) noexcept = 0;

private:
    void run_loop() {
        uint64_t seq = 0;
        auto next_wakeup = std::chrono::steady_clock::now();

        while (running_.load(std::memory_order_acquire)) {
            next_wakeup += period_;
            int32_t slot = shm_pool_.acquire_slot();
            if (slot < 0) {
                dropped_.fetch_add(1, std::memory_order_relaxed);
                std::this_thread::sleep_until(next_wakeup);
                continue;
            }

            uint8_t* ptr = shm_pool_.get_slot_ptr(static_cast<uint32_t>(slot));
            uint64_t hw_ts = now_monotonic_ns();
            fill_sensor_payload(ptr, payload_bytes_, seq);

            av_frame_descriptor_t desc{};
            desc.sequence_id = seq++;
            desc.hw_timestamp_ns = hw_ts;
            desc.ingest_timestamp_ns = now_monotonic_ns();
            desc.sensor_id = sensor_id_;
            desc.sensor_type = static_cast<uint32_t>(type_);
            desc.shm_slot_index = static_cast<uint32_t>(slot);
            desc.payload_size_bytes = static_cast<uint32_t>(payload_bytes_);
            desc.checksum_crc32 = av::core::compute_crc32(ptr, payload_bytes_);

            if (!output_queue_.try_push(desc)) {
                shm_pool_.release_slot(static_cast<uint32_t>(slot));
                dropped_.fetch_add(1, std::memory_order_relaxed);
            } else {
                published_.fetch_add(1, std::memory_order_relaxed);
            }

            std::this_thread::sleep_until(next_wakeup);
        }
    }

    std::string name_;
    uint32_t sensor_id_;
    av_sensor_type_t type_;
    size_t payload_bytes_;
    std::chrono::microseconds period_;
    av::core::ZeroCopyShmPool& shm_pool_;
    DescriptorQueue& output_queue_;
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> published_{0};
    std::atomic<uint64_t> dropped_{0};
    std::thread worker_;
};

class FrontCameraSensor final : public SimulatedSensorBase {
public:
    FrontCameraSensor(uint32_t id, av::core::ZeroCopyShmPool& pool, DescriptorQueue& q)
        : SimulatedSensorBase("Front_Camera_4K_30FPS", id, AV_SENSOR_CAMERA,
                              512 * 1024,
                              std::chrono::microseconds(33333), pool, q) {}

protected:
    void fill_sensor_payload(uint8_t* dst, size_t size, uint64_t seq) noexcept override {
        std::memset(dst, static_cast<int>(seq & 0xFF), size);
        std::memcpy(dst, &seq, sizeof(seq));
    }
};

class RoofLidarSensor final : public SimulatedSensorBase {
public:
    RoofLidarSensor(uint32_t id, av::core::ZeroCopyShmPool& pool, DescriptorQueue& q)
        : SimulatedSensorBase("Roof_LIDAR_128Ch_20Hz", id, AV_SENSOR_LIDAR,
                              256 * 1024,
                              std::chrono::microseconds(50000), pool, q) {}

protected:
    void fill_sensor_payload(uint8_t* dst, size_t size, uint64_t seq) noexcept override {
        float* points = reinterpret_cast<float*>(dst);
        size_t floats = size / sizeof(float);
        for (size_t i = 0; i < floats; i += 64) {
            points[i] = static_cast<float>(seq % 100) + 0.25f;
        }
    }
};

class VehicleCanImuSensor final : public SimulatedSensorBase {
public:
    VehicleCanImuSensor(uint32_t id, av::core::ZeroCopyShmPool& pool, DescriptorQueue& q)
        : SimulatedSensorBase("Vehicle_CAN_IMU_200Hz", id, AV_SENSOR_CAN,
                              256,
                              std::chrono::microseconds(5000), pool, q) {}

protected:
    void fill_sensor_payload(uint8_t* dst, size_t size, uint64_t seq) noexcept override {
        std::memset(dst, 0, size);
        double velocity_mps = 18.5 + (seq % 10) * 0.1;
        double yaw_rate_rps = 0.012;
        std::memcpy(dst, &velocity_mps, sizeof(double));
        std::memcpy(dst + 8, &yaw_rate_rps, sizeof(double));
    }
};

} // namespace av::sal
