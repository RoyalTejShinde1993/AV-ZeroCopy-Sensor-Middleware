#pragma once

#include "abi/sensor_c_abi.h"
#include <cstdint>
#include <deque>
#include <optional>

namespace av::core {

struct SynchronizedSensorBundle {
    av_frame_descriptor_t camera_frame;
    av_frame_descriptor_t lidar_frame;
    av_frame_descriptor_t can_imu_frame;
    uint64_t max_skew_ns{0};
};

class PtpTimeSynchronizer {
public:
    explicit PtpTimeSynchronizer(uint64_t max_allowed_skew_ns = 15'000'000ULL)
        : max_allowed_skew_ns_(max_allowed_skew_ns) {}

    void push_lidar(const av_frame_descriptor_t& desc) {
        if (lidar_history_.size() >= MAX_HISTORY) lidar_history_.pop_front();
        lidar_history_.push_back(desc);
    }

    void push_can_imu(const av_frame_descriptor_t& desc) {
        if (can_history_.size() >= MAX_HISTORY) can_history_.pop_front();
        can_history_.push_back(desc);
    }

    std::optional<SynchronizedSensorBundle> try_align_with_camera(const av_frame_descriptor_t& cam_desc) {
        auto best_lidar = find_nearest(lidar_history_, cam_desc.hw_timestamp_ns);
        auto best_can   = find_nearest(can_history_, cam_desc.hw_timestamp_ns);

        if (!best_lidar || !best_can) {
            return std::nullopt;
        }

        uint64_t lidar_skew = abs_diff(cam_desc.hw_timestamp_ns, best_lidar->hw_timestamp_ns);
        uint64_t can_skew   = abs_diff(cam_desc.hw_timestamp_ns, best_can->hw_timestamp_ns);
        uint64_t worst_skew = (lidar_skew > can_skew) ? lidar_skew : can_skew;

        if (worst_skew > max_allowed_skew_ns_) {
            return std::nullopt;
        }

        SynchronizedSensorBundle bundle{};
        bundle.camera_frame  = cam_desc;
        bundle.lidar_frame   = *best_lidar;
        bundle.can_imu_frame = *best_can;
        bundle.max_skew_ns   = worst_skew;
        return bundle;
    }

private:
    static constexpr size_t MAX_HISTORY = 64;
    uint64_t max_allowed_skew_ns_;
    std::deque<av_frame_descriptor_t> lidar_history_;
    std::deque<av_frame_descriptor_t> can_history_;

    static uint64_t abs_diff(uint64_t a, uint64_t b) noexcept {
        return (a >= b) ? (a - b) : (b - a);
    }

    static std::optional<av_frame_descriptor_t> find_nearest(
        const std::deque<av_frame_descriptor_t>& queue, uint64_t target_ns) noexcept {
        if (queue.empty()) return std::nullopt;

        const av_frame_descriptor_t* best = nullptr;
        uint64_t best_diff = UINT64_MAX;

        for (const auto& item : queue) {
            uint64_t diff = abs_diff(item.hw_timestamp_ns, target_ns);
            if (diff < best_diff) {
                best_diff = diff;
                best = &item;
            }
        }
        return best ? std::optional<av_frame_descriptor_t>(*best) : std::nullopt;
    }
};

} // namespace av::core
