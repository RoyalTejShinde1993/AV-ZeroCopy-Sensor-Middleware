#pragma once

#include <atomic>
#include <array>
#include <cstddef>
#include <optional>
#include <new>

namespace av::core {
constexpr size_t CACHE_LINE_SIZE = 64;
/*#ifdef __cpp_lib_hardware_interference_size
    constexpr size_t CACHE_LINE_SIZE = std::hardware_destructive_interference_size;
#else
    constexpr size_t CACHE_LINE_SIZE = 64;
#endif*/

template <typename T, size_t Capacity>
class LockFreeSpscRing {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of 2");

public:
    LockFreeSpscRing() : head_(0), tail_(0) {}

    bool try_push(const T& item) noexcept {
        const size_t current_tail = tail_.load(std::memory_order_relaxed);
        const size_t next_tail = (current_tail + 1) & (Capacity - 1);

        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Ring full
        }

        buffer_[current_tail] = item;
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    std::optional<T> try_pop() noexcept {
        const size_t current_head = head_.load(std::memory_order_relaxed);

        if (current_head == tail_.load(std::memory_order_acquire)) {
            return std::nullopt; // Ring empty
        }

        T item = buffer_[current_head];
        head_.store((current_head + 1) & (Capacity - 1), std::memory_order_release);
        return item;
    }

    size_t size_approx() const noexcept {
        const size_t t = tail_.load(std::memory_order_acquire);
        const size_t h = head_.load(std::memory_order_acquire);
        return (t - h) & (Capacity - 1);
    }

private:
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> head_;
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> tail_;
    alignas(CACHE_LINE_SIZE) std::array<T, Capacity> buffer_{};
};

} // namespace av::core
