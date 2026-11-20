#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/statvfs.h>
#include <unistd.h>
#include <vector>

namespace av::core {

class ZeroCopyShmPool {
public:
    ZeroCopyShmPool(std::string shm_name, size_t slot_count, size_t slot_size)
        : shm_name_(std::move(shm_name)),
          slot_count_(slot_count),
          slot_size_(slot_size),
          total_bytes_(slot_count * slot_size),
          ref_counts_(slot_count) {
        for (size_t i = 0; i < slot_count_; ++i) {
            ref_counts_[i].store(0, std::memory_order_relaxed);
        }

        // Check if /dev/shm has sufficient free quota (Docker/Codespaces defaults to 64 MB)
        bool use_posix_shm = false;
        struct statvfs vfs{};
        if (statvfs("/dev/shm", &vfs) == 0) {
            size_t avail_bytes = static_cast<size_t>(vfs.f_bavail) * static_cast<size_t>(vfs.f_frsize);
            if (avail_bytes > total_bytes_ + (8 * 1024 * 1024)) {
                use_posix_shm = true;
            }
        }

        if (use_posix_shm) {
            shm_unlink(shm_name_.c_str());
            fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
            if (fd_ >= 0 && ftruncate(fd_, static_cast<off_t>(total_bytes_)) == 0) {
                base_ptr_ = static_cast<uint8_t*>(
                    mmap(nullptr, total_bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));
                if (base_ptr_ != MAP_FAILED) {
                    posix_shm_active_ = true;
                }
            }
        }

        // Fallback to MAP_SHARED | MAP_ANONYMOUS if /dev/shm is container-restricted
        if (!posix_shm_active_) {
            if (fd_ >= 0) {
                close(fd_);
                fd_ = -1;
            }
            base_ptr_ = static_cast<uint8_t*>(
                mmap(nullptr, total_bytes_, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));
            if (base_ptr_ == MAP_FAILED) {
                throw std::runtime_error("Failed to mmap ZeroCopyShmPool");
            }
        }

        // Pre-fault pages to eliminate minor page faults during real-time execution
        std::memset(base_ptr_, 0, total_bytes_);
    }

    ~ZeroCopyShmPool() {
        if (base_ptr_ && base_ptr_ != MAP_FAILED) {
            munmap(base_ptr_, total_bytes_);
        }
        if (fd_ >= 0) {
            close(fd_);
        }
        if (posix_shm_active_) {
            shm_unlink(shm_name_.c_str());
        }
    }

    ZeroCopyShmPool(const ZeroCopyShmPool&) = delete;
    ZeroCopyShmPool& operator=(const ZeroCopyShmPool&) = delete;

    int32_t acquire_slot() noexcept {
        for (size_t attempt = 0; attempt < slot_count_; ++attempt) {
            size_t idx = next_hint_.fetch_add(1, std::memory_order_relaxed) % slot_count_;
            uint32_t expected = 0;
            if (ref_counts_[idx].compare_exchange_strong(
                    expected, 1, std::memory_order_acquire, std::memory_order_relaxed)) {
                return static_cast<int32_t>(idx);
            }
        }
        return -1;
    }

    void add_ref(uint32_t slot_idx) noexcept {
        if (slot_idx < slot_count_) {
            ref_counts_[slot_idx].fetch_add(1, std::memory_order_relaxed);
        }
    }

    void release_slot(uint32_t slot_idx) noexcept {
        if (slot_idx < slot_count_) {
            ref_counts_[slot_idx].fetch_sub(1, std::memory_order_release);
        }
    }

    uint8_t* get_slot_ptr(uint32_t slot_idx) noexcept {
        return base_ptr_ + (static_cast<size_t>(slot_idx) * slot_size_);
    }

    const uint8_t* get_slot_ptr(uint32_t slot_idx) const noexcept {
        return base_ptr_ + (static_cast<size_t>(slot_idx) * slot_size_);
    }

    size_t slot_size() const noexcept { return slot_size_; }
    size_t slot_count() const noexcept { return slot_count_; }
    const std::string& name() const noexcept { return shm_name_; }
    bool is_posix_shm_backed() const noexcept { return posix_shm_active_; }

private:
    std::string shm_name_;
    size_t slot_count_;
    size_t slot_size_;
    size_t total_bytes_;
    int fd_{-1};
    bool posix_shm_active_{false};
    uint8_t* base_ptr_{nullptr};
    std::atomic<size_t> next_hint_{0};
    std::vector<std::atomic<uint32_t>> ref_counts_;
};

} // namespace av::core
