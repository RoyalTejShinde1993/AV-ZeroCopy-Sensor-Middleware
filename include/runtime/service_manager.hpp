#pragma once

#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace av::runtime {

struct ServiceNode {
    std::string name;
    int priority_stage;
    std::function<bool()> init_fn;
    std::function<void()> start_fn;
    std::function<void()> stop_fn;
};

class FastBootOrchestrator {
public:
    void register_service(ServiceNode node) {
        nodes_.push_back(std::move(node));
    }

    bool boot_all() {
        auto boot_start = std::chrono::steady_clock::now();
        std::cout << "\n[BOOT-UX] Initiating Deterministic AV Stack Bring-Up...\n";

        for (auto& node : nodes_) {
            auto t0 = std::chrono::steady_clock::now();
            if (!node.init_fn()) {
                std::cerr << "[BOOT-UX][FAIL] Initialization failed for: " << node.name << "\n";
                return false;
            }
            node.start_fn();
            auto t1 = std::chrono::steady_clock::now();
            double us = std::chrono::duration<double, std::micro>(t1 - t0).count();

            std::cout << "  -> Stage " << node.priority_stage
                      << " | Service: " << std::left << std::setw(28) << node.name
                      << " | Ready in " << std::fixed << std::setprecision(2) << us << " us\n";
        }

        auto boot_end = std::chrono::steady_clock::now();
        double total_ms = std::chrono::duration<double, std::milli>(boot_end - boot_start).count();
        std::cout << "[BOOT-UX] Full AV Platform Operational in "
                  << std::fixed << std::setprecision(3) << total_ms << " ms\n\n";
        return true;
    }

    void shutdown_all() {
        std::cout << "\n[SHUTDOWN] Gracefully stopping AV services in reverse order...\n";
        for (auto it = nodes_.rbegin(); it != nodes_.rend(); ++it) {
            it->stop_fn();
            std::cout << "  -> Stopped: " << it->name << "\n";
        }
    }

private:
    std::vector<ServiceNode> nodes_;
};

} // namespace av::runtime
