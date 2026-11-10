#pragma once

#include "lumo_deploy/types.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <thread>
#include <vector>

namespace lumo_deploy {

// Single producer (control loop), single consumer (disk worker).
// Update never waits for disk or for the worker; Shutdown drains and joins.
class CsvRecorder {
public:
    using Clock = std::chrono::steady_clock;
    explicit CsvRecorder(std::filesystem::path directory, std::size_t capacity = 4096);
    ~CsvRecorder();
    CsvRecorder(const CsvRecorder &) = delete;
    CsvRecorder &operator=(const CsvRecorder &) = delete;

    void Update(const OperatorInput &input, const RobotState &state,
                const JointCommand &command, Mode mode, bool command_written,
                Clock::time_point now = Clock::now());
    void Shutdown();
    bool Busy() const { return busy_.load(std::memory_order_acquire); }
    bool Failed() const { return failed_.load(std::memory_order_acquire); }
    std::size_t Dropped() const { return dropped_.load(); }

private:
    struct Sample {
        double time_s = 0;
        std::size_t index = 0;
        std::size_t dropped = 0;
        Mode mode = Mode::kDamping;
        bool command_written = false;
        RobotState state;
        JointCommand command;
    };
    bool Pop(Sample *sample);
    void Worker();

    std::filesystem::path directory_;
    std::vector<Sample> queue_;
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
    std::atomic<std::size_t> dropped_{0};
    std::atomic<bool> busy_{false};
    std::atomic<bool> start_requested_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> shutdown_{false};
    std::atomic<bool> failed_{false};
    // Producer-only fields, never accessed by the disk worker.
    bool recording_ = false;
    bool start_down_ = false;
    bool back_down_ = false;
    std::size_t sample_index_ = 0;
    Clock::time_point start_time_{};
    std::thread worker_;
};

}  // namespace lumo_deploy
