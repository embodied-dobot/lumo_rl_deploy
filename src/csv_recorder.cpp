#include "lumo_deploy/csv_recorder.hpp"
#include "lumo_deploy/config.hpp"
#include "lumo_deploy/runtime_alert.hpp"

#include <cstdio>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>

namespace lumo_deploy {
namespace {

std::string Header()
{
    std::ostringstream out;
    out << "time_s,sample_index,dropped_samples,mode,command_written";
    for (const auto &name : JointNames()) {
        for (const auto *field : {"q_rad", "dq_rad_s", "tau_est_nm", "q_cmd_rad", "dq_cmd_rad_s",
                                 "tau_ff_cmd_nm", "kp", "kd"}) {
            out << ',' << name << '/' << field;
        }
    }
    out << '\n';
    return out.str();
}

bool Write(std::FILE *file, const std::string &line)
{
    return std::fwrite(line.data(), 1, line.size(), file) == line.size();
}

}  // namespace

CsvRecorder::CsvRecorder(std::filesystem::path directory, std::size_t capacity)
    : directory_(std::move(directory)), queue_(capacity)
{
    static_assert(std::atomic<std::size_t>::is_always_lock_free);
    if (capacity == 0) {
        throw std::invalid_argument("CSV queue capacity must be positive");
    }
    worker_ = std::thread(&CsvRecorder::Worker, this);
}

CsvRecorder::~CsvRecorder() { Shutdown(); }

void CsvRecorder::Shutdown()
{
    recording_ = false;
    stop_requested_.store(true, std::memory_order_release);
    shutdown_.store(true, std::memory_order_release);
    if (worker_.joinable()) {
        worker_.join();
    }
}

void CsvRecorder::Update(const OperatorInput &input, const RobotState &state,
                         const JointCommand &command, Mode mode, bool command_written,
                         Clock::time_point now)
{
    const bool start = input.start && !start_down_;
    const bool back = input.back && !back_down_;
    start_down_ = input.start;
    back_down_ = input.back;
    if (shutdown_.load(std::memory_order_acquire)) {
        return;
    }
    if (Failed()) {
        recording_ = false;
        stop_requested_.store(true, std::memory_order_release);
        return;
    }
    // Back wins if both keys are pressed. Held keys never create repeated files.
    if (back || input.back) {
        recording_ = false;
        stop_requested_.store(true, std::memory_order_release);
    } else if (start && !Busy()) {
        dropped_.store(0);
        sample_index_ = 0;
        start_time_ = now;
        stop_requested_.store(false, std::memory_order_release);
        busy_.store(true, std::memory_order_release);
        start_requested_.store(true, std::memory_order_release);
        recording_ = true;
    }
    if (!recording_) {
        return;
    }
    const auto index = sample_index_++;
    const auto head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) >= queue_.size()) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    Sample &sample = queue_[head % queue_.size()];
    sample.time_s = std::chrono::duration<double>(now - start_time_).count();
    sample.index = index;
    sample.dropped = dropped_.load(std::memory_order_relaxed);
    sample.mode = mode;
    sample.command_written = command_written;
    sample.state = state;
    sample.command = command;
    head_.store(head + 1, std::memory_order_release);
}

bool CsvRecorder::Pop(Sample *sample)
{
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
        return false;
    }
    *sample = queue_[tail % queue_.size()];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
}

void CsvRecorder::Worker()
{
    std::FILE *file = nullptr;
    std::filesystem::path path;
    std::size_t rows = 0;
    bool session_open = false;
    auto last_flush = Clock::now();
    auto report_error = [&](const std::string &message) {
        if (!failed_.exchange(true)) {
            PrintAlert("[ERROR] CSV 记录失败 / CSV RECORDING FAILED",
                       message + "\nRecording is disabled; robot control continues.\nPartial CSV: " +
                           (path.empty() ? directory_.string() : path.string()), true);
        }
    };
    try {
        while (true) {
            if (start_requested_.exchange(false, std::memory_order_acq_rel)) {
                session_open = true;
                std::filesystem::create_directories(directory_);
                const auto wall_time = std::time(nullptr);
                std::tm utc{};
                ::gmtime_r(&wall_time, &utc);
                std::ostringstream filename;
                filename << "lumo_" << std::put_time(&utc, "%Y%m%dT%H%M%SZ") << "_XXXXXX.csv";
                auto pattern = (directory_ / filename.str()).string();
                const int fd = ::mkstemps(pattern.data(), 4);
                path = pattern;
                if (fd < 0) {
                    throw std::runtime_error("Cannot create CSV in " + directory_.string() + ": " + std::strerror(errno));
                }
                // Allow a desktop PlotJuggler process to read logs recorded with sudo.
                if (::fchmod(fd, 0644) != 0) {
                    ::close(fd);
                    throw std::runtime_error("Cannot set CSV read permissions");
                }
                file = ::fdopen(fd, "w");
                if (!file) {
                    ::close(fd);
                    throw std::runtime_error("Cannot open CSV stream");
                }
                rows = 0;
                last_flush = Clock::now();
                if (!Write(file, Header()) || std::fflush(file) != 0) {
                    report_error("Cannot write CSV header (check disk space/permissions).");
                } else {
                    std::cout << "[CSV] Recording started / 开始记录: " << path << std::endl;
                }
            }
            Sample sample;
            std::size_t batch = 0;
            while (session_open && batch++ < 256 && Pop(&sample)) {
                if (!file || Failed()) {
                    continue;
                }
                std::ostringstream row;
                row.imbue(std::locale::classic());
                row << std::setprecision(9) << sample.time_s << ',' << sample.index << ','
                    << sample.dropped << ',' << static_cast<int>(sample.mode) << ','
                    << static_cast<int>(sample.command_written);
                for (std::size_t i = 0; i < kJointCount; ++i) {
                    row << ',' << sample.state.q[i] << ',' << sample.state.dq[i] << ','
                        << sample.state.tau_est[i] << ',' << sample.command.q[i] << ','
                        << sample.command.dq[i] << ',' << sample.command.tau[i] << ','
                        << sample.command.kp[i] << ',' << sample.command.kd[i];
                }
                row << '\n';
                if (!Write(file, row.str())) {
                    report_error("CSV write failed (check disk space). Data may be incomplete.");
                } else {
                    ++rows;
                }
            }
            if (file && !Failed() && Clock::now() - last_flush >= std::chrono::seconds(1)) {
                if (std::fflush(file) != 0) {
                    report_error("CSV flush failed (check disk space). Data may be incomplete.");
                }
                last_flush = Clock::now();
            }
            // Stop is published only after the producer has stopped enqueueing.
            const bool stopping = stop_requested_.load(std::memory_order_acquire);
            const bool empty = tail_.load() == head_.load(std::memory_order_acquire);
            if (stopping && empty && session_open) {
                if (file) {
                    const int result = std::fclose(file);
                    file = nullptr;
                    if (result != 0) {
                        report_error("CSV close/flush failed. Data may be incomplete.");
                    }
                }
                if (!Failed()) {
                    std::cout << "[CSV] Saved / 已保存: " << path << ", rows=" << rows
                              << ", dropped=" << Dropped() << std::endl;
                    if (Dropped() != 0) {
                        PrintAlert("[WARNING] CSV 样本丢失 / CSV SAMPLES DROPPED",
                                   "Disk writer could not keep up; inspect sample_index/time_s gaps. "
                                   "Dropped samples: " + std::to_string(Dropped()), false);
                    }
                }
                session_open = false;
                busy_.store(false, std::memory_order_release);
            }
            if (shutdown_.load(std::memory_order_acquire) && !Busy() && empty) {
                break;
            }
            if (empty) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
    } catch (const std::exception &exception) {
        report_error(exception.what());
        if (file) {
            std::fclose(file);
        }
        busy_.store(false, std::memory_order_release);
    }
}

}  // namespace lumo_deploy
