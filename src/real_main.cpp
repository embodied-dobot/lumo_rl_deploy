#include "lumo_deploy/config.hpp"
#include "lumo_deploy/csv_recorder.hpp"
#include "lumo_deploy/controller.hpp"
#include "lumo_deploy/types.hpp"

#include "lumo_deploy/sdk_adapter.hpp"
#include "lumo_deploy/runtime_alert.hpp"

#include <dobot/sdk/session.hpp>
#include <gamepad_statePubSubTypes.hpp>
#include <imu_statePubSubTypes.hpp>
#include <lower_cmdPubSubTypes.hpp>
#include <lower_statePubSubTypes.hpp>
#include <upper_cmdPubSubTypes.hpp>
#include <upper_statePubSubTypes.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/file.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

using lumo_deploy::Controller;
using lumo_deploy::JointCommand;
using lumo_deploy::Mode;
using lumo_deploy::OperatorInput;
using lumo_deploy::RobotState;
using lumo_deploy::kJointCount;

constexpr auto kControlPeriod = std::chrono::microseconds(2000);
volatile std::sig_atomic_t g_stop = 0;

void SignalHandler(int signal) { g_stop = signal; }

using lumo_deploy::ExitReport;
using lumo_deploy::PrintAlert;

struct ProcessPaths {
    std::filesystem::path executable;
    std::filesystem::path pid_file;
    std::filesystem::path log_file;
};

bool ResolveProcessPaths(ProcessPaths *paths, std::string *error)
{
    if (!paths) {
        if (error) {
            *error = "process path output is null";
        }
        return false;
    }
    std::error_code path_error;
    paths->executable = std::filesystem::read_symlink("/proc/self/exe", path_error);
    if (path_error || paths->executable.empty()) {
        if (error) {
            *error = "failed to resolve /proc/self/exe: " + path_error.message();
        }
        return false;
    }
    const std::filesystem::path directory = paths->executable.parent_path();
    paths->pid_file = directory / "lumo_real.pid";
    paths->log_file = directory / "lumo_real.log";
    if (error) {
        error->clear();
    }
    return true;
}

std::optional<pid_t> ReadPid(const std::filesystem::path &pid_file)
{
    std::ifstream input(pid_file);
    long long value = 0;
    if (!(input >> value) || value <= 0) {
        return std::nullopt;
    }
    return static_cast<pid_t>(value);
}

bool MatchesExecutable(pid_t pid, const std::filesystem::path &expected)
{
    std::error_code error;
    const std::filesystem::path actual =
        std::filesystem::read_symlink("/proc/" + std::to_string(pid) + "/exe", error);
    if (error) {
        return false;
    }
    return actual == expected || actual.string() == expected.string() + " (deleted)";
}

class BackgroundSession {
public:
    enum class StartResult {
        kChild,
        kParent,
        kError,
    };

    ~BackgroundSession()
    {
        if (!active_) {
            return;
        }
        std::error_code ignored;
        std::filesystem::remove(pid_file_, ignored);
        if (pid_fd_ >= 0) {
            ::flock(pid_fd_, LOCK_UN);
            ::close(pid_fd_);
        }
    }

    StartResult Start(const ProcessPaths &paths, std::string *error)
    {
        pid_file_ = paths.pid_file;
        pid_fd_ = ::open(paths.pid_file.c_str(), O_CREAT | O_RDWR, 0644);
        if (pid_fd_ < 0) {
            return Fail(error, "failed to open PID file: " + std::string(std::strerror(errno)));
        }
        if (::flock(pid_fd_, LOCK_EX | LOCK_NB) != 0) {
            ::close(pid_fd_);
            pid_fd_ = -1;
            return Fail(error, "another lumo_real background process is already running");
        }

        const int log_fd = ::open(paths.log_file.c_str(), O_CREAT | O_WRONLY | O_APPEND, 0644);
        if (log_fd < 0) {
            return Fail(error, "failed to open background log: " +
                                   std::string(std::strerror(errno)));
        }
        const int null_fd = ::open("/dev/null", O_RDONLY);
        if (null_fd < 0) {
            ::close(log_fd);
            return Fail(error, "failed to open /dev/null: " +
                                   std::string(std::strerror(errno)));
        }

        int ready_pipe[2]{};
        if (::pipe(ready_pipe) != 0) {
            ::close(null_fd);
            ::close(log_fd);
            return Fail(error, "failed to create startup pipe: " +
                                   std::string(std::strerror(errno)));
        }

        std::cout.flush();
        std::cerr.flush();
        const pid_t child = ::fork();
        if (child < 0) {
            ::close(ready_pipe[0]);
            ::close(ready_pipe[1]);
            ::close(null_fd);
            ::close(log_fd);
            return Fail(error, "failed to fork background process: " +
                                   std::string(std::strerror(errno)));
        }

        if (child > 0) {
            ::close(ready_pipe[1]);
            ::close(null_fd);
            ::close(log_fd);
            char ready = 0;
            const ssize_t count = ::read(ready_pipe[0], &ready, sizeof(ready));
            ::close(ready_pipe[0]);
            ::close(pid_fd_);
            pid_fd_ = -1;
            if (count != 1 || ready != 1) {
                return Fail(error, "background child failed during startup");
            }
            std::cout << "lumo_real started in background, PID=" << child << '\n'
                      << "log: " << paths.log_file << '\n';
            if (error) {
                error->clear();
            }
            return StartResult::kParent;
        }

        ::close(ready_pipe[0]);
        if (::setsid() < 0 || ::dup2(null_fd, STDIN_FILENO) < 0 ||
            ::dup2(log_fd, STDOUT_FILENO) < 0 || ::dup2(log_fd, STDERR_FILENO) < 0) {
            const char failed = 0;
            const ssize_t ignored = ::write(ready_pipe[1], &failed, sizeof(failed));
            static_cast<void>(ignored);
            ::_exit(1);
        }
        ::close(null_fd);
        ::close(log_fd);
        if (::ftruncate(pid_fd_, 0) != 0 || ::lseek(pid_fd_, 0, SEEK_SET) < 0 ||
            ::dprintf(pid_fd_, "%ld\n", static_cast<long>(::getpid())) < 0) {
            const char failed = 0;
            const ssize_t ignored = ::write(ready_pipe[1], &failed, sizeof(failed));
            static_cast<void>(ignored);
            ::_exit(1);
        }
        active_ = true;
        std::signal(SIGHUP, SIG_IGN);
        const char ready = 1;
        if (::write(ready_pipe[1], &ready, sizeof(ready)) != 1) {
            ::_exit(1);
        }
        ::close(ready_pipe[1]);
        if (error) {
            error->clear();
        }
        return StartResult::kChild;
    }

private:
    StartResult Fail(std::string *error, std::string message)
    {
        if (pid_fd_ >= 0) {
            ::flock(pid_fd_, LOCK_UN);
            ::close(pid_fd_);
            pid_fd_ = -1;
        }
        if (error) {
            *error = std::move(message);
        }
        return StartResult::kError;
    }

    std::filesystem::path pid_file_;
    int pid_fd_ = -1;
    bool active_ = false;
};

int PrintBackgroundStatus(const ProcessPaths &paths)
{
    const std::optional<pid_t> pid = ReadPid(paths.pid_file);
    if (!pid || !MatchesExecutable(*pid, paths.executable)) {
        std::cout << "lumo_real is not running in background\n";
        return 1;
    }
    std::cout << "lumo_real is running in background, PID=" << *pid << '\n'
              << "log: " << paths.log_file << '\n';
    return 0;
}

int StopBackground(const ProcessPaths &paths)
{
    const std::optional<pid_t> pid = ReadPid(paths.pid_file);
    if (!pid) {
        std::cout << "lumo_real background PID file was not found\n";
        return 1;
    }
    if (!MatchesExecutable(*pid, paths.executable)) {
        std::error_code ignored;
        std::filesystem::remove(paths.pid_file, ignored);
        std::cerr << "stale lumo_real PID file removed; no matching process was killed\n";
        return 1;
    }
    if (::kill(*pid, SIGTERM) != 0) {
        std::cerr << "failed to send SIGTERM: " << std::strerror(errno) << '\n';
        return 1;
    }
    for (int i = 0; i < 50; ++i) {
        if (!MatchesExecutable(*pid, paths.executable)) {
            std::error_code ignored;
            std::filesystem::remove(paths.pid_file, ignored);
            std::cout << "lumo_real stopped\n";
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cerr << "lumo_real did not stop after SIGTERM; use the emergency stop and inspect PID "
              << *pid << '\n';
    return 1;
}

class Hardware {
public:
    ~Hardware() { session_.Shutdown(); }

    bool Initialize(lumo_deploy::SdkModel model, const dobot::sdk::SessionConfig &config,
                    std::string *error)
    {
        const auto topics = lumo_deploy::TopicsForModel(model);
        if (!session_.Init(topics.profile, config, error)) {
            return false;
        }
        namespace msg = lumo_deploy::sdk_msg;
        auto lower = session_.Subscribe<msg::LowerState_PubSubType>(topics.lower_state);
        auto upper = session_.Subscribe<msg::UpperState_PubSubType>(topics.upper_state);
        auto imu = session_.Subscribe<msg::IMUState_PubSubType>(topics.imu);
        auto gamepad = session_.Subscribe<msg::GamepadState_PubSubType>(topics.gamepad);
        lower_writer_ = session_.Publish<msg::LowerCmd_PubSubType>(topics.lower_cmd);
        upper_writer_ = session_.Publish<msg::UpperCmd_PubSubType>(topics.upper_cmd);
        if (!lower.valid() || !upper.valid() || !imu.valid() || !gamepad.valid() ||
            !lower_writer_.valid() || !upper_writer_.valid()) {
            return Fail(error, "failed to create Dobot SDK state/command endpoints");
        }
        lower.On([this](const auto &sample) { lower_.Update(sample); });
        upper.On([this](const auto &sample) { upper_.Update(sample); });
        imu.On([this](const auto &sample) { imu_.Update(sample); });
        gamepad.On([this](const auto &sample) { gamepad_.Update(sample); });
        return true;
    }

    bool Read(RobotState *state, OperatorInput *input, std::string *error)
    {
        const auto lower = lower_.Fresh(lumo_deploy::kSdkStateTimeout);
        const auto upper = upper_.Fresh(lumo_deploy::kSdkStateTimeout);
        const auto imu = imu_.Fresh(lumo_deploy::kSdkStateTimeout);
        const auto gamepad = gamepad_.Fresh(lumo_deploy::kSdkGamepadTimeout);
        if (!lower || !upper || !imu || !gamepad) {
            return Fail(error, "missing/stale SDK topic(s):" +
                std::string(lower ? "" : " lower/state") + (upper ? "" : " upper/state") +
                (imu ? "" : " pelvis/imu/data") + (gamepad ? "" : " gamepad/state"));
        }
        if (lower_writer_.Matched() == 0 || upper_writer_.Matched() == 0) {
            return Fail(error, "waiting for robot readers on lower/cmd and upper/cmd");
        }
        RobotState decoded_state;
        OperatorInput decoded_input;
        if (!lumo_deploy::DecodeSdkState(*lower, *upper, *imu, &decoded_state, error) ||
            !gamepad_decoder_.Decode(*gamepad, &decoded_input, error)) {
            return false;
        }
        *state = decoded_state;
        *input = decoded_input;
        return true;
    }

    bool Send(const JointCommand &command, std::string *error)
    {
        lumo_deploy::sdk_msg::LowerCmd_ lower;
        lumo_deploy::sdk_msg::UpperCmd_ upper;
        if (!lumo_deploy::EncodeSdkCommand(command, &lower, &upper, error)) {
            return false;
        }
        // Attempt both writes even if the first fails. DDS does not provide an
        // atomic whole-body command or an acknowledgement of motor execution.
        const bool lower_ok = lower_writer_.Write(lower);
        const bool upper_ok = upper_writer_.Write(upper);
        if (!lower_ok || !upper_ok) {
            return Fail(error, "Dobot SDK command write failed");
        }
        return true;
    }

private:
    static bool Fail(std::string *error, std::string message)
    {
        if (error) {
            *error = std::move(message);
        }
        return false;
    }

    dobot::sdk::Session session_;
    lumo_deploy::ReceivedSdkSample<lumo_deploy::sdk_msg::LowerState_> lower_;
    lumo_deploy::ReceivedSdkSample<lumo_deploy::sdk_msg::UpperState_> upper_;
    lumo_deploy::ReceivedSdkSample<lumo_deploy::sdk_msg::IMUState_> imu_;
    lumo_deploy::ReceivedSdkSample<lumo_deploy::sdk_msg::GamepadState_> gamepad_;
    dobot::sdk::Publisher<lumo_deploy::sdk_msg::LowerCmd_> lower_writer_;
    dobot::sdk::Publisher<lumo_deploy::sdk_msg::UpperCmd_> upper_writer_;
    lumo_deploy::SdkGamepadDecoder gamepad_decoder_;
};

JointCommand DampingCommand(const RobotState &state)
{
    JointCommand command;
    command.q = state.q;
    command.kd = lumo_deploy::FixedKd();
    return command;
}

void PrintHelp()
{
    std::cout << "Usage: lumo_real --nic INTERFACE [--model sr_3588|sr_thro] [--robot-ip IP] [--domain N] [--background]\n"
              << "       lumo_real --status\n"
              << "       lumo_real --stop\n"
              << "       lumo_real --check | --help\n"
              << "  --model       Robot profile (default: sr_3588)\n"
              << "  --robot-ip    RPC address (default follows model; DDS uses NIC/domain)\n"
              << "  --nic         Wired network interface; use scripts/run_real.sh for route selection\n"
              << "  --domain      DDS domain, 0..232 (default: 0)\n"
              << "  Enter robot debug mode manually before starting this program.\n"
              << "  --background  Run detached from the launching terminal and write a PID/log file\n"
              << "  --status      Report the managed background process\n"
              << "  --stop        Send SIGTERM to the managed background process\n"
              << "  --check       Validate policy assets without connecting to hardware\n";
}

}  // namespace

int main(int argc, char **argv)
{
    ExitReport exit_report;
    lumo_deploy::SdkModel model = lumo_deploy::SdkModel::kSr3588;
    dobot::sdk::SessionConfig sdk_config;
    sdk_config.where = dobot::sdk::RunWhere::kComputer;
    bool check_only = false;
    bool run_in_background = false;
    bool show_status = false;
    bool stop_background = false;
    for (int i = 1; i < argc; ++i) {
        const std::string argument(argv[i]);
        if (argument == "--help" || argument == "-h") {
            PrintHelp();
            exit_report.enabled = false;
            return 0;
        }
        if (argument == "--model" || argument == "--robot-ip" ||
            argument == "--nic" || argument == "--domain") {
            if (i + 1 >= argc || std::string(argv[i + 1]).empty()) {
                std::cerr << argument << " requires a value\n";
                exit_report.code = 2;
                return 2;
            }
            const std::string value(argv[++i]);
            if (argument == "--model") {
                if (value == "sr_3588") {
                    model = lumo_deploy::SdkModel::kSr3588;
                } else if (value == "sr_thro") {
                    model = lumo_deploy::SdkModel::kSrThro;
                } else {
                    std::cerr << "--model must be sr_3588 or sr_thro\n";
                    exit_report.code = 2;
                    return 2;
                }
            } else if (argument == "--robot-ip") {
                in_addr address{};
                if (::inet_pton(AF_INET, value.c_str(), &address) != 1) {
                    std::cerr << "--robot-ip requires an IPv4 address\n";
                    exit_report.code = 2;
                    return 2;
                }
                sdk_config.robot_ip = value;
            } else if (argument == "--nic") {
                sdk_config.network_interface = value;
            } else {
                unsigned domain = 0;
                const auto parsed = std::from_chars(value.data(), value.data() + value.size(), domain);
                if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || domain > 232) {
                    std::cerr << "--domain must be an integer in 0..232\n";
                    exit_report.code = 2;
                    return 2;
                }
                sdk_config.domain_id = domain;
            }
            continue;
        }
        if (argument == "--check") {
            check_only = true;
            continue;
        }
        if (argument == "--background") {
            run_in_background = true;
            continue;
        }
        if (argument == "--status") {
            show_status = true;
            continue;
        }
        if (argument == "--stop") {
            stop_background = true;
            continue;
        }
        std::cerr << "Unknown option: " << argument << '\n';
        exit_report.code = 2;
        return 2;
    }

    if ((show_status && (stop_background || run_in_background || check_only)) ||
        (stop_background && (run_in_background || check_only))) {
        std::cerr << "--status and --stop cannot be combined with other options\n";
        exit_report.code = 2;
        return 2;
    }

    ProcessPaths process_paths;
    std::string error;
    if (!ResolveProcessPaths(&process_paths, &error)) {
        exit_report.reason = error;
        std::cerr << error << '\n';
        return 1;
    }
    if (show_status) {
        exit_report.enabled = false;
        return PrintBackgroundStatus(process_paths);
    }
    if (stop_background) {
        exit_report.enabled = false;
        return StopBackground(process_paths);
    }

    if (!check_only) {
        const auto &nic = sdk_config.network_interface;
        if (nic.empty() || nic == "lo" || ::if_nametoindex(nic.c_str()) == 0 ||
            std::filesystem::exists(std::filesystem::path("/sys/class/net") / nic / "wireless")) {
            std::cerr << "--nic must name the wired interface connected to the robot; "
                         "use scripts/run_real.sh to select it from the route\n";
            exit_report.code = 2;
            return 2;
        }
    }
    if (sdk_config.robot_ip.empty()) {
        sdk_config.robot_ip = lumo_deploy::TopicsForModel(model).profile.default_robot_ip;
    }

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    BackgroundSession background_session;
    if (run_in_background) {
        const BackgroundSession::StartResult result =
            background_session.Start(process_paths, &error);
        if (result == BackgroundSession::StartResult::kError) {
            std::cerr << error << '\n';
            return 1;
        }
        if (result == BackgroundSession::StartResult::kParent) {
            exit_report.enabled = false;
            return 0;
        }
    }

    const std::filesystem::path project_root(LUMO_PROJECT_ROOT);
    Controller controller;
    if (!controller.Initialize(project_root, &error)) {
        exit_report.reason = error;
        std::cerr << error << '\n';
        return 1;
    }
    if (check_only) {
        std::cout << "Policy assets are ready\n";
        exit_report.enabled = false;
        return 0;
    }

    lumo_deploy::CsvRecorder recorder(project_root / "logs" / "csv");
    Hardware hardware;
    if (!hardware.Initialize(model, sdk_config, &error)) {
        exit_report.reason = error;
        std::cerr << error << '\n';
        return 1;
    }
    std::cout << "Dobot " << lumo_deploy::TopicsForModel(model).profile.model
              << ", RPC IP=" << sdk_config.robot_ip
              << ", DDS NIC=" << sdk_config.network_interface
              << ", domain=" << sdk_config.domain_id << '\n'
              << "Waiting for robot state and command readers (10 s timeout)\n"
              << "RB+A initial pose, RB+X damping, RB+Up IsaacLab, RB+Down MJLab\n"
              << "Start: record CSV; Back/Select: stop and save CSV (logs/csv/)\n";
    Mode printed_mode = controller.CurrentMode();
    RobotState state;
    OperatorInput input;
    JointCommand command;
    bool have_state = false;
    bool failed = false;
    std::string last_controller_error;
    auto next_cycle = std::chrono::steady_clock::now();
    auto last_wait_message = next_cycle - std::chrono::seconds(2);
    const auto startup_deadline = next_cycle + std::chrono::seconds(10);

    while (g_stop == 0) {
        next_cycle += kControlPeriod;
        if (!hardware.Read(&state, &input, &error)) {
            const auto now = std::chrono::steady_clock::now();
            if (have_state || now >= startup_deadline) {
                exit_report.reason = std::string(have_state ? "State/connection failure: "
                                                            : "Startup timeout (10 s): ") + error;
                PrintAlert("[ERROR] 控制已中止 / CONTROL ABORTED",
                           exit_report.reason +
                           "\nCheck Ethernet, robot debug mode, state publication and DDS NIC/domain."
                           "\nState/IMU timeout: 100 ms; gamepad timeout: 500 ms."
                           "\nDo not assume the robot received damping commands.", true);
                failed = true;
                break;
            }
            if (now - last_wait_message >= std::chrono::seconds(1)) {
                std::cerr << error << '\n';
                last_wait_message = now;
            }
            std::this_thread::sleep_until(next_cycle);
            continue;
        }
        have_state = true;

        if (!controller.Step(state, input, 0.002F, &command, &error)) {
            if (error != last_controller_error) {
                PrintAlert("[ERROR] 策略异常，已切入阻尼 / CONTROLLER ENTERED DAMPING",
                           error + "\nThe process remains running in damping mode.", true);
                last_controller_error = error;
            }
        } else {
            last_controller_error.clear();
        }
        const bool command_written = hardware.Send(command, &error);
        recorder.Update(input, state, command, controller.CurrentMode(), command_written);
        if (!command_written) {
            exit_report.reason = "Command transmission failure: " + error;
            PrintAlert("[ERROR] 控制已中止 / CONTROL ABORTED", exit_report.reason, true);
            failed = true;
            break;
        }
        if (controller.CurrentMode() != printed_mode) {
            printed_mode = controller.CurrentMode();
            std::cout << "Mode: " << lumo_deploy::ModeName(printed_mode) << '\n';
        }
        std::this_thread::sleep_until(next_cycle);
    }

    exit_report.had_state = have_state;
    exit_report.code = failed ? 1 : 0;
    if (!failed) {
        exit_report.reason = g_stop == SIGINT ? "SIGINT (Ctrl+C) requested shutdown."
                                              : "SIGTERM requested shutdown.";
    }
    if (have_state) {
        const JointCommand damping = DampingCommand(state);
        for (int i = 0; i < 20; ++i) {
            std::string damping_error;
            if (!hardware.Send(damping, &damping_error)) {
                ++exit_report.damping_failures;
            }
            std::this_thread::sleep_for(kControlPeriod);
        }
    }
    if (exit_report.damping_failures > 0 && !failed) {
        exit_report.reason += " Exit damping writes failed.";
        exit_report.code = 1;
    }
    // Flush only after the control loop and exit damping have finished.
    recorder.Shutdown();
    return exit_report.code;
}
