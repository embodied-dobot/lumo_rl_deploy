#include "lumo_deploy/csv_recorder.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>

namespace {
void Expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}
std::vector<std::filesystem::path> Files(const std::filesystem::path &directory)
{
    std::vector<std::filesystem::path> paths;
    if (std::filesystem::exists(directory)) {
        for (const auto &entry : std::filesystem::directory_iterator(directory)) {
            if (entry.path().extension() == ".csv") paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());
    return paths;
}
void WaitIdle(const lumo_deploy::CsvRecorder &recorder)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (recorder.Busy() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Expect(!recorder.Busy(), "worker finishes saving");
}
std::vector<std::string> Split(const std::string &line)
{
    std::istringstream input(line);
    std::vector<std::string> parts;
    std::string value;
    while (std::getline(input, value, ',')) parts.push_back(value);
    return parts;
}
}  // namespace

int main()
{
    using namespace lumo_deploy;
    char pattern[] = "/tmp/lumo-csv-test-XXXXXX";
    const char *temporary = ::mkdtemp(pattern);
    Expect(temporary != nullptr, "temporary test directory");
    const std::filesystem::path root(temporary);
    RobotState state;
    JointCommand command;
    for (std::size_t i = 0; i < kJointCount; ++i) {
        state.q[i] = static_cast<float>(i) + 0.1F;
        state.dq[i] = static_cast<float>(i) + 0.2F;
        state.tau_est[i] = static_cast<float>(i) + 0.3F;
        command.q[i] = static_cast<float>(i) + 0.4F;
        command.dq[i] = static_cast<float>(i) + 0.5F;
        command.tau[i] = static_cast<float>(i) + 0.6F;
        command.kp[i] = static_cast<float>(i) + 0.7F;
        command.kd[i] = static_cast<float>(i) + 0.8F;
    }
    const auto t = CsvRecorder::Clock::now();
    OperatorInput input;
    {
        CsvRecorder recorder(root / "normal");
        recorder.Update(input, state, command, Mode::kDamping, true, t);
        Expect(!recorder.Busy(), "idle does not record");
        input.start = true;
        for (int i = 0; i < 10; ++i) {
            recorder.Update(input, state, command, Mode::kMjlabVelocity, i != 9,
                            t + std::chrono::milliseconds(i * 2));
        }
        input.start = false;
        input.back = true;
        recorder.Update(input, state, command, Mode::kDamping, true, t + std::chrono::milliseconds(20));
        WaitIdle(recorder);
        Expect(!recorder.Failed() && recorder.Dropped() == 0, "normal recording complete");
        auto files = Files(root / "normal");
        Expect(files.size() == 1, "held Start creates exactly one recording");
        std::ifstream csv(files[0]);
        std::string line;
        std::getline(csv, line);
        auto header = Split(line);
        Expect(header.size() == 245 && header[0] == "time_s", "PlotJuggler numeric time and 245 columns");
        Expect(header[5] == "left_hip_pitch_joint/q_rad" &&
               header[7] == "left_hip_pitch_joint/tau_est_nm" &&
               header[237] == "head_joint/q_rad", "named joints and distinct torque fields");
        int count = 0;
        while (std::getline(csv, line)) {
            const auto values = Split(line);
            Expect(values.size() == header.size(), "CSV row width");
            Expect(std::abs(std::stod(values[0]) - count * 0.002) < 1e-8, "actual elapsed seconds");
            Expect(std::stoi(values[1]) == count && std::stoi(values[3]) == 3, "sample index and mode");
            Expect(std::stoi(values[4]) == (count != 9), "local command write status");
            for (std::size_t joint = 0; joint < kJointCount; ++joint) {
                for (std::size_t field = 0; field < 8; ++field) {
                    const double expected = joint + (field + 1) * 0.1;
                    Expect(std::abs(std::stod(values[5 + joint * 8 + field]) - expected) < 1e-5,
                           "all 30 motors and eight channels preserved");
                }
            }
            ++count;
        }
        Expect(count == 10, "Back stops capture and drains all queued samples");
        struct stat permissions{};
        Expect(::stat(files[0].c_str(), &permissions) == 0 && (permissions.st_mode & 0444) == 0444,
               "CSV readable by non-root PlotJuggler after sudo recording");
        input = {};
        recorder.Update(input, state, command, Mode::kDamping, true);
        input.start = true;
        recorder.Update(input, state, command, Mode::kDamping, true);
        recorder.Shutdown();
        Expect(Files(root / "normal").size() == 2, "restart gets a new file; shutdown saves active session");
    }
    {
        CsvRecorder recorder(root / "both");
        input.start = input.back = true;
        recorder.Update(input, state, command, Mode::kDamping, true);
        recorder.Shutdown();
        Expect(Files(root / "both").empty(), "Back takes precedence over Start");
    }
    {
        CsvRecorder recorder(root / "overflow", 1);
        input = {};
        input.start = true;
        for (int i = 0; i < 20000; ++i) {
            recorder.Update(input, state, command, Mode::kDamping, true,
                            t + std::chrono::milliseconds(i * 2));
        }
        recorder.Shutdown();
        Expect(recorder.Dropped() > 0 && !recorder.Failed(), "bounded queue reports overflow without stopping control");
        Expect(Files(root / "overflow").size() == 1, "overflow still saves a readable CSV");
    }
    {
        CsvRecorder recorder(root / "rapid");
        for (int i = 0; i < 50; ++i) {
            input = {};
            recorder.Update(input, state, command, Mode::kDamping, true);
            input.start = true;
            recorder.Update(input, state, command, Mode::kDamping, true);
            input.start = false;
            input.back = true;
            recorder.Update(input, state, command, Mode::kDamping, true);
            WaitIdle(recorder);
        }
        recorder.Shutdown();
        const auto files = Files(root / "rapid");
        Expect(files.size() == 50, "rapid successive sessions do not merge or overwrite files");
        for (const auto &path : files) {
            std::ifstream csv(path);
            std::string line;
            int lines = 0;
            while (std::getline(csv, line)) ++lines;
            Expect(lines == 2, "immediate Back retains the first queued sample");
        }
    }
    {
        std::ofstream(root / "not_a_directory") << "test";
        CsvRecorder recorder(root / "not_a_directory" / "csv");
        input = {};
        input.start = true;
        recorder.Update(input, state, command, Mode::kDamping, true);
        recorder.Shutdown();
        Expect(recorder.Failed(), "filesystem error is reported, not silently called saved");
    }
    std::filesystem::remove_all(root);
    std::cout << "CSV recorder tests passed\n";
}
