#pragma once

#include "lumo_deploy/types.hpp"

#include <dobot/robot/sr_3588/topics.hpp>
#include <dobot/robot/sr_thro/topics.hpp>
#include <gamepad_state.hpp>
#include <imu_state.hpp>
#include <lower_cmd.hpp>
#include <lower_state.hpp>
#include <upper_cmd.hpp>
#include <upper_state.hpp>

#include <chrono>
#include <mutex>
#include <optional>
#include <string>

namespace lumo_deploy {

namespace sdk_msg = dobot::msg::dds_;
enum class SdkModel { kSr3588, kSrThro };

struct SdkTopics {
    dobot::sdk::RobotProfile profile;
    const char *lower_state;
    const char *upper_state;
    const char *imu;
    const char *gamepad;
    const char *lower_cmd;
    const char *upper_cmd;
};
SdkTopics TopicsForModel(SdkModel model);

// These compatibility conventions need firmware confirmation for both models.
// Do not infer joint semantics from array size alone.
inline constexpr std::size_t kSdkLowerJoints = 12;
inline constexpr std::size_t kSdkUpperJoints = 18;
inline constexpr uint8_t kSdkMitMode = 11;
inline constexpr auto kSdkStateTimeout = std::chrono::milliseconds(100);
inline constexpr auto kSdkGamepadTimeout = std::chrono::milliseconds(500);

bool DecodeSdkState(const sdk_msg::LowerState_ &lower, const sdk_msg::UpperState_ &upper,
                    const sdk_msg::IMUState_ &imu, RobotState *state, std::string *error);
bool EncodeSdkCommand(const JointCommand &command, sdk_msg::LowerCmd_ *lower,
                      sdk_msg::UpperCmd_ *upper, std::string *error);

class SdkGamepadDecoder {
public:
    bool Decode(const sdk_msg::GamepadState_ &gamepad, OperatorInput *input,
                std::string *error);

private:
    float forward_ = 0.0F;
    float lateral_ = 0.0F;
    float yaw_ = 0.0F;
};

// SDK Latest() retains the last sample indefinitely. Track local receipt time,
// independent of the robot clock; never refresh it just because Read() was called.
template <typename Msg>
class ReceivedSdkSample {
public:
    using Clock = std::chrono::steady_clock;
    void Update(const Msg &message, Clock::time_point now = Clock::now())
    {
        std::lock_guard lock(mutex_);
        message_ = message;
        received_at_ = now;
    }

    std::optional<Msg> Fresh(std::chrono::milliseconds timeout,
                             Clock::time_point now = Clock::now()) const
    {
        std::lock_guard lock(mutex_);
        if (!message_ || now - received_at_ > timeout) {
            return std::nullopt;
        }
        return message_;
    }

private:
    mutable std::mutex mutex_;
    std::optional<Msg> message_;
    Clock::time_point received_at_{};
};

}  // namespace lumo_deploy
