#include "lumo_deploy/sdk_adapter.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <tuple>

namespace lumo_deploy {
namespace {

bool Fail(std::string *error, const char *message)
{
    if (error) {
        *error = message;
    }
    return false;
}

// LowerState has 15 slots in both SDKs; only the first 12 leg joints are used.
static_assert(kSdkLowerJoints + kSdkUpperJoints == kJointCount);
static_assert(std::tuple_size_v<std::remove_reference_t<decltype(
                  std::declval<sdk_msg::LowerState_>().motor_state())>> == 15);
static_assert(std::tuple_size_v<std::remove_reference_t<decltype(
                  std::declval<sdk_msg::UpperState_>().motor_state())>> == kSdkUpperJoints);
static_assert(std::tuple_size_v<std::remove_reference_t<decltype(
                  std::declval<sdk_msg::LowerCmd_>().motor_cmd())>> == kSdkLowerJoints);
static_assert(std::tuple_size_v<std::remove_reference_t<decltype(
                  std::declval<sdk_msg::UpperCmd_>().motor_cmd())>> == kSdkUpperJoints);

}  // namespace

SdkTopics TopicsForModel(SdkModel model)
{
    if (model == SdkModel::kSrThro) {
        namespace robot = dobot::robot::sr_thro;
        return {robot::Profile(), robot::kTopicLowerState, robot::kTopicUpperState,
                robot::kTopicPelvisImu, robot::kTopicGamepadState,
                robot::kTopicLowerCmd, robot::kTopicUpperCmd};
    }
    namespace robot = dobot::robot::sr_3588;
    return {robot::Profile(), robot::kTopicLowerState, robot::kTopicUpperState,
            robot::kTopicPelvisImu, robot::kTopicGamepadState,
            robot::kTopicLowerCmd, robot::kTopicUpperCmd};
}

bool DecodeSdkState(const sdk_msg::LowerState_ &lower, const sdk_msg::UpperState_ &upper,
                    const sdk_msg::IMUState_ &imu, RobotState *state, std::string *error)
{
    if (!state) {
        return Fail(error, "robot state output is null");
    }
    RobotState decoded;
    for (std::size_t i = 0; i < kJointCount; ++i) {
        const auto &motor = i < kSdkLowerJoints ? lower.motor_state()[i]
                                              : upper.motor_state()[i - kSdkLowerJoints];
        decoded.q[i] = motor.q();
        decoded.dq[i] = motor.dq();
        decoded.tau_est[i] = motor.tau_est();
        if (!std::isfinite(decoded.q[i]) || !std::isfinite(decoded.dq[i])) {
            return Fail(error, "Dobot SDK returned a non-finite joint state");
        }
    }
    const auto &q = imu.quaternion();
    decoded.orientation = Eigen::Quaternionf(q[0], q[1], q[2], q[3]);
    const auto &w = imu.gyroscope();
    decoded.angular_velocity = Eigen::Vector3f(w[0], w[1], w[2]);
    const float norm = decoded.orientation.norm();
    if (!decoded.orientation.coeffs().allFinite() ||
        !decoded.angular_velocity.allFinite() || !std::isfinite(norm) || norm < 1.0e-6F) {
        return Fail(error, "Dobot SDK returned an invalid IMU state");
    }
    decoded.orientation.normalize();
    *state = decoded;
    if (error) {
        error->clear();
    }
    return true;
}

bool EncodeSdkCommand(const JointCommand &command, sdk_msg::LowerCmd_ *lower,
                      sdk_msg::UpperCmd_ *upper, std::string *error)
{
    if (!lower || !upper) {
        return Fail(error, "joint command output is null");
    }
    // Validate the entire frame before modifying either command.
    for (std::size_t i = 0; i < kJointCount; ++i) {
        if (!std::isfinite(command.q[i]) || !std::isfinite(command.dq[i]) ||
            !std::isfinite(command.kp[i]) || !std::isfinite(command.kd[i]) ||
            !std::isfinite(command.tau[i]) || command.kp[i] < 0.0F || command.kd[i] < 0.0F) {
            return Fail(error, "refusing an invalid joint command");
        }
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
        auto &motor = i < kSdkLowerJoints ? lower->motor_cmd()[i]
                                         : upper->motor_cmd()[i - kSdkLowerJoints];
        motor.mode(kSdkMitMode);
        motor.q(command.q[i]);
        motor.dq(command.dq[i]);
        motor.kp(command.kp[i]);
        motor.kd(command.kd[i]);
        motor.tau(command.tau[i]);
    }
    if (error) {
        error->clear();
    }
    return true;
}

bool SdkGamepadDecoder::Decode(const sdk_msg::GamepadState_ &gamepad, OperatorInput *input,
                               std::string *error)
{
    if (!input) {
        return Fail(error, "operator input output is null");
    }
    if (!std::isfinite(gamepad.lx()) || !std::isfinite(gamepad.ly()) ||
        !std::isfinite(gamepad.rx()) || !std::isfinite(gamepad.ry())) {
        return Fail(error, "Dobot SDK returned a non-finite gamepad axis");
    }
    // Preserve the legacy Bridge axis signs and the deployment's 500 Hz filter.
    constexpr float alpha = 0.2F;
    forward_ = alpha * -gamepad.ly() + (1.0F - alpha) * forward_;
    lateral_ = alpha * -gamepad.lx() + (1.0F - alpha) * lateral_;
    yaw_ = alpha * -gamepad.rx() + (1.0F - alpha) * yaw_;
    input->forward = std::clamp(forward_, -1.0F, 1.0F);
    input->lateral = std::clamp(lateral_, -1.0F, 1.0F);
    input->yaw = std::clamp(yaw_, -1.0F, 1.0F);
    // Legacy xKeySwitchUnion: R1=0, A=8, X=10, Up=12, Down=14.
    const auto buttons = gamepad.btn_state();
    input->start = (buttons & (1U << 2)) != 0;
    input->back = (buttons & (1U << 3)) != 0;
    // The legacy decoder recognizes a combo only when exactly two buttons are held.
    const bool rb = std::popcount(buttons) == 2 && (buttons & (1U << 0)) != 0;
    input->rb_a = rb && (buttons & (1U << 8)) != 0;
    input->rb_x = rb && (buttons & (1U << 10)) != 0;
    input->rb_up = rb && (buttons & (1U << 12)) != 0;
    input->rb_down = rb && (buttons & (1U << 14)) != 0;
    if (error) {
        error->clear();
    }
    return true;
}

}  // namespace lumo_deploy
