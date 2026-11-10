#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstddef>

namespace lumo_deploy {

inline constexpr std::size_t kJointCount = 30;
inline constexpr std::size_t kPolicyDofCount = 29;

enum class Mode {
    kDamping,
    kInitialPose,
    kIsaacLabVelocity,
    kMjlabVelocity,
};

struct RobotState {
    std::array<float, kJointCount> q{};
    std::array<float, kJointCount> dq{};
    // SDK-estimated feedback torque, used for diagnostics only.
    std::array<float, kJointCount> tau_est{};
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    Eigen::Vector3f angular_velocity = Eigen::Vector3f::Zero();
};

struct JointCommand {
    std::array<float, kJointCount> q{};
    std::array<float, kJointCount> dq{};
    std::array<float, kJointCount> kp{};
    std::array<float, kJointCount> kd{};
    std::array<float, kJointCount> tau{};
};

// Axes are normalized to [-1, 1]. Buttons are current levels; edge detection is
// owned by the state machine so the real and simulated backends behave identically.
struct OperatorInput {
    float forward = 0.0F;
    float lateral = 0.0F;
    float yaw = 0.0F;
    bool start = false;
    bool back = false;
    bool rb_a = false;
    bool rb_x = false;
    bool rb_up = false;
    bool rb_down = false;
};

struct VelocityCommand {
    float x = 0.0F;
    float y = 0.0F;
    float yaw = 0.0F;
};

const char *ModeName(Mode mode);

}  // namespace lumo_deploy

