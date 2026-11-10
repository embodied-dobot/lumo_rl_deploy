#pragma once

#include "lumo_deploy/types.hpp"

#include <array>
#include <filesystem>
#include <string>

namespace lumo_deploy {

enum class PolicyKind {
    kIsaacLabVelocity,
    kMjlabVelocity,
};

struct PolicyConfig {
    PolicyKind kind = PolicyKind::kIsaacLabVelocity;
    std::string name;
    std::filesystem::path model_path;
    int observation_size = 0;
    int history_length = 1;
    std::array<int, kPolicyDofCount> joint_mapping{};
    std::array<int, kJointCount> gain_mapping{};
    std::array<float, kJointCount> action_scale{};
    std::array<float, kPolicyDofCount> kp{};
    std::array<float, kPolicyDofCount> kd{};
    float max_forward = 1.0F;
    float min_forward = -1.0F;
    float max_lateral = 0.5F;
    float max_yaw = 1.0F;
    float deadzone = 0.05F;
};

const std::array<std::string, kJointCount> &JointNames();
const std::array<float, kJointCount> &DefaultPose();
const std::array<float, kJointCount> &FixedKp();
const std::array<float, kJointCount> &FixedKd();
const std::array<float, kJointCount> &TorqueLimits();

PolicyConfig IsaacLabVelocityConfig(const std::filesystem::path &project_root);
PolicyConfig MjlabVelocityConfig(const std::filesystem::path &project_root);

}  // namespace lumo_deploy

