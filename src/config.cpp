#include "lumo_deploy/config.hpp"

namespace lumo_deploy {
namespace {

constexpr std::array<float, kJointCount> kDefaultPose{
    -0.312F, 0.0F, 0.0F, 0.669F, -0.363F, 0.0F,
    -0.312F, 0.0F, 0.0F, 0.669F, -0.363F, 0.0F,
    0.0F, 0.0F, 0.0F,
    0.2F, 0.2F, 0.0F, 0.6F, 0.0F, 0.0F, 0.0F,
    0.2F, -0.2F, 0.0F, 0.6F, 0.0F, 0.0F, 0.0F,
    0.0F};

constexpr std::array<float, kJointCount> kFixedKp{
    100.0F, 100.0F, 100.0F, 100.0F, 40.0F, 40.0F,
    100.0F, 100.0F, 100.0F, 100.0F, 40.0F, 40.0F,
    100.0F, 100.0F, 100.0F,
    100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F,
    100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F,
    40.0F};

constexpr std::array<float, kJointCount> kFixedKd{
    2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F,
    2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F,
    10.0F, 5.0F, 5.0F,
    2.0F, 2.0F, 2.0F, 2.0F, 1.0F, 1.0F, 1.0F,
    2.0F, 2.0F, 2.0F, 2.0F, 1.0F, 1.0F, 1.0F,
    2.0F};

constexpr std::array<float, kJointCount> kTorqueLimits{
    130.0F, 130.0F, 130.0F, 130.0F, 60.0F, 45.0F,
    130.0F, 130.0F, 130.0F, 130.0F, 60.0F, 45.0F,
    130.0F, 60.0F, 50.0F,
    30.0F, 30.0F, 30.0F, 30.0F, 30.0F, 10.0F, 10.0F,
    30.0F, 30.0F, 30.0F, 30.0F, 30.0F, 10.0F, 10.0F,
    10.0F};

constexpr std::array<std::string_view, kJointCount> kJointNameViews{
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint", "head_joint"};

constexpr std::array<int, kPolicyDofCount> kMjlabMapping{
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

constexpr std::array<int, kJointCount> kMjlabGainMapping{
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 29};

constexpr std::array<int, kPolicyDofCount> kIsaacMapping{
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
    16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28};

constexpr std::array<int, kJointCount> kIsaacGainMapping{
    0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
    16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28, 29};

constexpr std::array<float, kJointCount> kMjlabActionScale{
    0.291260027F, 0.291260027F, 0.291260027F, 0.291260027F, 0.217457849F, 0.163093386F,
    0.291260027F, 0.291260027F, 0.291260027F, 0.291260027F, 0.217457849F, 0.163093386F,
    0.291260027F, 0.217457849F, 0.181214874F,
    0.217457849F, 0.217457849F, 0.217457849F, 0.217457849F, 0.217457849F, 0.184214220F, 0.184214220F,
    0.217457849F, 0.217457849F, 0.217457849F, 0.217457849F, 0.217457849F, 0.184214220F, 0.184214220F,
    0.0F};

constexpr std::array<float, kJointCount> kIsaacActionScale{
    0.291260F, 0.291260F, 0.291260F, 0.291260F, 0.217458F, 0.163093F,
    0.291260F, 0.291260F, 0.291260F, 0.291260F, 0.217458F, 0.163093F,
    0.291260F, 0.217458F, 0.181215F,
    0.120000F, 0.120000F, 0.120000F, 0.120000F, 0.217458F, 0.184214F, 0.184214F,
    0.120000F, 0.120000F, 0.120000F, 0.120000F, 0.217458F, 0.184214F, 0.184214F,
    0.0F};

constexpr std::array<float, kPolicyDofCount> kMjlabKp{
    111.584F, 68.979F, 68.979F,
    34.489F, 34.489F, 34.489F, 34.489F, 34.489F, 13.571F, 13.571F,
    34.489F, 34.489F, 34.489F, 34.489F, 34.489F, 13.571F, 13.571F,
    111.584F, 111.584F, 111.584F, 111.584F, 68.979F, 68.979F,
    111.584F, 111.584F, 111.584F, 111.584F, 68.979F, 68.979F};

constexpr std::array<float, kPolicyDofCount> kMjlabKd{
    8.880F, 4.391F, 4.391F,
    2.196F, 2.196F, 2.196F, 2.196F, 2.196F, 0.720F, 0.720F,
    2.196F, 2.196F, 2.196F, 2.196F, 2.196F, 0.720F, 0.720F,
    8.880F, 8.880F, 8.880F, 8.880F, 4.391F, 4.391F,
    8.880F, 8.880F, 8.880F, 8.880F, 4.391F, 4.391F};

constexpr std::array<float, kPolicyDofCount> kIsaacKp{
    111.5841F, 111.5841F, 111.5841F, 111.5841F, 111.5841F, 68.9789F,
    111.5841F, 111.5841F, 68.9789F, 111.5841F, 111.5841F,
    34.4894F, 34.4894F, 68.9789F, 68.9789F, 34.4894F, 34.4894F,
    68.9789F, 68.9789F, 34.4894F, 34.4894F, 34.4894F, 34.4894F,
    34.4894F, 34.4894F, 13.5712F, 13.5712F, 13.5712F, 13.5712F};

constexpr std::array<float, kPolicyDofCount> kIsaacKd{
    8.8796F, 8.8796F, 8.8796F, 8.8796F, 8.8796F, 4.3913F,
    8.8796F, 8.8796F, 4.3913F, 8.8796F, 8.8796F,
    2.1957F, 2.1957F, 4.3913F, 4.3913F, 2.1957F, 2.1957F,
    4.3913F, 4.3913F, 2.1957F, 2.1957F, 2.1957F, 2.1957F,
    2.1957F, 2.1957F, 0.7200F, 0.7200F, 0.7200F, 0.7200F};

}  // namespace

const std::array<std::string, kJointCount> &JointNames()
{
    static const std::array<std::string, kJointCount> names = [] {
        std::array<std::string, kJointCount> result{};
        for (std::size_t i = 0; i < result.size(); ++i) {
            result[i] = kJointNameViews[i];
        }
        return result;
    }();
    return names;
}

const std::array<float, kJointCount> &DefaultPose() { return kDefaultPose; }
const std::array<float, kJointCount> &FixedKp() { return kFixedKp; }
const std::array<float, kJointCount> &FixedKd() { return kFixedKd; }
const std::array<float, kJointCount> &TorqueLimits() { return kTorqueLimits; }

PolicyConfig IsaacLabVelocityConfig(const std::filesystem::path &project_root)
{
    PolicyConfig config;
    config.kind = PolicyKind::kIsaacLabVelocity;
    config.name = "isaaclab_velocity_opensource";
    config.model_path = project_root / "models/isaaclab_velocity_opensource/policy.onnx";
    config.observation_size = 480;
    config.history_length = 5;
    config.joint_mapping = kIsaacMapping;
    config.gain_mapping = kIsaacGainMapping;
    config.action_scale = kIsaacActionScale;
    config.kp = kIsaacKp;
    config.kd = kIsaacKd;
    config.max_forward = 1.0F;
    config.min_forward = -0.5F;
    config.max_lateral = 0.3F;
    config.max_yaw = 1.0F;
    return config;
}

PolicyConfig MjlabVelocityConfig(const std::filesystem::path &project_root)
{
    PolicyConfig config;
    config.kind = PolicyKind::kMjlabVelocity;
    config.name = "mjlab_velocity";
    config.model_path = project_root / "models/mjlab_velocity/policy.onnx";
    config.observation_size = 98;
    config.history_length = 1;
    config.joint_mapping = kMjlabMapping;
    config.gain_mapping = kMjlabGainMapping;
    config.action_scale = kMjlabActionScale;
    config.kp = kMjlabKp;
    config.kd = kMjlabKd;
    config.max_forward = 2.5F;
    config.min_forward = -1.5F;
    config.max_lateral = 1.0F;
    config.max_yaw = 1.5F;
    return config;
}

}  // namespace lumo_deploy

