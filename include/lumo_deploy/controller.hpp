#pragma once

#include "lumo_deploy/policy.hpp"
#include "lumo_deploy/types.hpp"

#include <filesystem>
#include <memory>
#include <string>

namespace lumo_deploy {

class StateMachine {
public:
    Mode Update(const OperatorInput &input, bool initial_pose_ready);
    void ForceDamping();
    Mode Current() const { return mode_; }

private:
    Mode mode_ = Mode::kDamping;
    OperatorInput previous_{};
};

class Controller {
public:
    bool Initialize(const std::filesystem::path &project_root, std::string *error);
    bool Step(const RobotState &state, const OperatorInput &input, float dt,
              JointCommand *command, std::string *error);

    Mode CurrentMode() const { return state_machine_.Current(); }
    bool InitialPoseReady() const { return initial_pose_ready_; }

private:
    void EnterMode(Mode next, const RobotState &state);
    void BuildDampingCommand(const RobotState &state, JointCommand *command) const;
    void BuildInitialPoseCommand(float dt, JointCommand *command);

    StateMachine state_machine_;
    std::unique_ptr<Policy> isaaclab_policy_;
    std::unique_ptr<Policy> mjlab_policy_;
    Mode command_mode_ = Mode::kDamping;
    std::array<float, kJointCount> initial_pose_start_{};
    JointCommand held_policy_command_{};
    float initial_pose_elapsed_ = 0.0F;
    float inference_elapsed_ = 0.0F;
    bool initial_pose_ready_ = false;
    bool policy_command_ready_ = false;
};

}  // namespace lumo_deploy
