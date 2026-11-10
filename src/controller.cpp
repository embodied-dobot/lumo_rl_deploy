#include "lumo_deploy/controller.hpp"

#include "lumo_deploy/config.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lumo_deploy {
namespace {

constexpr float kInitialPoseDuration = 2.0F;
constexpr float kInferencePeriod = 0.02F;

bool Rising(bool current, bool previous) { return current && !previous; }

}  // namespace

const char *ModeName(Mode mode)
{
    switch (mode) {
        case Mode::kDamping:
            return "damping";
        case Mode::kInitialPose:
            return "initial_pose";
        case Mode::kIsaacLabVelocity:
            return "isaaclab_velocity_opensource";
        case Mode::kMjlabVelocity:
            return "mjlab_velocity";
    }
    return "unknown";
}

Mode StateMachine::Update(const OperatorInput &input, bool initial_pose_ready)
{
    const bool damping_edge = Rising(input.rb_x, previous_.rb_x);
    const bool initial_edge = Rising(input.rb_a, previous_.rb_a);
    const bool isaac_edge = Rising(input.rb_up, previous_.rb_up);
    const bool mjlab_edge = Rising(input.rb_down, previous_.rb_down);
    previous_ = input;

    // Safety command wins if multiple combinations arrive in the same sample.
    if (damping_edge) {
        mode_ = Mode::kDamping;
        return mode_;
    }
    if (initial_edge) {
        mode_ = Mode::kInitialPose;
        return mode_;
    }

    const bool policy_selection_allowed =
        (mode_ == Mode::kInitialPose && initial_pose_ready) ||
        mode_ == Mode::kIsaacLabVelocity || mode_ == Mode::kMjlabVelocity;
    if (policy_selection_allowed && isaac_edge) {
        mode_ = Mode::kIsaacLabVelocity;
    } else if (policy_selection_allowed && mjlab_edge) {
        mode_ = Mode::kMjlabVelocity;
    }
    return mode_;
}

void StateMachine::ForceDamping() { mode_ = Mode::kDamping; }

bool Controller::Initialize(const std::filesystem::path &project_root, std::string *error)
{
    isaaclab_policy_ = std::make_unique<Policy>(IsaacLabVelocityConfig(project_root));
    mjlab_policy_ = std::make_unique<Policy>(MjlabVelocityConfig(project_root));
    if (!isaaclab_policy_->Load(error)) {
        return false;
    }
    if (!mjlab_policy_->Load(error)) {
        return false;
    }
    if (error) {
        error->clear();
    }
    return true;
}

void Controller::EnterMode(Mode next, const RobotState &state)
{
    command_mode_ = next;
    inference_elapsed_ = kInferencePeriod;
    policy_command_ready_ = false;
    if (next == Mode::kInitialPose) {
        initial_pose_start_ = state.q;
        initial_pose_elapsed_ = 0.0F;
        initial_pose_ready_ = false;
    } else if (next == Mode::kIsaacLabVelocity && isaaclab_policy_) {
        isaaclab_policy_->Reset();
    } else if (next == Mode::kMjlabVelocity && mjlab_policy_) {
        mjlab_policy_->Reset();
    }
}

bool Controller::Step(const RobotState &state, const OperatorInput &input, float dt,
                      JointCommand *command, std::string *error)
{
    if (!command || !std::isfinite(dt) || dt <= 0.0F) {
        if (error) {
            *error = "controller requires a positive finite dt and command output";
        }
        return false;
    }

    const Mode next = state_machine_.Update(input, initial_pose_ready_);
    if (next != command_mode_) {
        EnterMode(next, state);
    }

    switch (command_mode_) {
        case Mode::kDamping:
            BuildDampingCommand(state, command);
            break;
        case Mode::kInitialPose:
            BuildInitialPoseCommand(dt, command);
            break;
        case Mode::kIsaacLabVelocity:
        case Mode::kMjlabVelocity: {
            inference_elapsed_ += dt;
            Policy *policy = command_mode_ == Mode::kIsaacLabVelocity ? isaaclab_policy_.get()
                                                                       : mjlab_policy_.get();
            if (!policy) {
                if (error) {
                    *error = "selected policy is unavailable";
                }
                state_machine_.ForceDamping();
                EnterMode(Mode::kDamping, state);
                BuildDampingCommand(state, command);
                return false;
            }
            if (!policy_command_ready_ || inference_elapsed_ >= kInferencePeriod) {
                inference_elapsed_ = std::fmod(inference_elapsed_, kInferencePeriod);
                if (!policy->Infer(state, input, &held_policy_command_, error)) {
                    state_machine_.ForceDamping();
                    EnterMode(Mode::kDamping, state);
                    BuildDampingCommand(state, command);
                    return false;
                }
                policy_command_ready_ = true;
            }
            *command = held_policy_command_;
            break;
        }
    }
    if (error) {
        error->clear();
    }
    return true;
}

void Controller::BuildDampingCommand(const RobotState &state, JointCommand *command) const
{
    command->q = state.q;
    command->dq.fill(0.0F);
    command->kp.fill(0.0F);
    command->kd = FixedKd();
    command->tau.fill(0.0F);
}

void Controller::BuildInitialPoseCommand(float dt, JointCommand *command)
{
    initial_pose_elapsed_ = std::min(kInitialPoseDuration, initial_pose_elapsed_ + dt);
    const float alpha = initial_pose_elapsed_ / kInitialPoseDuration;
    const float smooth = alpha * alpha * (3.0F - 2.0F * alpha);
    for (std::size_t i = 0; i < kJointCount; ++i) {
        command->q[i] = initial_pose_start_[i] + smooth * (DefaultPose()[i] - initial_pose_start_[i]);
    }
    command->dq.fill(0.0F);
    command->kp = FixedKp();
    command->kd = FixedKd();
    command->tau.fill(0.0F);
    initial_pose_ready_ = initial_pose_elapsed_ >= kInitialPoseDuration;
}

}  // namespace lumo_deploy
