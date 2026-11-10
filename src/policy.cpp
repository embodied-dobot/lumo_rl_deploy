#include "lumo_deploy/policy.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <utility>
#include <vector>

namespace lumo_deploy {
namespace {

constexpr int kIsaacStepSize = 96;
constexpr int kMjlabObservationSize = 98;

float Deadzone(float value, float threshold)
{
    if (!std::isfinite(value) || std::abs(value) < threshold) {
        return 0.0F;
    }
    return std::clamp(value, -1.0F, 1.0F);
}

VelocityCommand ScaleVelocity(const OperatorInput &input, const PolicyConfig &config)
{
    const float forward = Deadzone(input.forward, config.deadzone);
    VelocityCommand command;
    command.x = forward >= 0.0F ? forward * config.max_forward
                                : forward * -config.min_forward;
    command.y = Deadzone(input.lateral, config.deadzone) * config.max_lateral;
    command.yaw = Deadzone(input.yaw, config.deadzone) * config.max_yaw;
    return command;
}

Eigen::Vector3f ProjectedGravity(const Eigen::Quaternionf &orientation)
{
    Eigen::Quaternionf normalized = orientation;
    if (!normalized.coeffs().allFinite() || normalized.norm() < 1.0e-6F) {
        normalized = Eigen::Quaternionf::Identity();
    } else {
        normalized.normalize();
    }
    return normalized.conjugate() * Eigen::Vector3f(0.0F, 0.0F, -1.0F);
}

bool ShapeEquals(const std::vector<int64_t> &shape, int expected)
{
    return shape.size() == 2 && shape[0] == 1 && shape[1] == expected;
}

}  // namespace

class Policy::Impl {
public:
    explicit Impl(const PolicyConfig *config)
        : config_(config), env_(ORT_LOGGING_LEVEL_WARNING, "lumo_p2_deploy"),
          memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
    {
    }

    bool Load(std::string *error)
    {
        if (!config_ || !std::filesystem::is_regular_file(config_->model_path)) {
            return Fail(error, "model not found: " +
                                   (config_ ? config_->model_path.string() : std::string("<null>")));
        }
        try {
            Ort::SessionOptions options;
            options.SetIntraOpNumThreads(1);
            options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            session_ = std::make_unique<Ort::Session>(env_, config_->model_path.c_str(), options);

            if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1) {
                return Fail(error, config_->name + " must expose exactly one input and output");
            }
            Ort::AllocatorWithDefaultOptions allocator;
            const auto input_name = session_->GetInputNameAllocated(0, allocator);
            const auto output_name = session_->GetOutputNameAllocated(0, allocator);
            if (std::string(input_name.get()) != "obs" ||
                std::string(output_name.get()) != "actions") {
                return Fail(error, config_->name + " requires obs -> actions tensor names");
            }
            const auto input_shape =
                session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            const auto output_shape =
                session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            if (!ShapeEquals(input_shape, config_->observation_size) ||
                !ShapeEquals(output_shape, static_cast<int>(kPolicyDofCount))) {
                std::ostringstream message;
                message << config_->name << " requires obs[1," << config_->observation_size
                        << "] -> actions[1," << kPolicyDofCount << "]";
                return Fail(error, message.str());
            }

            input_.assign(static_cast<std::size_t>(config_->observation_size), 0.0F);
            output_.assign(kPolicyDofCount, 0.0F);
            last_action_.assign(kPolicyDofCount, 0.0F);
            input_shape_ = {1, config_->observation_size};
            output_shape_ = {1, static_cast<int64_t>(kPolicyDofCount)};
            input_tensor_ = Ort::Value::CreateTensor<float>(
                memory_info_, input_.data(), input_.size(), input_shape_.data(), input_shape_.size());
            output_tensor_ = Ort::Value::CreateTensor<float>(
                memory_info_, output_.data(), output_.size(), output_shape_.data(),
                output_shape_.size());
            loaded_ = true;
            Reset();
            static constexpr const char *kInputNames[]{"obs"};
            static constexpr const char *kOutputNames[]{"actions"};
            session_->Run(Ort::RunOptions{nullptr}, kInputNames, &input_tensor_, 1,
                          kOutputNames, &output_tensor_, 1);
            if (!std::all_of(output_.begin(), output_.end(),
                             [](float value) { return std::isfinite(value); })) {
                loaded_ = false;
                return Fail(error, config_->name + " warmup produced a non-finite action");
            }
            Reset();
            if (error) {
                error->clear();
            }
            return true;
        } catch (const Ort::Exception &exception) {
            return Fail(error, config_->name + " ONNX load failed: " + exception.what());
        }
    }

    void Reset()
    {
        std::fill(input_.begin(), input_.end(), 0.0F);
        std::fill(output_.begin(), output_.end(), 0.0F);
        std::fill(last_action_.begin(), last_action_.end(), 0.0F);
        first_step_ = true;
        policy_step_ = 0;
    }

    bool Infer(const RobotState &state, const OperatorInput &input, JointCommand *command,
               std::string *error)
    {
        if (!loaded_ || !session_ || !command) {
            return Fail(error, config_->name + " is not ready");
        }
        const VelocityCommand velocity = ScaleVelocity(input, *config_);
        if (!BuildObservation(state, velocity, error)) {
            return false;
        }

        try {
            static constexpr const char *kInputNames[]{"obs"};
            static constexpr const char *kOutputNames[]{"actions"};
            session_->Run(Ort::RunOptions{nullptr}, kInputNames, &input_tensor_, 1,
                          kOutputNames, &output_tensor_, 1);
        } catch (const Ort::Exception &exception) {
            return Fail(error, config_->name + " inference failed: " + exception.what());
        }

        if (!std::all_of(output_.begin(), output_.end(),
                         [](float value) { return std::isfinite(value); })) {
            return Fail(error, config_->name + " produced a non-finite action");
        }
        BuildCommand(command);
        last_action_ = output_;
        ++policy_step_;
        if (error) {
            error->clear();
        }
        return true;
    }

    bool IsLoaded() const { return loaded_; }

private:
    static bool Fail(std::string *error, std::string message)
    {
        if (error) {
            *error = std::move(message);
        }
        return false;
    }

    bool BuildObservation(const RobotState &state, const VelocityCommand &velocity,
                          std::string *error)
    {
        if (!state.orientation.coeffs().allFinite() || !state.angular_velocity.allFinite()) {
            return Fail(error, "robot IMU state is non-finite");
        }
        for (std::size_t i = 0; i < kJointCount; ++i) {
            if (!std::isfinite(state.q[i]) || !std::isfinite(state.dq[i])) {
                return Fail(error, "robot joint state is non-finite");
            }
        }
        if (config_->kind == PolicyKind::kMjlabVelocity) {
            return BuildMjlabObservation(state, velocity, error);
        }
        return BuildIsaacObservation(state, velocity, error);
    }

    bool BuildMjlabObservation(const RobotState &state, const VelocityCommand &velocity,
                               std::string *error)
    {
        if (input_.size() != kMjlabObservationSize) {
            return Fail(error, "mjlab_velocity observation buffer has the wrong size");
        }
        std::size_t offset = 0;
        const auto append = [this, &offset](const auto &values) {
            for (const auto value : values) {
                input_[offset++] = std::clamp(static_cast<float>(value), -100.0F, 100.0F);
            }
        };

        append(std::array<float, 3>{state.angular_velocity.x(), state.angular_velocity.y(),
                                    state.angular_velocity.z()});
        const Eigen::Vector3f gravity = ProjectedGravity(state.orientation);
        append(std::array<float, 3>{gravity.x(), gravity.y(), gravity.z()});
        append(std::array<float, 3>{velocity.x, velocity.y, velocity.yaw});

        constexpr double kTwoPi = 6.28318530717958647692;
        float phase_sin = 0.0F;
        float phase_cos = 0.0F;
        if (std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                      velocity.yaw * velocity.yaw) >= 0.1F) {
            const double phase =
                std::fmod(static_cast<double>(policy_step_) * 0.02, 0.6) / 0.6;
            phase_sin = static_cast<float>(std::sin(kTwoPi * phase));
            phase_cos = static_cast<float>(std::cos(kTwoPi * phase));
        }
        append(std::array<float, 2>{phase_sin, phase_cos});

        for (std::size_t policy_index = 0; policy_index < kPolicyDofCount; ++policy_index) {
            const std::size_t motor = static_cast<std::size_t>(config_->joint_mapping[policy_index]);
            input_[offset++] = std::clamp(state.q[motor] - DefaultPose()[motor], -100.0F, 100.0F);
        }
        for (std::size_t policy_index = 0; policy_index < kPolicyDofCount; ++policy_index) {
            const std::size_t motor = static_cast<std::size_t>(config_->joint_mapping[policy_index]);
            input_[offset++] = std::clamp(state.dq[motor], -100.0F, 100.0F);
        }
        append(last_action_);
        return offset == input_.size() || Fail(error, "mjlab_velocity observation packing failed");
    }

    bool BuildIsaacObservation(const RobotState &state, const VelocityCommand &velocity,
                               std::string *error)
    {
        if (input_.size() != static_cast<std::size_t>(kIsaacStepSize * config_->history_length) ||
            config_->history_length != 5) {
            return Fail(error, "isaaclab_velocity_opensource history buffer has the wrong size");
        }

        std::array<float, kIsaacStepSize> step{};
        std::size_t offset = 0;
        const auto append = [&step, &offset](const auto &values) {
            for (const auto value : values) {
                step[offset++] = std::clamp(static_cast<float>(value), -100.0F, 100.0F);
            }
        };
        append(std::array<float, 3>{state.angular_velocity.x() * 0.2F,
                                    state.angular_velocity.y() * 0.2F,
                                    state.angular_velocity.z() * 0.2F});
        const Eigen::Vector3f gravity = ProjectedGravity(state.orientation);
        append(std::array<float, 3>{gravity.x(), gravity.y(), gravity.z()});
        append(std::array<float, 3>{velocity.x, velocity.y, velocity.yaw});

        for (std::size_t policy_index = 0; policy_index < kPolicyDofCount; ++policy_index) {
            const std::size_t motor = static_cast<std::size_t>(config_->joint_mapping[policy_index]);
            step[offset++] = std::clamp(state.q[motor] - DefaultPose()[motor], -100.0F, 100.0F);
        }
        for (std::size_t policy_index = 0; policy_index < kPolicyDofCount; ++policy_index) {
            const std::size_t motor = static_cast<std::size_t>(config_->joint_mapping[policy_index]);
            step[offset++] = std::clamp(state.dq[motor] * 0.05F, -100.0F, 100.0F);
        }
        append(last_action_);
        if (offset != step.size()) {
            return Fail(error, "isaaclab_velocity_opensource observation packing failed");
        }

        // Term-major history: each term owns a five-frame window. This matches
        // the exported IsaacLab policy contract exactly.
        constexpr std::array<int, 6> kTermDimensions{3, 3, 3, 29, 29, 29};
        std::size_t step_offset = 0;
        std::size_t history_offset = 0;
        for (const int dimension : kTermDimensions) {
            const std::size_t dim = static_cast<std::size_t>(dimension);
            const std::size_t window = dim * static_cast<std::size_t>(config_->history_length);
            if (first_step_) {
                for (int frame = 0; frame < config_->history_length; ++frame) {
                    std::copy_n(step.data() + step_offset, dim,
                                input_.data() + history_offset + static_cast<std::size_t>(frame) * dim);
                }
            } else {
                std::memmove(input_.data() + history_offset,
                             input_.data() + history_offset + dim,
                             (window - dim) * sizeof(float));
                std::copy_n(step.data() + step_offset, dim,
                            input_.data() + history_offset + window - dim);
            }
            step_offset += dim;
            history_offset += window;
        }
        first_step_ = false;
        return true;
    }

    void BuildCommand(JointCommand *command) const
    {
        command->q = DefaultPose();
        command->dq.fill(0.0F);
        command->kp = FixedKp();
        command->kd = FixedKd();
        command->tau.fill(0.0F);
        if (config_->kind == PolicyKind::kMjlabVelocity) {
            // This policy intentionally leaves the head passive.
            command->kp[kJointCount - 1] = 0.0F;
            command->kd[kJointCount - 1] = 0.0F;
        }

        for (std::size_t policy_index = 0; policy_index < kPolicyDofCount; ++policy_index) {
            const std::size_t motor = static_cast<std::size_t>(config_->joint_mapping[policy_index]);
            command->q[motor] = DefaultPose()[motor] + output_[policy_index] *
                                                           config_->action_scale[motor];
            const std::size_t gain_motor =
                static_cast<std::size_t>(config_->gain_mapping[policy_index]);
            command->kp[gain_motor] = config_->kp[policy_index];
            command->kd[gain_motor] = config_->kd[policy_index];
        }
    }

    const PolicyConfig *config_ = nullptr;
    Ort::Env env_;
    Ort::MemoryInfo memory_info_;
    std::unique_ptr<Ort::Session> session_;
    Ort::Value input_tensor_{nullptr};
    Ort::Value output_tensor_{nullptr};
    std::vector<float> input_;
    std::vector<float> output_;
    std::vector<float> last_action_;
    std::array<int64_t, 2> input_shape_{};
    std::array<int64_t, 2> output_shape_{};
    std::uint64_t policy_step_ = 0;
    bool first_step_ = true;
    bool loaded_ = false;
};

Policy::Policy(PolicyConfig config)
    : config_(std::move(config)), impl_(std::make_unique<Impl>(&config_))
{
}

Policy::~Policy() = default;

bool Policy::Load(std::string *error) { return impl_->Load(error); }
void Policy::Reset() { impl_->Reset(); }
bool Policy::Infer(const RobotState &state, const OperatorInput &input, JointCommand *command,
                   std::string *error)
{
    return impl_->Infer(state, input, command, error);
}
bool Policy::IsLoaded() const { return impl_->IsLoaded(); }

}  // namespace lumo_deploy
