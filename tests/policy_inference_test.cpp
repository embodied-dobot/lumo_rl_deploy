#include "lumo_deploy/config.hpp"
#include "lumo_deploy/policy.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

bool CheckPolicy(lumo_deploy::PolicyConfig config, const lumo_deploy::RobotState &state,
                 std::string *error)
{
    lumo_deploy::Policy policy(std::move(config));
    if (!policy.Load(error)) {
        return false;
    }
    lumo_deploy::JointCommand command;
    if (!policy.Infer(state, lumo_deploy::OperatorInput{}, &command, error)) {
        return false;
    }
    for (std::size_t i = 0; i < lumo_deploy::kJointCount; ++i) {
        if (!std::isfinite(command.q[i]) || !std::isfinite(command.kp[i]) ||
            !std::isfinite(command.kd[i])) {
            if (error) {
                *error = "policy produced a non-finite joint command";
            }
            return false;
        }
    }
    return true;
}

}  // namespace

int main()
{
    const std::filesystem::path root(LUMO_PROJECT_ROOT);
    lumo_deploy::RobotState state;
    state.q = lumo_deploy::DefaultPose();
    std::string error;
    if (!CheckPolicy(lumo_deploy::IsaacLabVelocityConfig(root), state, &error) ||
        !CheckPolicy(lumo_deploy::MjlabVelocityConfig(root), state, &error)) {
        std::cerr << error << '\n';
        return 1;
    }
    std::cout << "Policy inference checks passed\n";
    return 0;
}

