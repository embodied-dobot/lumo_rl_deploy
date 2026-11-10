#pragma once

#include "lumo_deploy/config.hpp"
#include "lumo_deploy/types.hpp"

#include <memory>
#include <string>

namespace lumo_deploy {

class Policy {
public:
    explicit Policy(PolicyConfig config);
    ~Policy();

    Policy(const Policy &) = delete;
    Policy &operator=(const Policy &) = delete;

    bool Load(std::string *error);
    void Reset();
    bool Infer(const RobotState &state, const OperatorInput &input, JointCommand *command,
               std::string *error);

    const PolicyConfig &Config() const { return config_; }
    bool IsLoaded() const;

private:
    class Impl;
    PolicyConfig config_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lumo_deploy

