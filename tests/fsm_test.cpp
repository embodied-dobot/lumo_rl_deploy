#include "lumo_deploy/controller.hpp"

#include <cstdlib>
#include <iostream>

namespace {

void Expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

lumo_deploy::OperatorInput Released()
{
    return {};
}

}  // namespace

int main()
{
    using lumo_deploy::Mode;
    using lumo_deploy::OperatorInput;
    using lumo_deploy::StateMachine;

    StateMachine fsm;
    Expect(fsm.Current() == Mode::kDamping, "startup must be damping");

    OperatorInput input;
    input.rb_a = true;
    Expect(fsm.Update(input, false) == Mode::kInitialPose, "RB+A enters initial pose");
    fsm.Update(Released(), false);

    input = {};
    input.rb_up = true;
    Expect(fsm.Update(input, false) == Mode::kInitialPose,
           "policy cannot start before initial pose is ready");
    fsm.Update(Released(), true);
    input.rb_up = true;
    Expect(fsm.Update(input, true) == Mode::kIsaacLabVelocity,
           "RB+Up starts IsaacLab velocity");
    fsm.Update(Released(), true);

    input = {};
    input.rb_down = true;
    Expect(fsm.Update(input, true) == Mode::kMjlabVelocity,
           "RB+Down switches directly to MJLab velocity");
    fsm.Update(Released(), true);

    input = {};
    input.rb_up = true;
    Expect(fsm.Update(input, true) == Mode::kIsaacLabVelocity,
           "RB+Up switches directly back to IsaacLab velocity");
    fsm.Update(Released(), true);

    input = {};
    input.rb_a = true;
    Expect(fsm.Update(input, true) == Mode::kInitialPose,
           "RB+A returns to initial pose from a policy");
    fsm.Update(Released(), true);

    input = {};
    input.rb_x = true;
    Expect(fsm.Update(input, true) == Mode::kDamping,
           "RB+X enters damping from any state");
    fsm.Update(Released(), true);

    input = {};
    input.rb_x = true;
    input.rb_a = true;
    Expect(fsm.Update(input, true) == Mode::kDamping,
           "RB+X has priority when safety combinations overlap");

    std::cout << "FSM checks passed\n";
    return 0;
}
