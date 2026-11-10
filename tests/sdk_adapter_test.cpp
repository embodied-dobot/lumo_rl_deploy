#include "lumo_deploy/sdk_adapter.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
void Expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}
bool Near(float a, float b) { return std::abs(a - b) < 1.0e-5F; }
}  // namespace

int main()
{
    using namespace lumo_deploy;
    const auto sr = TopicsForModel(SdkModel::kSr3588);
    const auto thro = TopicsForModel(SdkModel::kSrThro);
    Expect(std::string(sr.profile.default_robot_ip) == "192.168.5.2", "SR_3588 default IP");
    Expect(std::string(thro.profile.default_robot_ip) == "172.16.20.2", "SR_thro default IP");
    Expect(std::string(sr.profile.model) == "SR_3588" &&
           std::string(thro.profile.model) == "SR_thro", "model selection");
    Expect(std::string(sr.imu) == "rt/pelvis/imu/data" &&
           std::string(thro.imu) == "rt/pelvis/imu/data", "pelvis IMU selection");

    sdk_msg::LowerState_ lower;
    sdk_msg::UpperState_ upper;
    sdk_msg::IMUState_ imu;
    // Distinct values detect crossed limbs and waist/arm/head offsets.
    for (std::size_t i = 0; i < 30; ++i) {
        auto &m = i < 12 ? lower.motor_state()[i] : upper.motor_state()[i - 12];
        m.q(static_cast<float>(i) + 0.25F);
        m.dq(-static_cast<float>(i) - 0.5F);
        m.tau_est(static_cast<float>(i) + 0.75F);
    }
    // Extra lower-state slots are not the upper-body waist entries.
    lower.motor_state()[12].q(1000);
    lower.motor_state()[13].q(1001);
    lower.motor_state()[14].q(1002);
    imu.quaternion({2.0F, 0.0F, 0.0F, 2.0F});
    imu.gyroscope({0.3F, -0.4F, 0.5F});
    RobotState state;
    std::string error;
    Expect(DecodeSdkState(lower, upper, imu, &state, &error), "valid SDK state");
    for (std::size_t i = 0; i < 30; ++i) {
        Expect(Near(state.tau_est[i], static_cast<float>(i) + 0.75F), "all feedback torques map correctly");
        Expect(Near(state.q[i], static_cast<float>(i) + 0.25F), "all joint positions map correctly");
        Expect(Near(state.dq[i], -static_cast<float>(i) - 0.5F), "all velocities map correctly");
    }
    Expect(Near(state.orientation.w(), std::sqrt(0.5F)) &&
           Near(state.orientation.z(), std::sqrt(0.5F)), "wxyz quaternion normalization");
    Expect(Near(state.angular_velocity.x(), 0.3F) && Near(state.angular_velocity.y(), -0.4F) &&
           Near(state.angular_velocity.z(), 0.5F), "gyro axes and scale preserved");
    imu.quaternion({0, 0, 0, 0});
    Expect(!DecodeSdkState(lower, upper, imu, &state, &error), "zero quaternion rejected");
    imu.quaternion({1, 0, 0, 0});
    upper.motor_state()[17].dq(std::numeric_limits<float>::quiet_NaN());
    Expect(!DecodeSdkState(lower, upper, imu, &state, &error), "NaN joint rejected");
    Expect(Near(state.dq[29], -29.5F), "failed decode leaves prior state intact");
    upper.motor_state()[17].dq(0);
    imu.gyroscope()[1] = std::numeric_limits<float>::infinity();
    Expect(!DecodeSdkState(lower, upper, imu, &state, &error), "infinite gyro rejected");

    SdkGamepadDecoder decoder;
    sdk_msg::GamepadState_ gamepad;
    gamepad.ly(-1.0F);
    gamepad.lx(0.5F);
    gamepad.rx(-0.25F);
    OperatorInput input;
    Expect(decoder.Decode(gamepad, &input, &error), "valid gamepad");
    Expect(Near(input.forward, 0.2F) && Near(input.lateral, -0.1F) && Near(input.yaw, 0.05F),
           "legacy axis signs and low-pass filter");
    for (const unsigned bit : {8U, 10U, 12U, 14U}) {
        gamepad.btn_state(static_cast<uint16_t>(1U << bit));
        Expect(decoder.Decode(gamepad, &input, &error), "single button decode");
        Expect(!input.rb_a && !input.rb_x && !input.rb_up && !input.rb_down,
               "mode buttons require RB");
        gamepad.btn_state(static_cast<uint16_t>(1U | (1U << bit)));
        Expect(decoder.Decode(gamepad, &input, &error), "combo decode");
        Expect(input.rb_a == (bit == 8) && input.rb_x == (bit == 10) &&
               input.rb_up == (bit == 12) && input.rb_down == (bit == 14), "RB combo bit mapping");
    }
    gamepad.btn_state((1U << 0) | (1U << 8) | (1U << 10));
    Expect(decoder.Decode(gamepad, &input, &error) && !input.rb_a && !input.rb_x,
           "legacy decoder ignores ambiguous three-button combinations");
    gamepad.btn_state(0);
    Expect(decoder.Decode(gamepad, &input, &error) && !input.rb_down, "button release clears level");
    gamepad.btn_state(1U << 2);
    Expect(decoder.Decode(gamepad, &input, &error) && input.start && !input.back && !input.rb_a,
           "Start bit 2 does not switch motion mode");
    gamepad.btn_state(1U << 3);
    Expect(decoder.Decode(gamepad, &input, &error) && input.back && !input.start,
           "Back/Select bit 3");
    gamepad.btn_state(0);
    Expect(decoder.Decode(gamepad, &input, &error) && !input.start && !input.back,
           "recording buttons release");
    gamepad.lx(std::numeric_limits<float>::quiet_NaN());
    Expect(!decoder.Decode(gamepad, &input, &error), "NaN gamepad rejected");

    JointCommand command;
    for (std::size_t i = 0; i < 30; ++i) {
        const auto n = static_cast<float>(i);
        command.q[i] = n + 0.1F;
        command.dq[i] = -n - 0.2F;
        command.kp[i] = n + 0.3F;
        command.kd[i] = n + 0.4F;
        command.tau[i] = -n - 0.5F;
    }
    sdk_msg::LowerCmd_ lower_cmd;
    sdk_msg::UpperCmd_ upper_cmd;
    Expect(EncodeSdkCommand(command, &lower_cmd, &upper_cmd, &error), "valid command");
    for (std::size_t i = 0; i < 30; ++i) {
        const auto &m = i < 12 ? lower_cmd.motor_cmd()[i] : upper_cmd.motor_cmd()[i - 12];
        Expect(m.mode() == 11 && Near(m.q(), command.q[i]) && Near(m.dq(), command.dq[i]) &&
               Near(m.kp(), command.kp[i]) && Near(m.kd(), command.kd[i]) &&
               Near(m.tau(), command.tau[i]), "all six motor fields map correctly");
    }
    command.q[0] = 999;
    command.tau[29] = std::numeric_limits<float>::infinity();
    Expect(!EncodeSdkCommand(command, &lower_cmd, &upper_cmd, &error), "invalid final joint rejected");
    Expect(Near(lower_cmd.motor_cmd()[0].q(), 0.1F), "no partial command mutation on failure");
    command.tau[29] = 0;
    command.kd[0] = -1;
    Expect(!EncodeSdkCommand(command, &lower_cmd, &upper_cmd, &error), "negative damping rejected");

    ReceivedSdkSample<sdk_msg::LowerState_> cache;
    const auto t = decltype(cache)::Clock::now();
    Expect(!cache.Fresh(kSdkStateTimeout, t), "no fabricated initial state");
    cache.Update(lower, t);
    Expect(cache.Fresh(kSdkStateTimeout, t + std::chrono::milliseconds(100)).has_value(),
           "state at timeout boundary remains valid");
    Expect(!cache.Fresh(kSdkStateTimeout, t + std::chrono::milliseconds(101)), "stale state rejected");
    Expect(!cache.Fresh(kSdkStateTimeout, t + std::chrono::milliseconds(102)),
           "reading cached state does not refresh receipt time");
    cache.Update(lower, t + std::chrono::milliseconds(103));
    Expect(cache.Fresh(kSdkStateTimeout, t + std::chrono::milliseconds(104)).has_value(),
           "new sample refreshes state");
    std::cout << "SDK adapter tests passed (local conventions; not firmware validation)\n";
}
