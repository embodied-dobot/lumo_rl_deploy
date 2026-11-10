#include "lumo_deploy/config.hpp"
#include "lumo_deploy/controller.hpp"
#include "lumo_deploy/types.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace {

using lumo_deploy::Controller;
using lumo_deploy::JointCommand;
using lumo_deploy::Mode;
using lumo_deploy::OperatorInput;
using lumo_deploy::RobotState;
using lumo_deploy::kJointCount;

constexpr double kControlDt = 0.002;

struct ModelDeleter {
    void operator()(mjModel *model) const { mj_deleteModel(model); }
};

struct DataDeleter {
    void operator()(mjData *data) const { mj_deleteData(data); }
};

class Simulator {
public:
    bool Load(const std::filesystem::path &path, std::string *error)
    {
        std::array<char, 1024> load_error{};
        model_.reset(mj_loadXML(path.c_str(), nullptr, load_error.data(), load_error.size()));
        if (!model_) {
            return Fail(error, "MuJoCo load failed: " + std::string(load_error.data()));
        }
        model_->opt.timestep = kControlDt;
        data_.reset(mj_makeData(model_.get()));
        if (!data_) {
            return Fail(error, "MuJoCo data allocation failed");
        }
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const std::string &name = lumo_deploy::JointNames()[i];
            const int joint_id = mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str());
            const int actuator_id = mj_name2id(model_.get(), mjOBJ_ACTUATOR, name.c_str());
            if (joint_id < 0 || actuator_id < 0) {
                return Fail(error, "MuJoCo joint or actuator missing: " + name);
            }
            qpos_address_[i] = model_->jnt_qposadr[joint_id];
            dof_address_[i] = model_->jnt_dofadr[joint_id];
            actuator_id_[i] = actuator_id;
        }
        const int quaternion_sensor = mj_name2id(model_.get(), mjOBJ_SENSOR, "imu_quat");
        const int gyro_sensor =
            mj_name2id(model_.get(), mjOBJ_SENSOR, "imu-pelvis-angular-velocity");
        if (quaternion_sensor < 0 || gyro_sensor < 0) {
            return Fail(error, "MuJoCo IMU sensors are missing");
        }
        quaternion_address_ = model_->sensor_adr[quaternion_sensor];
        gyro_address_ = model_->sensor_adr[gyro_sensor];
        Reset();
        if (error) {
            error->clear();
        }
        return true;
    }

    void Reset()
    {
        mj_resetData(model_.get(), data_.get());
        data_->qpos[0] = 0.0;
        data_->qpos[1] = 0.0;
        data_->qpos[2] = 0.9;
        data_->qpos[3] = 1.0;
        data_->qpos[4] = 0.0;
        data_->qpos[5] = 0.0;
        data_->qpos[6] = 0.0;
        for (std::size_t i = 0; i < kJointCount; ++i) {
            data_->qpos[qpos_address_[i]] = lumo_deploy::DefaultPose()[i];
        }
        mj_forward(model_.get(), data_.get());
    }

    RobotState ReadState() const
    {
        RobotState state;
        for (std::size_t i = 0; i < kJointCount; ++i) {
            state.q[i] = static_cast<float>(data_->qpos[qpos_address_[i]]);
            state.dq[i] = static_cast<float>(data_->qvel[dof_address_[i]]);
        }
        const mjtNum *quaternion = data_->sensordata + quaternion_address_;
        state.orientation = Eigen::Quaternionf(static_cast<float>(quaternion[0]),
                                                static_cast<float>(quaternion[1]),
                                                static_cast<float>(quaternion[2]),
                                                static_cast<float>(quaternion[3]));
        const mjtNum *gyro = data_->sensordata + gyro_address_;
        state.angular_velocity = Eigen::Vector3f(static_cast<float>(gyro[0]),
                                                 static_cast<float>(gyro[1]),
                                                 static_cast<float>(gyro[2]));
        return state;
    }

    void Step(const JointCommand &command)
    {
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const float q = static_cast<float>(data_->qpos[qpos_address_[i]]);
            const float dq = static_cast<float>(data_->qvel[dof_address_[i]]);
            const float raw_torque = command.kp[i] * (command.q[i] - q) +
                                     command.kd[i] * (command.dq[i] - dq) + command.tau[i];
            const float limit = lumo_deploy::TorqueLimits()[i];
            data_->ctrl[actuator_id_[i]] = std::clamp(raw_torque, -limit, limit);
        }
        mj_step(model_.get(), data_.get());
    }

    mjModel *Model() const { return model_.get(); }
    mjData *Data() const { return data_.get(); }

private:
    static bool Fail(std::string *error, std::string message)
    {
        if (error) {
            *error = std::move(message);
        }
        return false;
    }

    std::unique_ptr<mjModel, ModelDeleter> model_;
    std::unique_ptr<mjData, DataDeleter> data_;
    std::array<int, kJointCount> qpos_address_{};
    std::array<int, kJointCount> dof_address_{};
    std::array<int, kJointCount> actuator_id_{};
    int quaternion_address_ = -1;
    int gyro_address_ = -1;
};

bool Key(GLFWwindow *window, int key)
{
    return glfwGetKey(window, key) == GLFW_PRESS;
}

struct SimulationInput {
    OperatorInput operator_input;
    bool reset_scene = false;
};

SimulationInput ReadSimulationInput(GLFWwindow *window)
{
    SimulationInput input;
    GLFWgamepadstate gamepad{};
    if (glfwGetGamepadState(GLFW_JOYSTICK_1, &gamepad) == GLFW_TRUE) {
        const bool rb = gamepad.buttons[GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER] == GLFW_PRESS;
        input.operator_input.rb_a =
            rb && gamepad.buttons[GLFW_GAMEPAD_BUTTON_A] == GLFW_PRESS;
        input.operator_input.rb_x =
            rb && gamepad.buttons[GLFW_GAMEPAD_BUTTON_X] == GLFW_PRESS;
        input.operator_input.rb_up =
            rb && gamepad.buttons[GLFW_GAMEPAD_BUTTON_DPAD_UP] == GLFW_PRESS;
        input.operator_input.rb_down =
            rb && gamepad.buttons[GLFW_GAMEPAD_BUTTON_DPAD_DOWN] == GLFW_PRESS;
        input.reset_scene = rb && gamepad.buttons[GLFW_GAMEPAD_BUTTON_Y] == GLFW_PRESS;
        input.operator_input.forward = -gamepad.axes[GLFW_GAMEPAD_AXIS_LEFT_Y];
        input.operator_input.lateral = -gamepad.axes[GLFW_GAMEPAD_AXIS_LEFT_X];
        input.operator_input.yaw = -gamepad.axes[GLFW_GAMEPAD_AXIS_RIGHT_X];
        return input;
    }

    // Keyboard fallback mirrors the gamepad combinations with R as RB.
    const bool rb = Key(window, GLFW_KEY_R);
    input.operator_input.rb_a = rb && Key(window, GLFW_KEY_A);
    input.operator_input.rb_x = rb && Key(window, GLFW_KEY_X);
    input.operator_input.rb_up = rb && Key(window, GLFW_KEY_UP);
    input.operator_input.rb_down = rb && Key(window, GLFW_KEY_DOWN);
    input.reset_scene = rb && Key(window, GLFW_KEY_Y);
    input.operator_input.forward = (Key(window, GLFW_KEY_W) ? 1.0F : 0.0F) -
                                   (Key(window, GLFW_KEY_S) ? 1.0F : 0.0F);
    input.operator_input.lateral = (Key(window, GLFW_KEY_A) ? 1.0F : 0.0F) -
                                   (Key(window, GLFW_KEY_D) ? 1.0F : 0.0F);
    input.operator_input.yaw = (Key(window, GLFW_KEY_Q) ? 1.0F : 0.0F) -
                               (Key(window, GLFW_KEY_E) ? 1.0F : 0.0F);
    return input;
}

void PrintHelp()
{
    std::cout << "Usage: lumo_sim [--check] [--help]\n"
              << "  --check  Validate both policies and the MuJoCo scene without opening a window\n";
}

}  // namespace

int main(int argc, char **argv)
{
    bool check_only = false;
    for (int i = 1; i < argc; ++i) {
        const std::string argument(argv[i]);
        if (argument == "--help" || argument == "-h") {
            PrintHelp();
            return 0;
        }
        if (argument == "--check") {
            check_only = true;
            continue;
        }
        std::cerr << "Unknown option: " << argument << '\n';
        return 2;
    }

    const std::filesystem::path project_root(LUMO_PROJECT_ROOT);
    Controller controller;
    Simulator simulator;
    std::string error;
    if (!controller.Initialize(project_root, &error) ||
        !simulator.Load(project_root / "assets/lumo_p2/mjcf/scene.xml", &error)) {
        std::cerr << error << '\n';
        return 1;
    }
    if (check_only) {
        std::cout << "Policies and MuJoCo scene are ready\n";
        return 0;
    }

    if (glfwInit() != GLFW_TRUE) {
        std::cerr << "GLFW initialization failed\n";
        return 1;
    }
    GLFWwindow *window = glfwCreateWindow(1280, 720, "Dobot Lumo P2 sim2sim", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        std::cerr << "GLFW window creation failed\n";
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    camera.type = mjCAMERA_FREE;
    camera.distance = 3.0;
    camera.azimuth = 90.0;
    camera.elevation = -20.0;
    mjv_makeScene(simulator.Model(), &scene, 2000);
    mjr_makeContext(simulator.Model(), &context, mjFONTSCALE_150);

    std::cout << "Controls: RB+Y reset scene, RB+A initial pose, RB+X damping, "
                 "RB+Up IsaacLab, RB+Down MJLab\n";
    std::cout << "Keyboard fallback: hold R with Y/A/X/Up/Down; "
                 "W/S, A/D, Q/E command velocity\n";
    Mode printed_mode = controller.CurrentMode();
    std::cout << "Mode: " << lumo_deploy::ModeName(printed_mode) << '\n';

    JointCommand command;
    bool reset_was_pressed = false;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const SimulationInput input = ReadSimulationInput(window);
        const bool reset_edge = input.reset_scene && !reset_was_pressed;
        reset_was_pressed = input.reset_scene;
        if (reset_edge) {
            simulator.Reset();
            command = {};
            std::cout << "Scene reset. Mode remains "
                      << lumo_deploy::ModeName(controller.CurrentMode()) << ".\n";
        }
        const double frame_end = simulator.Data()->time + 1.0 / 60.0;
        while (simulator.Data()->time < frame_end) {
            const RobotState state = simulator.ReadState();
            if (!controller.Step(state, input.operator_input, static_cast<float>(kControlDt),
                                 &command, &error)) {
                std::cerr << "Controller entered damping: " << error << '\n';
            }
            simulator.Step(command);
        }

        if (controller.CurrentMode() != printed_mode) {
            printed_mode = controller.CurrentMode();
            std::cout << "Mode: " << lumo_deploy::ModeName(printed_mode) << '\n';
        }
        camera.lookat[0] = simulator.Data()->qpos[0];
        camera.lookat[1] = simulator.Data()->qpos[1];
        camera.lookat[2] = 0.8;
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        const mjrRect viewport{0, 0, width, height};
        mjv_updateScene(simulator.Model(), simulator.Data(), &option, nullptr, &camera, mjCAT_ALL,
                        &scene);
        mjr_render(viewport, &scene, &context);
        glfwSwapBuffers(window);
    }

    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
