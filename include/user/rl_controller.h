//
// Created by cyy on 2020/12/27.
//

#include "onnx/onnxruntime_cxx_api.h"
#include "onnx_inference.h"
#include "utils/config.h"
#include "utils/cpp_types.h"
#include "unitree/g1/motors.hpp"
#include "unitree/g1/data_buffer.hpp"
#include "unitree/g1/base_state.hpp"
#include "utils/orientation_tools.h"
#include "unitree/g1/joystick.hpp"
#include <time.h>
#include <sys/time.h>
#include "joystick_reader.h"

class RLController {
public:
    explicit RLController() = default;

public:
    bool _is_first_run = true;
    int counter_print = 0;
    int counter_rl = 0;
    int task_mode = 0;
    float _rl_time_step = 0.01f;
    vector <vector<float>> sim_gait_data;
    float _record_yaw = 0.;
    float static_flag = 0.f;

    static const int NUM_LEGS = 2;
    static const int NUM_JOINTS = 10;
    static const int NUM_ACTUAT_JOINTS = 10;

    Vec2<float> target_command;
    Vec2<float> pm_f;
    Vec4<float> pm_phase_sin_cos;
    Vec3<float> base_rpy, base_rpy_rate, base_vel, base_acc;
    Vec4<float> base_quat;

    Vec10<float> joint_pos, joint_vel, joint_tau, joint_acc, joint_pos_error, joint_act, init_joint_act, motion_test_start_joint_act,
            motion_test_end_joint_act;
    Vec10<float> output_joint_act;

    Matrix<float, Dynamic, 1> action_increment;
    Matrix<float, Dynamic, 1> observation;
    DataBuffer<MotorCommand> *dds_motor_command = nullptr;
    DataBuffer<MotorState> *dds_motor_state = nullptr;
    DataBuffer<BaseState> *dds_base_state = nullptr;
    Gamepad *gamepad = nullptr;

    JoystickReader *jsreader = nullptr;


    Vec10<float> _kp, _kd;
    Vec10<float> _kp_soft, _kd_soft;

    ConfigParams configParams;


private:
    Vec10<int> jointIndex2Sim;

    Vec2<float> _pm_phase;
    Vec10<float> _ref_joint_act, _offset_joint_act;

    OnnxInference onnxInference;
    Ort::Session *motion_session = nullptr;
    pthread_mutex_t _rl_state_mutex{};
    Vec10<float> act_pos_low, act_pos_high;
    std::vector <Matrix<float, Dynamic, 1>> obs_stack;

    float smallest_signed_angle_between(float alpha, float beta);

    void joint_increment_control(Matrix<float, Dynamic, 1> increment);

    void compute_pm_phase(Vec2<float> f);

    Matrix<float, Dynamic, 1> get_observation();

    Matrix<float, Dynamic, -1> transform(Matrix<float, Dynamic, -1> data);

    float exp_filter(float history, float present, float weight);

    void joystick_command_process();


    void smooth_joint_action(float ratio, const Vec10<float> &end_joint_act);

    float get_true_loop_period();

public:
    void init();

    Vec3<float> quat_rotate_inverse(Vec4<float> q, Vec3<float> v);

    Vec4<float> quat_product(Vec4<float> &q1, Vec4<float> &q2);

    Vec4<float> rpy_to_quat(const Vec3<float> &rpy);

    Vec4<float> quat_mul(Vec4<float> a, Vec4<float> b);

    static Vec3<float> convert_world_frame_to_base_frame(const Vec3<float> &world_vec, const Vec3<float> &rpy);


    void rl_control();

    void sim_gait_control();

    void reset(bool is_test_local);

    void stand_control(float ratio);

    void sin_control(float amplitude, float f, float motion_time);

    void convert_dds_state2rl_state();

    void set_rl_joint_act2dds_motor_command(char mode);

public:
    virtual ~RLController() {
        delete motion_session;
        delete dds_base_state;
        delete dds_motor_state;
        delete dds_motor_command;
        delete jsreader;
        delete gamepad;
    }
};
