#include "user/rl_controller.h"
#include <algorithm>

void RLController::init() {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "q1");
    Ort::SessionOptions session_options;
    motion_session = new Ort::Session(env, "policy.onnx", session_options);
    _offset_joint_act.setZero();
    onnxInference.init(configParams.num_observations, configParams.num_actions, configParams.num_stacks);
    jointIndex2Sim << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
    base_rpy.setZero();
    base_vel.setZero();
    joint_pos.setZero();
    joint_vel.setZero();
    joint_tau.setZero();
    joint_acc.setZero();
    base_acc.setZero();
    base_quat << 1, 0, 0, 0;
    base_rpy_rate.setZero();
    target_command.setZero();
    joint_pos_error.setZero();
    pm_f.setConstant(0.5f);
    _pm_phase << 0, 0;
    pm_phase_sin_cos.setZero();
    action_increment.resize(onnxInference.output_dim);
    action_increment.setZero();

    for (int i = 0; i < NUM_JOINTS; ++i) {
        act_pos_low[i] = configParams.act_pos_low.at(i);
        act_pos_high[i] = configParams.act_pos_high.at(i);
        _ref_joint_act[i] = configParams.ref_joint_act.at(i);
        _kp[i] = configParams.kp.at(i);
        _kd[i] = configParams.kd.at(i);
        _kp_soft[i] = configParams.kp_soft.at(i);
        _kd_soft[i] = configParams.kd_soft.at(i);
    }
    joint_act = _ref_joint_act;
    observation.resize(onnxInference.input_dim * onnxInference.stack_dim);
    observation.setZero();
    obs_stack.resize(onnxInference.stack_dim);
    for (int i(0); i < onnxInference.stack_dim; i++)obs_stack.at(i).resize(onnxInference.input_dim);
    for (int i(0); i < onnxInference.stack_dim; i++)obs_stack.at(i).setZero(onnxInference.input_dim);

}

void RLController::reset(bool is_test_local) {
    pm_f.setConstant(0.5f);
    _pm_phase << 0, 0;
    target_command.setZero();
    _is_first_run = true;
    counter_rl = 0;
    if (is_test_local) {
        init_joint_act = joint_act;
        joint_pos = joint_act;
    } else {
        convert_dds_state2rl_state();
        joint_act = joint_pos;
        init_joint_act = joint_pos;
        _record_yaw = base_rpy[2];//todo
        cout << "Reset done! Rpy: " << base_rpy.transpose() << endl;
    }
    auto o = get_observation();
}


void RLController::rl_control() {
    counter_rl++;
    Matrix<float, Dynamic, 1> net_out;
    net_out = onnxInference.inference(motion_session, get_observation());
    action_increment = transform(net_out);
    joint_increment_control(action_increment);
    _rl_time_step = get_true_loop_period();
}

void RLController::joint_increment_control(Matrix<float, Dynamic, 1> increment) {
    pm_f = increment.segment(0, NUM_LEGS);
    compute_pm_phase(pm_f);
    joint_act.segment(0, NUM_ACTUAT_JOINTS) += increment.segment(NUM_LEGS, NUM_ACTUAT_JOINTS) * _rl_time_step;
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
    // cout << "joint_act: " << joint_act.transpose() << endl;
    // exit(1);
}


Matrix<float, Dynamic, 1> RLController::get_observation() {
    Matrix<float, Dynamic, 1> obs;
    Vec2<float> con_1;
    con_1.setOnes();
    obs.resize(onnxInference.input_dim);
    obs.setZero();
    pthread_mutex_lock(&_rl_state_mutex);
    joint_pos_error = joint_act - joint_pos;
    for (int i(0); i < NUM_LEGS; i++) {
        pm_phase_sin_cos(i) = sin(_pm_phase[i]);
        pm_phase_sin_cos(NUM_LEGS + i) = cos(_pm_phase[i]);
    }
    joystick_command_process();
    if (sqrt(pow(target_command(0), 2) + pow(target_command(1), 2)) < 0.15)
        static_flag = 0.f;
    else
        static_flag = 1.f;
    obs << target_command,
            base_rpy.segment(0, 2),
            base_rpy_rate * 0.5,
            joint_pos.segment(0, NUM_ACTUAT_JOINTS) - _ref_joint_act,
            joint_vel.segment(0, NUM_ACTUAT_JOINTS) * 0.1f,
            joint_pos_error.segment(0, NUM_ACTUAT_JOINTS),
            pm_phase_sin_cos * static_flag,
            (pm_f * 0.3 - con_1) * static_flag;
    obs = obs.cwiseMax(-3.).cwiseMin(3.);


    pthread_mutex_unlock(&_rl_state_mutex);
    if (int(observation.size()) != onnxInference.input_dim * onnxInference.stack_dim) {
        cout << "The dimension of the input size observation is error!!!" << endl;
        cout << "True state size:" << observation.size() << "Policy input size:" << onnxInference.input_dim * onnxInference.stack_dim << endl;
        exit(1);
    }

    if (_is_first_run) {
        for (int i(0); i < onnxInference.stack_dim; i++) {
            obs_stack.erase(obs_stack.begin());
            obs_stack.push_back(obs);
        }
        _is_first_run = false;
        cout << endl << "Reset observation history: Done!" << endl;
    } else {
        obs_stack.erase(obs_stack.begin());
        obs_stack.push_back(obs);
    }
    for (int i(0); i < onnxInference.stack_dim; i++) {
        for (int j(0); j < onnxInference.input_dim; j++) {
            observation[onnxInference.input_dim * i + j] = obs_stack.at(i)[j];
        }
    }
    return observation;
}


void RLController::joystick_command_process() {
    float vx_cmd = 0,  yr_cmd = 0;
    auto yr_max = configParams.yr_cmd_range.at(1);
    auto vx_min = configParams.vx_cmd_range.at(0);
    auto vx_max = configParams.vx_cmd_range.at(1);
    if (task_mode == 3 or task_mode == 4) {
        ///stand
        vx_cmd = -vx_max * jsreader->Axis[1];
        yr_cmd = -yr_max * jsreader->Axis[2];

        if (fabs(yr_cmd) > 0.1 or configParams.kp_yaw_ctrl < 1e-2 or static_flag < 0.1) {
            _record_yaw = base_rpy[2];//todo
        } else {
            yr_cmd = configParams.kp_yaw_ctrl * smallest_signed_angle_between(base_rpy[2], _record_yaw);
        }

        yr_cmd = std::clamp(yr_cmd, -yr_max, yr_max);
        vx_cmd = std::clamp(vx_cmd, vx_min, vx_max);
    }
    target_command << vx_cmd, yr_cmd;
}

void RLController::set_rl_joint_act2dds_motor_command(char mode) {
    MotorCommand motor_command_tmp;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        motor_command_tmp.q_target[i] = joint_act[jointIndex2Sim[i]];
        if (mode=='q') {
            motor_command_tmp.kp[i] = 0.;
            motor_command_tmp.kd[i] = 0.;
        } else if (mode=='1') {
            motor_command_tmp.kp[i] = _kp_soft[jointIndex2Sim[i]];
            motor_command_tmp.kd[i] = _kd_soft[jointIndex2Sim[i]];
        }
        else {
            motor_command_tmp.kp[i] = _kp[jointIndex2Sim[i]];
            motor_command_tmp.kd[i] = _kd[jointIndex2Sim[i]];
        }
        motor_command_tmp.tau_ff[i] = 0.;
        motor_command_tmp.dq_target[i] = 0.;
    }
    dds_motor_command->SetData(motor_command_tmp);
}

void RLController::convert_dds_state2rl_state() {
    Vec3<float> trans_axis(-1., 1, -1);
    if (dds_motor_state->GetData()) {
        for (int i = 0; i < NUM_JOINTS; ++i) {
            joint_pos[jointIndex2Sim[i]] = exp_filter(joint_pos[jointIndex2Sim[i]], dds_motor_state->GetData()->q[i], 0.2);
            joint_vel[jointIndex2Sim[i]] = exp_filter(joint_vel[jointIndex2Sim[i]], dds_motor_state->GetData()->dq[i], 0.1);
            joint_tau[jointIndex2Sim[i]] = dds_motor_state->GetData()->tau_est[i];
            joint_acc[jointIndex2Sim[i]] = dds_motor_state->GetData()->ddq[i];
        }
    }
    if (dds_base_state->GetData()) {
        for (int i(0); i < 3; i++) {
            base_rpy(i) = exp_filter(base_rpy(i), fmod(dds_base_state->GetData()->rpy.at(i) * trans_axis(i), 2 * M_PI), 0.2);
            base_rpy_rate(i) = exp_filter(base_rpy_rate(i), dds_base_state->GetData()->omega.at(i) * trans_axis(i), 0.1);
            base_acc(i) = exp_filter(base_acc(i), dds_base_state->GetData()->acc.at(i) * trans_axis(i), 0.1);
        }
    }
    counter_print++;
}


void RLController::compute_pm_phase(Vec2<float> f) {
    for (int leg(0); leg < NUM_LEGS; leg++) {
        _pm_phase[leg] += 2. * M_PI * f[leg] * _rl_time_step;
        _pm_phase[leg] = fmod(_pm_phase[leg], 2 * M_PI);
    }
}

Matrix<float, Dynamic, -1> RLController::transform(Matrix<float, Dynamic, -1> data) {
    auto net = (data.array() + 1.) / 2.;
    int ii = 0;
    for (int i(0); i < onnxInference.output_dim; i++) {
        if (i < NUM_LEGS)
            ii = 0;
        else if (i < NUM_LEGS + NUM_ACTUAT_JOINTS + 1)
            ii = 1;
        else
            ii = 2;
        action_increment(i) = net(i) * (configParams.act_inc_high[ii] - configParams.act_inc_low[ii]) + configParams.act_inc_low[ii];
    }
    return action_increment;
}


void RLController::smooth_joint_action(float ratio, const Vec10<float> &end_joint_act) {
    joint_act = (1 - ratio) * init_joint_act + ratio * end_joint_act;
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
}

float RLController::exp_filter(float history, float present, float weight) {
    auto result = history * weight + present * (1. - weight);
    return result;
}


void RLController::sin_control(float amplitude, float f, float motion_time) {
    Vec10<float> sin_joint_act;
    sin_joint_act.setConstant(amplitude * sin(2.f * M_PI * f * motion_time));
    if (configParams.sin_joint_idx == -1)
        joint_act.segment(0, NUM_ACTUAT_JOINTS) = init_joint_act.segment(0, NUM_ACTUAT_JOINTS) + sin_joint_act;
    else
        joint_act[configParams.sin_joint_idx] = init_joint_act[configParams.sin_joint_idx] + sin_joint_act[configParams.sin_joint_idx];
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
}


float RLController::get_true_loop_period() {
    static struct timeval last_time;
    static struct timeval now_time;
    static bool first_get_time = true;
    if (first_get_time) {
        gettimeofday(&last_time, nullptr);
        first_get_time = false;
    }
    gettimeofday(&now_time, nullptr);
    auto d_time = (float) (now_time.tv_sec - last_time.tv_sec) +
                  (float) (now_time.tv_usec - last_time.tv_usec) / 1000000;
    last_time = now_time;
    if (fabs(d_time - _rl_time_step) * 1000. > 2.)
        cout << "True period: " << d_time * 1000. << " ms" << endl;
    return d_time;
}

void RLController::stand_control(float ratio) {
    smooth_joint_action(ratio, _ref_joint_act);
}

void RLController::sim_gait_control() {
    static int data_index = 0;
    if (data_index <= sim_gait_data.size() - 2)
        data_index++;
    for (int i(0); i < NUM_ACTUAT_JOINTS; i++) {
        joint_act(i) = sim_gait_data.at(data_index).at(i);
    }
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
}

Vec3<float> RLController::convert_world_frame_to_base_frame(const Vec3<float> &world_vec, const Vec3<float> &rpy) {
    return ori::rpy_to_rotMat(rpy) * world_vec;
}

Vec3<float> RLController::quat_rotate_inverse(Vec4<float> q, Vec3<float> v) {
    Vec3<float> a, b, c;
    // q={w,x,y,z}
    // q << 0.99981624, -0.013256095, 0.012793032, -0.005295367; //todo: in isaac gym: x,y,z,w ;here IMU(q): w,x,y,z !!
    // v << -0.13947208, -0.08728597, 0.19939381;
    // result: lin_v(-0.14353749,-0.09399233,0.19336908)
    float q_w = q[0];
    Vec3<float> q_vec(q[1], q[2], q[3]);
    a = v * (2.0 * q_w * q_w - 1.0);
    b = q_vec.cross(v) * q_w * 2.0;
    c = q_vec * q_vec.transpose() * v * 2.0;
    // cout << "quat_rotate_inverse: " << (a - b + c).transpose() << endl << endl;
    return a - b + c;
}

/*!
 * Take the product of two quaternions
 */
Vec4<float> RLController::quat_product(Vec4<float> &q1, Vec4<float> &q2) {
    float r1 = q1[0];
    float r2 = q2[0];

    Vec3<float> v1(q1[1], q1[2], q1[3]);
    Vec3<float> v2(q2[1], q2[2], q2[3]);

    float r = r1 * r2 - v1.dot(v2);
    Vec3<float> v = r1 * v2 + r2 * v1 + v1.cross(v2);
    Vec4<float> q(r, v[0], v[1], v[2]);
    return q;
}

float RLController::smallest_signed_angle_between(float alpha, float beta) {
    auto a = beta - alpha;
    a += (a > M_PI) ? -2 * M_PI : (a < -M_PI) ? 2 * M_PI : 0.;
    return a;
}

Vec4<float> RLController::rpy_to_quat(const Vec3<float> &rpy) {
    auto R = ori::rpy_to_rotMat(rpy);
    auto q = ori::rotMat_to_quat(R);
    return q;
}

Vec4<float> RLController::quat_mul(Vec4<float> a, Vec4<float> b) {
    float x1 = a[1];
    float y1 = a[2];
    float z1 = a[3];
    float w1 = a[0];

    float x2 = b[1];
    float y2 = b[2];
    float z2 = b[3];
    float w2 = b[0];

    float ww = (z1 + x1) * (x2 + y2);
    float yy = (w1 - y1) * (w2 + z2);
    float zz = (w1 + y1) * (w2 - z2);
    float xx = ww + yy + zz;
    float qq = 0.5 * (xx + (z1 - x1) * (x2 - y2));

    float w = qq - ww + (z1 - y1) * (y2 - z2);
    float x = qq - xx + (x1 + w1) * (x2 + w2);
    float y = qq - yy + (w1 - x1) * (y2 + z2);
    float z = qq - zz + (z1 + y1) * (w2 - x2);

    Vec4<float> quat(w, x, y, z);

    return quat;
}