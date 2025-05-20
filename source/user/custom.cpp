//
// Created by cyy on 24-10-7.
//

#include "user/custom.hpp"


void G1::ModeProcess() {
    if (_is_test_local) {
        selected_mode = modeSwitcher.get_selected_key(current_mode);
        ModeSwitcher::print_selected_mode(selected_mode);
    } else {
//        selected_mode = modeSwitcher.get_selected_stick(current_mode);
        selected_mode = modeSwitcher.get_selected_jskey(current_mode);
    }
    if (selected_mode != current_mode) {
        ModeSwitcher::print_selected_mode(selected_mode);
        /// mode transition
        relative_time = 0.;
        current_mode = selected_mode;
        rlController->reset(_is_test_local);
    }
    rlController->task_mode = modeSwitcher.rl_task_mode;
}

void G1::Control() {
    relative_time += control_dt_;
    float ratio = fmin(relative_time / MOVE_DURATION, 1.f);
    rlController->convert_dds_state2rl_state();
    switch (current_mode) {
        case 'q':
            rlController->_kp.setZero();
            rlController->_kd.setZero();
            dataReporter.close();
            usleep(1e3);
            exit(1);
        case '2':
            ///to stand
            rlController->stand_control(ratio);
            break;
        case '3':
            ///rl
            rlController->rl_control();
            if (rlController->counter_rl < 2)
                rlController->stand_control(ratio);
            break;
        case '5':
            // sin test (select)
            if (rlController->configParams.use_sim_gait)
                rlController->sim_gait_control();
            else
                rlController->sin_control(0.2, 2., relative_time);
            break;
        default: /// case '1'
            rlController->stand_control(ratio);
            break;
    }
    rlController->set_rl_joint_act2dds_motor_command(current_mode);
    control_count++;
    if (control_count % 150 == 0) {
        control_count = 0;
        cout << "q: " << rlController->joint_pos.transpose() << endl;
//        cout << "rpy: " << rlController->base_rpy.transpose() << endl;
    }
}

void G1::ReportData() {
    if (current_mode >= '1' and current_mode != 'q') {
         if (rlController->counter_rl >= 2) {
            dataReporter.report_data(rlController);
         }
    }
}

void G1::RunJoystick() {
    xRockerGamepad.Step();
}

void G1::RecordMotorState(const std::array<MotorData, 10> &data) {
    MotorState ms_tmp;
    for (int i = 0; i < 10; ++i) {
        ms_tmp.q.at(i) = data[i].q;
        ms_tmp.dq.at(i) = data[i].dq;
        ms_tmp.ddq.at(i) = 0.;
        ms_tmp.tau_est.at(i) = 0.;
    }
    motor_state_buffer_.SetData(ms_tmp);
//    std::cout << "q: " << ms_tmp.q.at(0)<<endl;
}

unitree_hg::msg::dds_::LowCmd_ G1::SetMotorCmd() {
    unitree_hg::msg::dds_::LowCmd_ dds_low_command;
    //    std::cout << "qd0: " << dds_low_command.motor_cmd().at(0).q() << endl;

    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();

    for (size_t i = 0; i < rlController->NUM_JOINTS; i++) {
        dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
        dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
        dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
        dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i) * 1.;
        dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i) * 1.;
    }
    return dds_low_command;
}

void G1::RecordBaseState() {
    BaseState bs_tmp;
    bs_tmp.omega = {imuReader.RollSpeed, imuReader.PitchSpeed, imuReader.HeadingSpeed};
    bs_tmp.rpy = {imuReader.Roll, imuReader.Pitch, imuReader.Heading};
    bs_tmp.acc = {imuReader.Accelerometer_X, imuReader.Accelerometer_Y, imuReader.Accelerometer_Z};
    bs_tmp.quat = {imuReader.qw, imuReader.qx, imuReader.qy, imuReader.qz};
    base_state_buffer_.SetData(bs_tmp);

}

void G1::JointStateReadWriter() {
    Motor_control.Run(SetMotorCmd());
    RecordMotorState(Motor_control.GetData());
 //   std::cout<<"被触发"<<std::endl;
//    ///IMU
//    if (imuReader.fetchIMUData()) {
//        RecordBaseState();
//    } else {
//        std::cerr << "Failed to fetch IMU data" << std::endl;
//    }
}

void G1::IMUStateReader() {
    if (imuReader.fetchIMUData()) {
        RecordBaseState();
    } else {
        std::cerr << "Failed to fetch IMU data" << std::endl;
    }
}