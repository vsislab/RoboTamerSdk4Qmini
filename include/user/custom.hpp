#pragma once

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <Python.h>
#include "unitree/common/thread/thread.hpp"


#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include "unitree/g1/PRorAB.hpp"
#include "unitree/g1/data_buffer.hpp"
#include "unitree/g1/motors.hpp"
#include "unitree/g1/base_state.hpp"


#include "IMUReader.h"
#include "mode_switcher.h"
#include "data_report.h"
#include "read_txt_file.h"

#include "Motor_thread.hpp"


static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;


class G1 {
public:
    explicit G1(const std::string &networkInterface, bool is_test_local) : PRorAB_mode_(PR),
                                                                           mode_machine_(0) {

        ChannelFactory::Instance()->Init(0, networkInterface);
        std::cout << "Initialize channel factory." << std::endl;
        _is_test_local = is_test_local;

        xRockerGamepad.setGamepadSmooth(0.2);
        xRockerGamepad.SetGamepadDeadZone(0.1);
        xRockerGamepad.InitDdsModel();
        modeSwitcher.joystickBtn = &xRockerGamepad.gamepad.joystickBtn;


        rlController = new RLController();
        rlController->init();
        rlController->dds_motor_command = &motor_command_buffer_;
        rlController->dds_motor_state = &motor_state_buffer_;
        rlController->dds_base_state = &base_state_buffer_;
        rlController->gamepad = &xRockerGamepad.gamepad;
        rlController->reset(_is_test_local);
        rlController->jsreader=&modeSwitcher.jsreader;


        if (rlController->configParams.use_sim_gait)
            ReadTxtFile::get_data_to_vector(rlController->sim_gait_data);

        ///h1 control 0.01s
        control_dt_ = rlController->configParams.control_dt;
        rlController->_rl_time_step = control_dt_;
        float motor_dt_ = 0.002;
        control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, control_dt_ * 1e6,
                                                      &G1::Control, this);
        usleep(0.1 * 1e6);

        ///command_writer 0.002s
        lowcmd_publisher_.reset(new ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(HG_CMD_TOPIC));
        lowcmd_publisher_->InitChannel();
        command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, motor_dt_ * 1e6,
                                                      &G1::JointStateReadWriter, this);

                                          
        
        usleep(0.1 * 1e6);

        ///command_writer 0.002s
        PyEval_ReleaseLock();
        imu_thread_ptr_ = CreateRecurrentThreadEx("imu", UT_CPU_ID_NONE, 0.003 * 1e6,
                                                  &G1::IMUStateReader, this);
        usleep(0.1 * 1e6);

        ///joystick 0.03s
        joystick_thread_ptr_ = CreateRecurrentThreadEx("joystick", UT_CPU_ID_NONE, 0.004 * 1e6,
                                                       &G1::RunJoystick, this);
        usleep(0.1 * 1e6);


        ///report_rpy 0.02s
        dataReporter.init(true, true);
        report_rpy_ptr_ = CreateRecurrentThreadEx("report_rpy", UT_CPU_ID_NONE, control_dt_ * 1e6,
                                                  &G1::ReportData, this);
        usleep(0.1 * 1e6);

        ///mode_process 0.05s
        mode_process_ptr_ = CreateRecurrentThreadEx("mode_process", UT_CPU_ID_NONE, 0.02 * 1e6,
                                                    &G1::ModeProcess, this);
        usleep(0.1 * 1e6);

        ModeSwitcher::print_selected_mode(current_mode);

    }


    virtual ~G1() {
        delete rlController;
    };

public:
    char current_mode = '1';
    char selected_mode = '2';

    ModeSwitcher modeSwitcher; // NOTE! There is no WirelessController_.hpp in robot type 'hg'. This is for go2.
    DataReporter dataReporter;
    XRockerGamepad xRockerGamepad;

    RLController *rlController = nullptr;
    IMUReader imuReader;

    MotorController Motor_control;


public:
    void JointStateReadWriter();

    void IMUStateReader();


    void Control();

    void ReportData();

    void RunJoystick();

    void ModeProcess();

private:
    ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;

    DataBuffer<MotorState> motor_state_buffer_;
    DataBuffer<MotorCommand> motor_command_buffer_;
    DataBuffer<BaseState> base_state_buffer_;

    bool _is_test_local = false;
    PRorAB PRorAB_mode_;
    uint8_t mode_machine_;

    float control_dt_ = 0.01f;
    float relative_time = 0.f;
    int control_count = 0;
    float MOVE_DURATION = 3.f;

    // multithreading
    ThreadPtr command_writer_ptr_;
    ThreadPtr control_thread_ptr_;
    ThreadPtr imu_thread_ptr_;
    // ThreadPtr run_threads_[4];
    // // ThreadPtr run_threads_[1];
    // // ThreadPtr run_threads_[2];
    // // ThreadPtr run_threads_[3];
    // ThreadPtr motor_threads;


    ThreadPtr report_rpy_ptr_;
    ThreadPtr mode_process_ptr_;
    ThreadPtr joystick_thread_ptr_;

private:
    void RecordMotorState(const std::array<MotorData, 10> &data);

    void RecordBaseState();

    unitree_hg::msg::dds_::LowCmd_ SetMotorCmd();
};
