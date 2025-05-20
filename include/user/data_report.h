#ifndef DATA_REPORT_H
#define DATA_REPORT_H

#include <sys/time.h>
#include <mutex>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <fstream>
#include <string>
#include "utils/cpp_types.h"
#include "rl_controller.h"

using namespace std;


class DataReporter {
public:
    DataReporter() = default;

    virtual ~DataReporter() = default;

public:
    Vec10<float> joint_pos, joint_vel, joint_act;
    Vec3<float> base_rpy, base_rpy_rate, base_vel;

    void init(bool general, bool rl) {
        if (general) {
            if (general_fp == nullptr) {
                general_fp = fopen(general_file, "w");
                if (general_fp == nullptr) {
                    cout << "The general file can not be opened." << endl;
                    //exit(-1);
                }
            }
        }
        if (rl) {
            if (rl_fp == nullptr) {
                rl_fp = fopen(rl_file, "w");
                if (rl_fp == nullptr) {
                    cout << "The rl file can not be opened." << endl;
                    exit(-1);
                }
            }
        }
        report_data_title();
    }

    void close() {
        if (general_fp) {
            if (fclose(general_fp)) {
                cout << fclose(general_fp) << endl;
            }
            general_fp = nullptr;
        }
        if (rl_fp) {
            if (fclose(rl_fp)) {
                cout << fclose(rl_fp) << endl;
            }
            rl_fp = nullptr;
        }
    }

    void report_data(RLController *rlController) {
        push_data2buffer(rlController);
        if (general_fp) {
            ///general data
            for (float i: _general_buffer)
                fprintf(general_fp, "%lf\t", i);
            fprintf(general_fp, "\n");
        }
        if (rl_fp) {
            ///rl data
            for (float i: _rl_buffer)
                fprintf(rl_fp, "%lf\t", i);
            fprintf(rl_fp, "\n");
        }
    }

private:
    std::mutex _data_report_mutex;

    const char *general_file = "general.txt";
    const char *rl_file = "rl.txt";
    FILE *general_fp = nullptr;
    FILE *rl_fp = nullptr;

    std::vector<float> _general_buffer;
    std::vector<float> _rl_buffer;

    const string LEG_NAMES[2] = {"l", "r"};

    const string AXIS[3] = {"x", "y", "z"};
    const string QUATNAME[4] = {"w", "x", "y", "z"};

    const string LEG_JOINT_NAMES[5] = {"hyaw", "hrol", "hpit", "knee", "apit",};

    const string RPY_NAMES[3] = {"rol", "pit", "yaw"};
    const string JOINT_STATE_NAMES[4] = {"C", "P", "V", "T"};

    const string JOINT_STATE_NAMES2RL[3] = {"C", "E", "V"};

    const string SINCOS_NAMES[2] = {"sin", "cos"};

    const int ACT_JOINTS_NUM = 10;

private:
    void report_data_title() {
        ///for general data title
        if (general_fp) {
            //qC, qP, qV
            for (auto &joint_state_name: JOINT_STATE_NAMES) {
                for (auto &leg_name: LEG_NAMES) {
                    for (auto &leg_joint_name: LEG_JOINT_NAMES) {
                        fprintf(general_fp, "%s\t", (joint_state_name + "_" + leg_name + "_" + leg_joint_name).c_str());
                    }
                }
            }
            //rpy
            for (auto &rpy_name: RPY_NAMES) {
                fprintf(general_fp, "%s\t", rpy_name.c_str());
            }
            //rpy rate
            for (auto &rpy_name: RPY_NAMES) {
                fprintf(general_fp, "%s\t", (rpy_name + "_" + "rate").c_str());
            }
            //base acc
            for (auto &rpy_name: AXIS) {
                fprintf(general_fp, "%s\t", (rpy_name + "_" + "acc").c_str());
            }
            //base quat
            for (auto &q_name: QUATNAME) {
                fprintf(general_fp, "%s\t", (q_name + "_" + "quat").c_str());
            }
            fprintf(general_fp, "\n");
        }
        ///for rl data title
        if (rl_fp) {
            //net_out
            for (auto &leg_name: LEG_NAMES) {
                fprintf(rl_fp, "%s\t", (leg_name + "_" + "f").c_str());
            }
            for (auto &leg_name: LEG_NAMES) {
                for (auto &leg_joint_name: LEG_JOINT_NAMES) {
                    fprintf(rl_fp, "%s\t", (leg_name + "_" + leg_joint_name + "_out").c_str());
                }
            }
            for (int jj(0); jj < 3; jj++) {
                //cmd
                fprintf(rl_fp, "vx_cmd\tyr_cmd\t");
                //rp
                fprintf(rl_fp, "rol\tpit\t");
                //rpy rate
                for (auto &rpy_name: RPY_NAMES) {
                    fprintf(rl_fp, "%s\t", (rpy_name + "_" + "rate").c_str());
                }
                //joint state
                for (auto &joint_state_name: JOINT_STATE_NAMES2RL) {
                    for (auto &leg_name: LEG_NAMES) {
                        for (auto &leg_joint_name: LEG_JOINT_NAMES) {
                            fprintf(rl_fp, "%s\t", (joint_state_name + "_" + leg_name + "_" + leg_joint_name).c_str());
                        }
                    }
                }
                //pm phase
                for (auto &sincos_name: SINCOS_NAMES) {
                    for (auto &leg_name: LEG_NAMES) {
                        fprintf(rl_fp, "%s\t", (leg_name + "_" + sincos_name).c_str());
                    }
                }
                //base acc
                // for (auto &rpy_name: AXIS) {
                // fprintf(rl_fp, "%s\t", (rpy_name + "_" + "acc").c_str());
                // }
                //pmf
                for (auto &leg_name: LEG_NAMES) {
                    fprintf(rl_fp, "%s\t", (leg_name + "_" + "pf").c_str());
                }
            }

            fprintf(rl_fp, "\n");
        }
    }

    void push_data2buffer(RLController *rlController) {
        std::vector<float> temp(0, 0.);
        _data_report_mutex.lock();
        temp.clear();
        /// todo general data
        if (general_fp) {
            //q_des
            for (int i = 0; i < ACT_JOINTS_NUM; ++i) {
                temp.push_back(rlController->joint_act(i));
            }

            //q
            for (int i = 0; i < ACT_JOINTS_NUM; ++i) {
                temp.push_back(rlController->joint_pos(i));
            }
            //dq
            for (int i = 0; i < ACT_JOINTS_NUM; ++i) {
                temp.push_back(rlController->joint_vel(i));
            }
            //tau_est
            for (int i = 0; i < ACT_JOINTS_NUM; ++i) {
                temp.push_back(rlController->joint_tau(i));
            }


            //rpy
            for (float i: rlController->base_rpy)temp.push_back(i);
            //rpy rate
            for (float i: rlController->base_rpy_rate)temp.push_back(i);
            //base acc
            for (float i: rlController->base_acc)temp.push_back(i);
            //base quat
            for (float i: rlController->base_quat)temp.push_back(i);
            _general_buffer = temp;
            temp.clear();

        }
        /// todo rl data
        if (rl_fp) {
            for (float i: rlController->action_increment)
                temp.push_back(i);
            for (float i: rlController->observation)
                temp.push_back(i);
            _rl_buffer = temp;
            temp.clear();
        }
        _data_report_mutex.unlock();
    }
};

#endif
