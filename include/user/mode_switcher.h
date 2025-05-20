//
// Created by cyy on 24-10-7.
//

#ifndef UNITREE_SDK2_MODE_SWITCHER_H
#define UNITREE_SDK2_MODE_SWITCHER_H


#include <stdio.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <string>
#include <ctime>
#include <sys/time.h>
#include <eigen3/Eigen/Dense>
#include "unitree/g1/joystick.hpp"
#include "joystick_reader.h"

using namespace std;

class ModeSwitcher {
public:
    ModeSwitcher() {
        jsreader.initJoystickReader();
    }

    virtual ~ModeSwitcher() {
        delete joystickBtn;
    }

public:
    JoystickReader jsreader;
    unitree::common::xKeySwitchUnion *joystickBtn = nullptr;
    int rl_task_mode = 0;///[forward, stand, lateral]

public:
    char get_selected_key(char &mode) {
        char key = '0';
        while (true) {
            for (int i = 0; i < 100; i++) { printf("\033[33m="); }
            printf("\n\033[0m");
            cout << "Current mode: " << mode <<"\tPlease input a mode: ";
            cin >> key;
            if (key == 'q' and mode == '1')
                return key;
            else if (key >= '1' && key <= '9') {
                if (key == '3') {
                    rl_task_mode = 3;/// key==3 (forward)
                }else if (key == '4') {
                    rl_task_mode = 4; /// key==4 (stand)
                } else if (key == '5') {
                    rl_task_mode = 5; /// /// key==5 (sin test)
                } else if (key == '6') {
                    rl_task_mode = 6;
                } else if (key == '7') {
                    rl_task_mode = 7;
                } else if (key == '8') {
                    rl_task_mode = 8;
                } else if (key == '9') {
                    rl_task_mode = 9;
                }
                if (std::abs(min(key, '3') - min(mode, '3')) <= 1  or mode>='3') {
                    if (key>='3' and key!='5')
                        key='3';
                    return key;
                }
            }
            printf("\033[31mInvalid mode, please input again...\n\033[0m");
        }
    }

    char get_selected_jskey(char &mode) {
        char key = '0';
        jsreader.fetchJoystickData();
        if ((int) jsreader.But[9]== 1) {///start ready
            key = '1';
        } else if ((int) jsreader.But[0] == 1) { ///A stand
            key = '2';
        } else if ((int) jsreader.But[3] == 1) {///Y motion
            rl_task_mode = 3;
            key = '3';
        }else if ((int) jsreader.But[2] == 1) {///RL stand
            rl_task_mode = 4;
            key = '4';
        } else if ((int) jsreader.But[8] == 1) {///SELECT sin test
            rl_task_mode = 5;
            key = '5';
        }  else if ((int) jsreader.But[4] == 1) {///lateral
            rl_task_mode = 6;
            key = '6';
        } else if ((int) jsreader.But[5] == 1) {
            rl_task_mode = 7;
            key = '7';
        } else if ((int) jsreader.But[6] == 1) {
            rl_task_mode = 8;
            key = '8';
        } else if ((int) jsreader.But[7] == 1) {
            rl_task_mode = 9;
            key = '9';
        } else if ((int) jsreader.But[1] == 1) { key = 'q'; } //B exit
        if (key == 'q')
            return key;
        if (key >= '1') {
            if (std::abs(min(key, '3') - min(mode, '3')) <= 1 or mode>='3') {
                if (key>='3' and key !='5')
                    key='3';
                return key;
            }
        }
        return mode;
    }

    char get_selected_stick(char &mode) {
        char key = '0';
        if ((int) joystickBtn->components.start == 1) {///ready
            key = '1';
        } else if ((int) joystickBtn->components.A == 1) { ///stand
            key = '2';
        } else if ((int) joystickBtn->components.Y == 1) {///motion
            rl_task_mode = 3;
            key = '3';
        }else if ((int) joystickBtn->components.X == 1) {///stand
            rl_task_mode = 4;
            key = '4';
        } else if ((int) joystickBtn->components.select == 1) {///sin test
            rl_task_mode = 5;
            key = '5';
        }  else if ((int) joystickBtn->components.down == 1) {///lateral
            rl_task_mode = 6;
            key = '6';
        } else if ((int) joystickBtn->components.left == 1) {
            rl_task_mode = 7;
            key = '7';
        } else if ((int) joystickBtn->components.up == 1) {
            rl_task_mode = 8;
            key = '8';
        } else if ((int) joystickBtn->components.right == 1) {
            rl_task_mode = 9;
            key = '9';
        } else if ((int) joystickBtn->components.B == 1) { key = 'q'; }
        if (key == 'q')
            return key;
        if (key >= '1') {
            if (std::abs(min(key, '3') - min(mode, '3')) <= 1 or mode>='3') {
                if (key>='3' and key !='5')
                    key='3';
                return key;
            }
        }
        return mode;
    }

    static void print_selected_mode(char mode) {
        switch (mode) {
            case '1':
                printf("\033[32mCurrent mode: folding...\n\033[0m");
                break;
            case '2':
                printf("\033[32mCurrent mode: standing...\n\033[0m");
                break;
            case '3':
                printf("\033[32mCurrent mode: RL walking...\n\033[0m");
                break;
            case '4':
                printf("\033[32mCurrent mode: sin waving(step in place)...\n\033[0m");
                break;
            default:
                break;
        }
    }

};


#endif //UNITREE_SDK2_MODE_SWITCHER_H
