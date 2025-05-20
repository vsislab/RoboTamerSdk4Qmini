//
// Created by cyy on 24-10-7.
//

#ifndef UNITREE_SDK2_JOYSTICK_HPP
#define UNITREE_SDK2_JOYSTICK_HPP


#include <stdint.h>
#include <cmath>
#include "unitree/idl/go2/WirelessController_.hpp"
#include <mutex>

// #include <go2_idl/WirelessController_.hpp>
#include "unitree/idl/go2/WirelessController_.hpp"
#include "unitree/common/thread/thread.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

#include "joystick.hpp"

#define TOPIC_JOYSTICK "rt/wirelesscontroller"


namespace unitree::common {
    // union for keys
    typedef union {
        struct {
            uint8_t R1: 1;
            uint8_t L1: 1;
            uint8_t start: 1;
            uint8_t select: 1;
            uint8_t R2: 1;
            uint8_t L2: 1;
            uint8_t F1: 1;
            uint8_t F2: 1;
            uint8_t A: 1;
            uint8_t B: 1;
            uint8_t X: 1;
            uint8_t Y: 1;
            uint8_t up: 1;
            uint8_t right: 1;
            uint8_t down: 1;
            uint8_t left: 1;
        } components;

        uint16_t value;
    } xKeySwitchUnion;

    // single button class
    class Button {
    public:
        Button() {
        }

        void update(bool state) {
            on_press = state ? state != pressed : false;
            on_release = state ? false : state != pressed;
            pressed = state;
        }

        bool pressed = false;
        bool on_press = false;
        bool on_release = false;
    };

    // full gamepad
    class Gamepad {
    public:
        Gamepad() {
        }

        void Update(unitree_go::msg::dds_::WirelessController_ &key_msg) {
            // update stick values with smooth and deadzone
            lx = lx * (1 - smooth) + (std::fabs(key_msg.lx()) < dead_zone ? 0.0 : key_msg.lx()) * smooth;
            rx = rx * (1 - smooth) + (std::fabs(key_msg.rx()) < dead_zone ? 0.0 : key_msg.rx()) * smooth;
            ry = ry * (1 - smooth) + (std::fabs(key_msg.ry()) < dead_zone ? 0.0 : key_msg.ry()) * smooth;
            // ly = ly * (1 - smooth) + (std::fabs(key_msg.ly()) < dead_zone ? 0.0 : key_msg.ly()) * smooth; //todo
            ly = ly * (1 - 0.1) + (std::fabs(key_msg.ly()) < dead_zone ? 0.0 : key_msg.ly()) * 0.1;

            // update button states
            joystickBtn.value = key_msg.keys();

            R1.update(joystickBtn.components.R1);
            L1.update(joystickBtn.components.L1);
            start.update(joystickBtn.components.start);
            select.update(joystickBtn.components.select);
            R2.update(joystickBtn.components.R2);
            L2.update(joystickBtn.components.L2);
            F1.update(joystickBtn.components.F1);
            F2.update(joystickBtn.components.F2);
            A.update(joystickBtn.components.A);
            B.update(joystickBtn.components.B);
            X.update(joystickBtn.components.X);
            Y.update(joystickBtn.components.Y);
            up.update(joystickBtn.components.up);
            right.update(joystickBtn.components.right);
            down.update(joystickBtn.components.down);
            left.update(joystickBtn.components.left);
        }

        float smooth = 0.03;
        float dead_zone = 0.01;

        float lx = 0.;
        float rx = 0.;
        float ry = 0.;
        float ly = 0.;

        Button R1;
        Button L1;
        Button start;
        Button select;
        Button R2;
        Button L2;
        Button F1;
        Button F2;
        Button A;
        Button B;
        Button X;
        Button Y;
        Button up;
        Button right;
        Button down;
        Button left;

        xKeySwitchUnion joystickBtn;
    };
} // namespace unitree::common


using namespace unitree::common;
using namespace unitree::robot;

class XRockerGamepad {
public:
    XRockerGamepad() {
    }

public:
    unitree::common::Gamepad gamepad;

public:
    // setup dds model
    void InitDdsModel(const std::string &networkInterface = "") {
        ChannelFactory::Instance()->Init(0, networkInterface);
        joystick_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));

        joystick_subscriber->InitChannel(std::bind(&XRockerGamepad::MessageHandler, this, std::placeholders::_1), 1);
    }

    // set gamepad dead_zone parameter
    void SetGamepadDeadZone(float deadzone) {
        gamepad.dead_zone = deadzone;
    }

    // set gamepad smooth parameter
    void setGamepadSmooth(float smooth) {
        gamepad.smooth = smooth;
    }

    // callback function to save joystick message
    void MessageHandler(const void *message) {
        std::lock_guard<std::mutex> lock(joystick_mutex);
        joystick_msg = *(unitree_go::msg::dds_::WirelessController_ *) message;
    }

    // work thread
    void Step() { {
            std::lock_guard<std::mutex> lock(joystick_mutex);
            gamepad.Update(joystick_msg);
        }

        // some operations
        if (gamepad.A.on_press) {
            press_count += 1;
            // print gamepad state
            std::cout << "lx: " << gamepad.lx << "  ly: " << gamepad.ly
                    << "  rx: " << gamepad.rx << "  ry: " << gamepad.ry << std::endl
                    << "A: pressed: " << gamepad.A.pressed
                    << "; on_press: " << gamepad.A.on_press
                    << "; on_release: " << gamepad.A.on_release
                    << std::endl << "press count: " << press_count
                    << std::endl << "=======================================================" << std::endl;
        }
    }

protected:
    ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_subscriber;
    unitree_go::msg::dds_::WirelessController_ joystick_msg;


    ThreadPtr control_thread_ptr;

    std::mutex joystick_mutex;

    int press_count = 0;
};


#endif //UNITREE_SDK2_JOYSTICK_HPP
