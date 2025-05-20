//
// Created by cyy on 24-10-7.
//

#ifndef UNITREE_SDK2_BASE_STATE_HPP
#define UNITREE_SDK2_BASE_STATE_HPP


#include <array>

struct BaseState {
    std::array<float, 3> rpy = {};
    std::array<float, 3> omega = {};
    std::array<float, 3> acc = {};
    std::array<float, 4> quat = {};
};


#endif //UNITREE_SDK2_BASE_STATE_HPP