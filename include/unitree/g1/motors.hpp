//
// Created by cyy on 24-10-7.
//

#ifndef UNITREE_SDK2_MOTOR_HPP
#define UNITREE_SDK2_MOTOR_HPP


#include <unitree/idl/hg/LowCmd_.hpp>
const int G1_NUM_MOTOR = 10;

struct MotorState {
    std::array<float, G1_NUM_MOTOR> q = {};
    std::array<float, G1_NUM_MOTOR> dq = {};
    std::array<float, G1_NUM_MOTOR> ddq = {};
    std::array<float, G1_NUM_MOTOR> tau_est = {};
};

struct MotorCommand {
    std::array<float, G1_NUM_MOTOR> q_target = {};
    std::array<float, G1_NUM_MOTOR> dq_target = {};
    std::array<float, G1_NUM_MOTOR> kp = {};
    std::array<float, G1_NUM_MOTOR> kd = {};
    std::array<float, G1_NUM_MOTOR> tau_ff = {};
};

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else
                CRC32 <<= 1;
            if (data & xbit) CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
};

#endif //UNITREE_SDK2_MOTOR_HPP
