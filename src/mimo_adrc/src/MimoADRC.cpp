#include "mimo_adrc/MimoADRC.h"

MimoADRC::MimoADRC() {}

void MimoADRC::init(float h, float r, float b0) {
    for(int i = 0; i < 10; i++) adrcs_[i].init(h, r, b0);
}

void MimoADRC::setGains(float b1, float b2, float b3, float k1, float k2, float c) {
    for(int i = 0; i < 10; i++) adrcs_[i].setGains(b1, b2, b3, k1, k2, c);
}

void MimoADRC::observe(float y_total[10], float u_prev[10], float f_known_scalars[4]) {
    // 1. 处理前 4 个标量通道 (利用已知量)
    for(int i = 0; i < 4; i++) {
        adrcs_[i].observe(y_total[i], u_prev[i], f_known_scalars[i]);
    }
    
    // 2. 处理后 6 个分量 (即两个 3x1 向量，扰动未知，传 0 让 z3 去跟踪)
    for(int i = 4; i < 10; i++) {
        adrcs_[i].observe(y_total[i], u_prev[i], 0.0f);
    }
}

void MimoADRC::calculateU0(float v0[8], float u0[8]) {
    for(int i = 0; i < 8; i++) u0[i] = adrcs_[i].calculateU0(v0[i]);
}

// 这里的 MimoADRC:: 是必须的，因为它在类外部定义实现
void MimoADRC::getZ3(float z3[8]) {
    for(int i = 0; i < 8; i++) {
        z3[i] = adrcs_[i].getZ3();
    }
}

void MimoADRC::getStates(int idx, float s[3]) {
    if (idx >= 0 && idx < 8) {
        s[0] = adrcs_[idx].getZ1();
        s[1] = adrcs_[idx].getZ2();
        s[2] = adrcs_[idx].getZ3();
    }
}

void MimoADRC::getObservedVectors(float obs_vec1[3], float obs_vec2[3]) {
    // 提取后 6 个通道的 z3 值
    for(int i = 0; i < 3; i++) obs_vec1[i] = adrcs_[i+4].getObservedDisturbance();
    for(int i = 0; i < 3; i++) obs_vec2[i] = adrcs_[i+7].getObservedDisturbance();
}