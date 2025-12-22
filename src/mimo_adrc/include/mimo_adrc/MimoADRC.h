#ifndef MIMO_ADRC_H
#define MIMO_ADRC_H

#include "ADRC.h"

class MimoADRC {
public:
    MimoADRC();
    void init(float h, float r, float b0);
    void setGains(float b1, float b2, float b3, float k1, float k2, float c);
    // 修改：observe 接收 4个已知标量扰动，以及后 6 个通道的测量值
    // y_total: 10个通道的测量值
    // u_prev: 10个通道的控制量
    // f_known_scalars: 前4个通道已知的扰动值
    void observe(float y_total[10], float u_prev[10], float f_known_scalars[4]);
    // 获取后两个 3x1 向量的扰动观测结果 (z3)
    void getObservedVectors(float obs_vec1[3], float obs_vec2[3]);
    void calculateU0(float v0[8], float u0[8]);
    void getZ3(float z3[8]); 
    void getStates(int idx, float s[3]);

private:
    // 总共 10 个 ADRC 通道：4 (scalars) + 3 (vecA) + 3 (vecB)
    ADRC adrcs_[10];
};

#endif