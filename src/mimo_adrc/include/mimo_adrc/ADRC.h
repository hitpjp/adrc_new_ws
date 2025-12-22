#ifndef ADRC_H
#define ADRC_H

#include <cmath>
#include <algorithm>

class ADRC {
public:
    ADRC();
    void init(float h, float r, float b0);
    void setGains(float beta1, float beta2, float beta3, float k1, float k2, float c);
    
    // 修改：增加 f_known 参数，默认为 0
    void observe(float y, float u_prev, float f_known = 0.0f);
    float calculateU0(float v0);
    // 增加：专门获取扰动观测值的接口，语义更清晰
    float getObservedDisturbance() const { return z3_; }
    // 观测值接口
    float getZ1() const { return z1_; }
    float getZ2() const { return z2_; }
    float getZ3() const { return z3_; }

private:
    float sgn(float x);
    float fal(float e, float alpha, float delta);
    float fhan(float x1, float x2, float r, float h);

    float h_, r_, b0_;
    float beta1_, beta2_, beta3_;
    float k1_, k2_, c_;
    float v1_, v2_, z1_, z2_, z3_;
};

#endif