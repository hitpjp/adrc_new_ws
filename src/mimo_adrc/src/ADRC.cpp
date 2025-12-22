#include "mimo_adrc/ADRC.h"

ADRC::ADRC() {
    v1_ = v2_ = z1_ = z2_ = z3_ = 0.0f;
}

void ADRC::init(float h, float r, float b0) {
    h_ = h; r_ = r; b0_ = b0;
}

void ADRC::setGains(float b1, float b2, float b3, float k1, float k2, float c) {
    beta1_ = b1; beta2_ = b2; beta3_ = b3;
    k1_ = k1; k2_ = k2; c_ = c;
}

float ADRC::sgn(float x) {
    return (x > 1e-6f) ? 1.0f : ((x < -1e-6f) ? -1.0f : 0.0f);
}

float ADRC::fal(float e, float alpha, float delta) {
    if (std::abs(e) <= delta) return e / std::pow(delta, 1.0f - alpha);
    return std::pow(std::abs(e), alpha) * sgn(e);
}

float ADRC::fhan(float x1, float x2, float r, float h) {
    float d = r * h * h;
    float a0 = h * x2;
    float y = x1 + a0;
    float a1 = std::sqrt(d * (d + 8.0f * std::abs(y)));
    float a2 = a0 + sgn(y) * (a1 - d) / 2.0f;
    float sy = (sgn(y + d) - sgn(y - d)) / 2.0f;
    float a = (a0 + y - a2) * sy + a2;
    float sa = (sgn(a + d) - sgn(a - d)) / 2.0f;
    return -r * (a / d - sgn(a)) * sa - r * sgn(a);
}

void ADRC::observe(float y, float u_prev, float f_known) {
    float e = z1_ - y;
    z1_ += h_ * (z2_ - beta1_ * e);
    
    // 修改点：在 z2_ 更新公式中加入 f_known
    // 对于前4个通道，f_known 是已知的标量；
    // 对于后6个通道，f_known 为 0，z3_ 将承担全部的“未知扰动”跟踪任务。
    z2_ += h_ * (z3_ + b0_ * u_prev + f_known - beta2_ * fal(e, 0.50f, h_));
    
    z3_ += h_ * (-beta3_ * fal(e, 0.25f, h_));
}

float ADRC::calculateU0(float v0) {
    float fh = fhan(v1_ - v0, v2_, r_, h_);
    v1_ += h_ * v2_;
    v2_ += h_ * fh;
    return -fhan(v1_ - z1_, c_ * (v2_ - z2_), r_, h_);
}