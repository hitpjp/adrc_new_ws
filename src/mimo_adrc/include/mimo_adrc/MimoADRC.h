#ifndef MIMO_ADRC_H
#define MIMO_ADRC_H

#include "ADRC.h"

class MimoADRC {
public:
    MimoADRC();
    void init(float h, float r, float b0);
    void setGains(float b1, float b2, float b3, float k1, float k2, float c);
    void observe(float y[8], float u_prev[8]);
    void calculateU0(float v0[8], float u0[8]);
    void getZ3(float z3[8]); 
    void getStates(int idx, float s[3]);

private:
    ADRC adrcs_[8];
};

#endif