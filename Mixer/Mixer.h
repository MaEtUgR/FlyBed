// by MaEtUgR

#ifndef MIXER_H
#define MIXER_H

#define RT          0.70710678118654752440084436210485      // 1 / squrt(2) for the X configuration

#include "mbed.h"

class Mixer
{
    public:
        Mixer(int Configuration);
        void compute(unsigned long dt, int Throttle, const float * controller_value);
        float Motor_speed[4];       // calculated motor speeds to send to the ESCs
    private:
        int Configuration;          // number of the configuration used (for example +)
};

#endif