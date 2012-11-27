// by MaEtUgR

#ifndef MIXER_H
#define MIXER_H

#include "mbed.h"

class Mixer
{
    public:
        Mixer();
        void compute(unsigned long dt, const float * angle, int Throttle, const float * controller_value);
        float Motor_speed[4];
    private:
        
};

#endif