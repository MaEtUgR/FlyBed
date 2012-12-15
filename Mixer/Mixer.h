// by MaEtUgR

#ifndef MIXER_H
#define MIXER_H

#include "mbed.h"

class Mixer
{
    public:
        Mixer();
        void compute(unsigned long dt, int Throttle, const float * controller_value);
        float Motor_speed[4];
    private:
        
};

#endif