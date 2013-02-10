// by MaEtUgR

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "mbed.h"

#define Rad2Deg         57.295779513082320876798154814105 // factor between radians and degrees of angle (180/Pi)

class IMU_Filter
{
    public:
        IMU_Filter();
        void compute(unsigned long dt, const float * gyro_data, const int * acc_data);
        float angle[3];                                 // calculated values of the position [0: x,roll | 1: y,pitch | 2: z,yaw]
    private:
        float d_Gyro_angle[3];
        void get_Acc_angle(const int * Acc_data);
        float Acc_angle[3];
};

#endif