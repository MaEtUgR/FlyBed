// by MaEtUgR

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "mbed.h"

#define Rad2Deg         57.295779513082320876798154814105 // factor between radians and degrees of angle (180/Pi)

class IMU_Filter
{
    public:
        IMU_Filter();
        void compute(float dt, const float * gyro_data, const float * acc_data);
        float angle[3];                                 // calculated values of the position [0: x,roll | 1: y,pitch | 2: z,yaw]
        
        // MARG
        float q0, q1, q2, q3;   // quaternion elements representing the estimated orientation
        float exInt , eyInt , ezInt;  // scaled integral error
        void IMUupdate(float halfT, float gx, float gy, float gz, float ax, float ay, float az);
    private:
        float d_Gyro_angle[3];
        void get_Acc_angle(const float * Acc_data);
        float Acc_angle[3];
};

#endif