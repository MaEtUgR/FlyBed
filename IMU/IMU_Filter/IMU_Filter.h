//=====================================================================================================
// ALGORITHM COPIED FROM http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// Algorithm Author: S.O.H. Madgwick
// Algorithm Date: 25th August 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================

// by MaEtUgR

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "mbed.h"

class IMU_Filter
{
    public:
        IMU_Filter();
        void compute(float dt, const float * gyro_data, const float * acc_data, const float * Comp_data);
        float angle[3];                                 // calculated values of the position [0: x,roll | 1: y,pitch | 2: z,yaw]
        
        // IMU/AHRS
        float q0, q1, q2, q3;   // quaternion elements representing the estimated orientation
        float exInt , eyInt , ezInt;  // scaled integral error
        void IMUupdate(float halfT, float gx, float gy, float gz, float ax, float ay, float az);                                // 6DOF without compass
        void AHRSupdate(float halfT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz); // 9DOF with compass
    private:
        float d_Gyro_angle[3];
        void get_Acc_angle(const float * Acc_data);
        float Acc_angle[3];
};

#endif