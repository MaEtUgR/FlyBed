// by MaEtUgR

#ifndef PID_H
#define PID_H

#include "mbed.h"

class PID {
    public:
        PID(float P, float I, float D, float Integral_Max);
        void compute(float DesiredAngle, float Angle, float Gyro_data);
        void setIntegrate(bool Integrate);
        void setPID(float P, float I, float D);
        
        float Value;
    
    private:
        float P, I, D; // PID Values and limits
        
        Timer dtTimer;  // Timer to measure time between every compute
        float LastTime; // Time when last loop was
        
        float Integral; // the sum of all errors (constaind so it doesn't get infinite)
        float Integral_Max; // maximum that the sum of all errors can get (not important: last error not counted)
        bool Integrate; // if the integral is used / the controller is in use
        
        float PreviousGyro_data; // the Gyro measurement of the last computation to get derivative
};

#endif