#include "mbed.h"

#ifndef PID_H
#define PID_H

class PID {
    public:
        PID(float P, float I, float D, float Integral_Max);
        float compute(float SetPoint, float ProcessValue);
        void setIntegrate(bool Integrate);
    
    private:
        float P,I,D; // PID Values
        
        Timer dtTimer;  // Timer to measure time between every compute
        float LastTime; // Time when last loop was
        
        float SetPoint; // the Point you want to reach
        float Integral; // the sum of all errors (constaind so it doesn't get infinite)
        float Integral_Max; // maximum that the sum of all errors can get (not important: last error not counted)
        float PreviousError; // the Error of the last computation to get derivative
        bool Integrate; // if the integral is used / the controller is in use
};

#endif