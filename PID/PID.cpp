#include "PID.h"

PID::PID(float P, float I, float D, float Integral_Max) {
    Value = 0;
    Integral = 0;
    LastTime = 0;
    Integrate = true;
    PID::P = P;
    PID::I = I;
    PID::D = D;
    PID::Integral_Max = Integral_Max;
    dtTimer.start();
}

void PID::compute(float DesiredAngle, float Angle, float Gyro_data) {
    // meassure dt
    float dt = dtTimer.read() - LastTime;    // time in us since last loop
    LastTime = dtTimer.read();                   // set new time for next measurement
    
    // Derivative (most important for QC stability and Gyro only because very sensitive!)
    float Derivative = Gyro_data;
    //PreviousGyro_data = Gyro_data;
    
    // Proportional
    float Error =  Angle - DesiredAngle;
    
    // Integral
    if (dt > 2 || !Integrate) // TODO: 2 secs is the maximal time between two computations
        Integral = 0;
    else if (abs(Integral + Error) <= Integral_Max)
        Integral += Error * dt;
    
    // Final Formula
    Value = P * Error + I * Integral + D * Derivative;
}

void PID::setPID(float P, float I, float D) {
    PID::P = P;
    PID::I = I;
    PID::D = D;
}

void PID::setIntegrate(bool Integrate) {
    PID::Integrate = Integrate;
}