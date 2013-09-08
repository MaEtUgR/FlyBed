#include "PID.h"

PID::PID(float P, float I, float D, float Integral_Max)
{
    Integral = 0;
    LastTime = 0;
    Integrate = true;
    PID::P = P;
    PID::I = I;
    PID::D = D;
    PID::Integral_Max = Integral_Max;
    dtTimer.start();
}

float PID::compute(float SetPoint, float ProcessValue)
{
    // meassure dt
    float dt = dtTimer.read() - LastTime;    // time in us since last loop
    LastTime = dtTimer.read();                   // set new time for next measurement
    
    // Proportional
    float Error =  ProcessValue - SetPoint;
    
    // Integral
    if (dt > 2 || !Integrate) // Todo: 2 secs is the maximal time between two computations
        Integral = 0;
    else if (abs(Integral + Error) <= Integral_Max)
        Integral += Error * dt;
        
    // Derivative
    float Derivative = (Error - PreviousError) / dt;
    
    // Final Formula
    float Result = P * Error + I * Integral + D * Derivative;
    
    PreviousError = Error;
    
    return Result;
}

void PID::setPID(float P, float I, float D)
{
    PID::P = P;
    PID::I = I;
    PID::D = D;
}

void PID::setIntegrate(bool Integrate)
{
    PID::Integrate = Integrate;
}