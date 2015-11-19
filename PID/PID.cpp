#include "PID.h"

PID::PID(float P, float I, float D, float Integral_Max)
{
    Value = 0;
    Integral = 0;
    LastTime = 0;
    Integrate = true;
    RollBufferIndex = 0;
    PID::P = P;
    PID::I = I;
    PID::D = D;
    PID::Integral_Max = Integral_Max;
    dtTimer.start();
}

void PID::compute(float SetPoint, float ProcessValue)
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
    RollBuffer[RollBufferIndex] = (Error - PreviousError) / dt;
    RollBufferIndex++;
    if (RollBufferIndex == BUFFERSIZE)
        RollBufferIndex = 0;
    float Derivative = 0;
    for(int i=0; i<BUFFERSIZE ;i++)
        Derivative += RollBuffer[i];
    Derivative /= BUFFERSIZE;
    PreviousError = Error;
    
    // Final Formula
    Value = P * Error + I * Integral + D * Derivative;
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