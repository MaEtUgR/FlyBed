#include "Mixer.h"

Mixer::Mixer()
{
    for(int i=0; i<4; i++)
        Motor_speed[i]=0;
}

void Mixer::compute(unsigned long dt, const float * angle, int Throttle, const float * controller_value)
{
    // Calculate new motorspeeds
    // Pitch
    Motor_speed[0] = Throttle +controller_value[1];
    Motor_speed[2] = Throttle -controller_value[1];
    
    #if 1
        // Roll
        Motor_speed[1] = Throttle +controller_value[0];
        Motor_speed[3] = Throttle -controller_value[0];
        
        /*// Yaw
        Motor_speed[0] -= controller_value[2];
        Motor_speed[2] -= controller_value[2];
        
        Motor_speed[1] += controller_value[2];
        Motor_speed[3] += controller_value[2];*/
    #endif
    
    for(int i = 0; i < 4; i++) // make shure no motor stands still
        Motor_speed[i] = Motor_speed[i] > 50 ? Motor_speed[i] : 50;
}