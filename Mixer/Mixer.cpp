#include "Mixer.h"

Mixer::Mixer(int Configuration)
{
    Mixer::Configuration = Configuration;
    
    for(int i=0; i<4; i++)
        Motor_speed[i]=0;
}

void Mixer::compute(unsigned long dt, int Throttle, const float * controller_value)
{
    // Mixing tables for each configuration
    float mix_table[2][4][3] = {
    {
        {   0,  1,  1},       // + configuration
        {   1,  0, -1},
        {   0, -1,  1},
        {  -1,  0, -1}
    },
    {
        {  RT,  RT,  1},       // X configuration
        { -RT,  RT, -1},
        { -RT, -RT,  1},
        {  RT, -RT, -1}
    }
    };
    
    // Calculate new motorspeeds
    for(int i=0; i<4; i++) {
        Motor_speed[i] = Throttle;
        for(int j = 0; j < 3; j++)
            Motor_speed[i] += mix_table[Configuration][i][j] * controller_value[j];
    }
    
    for(int i = 0; i < 4; i++) // make sure no motor stands still
        Motor_speed[i] = Motor_speed[i] > 50 ? Motor_speed[i] : 50;
}