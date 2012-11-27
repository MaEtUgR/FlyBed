#include "IMU_Filter.h"

IMU_Filter::IMU_Filter()
{
    for(int i=0; i<3; i++)
        angle[i]=0;
}

void IMU_Filter::compute(unsigned long dt, const float * Gyro_data, const int * Acc_data)
{
    get_Acc_angle(Acc_data);
    for(int i = 0; i < 3; i++)
        d_Gyro_angle[i] = Gyro_data[i] *dt/15000000.0;
    
    // calculate angles for roll, pitch an yaw
    #if 0 // alte berechnung, vielleicht Accelerometer zu stark gewichtet
        angle[0] += (Acc.angle[0] - angle[0])/50 + d_Gyro_angle[0];
        angle[1] += (Acc.angle[1] - angle[1])/50 + d_Gyro_angle[1];// TODO Offset accelerometer einstellen
        //tempangle += (Comp.get_angle() - tempangle)/50 + Gyro.data[2] *dt/15000000.0;
        angle[2] = Gyro_angle[2]; // gyro only here
    #endif
    
    #if 1 // neuer Test 1 (Formel von http://diydrones.com/m/discussion?id=705844%3ATopic%3A669858)
        angle[0] = (0.98*(angle[0]+(Gyro_data[0] *dt/15000000.0)))+(0.02*(Acc_angle[0]));
        angle[1] = (0.98*(angle[1]+(Gyro_data[1] *dt/15000000.0)))+(0.02*(Acc_angle[1] + 3)); // TODO Offset accelerometer einstellen
        //tempangle += (Comp.get_angle() - tempangle)/50 + Gyro.data[2] *dt/15000000.0;
        angle[2] += d_Gyro_angle[2]; // gyro only here
    #endif
    
    #if 0 // neuer Test 2 (funktioniert wahrscheinlich nicht, denkfehler)
        angle[0] += Gyro_angle[0] * 0.98 + Acc.angle[0] * 0.02;
        angle[1] += Gyro_angle[1] * 0.98 + (Acc.angle[1] + 3) * 0.02; // TODO: Calibrierung Acc
        angle[2] = Gyro_angle[2]; // gyro only here
    #endif
    
    #if 0 // rein Gyro
        for(int i = 0; i < 3; i++)
            angle[i] = Gyro_angle[i];
    #endif
}

void IMU_Filter::get_Acc_angle(const int * Acc_data)
{
    // calculate the angles for roll and pitch (0,1)
    float R = sqrt(pow((float)Acc_data[0],2) + pow((float)Acc_data[1],2) + pow((float)Acc_data[2],2));
    float temp[3];
    
    temp[0] = -(Rad2Deg * acos((float)Acc_data[1] / R)-90);
    temp[1] =   Rad2Deg * acos((float)Acc_data[0] / R)-90;
    temp[2] =   Rad2Deg * acos((float)Acc_data[2] / R);
    
    for(int i = 0;i < 3; i++)
        if (temp[i] > -360 && temp[i] < 360)
            Acc_angle[i] = temp[i];
}