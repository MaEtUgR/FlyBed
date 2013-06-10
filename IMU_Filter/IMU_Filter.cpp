#include "IMU_Filter.h"

// MARG
#define PI 3.1415926535897932384626433832795
#define Kp 2.0f         // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f       // integral gain governs rate of convergence of gyroscope biases

IMU_Filter::IMU_Filter()
{
    for(int i=0; i<3; i++)
        angle[i]=0;
    
    // MARG
    q0 = 1; q1 = 0; q2 = 0; q3 = 0;
    exInt = 0; eyInt = 0; ezInt = 0;
}

void IMU_Filter::compute(float dt, const float * Gyro_data, const float * Acc_data)
{
    // calculate angles for each sensor
    for(int i = 0; i < 3; i++)
        d_Gyro_angle[i] = Gyro_data[i] *dt;
    get_Acc_angle(Acc_data);
    
    // Complementary Filter
    #if 0 // (formula from http://diydrones.com/m/discussion?id=705844%3ATopic%3A669858)
        angle[0] = (0.999*(angle[0] + d_Gyro_angle[0]))+(0.001*(Acc_angle[0]));
        angle[1] = (0.999*(angle[1] + d_Gyro_angle[1]))+(0.001*(Acc_angle[1]));// + 3)); // TODO Offset accelerometer einstellen
        angle[2] += d_Gyro_angle[2]; // gyro only here TODO: Compass + 3D
    #endif
    
    #if 0 // alte berechnung, vielleicht Accelerometer zu stark gewichtet
        angle[0] += (Acc.angle[0] - angle[0])/50 + d_Gyro_angle[0];
        angle[1] += (Acc.angle[1] - angle[1])/50 + d_Gyro_angle[1];// TODO Offset accelerometer einstellen
        //tempangle += (Comp.get_angle() - tempangle)/50 + Gyro.data[2] *dt/15000000.0;
        angle[2] = Gyro_angle[2]; // gyro only here
    #endif
    
    #if 0 // neuer Test 2 (funktioniert wahrscheinlich nicht, denkfehler)
        angle[0] += Gyro_angle[0] * 0.98 + Acc.angle[0] * 0.02;
        angle[1] += Gyro_angle[1] * 0.98 + (Acc.angle[1] + 3) * 0.02; // TODO: Calibrierung Acc
        angle[2] = Gyro_angle[2]; // gyro only here
    #endif
    
    #if 0 // all gyro only
        for(int i = 0; i < 3; i++)
            angle[i] += d_Gyro_angle[i];
    #endif
    
    // MARG
    #if 1 // (from http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
        float radGyro[3];
        
        for(int i=0; i<3; i++)  // Radians per second
            radGyro[i] = Gyro_data[i] * PI / 180;
        
        IMUupdate(dt/2, radGyro[0], radGyro[1], radGyro[2], Acc_data[0], Acc_data[1], Acc_data[2]);
        
        float rangle[3]; // calculate angles in radians from quternion output
        rangle[0] = atan2(2*q0*q1 + 2*q2*q3, 1 - 2*(q1*q1 + q2*q2)); // from Wiki
        rangle[1] = asin(2*q0*q2 - 2*q3*q1);
        rangle[2] = atan2(2*q0*q3 + 2*q1*q2, 1 - 2*(q2*q2 + q3*q3));
        
        for(int i=0; i<3; i++)  // angle in degree
            angle[i] = rangle[i] * 180 / PI;
    #endif 
}

void IMU_Filter::get_Acc_angle(const float * Acc_data)
{
    // calculate the angles for roll and pitch (0,1)
    float R = sqrt(pow((float)Acc_data[0],2) + pow((float)Acc_data[1],2) + pow((float)Acc_data[2],2));
    float temp[3];
    
    temp[0] = -(Rad2Deg * acos(Acc_data[1] / R)-90);
    temp[1] =   Rad2Deg * acos(Acc_data[0] / R)-90;
    temp[2] =   Rad2Deg * acos(Acc_data[2] / R);
    
    for(int i = 0;i < 3; i++)
        if (temp[i] > -360 && temp[i] < 360)
            Acc_angle[i] = temp[i];
}

// MARG
void IMU_Filter::IMUupdate(float halfT, float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;         
    
    // normalise the measurements
    norm = sqrt(ax*ax + ay*ay + az*az);
    if(norm == 0.0f) return;   
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;      
    
    // estimated direction of gravity
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
    // error is sum of cross product between reference direction of field and direction measured by sensor
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
    
    // integral error scaled integral gain
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;
    
    // adjusted gyroscope measurements
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;
    
    // integrate quaternion rate and normalise
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
    
    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
}