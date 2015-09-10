#include "IMU_Filter.h"

// IMU/AHRS
#define PI 3.1415926535897932384626433832795
#define Kp 0.5f         // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0f//0.005f       // integral gain governs rate of convergence of gyroscope biases

IMU_Filter::IMU_Filter()
{
    for(int i=0; i<3; i++)
        angle[i]=0;
    
    // IMU/AHRS
    q0 = 1; q1 = 0; q2 = 0; q3 = 0;
    exInt = 0; eyInt = 0; ezInt = 0;
}

void IMU_Filter::compute(float dt, const float * Gyro_data, const float * Acc_data, const float * Comp_data)
{
    #if 0 // all gyro only
        for(int i = 0; i < 3; i++) {
            d_Gyro_angle[i] = Gyro_data[i] *dt;
            angle[i] += d_Gyro_angle[i];
        }
    #endif
    
    #if 1 // IMU/AHRS (from http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/)   
        float radGyro[3]; // Gyro in radians per second
        for(int i=0; i<3; i++)
            radGyro[i] = Gyro_data[i] * PI / 180;
        
        IMUupdate(dt/2, radGyro[0], radGyro[1], radGyro[2], Acc_data[0], Acc_data[1], Acc_data[2]);
        //AHRSupdate(dt/2, radGyro[0], radGyro[1], radGyro[2], Acc_data[0], Acc_data[1], Acc_data[2], Comp_data[0], Comp_data[1], Comp_data[2]);
        
        float rangle[3]; // calculate angles in radians from quternion output, formula from Wiki (http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
        
        rangle[0] = atan2(2*q0*q1 + 2*q2*q3, 1 - 2*(q1*q1 + q2*q2));
        rangle[1] = asin (2*q0*q2 - 2*q3*q1);
        rangle[2] = atan2(2*q0*q3 + 2*q1*q2, 1 - 2*(q2*q2 + q3*q3));
        
        // TODO
        // Pitch should have a range of +/-90 degrees. 
        // After you pitch past vertical (90 degrees) your roll and yaw value should swing 180 degrees. 
        // A pitch value of 100 degrees is measured as a pitch of 80 degrees and inverted flight (roll = 180 degrees). 
        // Another example is a pitch of 180 degrees (upside down). This is measured as a level pitch (0 degrees) and a roll of 180 degrees.
        // And I think this solves the upside down issue...
        // Handle roll reversal when inverted
        /*if (Acc_data[2] < 0) {
            if (Acc_data[0] < 0) {
                rangle[1] = (180 - rangle[1]);
            } else {
                rangle[1] = (-180 - rangle[1]);
            }
        }*/
        
        for(int i=0; i<2; i++)  // angle in degree
            angle[i] = rangle[i] * 180 / PI;
    #endif
    
    angle[2] += Gyro_data[2] *dt; // ATTENTION YAW IS GYRO ONLY
}

//------------------------------------------------------------------------------------------------------------------
// Code copied from S.O.H. Madgwick (File IMU.c and AHRS.c from https://code.google.com/p/imumargalgorithm30042010sohm/)
//------------------------------------------------------------------------------------------------------------------

// IMU
void IMU_Filter::IMUupdate(float halfT, float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;         
    
    // normalise the measurements
    norm = sqrt(ax*ax + ay*ay + az*az);
    if(norm == 0.0f) return;   
    ax /= norm;
    ay /= norm;
    az /= norm;      
    
    // estimated direction of gravity
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
    // error is sum of cross product between reference direction of field and direction measured by sensor
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
    
    // integral error scaled integral gain
    exInt += ex*Ki;
    eyInt += ey*Ki;
    ezInt += ez*Ki;
    
    // adjusted gyroscope measurements
    gx += Kp*ex + exInt;
    gy += Kp*ey + eyInt;
    gz += Kp*ez + ezInt;
    
    // integrate quaternion rate and normalise
    float q0o = q0; // he did the MATLAB to C error by not thinking of the beginning vector elements already being changed for the calculation of the rest!
    float q1o = q1;
    float q2o = q2;
    float q3o = q3;
    q0 += (-q1o*gx - q2o*gy - q3o*gz)*halfT;
    q1 += (q0o*gx + q2o*gz - q3o*gy)*halfT;
    q2 += (q0o*gy - q1o*gz + q3o*gx)*halfT;
    q3 += (q0o*gz + q1o*gy - q2o*gx)*halfT;  
    
    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
}

// AHRS
void IMU_Filter::AHRSupdate(float halfT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    // auxiliary variables to reduce number of repeated operations
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;          
    
    // normalise the measurements
    norm = sqrt(ax*ax + ay*ay + az*az);
    if(norm == 0.0f) return;
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    norm = sqrt(mx*mx + my*my + mz*mz);
    if(norm == 0.0f) return;
    mx = mx / norm;
    my = my / norm;
    mz = mz / norm;         
    
    // compute reference direction of flux
    hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
    hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
    hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;        
    
    // estimated direction of gravity and flux (v and w)
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
    wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
    wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
    
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
    
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