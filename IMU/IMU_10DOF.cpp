#include "IMU_10DOF.h"

IMU_10DOF::IMU_10DOF(PinName MOSI, PinName MISO, PinName SCLK, PinName CS) : mpu(MOSI, MISO, SCLK, CS)
{
    dt = 0;
    dt_sensors = 0;
    time_for_dt = 0;
    time_for_dt_sensors = 0;
    
    angle = Filter.angle;           // initialize array pointer
    
    LocalTimer.start();
}

void IMU_10DOF::readAngles()
{
    time_for_dt_sensors = LocalTimer.read(); // start time for measuring sensors
    //mpu.readGyro(); // reading sensor data
    //mpu.readAcc();
    dt_sensors = LocalTimer.read() - time_for_dt_sensors; // stop time for measuring sensors

    // meassure dt since last measurement for the filter
    dt = LocalTimer.read() - time_for_dt; // time in s since last loop
    time_for_dt = LocalTimer.read();      // set new time for next measurement
    
    Filter.compute(dt, mpu.Gyro, mpu.Acc, mpu.Acc);
}