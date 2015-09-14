#include "IMU_10DOF.h"

IMU_10DOF::IMU_10DOF(PinName MOSI, PinName MISO, PinName SCLK, PinName CS) : mpu(MOSI, MISO, SCLK, CS)
{
    dt = 0;
    dt_sensors = 0;
    
    angle = Filter.angle;           // initialize array pointer
    
    LoopTimer.start();
}

void IMU_10DOF::readAngles()
{
    SensorTimer.start(); // start time for measuring sensors
    mpu.readGyro(); // reading sensor data
    mpu.readAcc();
    SensorTimer.stop(); // stop time for measuring sensors
    dt_sensors = SensorTimer.read();
    SensorTimer.reset();

    // meassure dt since last measurement for the filter
    dt = LoopTimer.read(); // time in s since last loop
    LoopTimer.reset();
    
    Filter.compute(dt, mpu.Gyro, mpu.Acc, mpu.Acc);
}