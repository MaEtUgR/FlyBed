#include "IMU_10DOF.h"

IMU_10DOF::IMU_10DOF(PinName sda, PinName scl) : Gyro(sda, scl), Acc(sda, scl), Comp(sda, scl), Alt(sda,scl)
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
    Gyro.read(); // reading sensor data
    Acc.read();
    Comp.read();    
    dt_sensors = LocalTimer.read() - time_for_dt_sensors; // stop time for measuring sensors

    // meassure dt for the filter
    dt = LocalTimer.read() - time_for_dt; // time in s since last loop
    time_for_dt = LocalTimer.read();      // set new time for next measurement
    
    Filter.compute(dt, Gyro.data, Acc.data, Comp.data);
}

void IMU_10DOF::readAltitude()
{
    Alt.read();
    temperature = Alt.Temperature; // copy all resulting measurements
    pressure    = Alt.Pressure;
    altitude    = Alt.Altitude;
}