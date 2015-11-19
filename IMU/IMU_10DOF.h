// by MaEtUgR

#ifndef IMU_10DOF_H
#define IMU_10DOF_H

#include "mbed.h"
#include "MPU9250.h"    // Combined Gyroscope & Accelerometer & Magnetometer over SPI
#include "MPU6050.h"    // Combined Gyroscope & Accelerometer
#include "IMU_Filter.h" // Class to calculate position angles  (algorithm from S.O.H. Madgwick, see header file for info)

class IMU_10DOF
{           
    public:
        IMU_10DOF(PinName MOSI, PinName MISO, PinName SCLK, PinName CS, PinName SDA, PinName SCL);
        void readAngles();              // read all sensors and calculate angles
        
        float * angle;                  // where the measured and calculated data is saved
        float temperature;
        float pressure;
        float altitude;
        
        float dt;                       // time for entire loop
        float dt_sensors;               // time only to read sensors
        
        MPU9250     mpu;                // The sensor Hardware Driver
        MPU6050     mpu2;
            
    private:                            
        Timer LoopTimer;               // local time to calculate processing speed for entire loop
        Timer SensorTimer;             // local time to calculate processing speed for just reading sensors
        
        IMU_Filter  Filter;             // Filterclass to join sensor data
};

#endif