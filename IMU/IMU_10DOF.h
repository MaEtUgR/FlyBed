// by MaEtUgR

#ifndef IMU_10DOF_H
#define IMU_10DOF_H

#include "mbed.h"
#include "MPU9250.h"    // Combined Gyroscope & Accelerometer & Magnetometer over SPI
#include "IMU_Filter.h" // Class to calculate position angles  (algorithm from S.O.H. Madgwick, see header file for info)

class IMU_10DOF
{           
    public:
        IMU_10DOF(PinName MOSI, PinName MISO, PinName SCLK, PinName CS);
        void readAngles();              // read all sensors and calculate angles
        
        float * angle;                  // where the measured and calculated data is saved
        float temperature;
        float pressure;
        float altitude;
        
        float dt;                       // time for entire loop
        float dt_sensors;               // time only to read sensors
        
        MPU9250     mpu;                // The sensor Hardware Driver
            
    private:                            
        Timer LocalTimer;               // local time to calculate processing speed for entire loop and just reading sensors
        float time_for_dt;              // |
        float time_for_dt_sensors;      // |
        
        IMU_Filter  Filter;             // Filterclass to join sensor data
};

#endif