// by MaEtUgR

#ifndef IMU_10DOF_H
#define IMU_10DOF_H

#include "mbed.h"
#include "L3G4200D.h"   // Gyro (Gyroscope)
#include "ADXL345.h"    // Acc (Accelerometer)
#include "HMC5883.h"    // Comp (Compass)
#include "BMP085.h"     // Alt (Altitude sensor or Barometer)
#include "IMU_Filter.h" // Class to calculate position angles  (algorithm from S.O.H. Madgwick, see header file for info)

class IMU_10DOF
{           
    public:
        IMU_10DOF(PinName sda, PinName scl);
        void readAngles();              // read all axis from register to array data
        void readAltitude();            // read all axis from register to array data
        
        float * angle;                  // where the measured and calculated data is saved
        float temperature;
        float pressure;
        float altitude;
        
        float dt;                       // time for entire loop
        float dt_sensors;               // time only to read sensors
        
        L3G4200D    Gyro;               // All sensors Hardwaredrivers
        ADXL345     Acc;
        HMC5883     Comp;
        BMP085      Alt;
            
    private:                            
        Timer LocalTimer;               // local time to calculate processing speed for entire loop and just reading sensors
        float time_for_dt;              // |
        float time_for_dt_sensors;      // |
        
        IMU_Filter  Filter;             // Filterclass to join sensor data
};

#endif