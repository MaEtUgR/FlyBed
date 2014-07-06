// by MaEtUgR

#ifndef I2C_Sensor_H
#define I2C_Sensor_H

#include "mbed.h"

class I2C_Sensor
{           
    public:
        I2C_Sensor(PinName sda, PinName scl, char address);
        
        float data[3];                  // where the measured data is saved
        //TODO: virtual void calibrate() = 0;   // calibrate the sensor and if desired write calibration values to a file
        
    //protected:
        // Calibration
        void saveCalibrationValues(float values[], int size, char * filename);
        void loadCalibrationValues(float values[], int size, char * filename);
    
        // I2C functions
        char readRegister(char reg);
        void writeRegister(char reg, char data);
        int readMultiRegister(char reg, char* output, int size);
        
        // raw data and function to measure it
        short raw[3];
        
    private:
        I2C i2c;            // original mbed I2C-library just to initialise the control registers
        char i2c_address;   // address
        
        LocalFileSystem local; // file access to save calibration values
};

#endif