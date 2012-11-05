// by MaEtUgR

#ifndef I2C_Sensor_H
#define I2C_Sensor_H

#include "mbed.h"

class I2C_Sensor
{           
    public:
        I2C_Sensor(PinName sda, PinName scl, int8_t address);
        
        float data[3];                  // where the measured data is saved
        virtual void read() = 0;        // read all axis from register to array data
        //TODO: virtual void calibrate() = 0;   // calibrate the sensor and if desired write calibration values to a file
        
    protected:
        // Calibration value saving
        void saveCalibrationValues(float values[], int size, char * filename);
        void loadCalibrationValues(float values[], int size, char * filename);
    
        // I2C functions
        char readRegister(char reg);
        void writeRegister(char reg, char data);
        void readMultiRegister(char reg, char* output, int size);
        
        // raw data and function to measure it
        int raw[3];
        virtual void readraw() = 0;
        
    private:
        I2C i2c;            // I2C-Bus
        int8_t i2c_address; // address
        
        LocalFileSystem local; // file access to save calibration values
};

#endif
