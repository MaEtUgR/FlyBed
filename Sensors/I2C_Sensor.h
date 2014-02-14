// by MaEtUgR

#ifndef I2C_Sensor_H
#define I2C_Sensor_H

#include "mbed.h"
#include "MODI2C.h"

class I2C_Sensor
{           
    public:
        I2C_Sensor(PinName sda, PinName scl, char address);
        
    protected:
        // Calibration
        void saveCalibrationValues(float values[], int size, char * filename);
        void loadCalibrationValues(float values[], int size, char * filename);
    
        // I2C functions
        char readRegister(char reg);
        void writeRegister(char reg, char data);
        void readMultiRegister(char reg, char* output, int size);
        
    private:
        I2C i2c_init;       // original mbed I2C-library just to initialise the control registers
        MODI2C i2c;         // I2C-Bus
        char i2c_address;   // address
        
        LocalFileSystem local; // file access to save calibration values
};

#endif
