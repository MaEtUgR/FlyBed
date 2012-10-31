#ifndef BMP085_H
#define BMP085_H

#include "mbed.h"
#include "I2C_Sensor.h"

// I2C address
#define BMP085_I2C_ADDRESS 0xEE

class BMP085 : public I2C_Sensor
{           
    public:
        BMP085(PinName sda, PinName scl);
        
        float data[3];
        
        void read();
        
        void calibrate(int s);
        
        float get_angle();
         
    private:
        // raw data and function to measure it
        int raw[3];
        void readraw();
        
        // calibration parameters and their saving
        int Min[3];
        int Max[3];
        float scale[3];
        float offset[3];
};

#endif
