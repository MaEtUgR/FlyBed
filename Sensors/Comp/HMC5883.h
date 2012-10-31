// based on http://mbed.org/users/BlazeX/code/HMC5883/

#ifndef HMC5883_H
#define HMC5883_H

#include "mbed.h"
#include "I2C_Sensor.h"

#define HMC5883_CONF_REG_A      0x00
#define HMC5883_CONF_REG_B      0x01
#define HMC5883_MODE_REG        0x02
#define HMC5883_DATA_OUT_X_MSB  0x03

// I2C addresses
#define HMC5883_I2C_ADDRESS 0x1E

class HMC5883 : public I2C_Sensor
{           
    public:
        HMC5883(PinName sda, PinName scl);
        
        float data[3];
        void read();
        void calibrate(int s);
        float get_angle();
         
    private:
        // raw data and function to get it
        int raw[3];
        void readraw();
        
        // calibration parameters and their saving
        int Min[3];
        int Max[3];
        float scale[3];
        float offset[3];
        LocalFileSystem local;
};

#endif
