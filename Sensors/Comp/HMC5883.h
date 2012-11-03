// based on http://mbed.org/users/BlazeX/code/HMC5883/

#ifndef HMC5883_H
#define HMC5883_H

#include "mbed.h"
#include "I2C_Sensor.h"

#define HMC5883_I2C_ADDRESS 0x1E

#define HMC5883_CONF_REG_A      0x00
#define HMC5883_CONF_REG_B      0x01
#define HMC5883_MODE_REG        0x02
#define HMC5883_DATA_OUT_X_MSB  0x03

class HMC5883 : public I2C_Sensor
{           
    public:
        HMC5883(PinName sda, PinName scl);
        
        virtual void read();            // read all axis from register to array data
        void calibrate(int s);
        float get_angle();
         
    private:
        void readraw();                 // function to get raw data
        
        float scale[3];                 // calibration parameters
        float offset[3];
};

#endif
