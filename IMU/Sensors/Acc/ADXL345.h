// based on http://mbed.org/users/Digixx/code/ADXL345/

#ifndef ADXL345_H
#define ADXL345_H

#include "mbed.h"
#include "I2C_Sensor.h"

#define ADXL345_I2C_ADDRESS    0xA6
//the ADXL345 7-bit address is 0x53 when ALT ADDRESS is low as it is on the sparkfun chip: when ALT ADDRESS is high the address is 0x1D
//when ALT ADDRESS pin is high:  
//#define ADXL345_I2C_ADDRESS    0x3A 

// register addresses
#define ADXL345_DEVID_REG          0x00
#define ADXL345_THRESH_TAP_REG     0x1D
#define ADXL345_OFSX_REG           0x1E
#define ADXL345_OFSY_REG           0x1F
#define ADXL345_OFSZ_REG           0x20
#define ADXL345_DUR_REG            0x21
#define ADXL345_LATENT_REG         0x22
#define ADXL345_WINDOW_REG         0x23
#define ADXL345_THRESH_ACT_REG     0x24
#define ADXL345_THRESH_INACT_REG   0x25
#define ADXL345_TIME_INACT_REG     0x26
#define ADXL345_ACT_INACT_CTL_REG  0x27
#define ADXL345_THRESH_FF_REG      0x28
#define ADXL345_TIME_FF_REG        0x29
#define ADXL345_TAP_AXES_REG       0x2A
#define ADXL345_ACT_TAP_STATUS_REG 0x2B
#define ADXL345_BW_RATE_REG        0x2C
#define ADXL345_POWER_CTL_REG      0x2D
#define ADXL345_INT_ENABLE_REG     0x2E
#define ADXL345_INT_MAP_REG        0x2F
#define ADXL345_INT_SOURCE_REG     0x30
#define ADXL345_DATA_FORMAT_REG    0x31
#define ADXL345_DATAX0_REG         0x32
#define ADXL345_DATAX1_REG         0x33
#define ADXL345_DATAY0_REG         0x34
#define ADXL345_DATAY1_REG         0x35
#define ADXL345_DATAZ0_REG         0x36
#define ADXL345_DATAZ1_REG         0x37
#define ADXL345_FIFO_CTL           0x38
#define ADXL345_FIFO_STATUS        0x39

#define ADXL345_X           0x00
#define ADXL345_Y           0x01
#define ADXL345_Z           0x02

typedef char byte;

class ADXL345 : public I2C_Sensor
{
    public:
        ADXL345(PinName sda, PinName scl);                  // constructor, uses I2C_Sensor class
        virtual void read();                                // read all axis to array
        
        float offset[3];                                    // offset that's subtracted from every measurement
        void calibrate(int times, float separation_time);   // calibration from 'times' measurements with 'separation_time' time between (get an offset while not moving)
       
    private:
        virtual void readraw();
};

#endif
