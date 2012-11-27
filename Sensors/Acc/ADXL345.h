// based on http://mbed.org/users/Digixx/code/ADXL345/

#ifndef ADXL345_H
#define ADXL345_H

#include "mbed.h"

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

// data rate codes
#define ADXL345_3200HZ      0x0F
#define ADXL345_1600HZ      0x0E
#define ADXL345_800HZ       0x0D
#define ADXL345_400HZ       0x0C
#define ADXL345_200HZ       0x0B
#define ADXL345_100HZ       0x0A
#define ADXL345_50HZ        0x09
#define ADXL345_25HZ        0x08
#define ADXL345_12HZ5       0x07
#define ADXL345_6HZ25       0x06

// read or write bytes
#define ADXL345_READ    0xA7  
#define ADXL345_WRITE   0xA6 
#define ADXL345_ADDRESS 0x53

//the ADXL345 7-bit address is 0x53 when ALT ADDRESS is low as it is on the sparkfun chip: when ALT ADDRESS is high the address is 0x1D
//when ALT ADDRESS pin is high:
//#define ADXL345_READ    0x3B   
//#define ADXL345_WRITE   0x3A
//#define ADXL345_ADDRESS 0x1D 

#define ADXL345_X           0x00
#define ADXL345_Y           0x01
#define ADXL345_Z           0x02

typedef char byte;

class ADXL345
{
    public:
        ADXL345(PinName sda, PinName scl); // constructor, uses i2c
        void read(); // read all axis to array
        int data[3]; // where the measured data is saved
       
    private:
        I2C i2c; // i2c object to communicate
        void writeReg(byte reg, byte value); // write one single register to sensor
        byte readReg(byte reg); // read one single register from sensor
        void readMultiReg(char startAddress, char* ptr_output, int size); // read multiple regs
        void setDataRate(char rate); // data rate configuration (not only a reg to write)
};

#endif
