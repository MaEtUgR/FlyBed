#include "mbed.h"
#include "BMP085.h"

BMP085::BMP085(PinName sda, PinName scl) : I2C_Sensor(sda, scl, BMP085_I2C_ADDRESS)
{
    // initialize BMP085 with settings
    //writeRegister(0xf4, 0x2e); // TODO: was macht das + register in header!
    
}