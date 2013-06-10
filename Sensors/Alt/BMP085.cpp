#include "BMP085.h"

BMP085::BMP085(PinName sda, PinName scl) : I2C_Sensor(sda, scl, BMP085_I2C_ADDRESS)
{
    // initialize BMP085 with settings
    //writeRegister(0xf4, 0x2e); // TODO: was macht das + register in header!
    
}

/*void BMP085::read() 
    {
    long P, UTemp, UPressure, X1, X2, X3, B3, B5, B6;
    unsigned long B4, B7;

    // TODO: writeRegister(0xf4, 0x2e); ?!!!
    twi_writechar(BMP085_ADRESS, 0xf4, 0x2e);
    // Wait at least 4.5ms
    wait(0.005);
    UTemp = twi_readshort(BMP085_ADRESS, 0xf6);
    
    X1 = ((UTemp - AC6) * AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;
    Temperature = (float)((B5 + 8) >> 4)/10.0;

    twi_writechar(BMP085_ADRESS, 0xf4, 0x34 + (oss << 6));
    // Wait at least 4.5ms
    wait(0.005);
    UPressure = twi_readlong(BMP085_ADRESS, 0xf6) >> (8 - oss);

    B6 = B5 - 4000;
    X1 = (B2 * (B6 * B6) >> 12) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((AC1 * 4 + X3) << oss) >> 2;    

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * (B6 * B6) >> 12) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = AC4 * (X3 + 32768) >> 15;
    
    B7 = (unsigned long)(UPressure - B3) * (50000 >> oss);
    
    if (B7 < 0x80000000) 
        {
        P = (2 * B7) / B4;
        }
    else 
        {
        P = 2* (B7 / B4);
        }
    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;    
    P = P + ((X1 + X2 + 3791) >> 4);
    Pressure = (float)P / 100.0;
    }*/