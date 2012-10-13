 
#ifndef BMP085_I2C_H
#define BMP085_I2C_H

#include "mbed.h"

class BMP085
    {
    private:
        I2C i2c_;
        
    public:
        BMP085(PinName sda, PinName scl);
    
        void    Update();
        float   Temperature;
        float   Pressure;
        float   CalcAltitude(float Press);
        short   oss;
    
    protected:
        void Init();
        unsigned short twi_readshort (int, int);
        unsigned long twi_readlong (int, int);
        void twi_writechar (int, int, int);
    
    
    private:
    
        short AC1, AC2, AC3, B1, B2, MB, MC, MD;
        unsigned short AC4, AC5, AC6;
    };

#endif