
#include "mbed.h"
#include "BMP085.h"

//I2C Adresse
#define BMP085_ADRESS 0xEE

#define xpow(x, y) ((long)1 << y)


// Constructor
// -----------------------------------------------
BMP085::BMP085(PinName sda, PinName scl) : i2c_(sda, scl)
    {
    Init();
    }
    

// Temperatur und Druck auslesen und berechnen
// -----------------------------------------------
void BMP085::Update () 
    {
    long P, UTemp, UPressure, X1, X2, X3, B3, B5, B6;
    unsigned long B4, B7;

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
    }


// Hoehe u.M. berechnen  (Druck in hPa)
// -----------------------------------------------
float BMP085::CalcAltitude(float Press)
    {
    float A = Press/1013.25;
    float B = 1/5.25588;
    float C = pow(A,B);
    
    C = 1 - C;
    C = C / 22.5577e-6;
    return C;
    }
    
        
// Drucksensor initialisieren  
// -----------------------------------------------
void BMP085::Init () 
    {
    AC1 = twi_readshort(BMP085_ADRESS, 0xaa);
    AC2 = twi_readshort(BMP085_ADRESS, 0xac);
    AC3 = twi_readshort(BMP085_ADRESS, 0xae);
    AC4 = twi_readshort(BMP085_ADRESS, 0xb0);
    AC5 = twi_readshort(BMP085_ADRESS, 0xb2);
    AC6 = twi_readshort(BMP085_ADRESS, 0xb4);
    B1 = twi_readshort(BMP085_ADRESS, 0xb6);
    B2 = twi_readshort(BMP085_ADRESS, 0xb8);
    MB = twi_readshort(BMP085_ADRESS, 0xba);
    MC = twi_readshort(BMP085_ADRESS, 0xbc);
    MD = twi_readshort(BMP085_ADRESS, 0xbe);
    }
    
    
// -----------------------------------------------
unsigned short BMP085::twi_readshort (int id, int addr) {
    unsigned short i;

    i2c_.start();
    i2c_.write(id);
    i2c_.write(addr);

    i2c_.start();
    i2c_.write(id | 1);
    i = i2c_.read(1) << 8;
    i |= i2c_.read(0);
    i2c_.stop();

    return i;
}


// -----------------------------------------------
unsigned long BMP085::twi_readlong (int id, int addr) {
    unsigned long i;

    i2c_.start();
    i2c_.write(id);
    i2c_.write(addr);

    i2c_.start();
    i2c_.write(id | 1);
    i = i2c_.read(1) << 16;
    i |= i2c_.read(1) << 8;
    i |= i2c_.read(0);
    i2c_.stop();

    return i;
}


// -----------------------------------------------
void BMP085::twi_writechar (int id, int addr, int dat) {

    i2c_.start();
    i2c_.write(id);
    i2c_.write(addr);
    i2c_.write(dat);
    i2c_.stop();
}
