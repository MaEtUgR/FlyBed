#include "mbed.h"
#include "L3G4200D.h"
#include <math.h>

#define L3G4200D_I2C_ADDRESS 0xD0


L3G4200D::L3G4200D(PinName sda, PinName scl):
    i2c(sda, scl)
{
    i2c.frequency(400000);
    // Turns on the L3G4200D's gyro and places it in normal mode.
    // 0x0F = 0b00001111
    // Normal power mode, all axes enabled
    
    writeReg(L3G4200D_CTRL_REG2, 0x05); // Filter steuern
    writeReg(L3G4200D_CTRL_REG3, 0x00);
    writeReg(L3G4200D_CTRL_REG4, 0x20); // Genauigkeit 2000 dps
    
    writeReg(L3G4200D_REFERENCE, 0x00);
    //writeReg(L3G4200D_STATUS_REG, 0x0F);
    writeReg(L3G4200D_INT1_THS_XH, 0x2C);
    writeReg(L3G4200D_INT1_THS_XL, 0xA4);
    writeReg(L3G4200D_INT1_THS_YH, 0x2C);
    writeReg(L3G4200D_INT1_THS_YL, 0xA4);
    writeReg(L3G4200D_INT1_THS_ZH, 0x2C);
    writeReg(L3G4200D_INT1_THS_ZL, 0xA4);
    //writeReg(L3G4200D_INT1_DURATION, 0x00);
    writeReg(L3G4200D_CTRL_REG5, 0x12);  // Filter einschalten
    
    writeReg(L3G4200D_CTRL_REG1, 0x0F); // Gogo
}

// Writes a gyro register
void L3G4200D::writeReg(byte reg, byte value)
{
    data[0] = reg;
    data[1] = value;
    
    i2c.write(L3G4200D_I2C_ADDRESS, data, 2);
}

// Reads a gyro register
byte L3G4200D::readReg(byte reg)
{
    byte value = 0;
    
    i2c.write(L3G4200D_I2C_ADDRESS, &reg, 1);
    i2c.read(L3G4200D_I2C_ADDRESS, &value, 1);

    return value;
}

// Reads the 3 gyro channels and stores them in vector g
void L3G4200D::read(int g[3])
{
    // assert the MSB of the address to get the gyro 
    // to do slave-transmit subaddress updating.
    data[0] = L3G4200D_OUT_X_L | (1 << 7);
    i2c.write(L3G4200D_I2C_ADDRESS, data, 1); 

//    Wire.requestFrom(GYR_ADDRESS, 6);
//    while (Wire.available() < 6);
    
    i2c.read(L3G4200D_I2C_ADDRESS, data, 6); 

    uint8_t xla = data[0];
    uint8_t xha = data[1];
    uint8_t yla = data[2];
    uint8_t yha = data[3];
    uint8_t zla = data[4];
    uint8_t zha = data[5];

    g[0] = (short) (xha << 8 | xla);
    g[1] = (short) (yha << 8 | yla);
    g[2] = (short) (zha << 8 | zla);
}

// Reads the gyros Temperature
int L3G4200D::readTemp()
{
    return (short) readReg(L3G4200D_OUT_TEMP);
}