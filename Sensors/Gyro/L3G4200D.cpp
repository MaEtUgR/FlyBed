#include "mbed.h"
#include "L3G4200D.h"
#include <math.h>

#define L3G4200D_I2C_ADDRESS 0xD0


L3G4200D::L3G4200D(PinName sda, PinName scl) : i2c(sda, scl)
{
    i2c.frequency(400000);
    // Turns on the L3G4200D's gyro and places it in normal mode.
    // Normal power mode, all axes enabled
    
    //writeReg(L3G4200D_CTRL_REG2, 0x05); // control filter
    writeReg(L3G4200D_CTRL_REG2, 0x00); // highpass filter disabled
    writeReg(L3G4200D_CTRL_REG3, 0x00);
    writeReg(L3G4200D_CTRL_REG4, 0x20); // acuracy 2000 dps
    
    writeReg(L3G4200D_REFERENCE, 0x00);
    //writeReg(L3G4200D_STATUS_REG, 0x0F);
    writeReg(L3G4200D_INT1_THS_XH, 0x2C);
    writeReg(L3G4200D_INT1_THS_XL, 0xA4);
    writeReg(L3G4200D_INT1_THS_YH, 0x2C);
    writeReg(L3G4200D_INT1_THS_YL, 0xA4);
    writeReg(L3G4200D_INT1_THS_ZH, 0x2C);
    writeReg(L3G4200D_INT1_THS_ZL, 0xA4);
    //writeReg(L3G4200D_INT1_DURATION, 0x00);
    //writeReg(L3G4200D_CTRL_REG5, 0x12);  // Filter einschalten
    //writeReg(L3G4200D_CTRL_REG5, 0x01);  // hochpass Filter einschalten
    writeReg(L3G4200D_CTRL_REG5, 0x00);  // Filter ausschalten
    
    writeReg(L3G4200D_CTRL_REG1, 0x0F); // Gogo
    
    // calibrate gyro with an average of count samples (result to offset)
    #define count 50
    for (int j = 0; j < 3; j++)
            offset[j] = 0;
            
    float Gyro_calib[3] = {0,0,0}; // temporary to sum up
    
    for (int i = 0; i < count; i++) {
        read();
        for (int j = 0; j < 3; j++)
            Gyro_calib[j] += data[j];
        wait(0.001); // maybe less or no wait !!
    }
    
    for (int j = 0; j < 3; j++)
        offset[j] = Gyro_calib[j]/count;
}

// Writes a gyro register
void L3G4200D::writeReg(byte reg, byte value)
{
    byte buffer[2];
    buffer[0] = reg;
    buffer[1] = value;
    
    i2c.write(L3G4200D_I2C_ADDRESS, buffer, 2);
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
void L3G4200D::read()
{
    byte buffer[6]; // 8-Bit pieces of axis data
    // assert the MSB of the address to get the gyro 
    // to do slave-transmit subaddress updating.
    buffer[0] = L3G4200D_OUT_X_L | (1 << 7);
    i2c.write(L3G4200D_I2C_ADDRESS, buffer, 1); 
    
    i2c.read(L3G4200D_I2C_ADDRESS, buffer, 6); 

    data[0] = (short) (buffer[1] << 8 | buffer[0]);
    data[1] = (short) (buffer[3] << 8 | buffer[2]);
    data[2] = (short) (buffer[5] << 8 | buffer[4]);
    
    //with offset of calibration
    for (int j = 0; j < 3; j++)
            data[j] -= offset[j];
}

// Reads the gyros Temperature
int L3G4200D::readTemp()
{
    return (short) readReg(L3G4200D_OUT_TEMP);
}