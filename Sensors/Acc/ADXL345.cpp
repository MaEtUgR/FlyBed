#include "ADXL345.h"

ADXL345::ADXL345(PinName sda, PinName scl) : I2C_Sensor(sda, scl, ADXL345_I2C_ADDRESS)
{
    // initialize the BW data rate
    writeRegister(ADXL345_BW_RATE_REG, ADXL345_1600HZ); //value greater than or equal to 0x0A is written into the rate bits (Bit D3 through Bit D0) in the BW_RATE register

    //Data format (for +-16g) - This is done by setting Bit D3 of the DATA_FORMAT register (Address 0x31) and writing a value of 0x03 to the range bits (Bit D1 and Bit D0) of the DATA_FORMAT register (Address 0x31).
    writeRegister(ADXL345_DATA_FORMAT_REG, 0x0B);       // full res and +_16g
 
    // Set Offset  - programmed into the OFSX, OFSY, and OFXZ registers, respectively, as 0xFD, 0x03 and 0xFE.
    writeRegister(ADXL345_OFSX_REG, 0x00);              // TODO: 0xFD (sein offset! brauch ich auch?)
    writeRegister(ADXL345_OFSY_REG, 0x00);              // y[1] = 0x03;
    writeRegister(ADXL345_OFSZ_REG, 0x00);              // z[1] = 0xFE;
 
    writeRegister(ADXL345_POWER_CTL_REG, 0x00);         // set power control
    writeRegister(ADXL345_DATA_FORMAT_REG, 0x0B);       // set data format
    setDataRate(ADXL345_3200HZ);                        // set data rate
    writeRegister(ADXL345_POWER_CTL_REG, 0x08);         // set mode
}

void ADXL345::read()
{
    char buffer[6];                                     // 8-Bit pieces of axis data
    
    readMultiRegister(ADXL345_DATAX0_REG, buffer, 6);   // read axis registers using I2C
    
    data[0] = (float)(short) (buffer[1] << 8 | buffer[0]);     // join 8-Bit pieces to 16-bit short integers
    data[1] = (float)(short) (buffer[3] << 8 | buffer[2]);
    data[2] = (float)(short) (buffer[5] << 8 | buffer[4]);
    
    // calculate the angles for roll and pitch (0,1)
    float R = sqrt(pow((float)data[0],2) + pow((float)data[1],2) + pow((float)data[2],2));  // calculate the absolute of the magnetic field vector
    angle[0] = -(RAD2DEG * acos((float)data[1] / R)-90);                                    // roll  - angle of magnetic field vector in x direction
    angle[1] =   RAD2DEG * acos((float)data[0] / R)-90;                                     // pitch - angle of magnetic field vector in y direction
    angle[2] =   RAD2DEG * acos((float)data[2] / R);                                        // angle from magnetic field vector in direction which it has
}

void ADXL345::setDataRate(char rate)
{
    char registerContents = readRegister(ADXL345_BW_RATE_REG); // get the current register contents, so we don't clobber the power bit

    registerContents &= 0x10;
    registerContents |= rate;

    writeRegister(ADXL345_BW_RATE_REG, registerContents);
}