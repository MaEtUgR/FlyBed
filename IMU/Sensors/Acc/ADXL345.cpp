#include "ADXL345.h"

ADXL345::ADXL345(PinName sda, PinName scl) : I2C_Sensor(sda, scl, ADXL345_I2C_ADDRESS)
{
    #warning these three offsets are calibration values to make shure the measurement is 0 when no acceleration is present
    offset[0] = -18; // offset calculated by hand... (min + ((max - min) / 2)
    offset[1] = -17; // TODO: make this automatic with saving to filesystem
    offset[2] = 20;
    
    // Set Offset  - programmed into the OFSX, OFSY, and OFXZ registers, respectively, as 0xFD, 0x03 and 0xFE.
    writeRegister(ADXL345_OFSX_REG, 0xFA); // to get these offsets just lie your sensor down on the table always the axis pointing down to earth has 200+ and the others should have more or less 0
    writeRegister(ADXL345_OFSY_REG, 0xFE);
    writeRegister(ADXL345_OFSZ_REG, 0x0A);
 
    writeRegister(ADXL345_BW_RATE_REG, 0x0F); // 3200Hz BW-Rate
    writeRegister(ADXL345_DATA_FORMAT_REG, 0x0B); // set data format to full resolution and +-16g
    writeRegister(ADXL345_POWER_CTL_REG, 0x08); // set mode
}

void ADXL345::read(){
    readraw();
    for (int i = 0; i < 3; i++)
        data[i] = raw[i] - offset[i]; // TODO: didnt care about units 
}

void ADXL345::readraw(){
    char buffer[6];    
    readMultiRegister(ADXL345_DATAX0_REG, buffer, 6);
    
    raw[0] = (short) ((int)buffer[1] << 8 | (int)buffer[0]);
    raw[1] = (short) ((int)buffer[3] << 8 | (int)buffer[2]);
    raw[2] = (short) ((int)buffer[5] << 8 | (int)buffer[4]);
}

void ADXL345::calibrate(int times, float separation_time)
{
    // calibrate sensor with an average of count samples (result of calibration stored in offset[])
    float calib[3] = {0,0,0};                           // temporary array for the sum of calibration measurement
    
    for (int j = 0; j < times; j++) {                   // read 'times' times the data in a very short time
        readraw();
        for (int i = 0; i < 3; i++)
            calib[i] += raw[i];
        wait(separation_time);
    }
    
    for (int i = 0; i < 3; i++)
        offset[i] = calib[i]/times;                     // take the average of the calibration measurements
}