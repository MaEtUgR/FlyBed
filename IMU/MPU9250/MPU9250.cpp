#include "MPU9250.h"

MPU9250::MPU9250(PinName MOSI, PinName MISO, PinName SCLK, PinName CS) : spi(MOSI, MISO, SCLK), cs(CS) {
    deselect();                         // chip must be deselected first
    spi.format(8,0);                    // setup the spi for standard 8 bit data and SPI-Mode 0
    spi.frequency(5e6);                 // with a 5MHz clock rate
    
    /*
    last 3 Bits of|Accelerometer(Fs=1kHz) |Gyroscope 
    MPU9250_CONFIG|Bandwidth(Hz)|Delay(ms)|Bandwidth(Hz)|Delay(ms)|Fs(kHz)
    ------------------------------------------------------------------------- 
    0             |260          |0        |256          |0.98     |8 
    1             |184          |2.0      |188          |1.9      |1 
    2             |94           |3.0      |98           |2.8      |1 
    3             |44           |4.9      |42           |4.8      |1 
    4             |21           |8.5      |20           |8.3      |1 
    5             |10           |13.8     |10           |13.4     |1 
    6             |5            |19.0     |5            |18.6     |1 
    */
    writeRegister8(MPU9250_CONFIG, 0x00);
    writeRegister8(MPU9250_GYRO_CONFIG, 0x18);              // scales gyros range to +-2000dps
    writeRegister8(MPU9250_ACCEL_CONFIG, 0x08);             // scales accelerometers range to +-4g
}

uint8_t MPU9250::getWhoami() {
    return readRegister8(MPU9250_WHO_AM_I);
}

float MPU9250::getTemperature() {
    int16_t data = readRegister16(MPU9250_TEMP_OUT_H);
    return ((data - 21) / 333.87) + 21;                     // formula from register map p.33
}

void MPU9250::readGyro() {
    int16_t rawGyro[3];
    readRegister48(MPU9250_GYRO_XOUT_H, rawGyro);
    
    int16_t offsetGyro[3] = {-31, -16, -12};                // TODO: make better calibration
    
    for (int i = 0; i < 3; i++)
        Gyro[i] = (rawGyro[i] - offsetGyro[i]) * 0.07 * 0.87;        // subtract offset from calibration and multiply unit factor to get degree per second (datasheet p.10)
        


    float tmp = Gyro[0];
    Gyro[0] = -Gyro[1];
    Gyro[1] = -tmp;
    Gyro[2] = -Gyro[2];
}

void MPU9250::readAcc() {
    int16_t rawAcc[3];
    readRegister48(MPU9250_ACCEL_XOUT_H, rawAcc);
    
    int16_t offsetAcc[3] = {-120, -48, -438};                       // TODO: make better calibration
    
    for (int i = 0; i < 3; i++)
        Acc[i] = (rawAcc[i] - offsetAcc[i])/8192.0;                // TODO: didn't care about units because IMU-algorithm just uses vector direction
        
        
        
    float tmp = Acc[0];
    Acc[0] = -Acc[1];
    Acc[1] = -tmp;
    Acc[2] = -Acc[2];
}

// PRIVATE Methods ------------------------------------------------------------------------------------


// SPI Interface --------------------------------------------------------------------------------------
uint8_t MPU9250::readRegister8(uint8_t reg) {
    uint8_t result;
    readRegister(reg, &result, 1);
    return result;
}

uint16_t MPU9250::readRegister16(uint8_t reg) {
    uint8_t result[2];
    readRegister(reg, result, 2);
    return result[0]<<8 | result[1];                        // join 8-Bit pieces to 16-bit short integer
}

void MPU9250::readRegister48(uint8_t reg, int16_t *buffer) {
    uint8_t result[6];
    readRegister(reg, result, 6);
    buffer[0] = (int16_t) (result[0] << 8 | result[1]);     // join 8-Bit pieces to 16-bit short integers
    buffer[1] = (int16_t) (result[2] << 8 | result[3]);
    buffer[2] = (int16_t) (result[4] << 8 | result[5]);
}

void MPU9250::writeRegister8(uint8_t reg, uint8_t buffer) {
    writeRegister(reg, &buffer, 1);
}

void MPU9250::readRegister(uint8_t reg, uint8_t *buffer, int length) {
    select();
    spi.write(reg | 0x80);                                  // send the register address we want to read and the read flag
    for(int i=0; i<length; i++)                             // get data
        buffer[i] = spi.write(0x00);
    deselect();
}

void MPU9250::writeRegister(uint8_t reg, uint8_t *buffer, int length) {
    select();
    spi.write(reg & ~0x80);                                 // send the register address we want to write and the write flag
    for(int i=0; i<length; i++)                             // put data
        spi.write(buffer[i]);
    deselect();
}

void MPU9250::select() { cs = 0; }                          // set Cable Select pin low to start SPI transaction
void MPU9250::deselect() { cs = 1; }                        // set Cable Select pin high to stop SPI transaction