#include "ADXL345.h"

ADXL345::ADXL345(PinName sda, PinName scl) : i2c(sda, scl) {
    //400kHz, allowing us to use the fastest data rates.
    //there are other chips on board, sorry
    i2c.frequency(400000);   
    // initialize the BW data rate
    char tx[2];
    tx[0] = ADXL345_BW_RATE_REG;
    tx[1] = ADXL345_1600HZ; //value greater than or equal to 0x0A is written into the rate bits (Bit D3 through Bit D0) in the BW_RATE register 
    i2c.write( ADXL345_WRITE , tx, 2);  
    //Data format (for +-16g) - This is done by setting Bit D3 of the DATA_FORMAT register (Address 0x31) and writing a value of 0x03 to the range bits (Bit D1 and Bit D0) of the DATA_FORMAT register (Address 0x31).
   
 char rx[2];
    rx[0] = ADXL345_DATA_FORMAT_REG;
    rx[1] = 0x0B; 
     // full res and +_16g
 i2c.write( ADXL345_WRITE , rx, 2); 
 
    // Set Offset  - programmed into the OFSX, OFSY, and OFXZ registers, respectively, as 0xFD, 0x03 and 0xFE.
  char x[2];
    x[0] = ADXL345_OFSX_REG ;
    //x[1] = 0xFD; 
    x[1] = 0x00; 
 i2c.write( ADXL345_WRITE , x, 2);
  char y[2];
    y[0] = ADXL345_OFSY_REG ;
    //y[1] = 0x03;
    y[1] = 0x00; 
 i2c.write( ADXL345_WRITE , y, 2);
 char z[2];
    z[0] = ADXL345_OFSZ_REG ;
    //z[1] = 0xFE;
    z[1] = 0x00;
 i2c.write( ADXL345_WRITE , z, 2);
 
 // MY INITIALISATION -------------------------------------------------------
 
    writeReg(ADXL345_POWER_CTL_REG, 0x00); // set power control
    writeReg(ADXL345_DATA_FORMAT_REG, 0x0B); // set data format
    setDataRate(ADXL345_3200HZ); // set data rate
    writeReg(ADXL345_POWER_CTL_REG, 0x08); // set mode
}

void ADXL345::read(){
    char buffer[6];    
    readMultiReg(ADXL345_DATAX0_REG, buffer, 6);
    
    data[0] = (short) ((int)buffer[1] << 8 | (int)buffer[0]);
    data[1] = (short) ((int)buffer[3] << 8 | (int)buffer[2]);
    data[2] = (short) ((int)buffer[5] << 8 | (int)buffer[4]);
}

void ADXL345::writeReg(char address, char data){ 
   char tx[2];
   tx[0] = address;
   tx[1] = data;
   i2c.write(ADXL345_WRITE, tx, 2);
}

char ADXL345::readReg(char address){   
   char tx = address;
   char output; 
    i2c.write( ADXL345_WRITE , &tx, 1);  //tell it what you want to read
    i2c.read( ADXL345_READ , &output, 1);    //tell it where to store the data
    return output; 
}

void ADXL345::readMultiReg(char address, char* output, int size) {
    i2c.write(ADXL345_WRITE, &address, 1, true); //tell it where to read from
    i2c.read(ADXL345_READ , output, size, true); //tell it where to store the data read
}

void ADXL345::setDataRate(char rate) {
    //Get the current register contents, so we don't clobber the power bit.
    char registerContents = readReg(ADXL345_BW_RATE_REG);

    registerContents &= 0x10;
    registerContents |= rate;

    writeReg(ADXL345_BW_RATE_REG, registerContents);
}