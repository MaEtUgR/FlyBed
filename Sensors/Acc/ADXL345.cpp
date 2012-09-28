#include "ADXL345.h"
#include "mbed.h"

ADXL345::ADXL345(PinName sda, PinName scl) : i2c_(sda, scl) {

    //400kHz, allowing us to use the fastest data rates.
    //there are other chips on board, sorry
    i2c_.frequency(400000);   
// initialize the BW data rate
    char tx[2];
    tx[0] = ADXL345_BW_RATE_REG;
    tx[1] = ADXL345_1600HZ; //value greater than or equal to 0x0A is written into the rate bits (Bit D3 through Bit D0) in the BW_RATE register 
 i2c_.write( ADXL345_WRITE , tx, 2);  

//Data format (for +-16g) - This is done by setting Bit D3 of the DATA_FORMAT register (Address 0x31) and writing a value of 0x03 to the range bits (Bit D1 and Bit D0) of the DATA_FORMAT register (Address 0x31).
   
 char rx[2];
    rx[0] = ADXL345_DATA_FORMAT_REG;
    rx[1] = 0x0B; 
     // full res and +_16g
 i2c_.write( ADXL345_WRITE , rx, 2); 
 
 // Set Offset  - programmed into the OFSX, OFSY, and OFXZ registers, respectively, as 0xFD, 0x03 and 0xFE.
  char x[2];
    x[0] = ADXL345_OFSX_REG ;
    x[1] = 0xFD; 
 i2c_.write( ADXL345_WRITE , x, 2);
  char y[2];
    y[0] = ADXL345_OFSY_REG ;
    y[1] = 0x03; 
 i2c_.write( ADXL345_WRITE , y, 2);
 char z[2];
    z[0] = ADXL345_OFSZ_REG ;
    z[1] = 0xFE; 
 i2c_.write( ADXL345_WRITE , z, 2);
 
 // MY INITIALISATION -------------------------------------------------------
 
 setPowerControl(0x00);
 setDataFormatControl(0x0B);
 setDataRate(ADXL345_3200HZ);
 setPowerControl(MeasurementMode);
}


char ADXL345::SingleByteRead(char address){   
   char tx = address;
   char output; 
    i2c_.write( ADXL345_WRITE , &tx, 1);  //tell it what you want to read
    i2c_.read( ADXL345_READ , &output, 1);    //tell it where to store the data
    return output;
  
}


/*
***info on the i2c_.write***
address     8-bit I2C slave address [ addr | 0 ]
data        Pointer to the byte-array data to send
length        Number of bytes to send
repeated    Repeated start, true - do not send stop at end
returns     0 on success (ack), or non-0 on failure (nack)
*/

int ADXL345::SingleByteWrite(char address, char data){ 
   int ack = 0;
   char tx[2];
   tx[0] = address;
   tx[1] = data;
   return   ack | i2c_.write( ADXL345_WRITE , tx, 2);   
}



void ADXL345::multiByteRead(char address, char* output, int size) {
    i2c_.write( ADXL345_WRITE, &address, 1);  //tell it where to read from
    i2c_.read( ADXL345_READ , output, size);      //tell it where to store the data read
}


int ADXL345::multiByteWrite(char address, char* ptr_data, int size) {
        int ack;
   
               ack = i2c_.write( ADXL345_WRITE, &address, 1);  //tell it where to write to
        return ack | i2c_.write( ADXL345_READ, ptr_data, size);  //tell it what data to write
                                    
}


void ADXL345::getOutput(int* readings){
    char buffer[6];    
    multiByteRead(ADXL345_DATAX0_REG, buffer, 6);
    
    readings[0] = (int)buffer[1] << 8 | (int)buffer[0];
    readings[1] = (int)buffer[3] << 8 | (int)buffer[2];
    readings[2] = (int)buffer[5] << 8 | (int)buffer[4];

}



char ADXL345::getDeviceID() {  
    return SingleByteRead(ADXL345_DEVID_REG);
    }
//
int ADXL345::setPowerMode(char mode) { 

    //Get the current register contents, so we don't clobber the rate value.
    char registerContents = (mode << 4) | SingleByteRead(ADXL345_BW_RATE_REG);

   return SingleByteWrite(ADXL345_BW_RATE_REG, registerContents);

}

char ADXL345::getPowerControl() {    
    return SingleByteRead(ADXL345_POWER_CTL_REG);
}

int ADXL345::setPowerControl(char settings) {    
    return SingleByteWrite(ADXL345_POWER_CTL_REG, settings);

}



char ADXL345::getDataFormatControl(void){

    return SingleByteRead(ADXL345_DATA_FORMAT_REG);
}

int ADXL345::setDataFormatControl(char settings){

   return SingleByteWrite(ADXL345_DATA_FORMAT_REG, settings);
    
}

int ADXL345::setDataRate(char rate) {

    //Get the current register contents, so we don't clobber the power bit.
    char registerContents = SingleByteRead(ADXL345_BW_RATE_REG);

    registerContents &= 0x10;
    registerContents |= rate;

    return SingleByteWrite(ADXL345_BW_RATE_REG, registerContents);

}


char ADXL345::getOffset(char axis) {     

    char address = 0;

    if (axis == ADXL345_X) {
        address = ADXL345_OFSX_REG;
    } else if (axis == ADXL345_Y) {
        address = ADXL345_OFSY_REG;
    } else if (axis == ADXL345_Z) {
        address = ADXL345_OFSZ_REG;
    }

   return SingleByteRead(address);
}

int ADXL345::setOffset(char axis, char offset) {        

    char address = 0;

    if (axis == ADXL345_X) {
        address = ADXL345_OFSX_REG;
    } else if (axis == ADXL345_Y) {
        address = ADXL345_OFSY_REG;
    } else if (axis == ADXL345_Z) {
        address = ADXL345_OFSZ_REG;
    }

   return SingleByteWrite(address, offset);

}


char ADXL345::getFifoControl(void){

    return SingleByteRead(ADXL345_FIFO_CTL);

}

int ADXL345::setFifoControl(char settings){
   return SingleByteWrite(ADXL345_FIFO_STATUS, settings);

}

char ADXL345::getFifoStatus(void){

    return SingleByteRead(ADXL345_FIFO_STATUS);

}



char ADXL345::getTapThreshold(void) {

    return SingleByteRead(ADXL345_THRESH_TAP_REG);
}

int ADXL345::setTapThreshold(char threshold) {   

   return SingleByteWrite(ADXL345_THRESH_TAP_REG, threshold);

}


float ADXL345::getTapDuration(void) {     

    return (float)SingleByteRead(ADXL345_DUR_REG)*625;
}

int ADXL345::setTapDuration(short int duration_us) {

    short int tapDuration = duration_us / 625;
    char tapChar[2];
     tapChar[0] = (tapDuration & 0x00FF);
     tapChar[1] = (tapDuration >> 8) & 0x00FF;
    return multiByteWrite(ADXL345_DUR_REG, tapChar, 2);

}

float ADXL345::getTapLatency(void) {

    return (float)SingleByteRead(ADXL345_LATENT_REG)*1.25;
}

int ADXL345::setTapLatency(short int latency_ms) {

    latency_ms = latency_ms / 1.25;
    char latChar[2];
     latChar[0] = (latency_ms & 0x00FF);
     latChar[1] = (latency_ms << 8) & 0xFF00;
    return multiByteWrite(ADXL345_LATENT_REG, latChar, 2);

}

float ADXL345::getWindowTime(void) {

    return (float)SingleByteRead(ADXL345_WINDOW_REG)*1.25;
}

int ADXL345::setWindowTime(short int window_ms) {

    window_ms = window_ms / 1.25;
    char windowChar[2];
    windowChar[0] = (window_ms & 0x00FF);
    windowChar[1] = ((window_ms << 8) & 0xFF00);
   return multiByteWrite(ADXL345_WINDOW_REG, windowChar, 2);

}

char ADXL345::getActivityThreshold(void) {

    return SingleByteRead(ADXL345_THRESH_ACT_REG);
}

int ADXL345::setActivityThreshold(char threshold) {
    return SingleByteWrite(ADXL345_THRESH_ACT_REG, threshold);

}

char ADXL345::getInactivityThreshold(void) {
    return SingleByteRead(ADXL345_THRESH_INACT_REG);
       
}

//int FUNCTION(short int * ptr_Output)
//short int FUNCTION ()

int ADXL345::setInactivityThreshold(char threshold) {
    return SingleByteWrite(ADXL345_THRESH_INACT_REG, threshold);

}

char ADXL345::getTimeInactivity(void) {

    return SingleByteRead(ADXL345_TIME_INACT_REG);

}

int ADXL345::setTimeInactivity(char timeInactivity) {
    return SingleByteWrite(ADXL345_TIME_INACT_REG, timeInactivity);

}

char ADXL345::getActivityInactivityControl(void) {

    return SingleByteRead(ADXL345_ACT_INACT_CTL_REG);

}

int ADXL345::setActivityInactivityControl(char settings) {
    return SingleByteWrite(ADXL345_ACT_INACT_CTL_REG, settings);
    
}

char ADXL345::getFreefallThreshold(void) {

    return SingleByteRead(ADXL345_THRESH_FF_REG);

}

int ADXL345::setFreefallThreshold(char threshold) {
   return SingleByteWrite(ADXL345_THRESH_FF_REG, threshold);

}

char ADXL345::getFreefallTime(void) {

    return SingleByteRead(ADXL345_TIME_FF_REG)*5;

}

int ADXL345::setFreefallTime(short int freefallTime_ms) {
     freefallTime_ms = freefallTime_ms / 5;
     char fallChar[2];
     fallChar[0] = (freefallTime_ms & 0x00FF);
     fallChar[1] = (freefallTime_ms << 8) & 0xFF00;
    
    return multiByteWrite(ADXL345_TIME_FF_REG, fallChar, 2);

}

char ADXL345::getTapAxisControl(void) {

    return SingleByteRead(ADXL345_TAP_AXES_REG);

}

int ADXL345::setTapAxisControl(char settings) {
   return SingleByteWrite(ADXL345_TAP_AXES_REG, settings);

}

char ADXL345::getTapSource(void) {

    return SingleByteRead(ADXL345_ACT_TAP_STATUS_REG);

}



char ADXL345::getInterruptEnableControl(void) {

    return SingleByteRead(ADXL345_INT_ENABLE_REG);

}

int ADXL345::setInterruptEnableControl(char settings) {
   return SingleByteWrite(ADXL345_INT_ENABLE_REG, settings);

}

char ADXL345::getInterruptMappingControl(void) {

    return SingleByteRead(ADXL345_INT_MAP_REG);

}

int ADXL345::setInterruptMappingControl(char settings) {
    return SingleByteWrite(ADXL345_INT_MAP_REG, settings);

}

char ADXL345::getInterruptSource(void){

    return SingleByteRead(ADXL345_INT_SOURCE_REG);

}




