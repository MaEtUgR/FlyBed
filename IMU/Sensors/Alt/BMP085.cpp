#include "BMP085.h"

BMP085::BMP085(PinName sda, PinName scl) : I2C_Sensor(sda, scl, BMP085_I2C_ADDRESS) {
    // read calibration data from E2PROM, needed for formulas in read function
    char buffer[22];                                  // 8-Bit pieces of axis data
    readMultiRegister(BMP085_CAL_AC1, buffer, 22);       // read all calibration registers in one time 22 Byte = 176 Bit
    
    AC1 = (short) (buffer[0] << 8 | buffer[1]);         // join 8-Bit pieces to 16-bit short integers
    AC2 = (short) (buffer[2] << 8 | buffer[3]);
    AC3 = (short) (buffer[4] << 8 | buffer[5]);
    AC4 = (unsigned short) (buffer[6] << 8 | buffer[7]);      // unsigned !!
    AC5 = (unsigned short) (buffer[8] << 8 | buffer[9]);
    AC6 = (unsigned short) (buffer[10] << 8 | buffer[11]);
    B1 = (short) (buffer[12] << 8 | buffer[13]);
    B2 = (short) (buffer[14] << 8 | buffer[15]);
    MB = (short) (buffer[16] << 8 | buffer[17]);
    MC = (short) (buffer[18] << 8 | buffer[19]);
    MD = (short) (buffer[20] << 8 | buffer[21]);
    
    
    oss = 0; // set Oversampling of Sensor
}

void BMP085::readraw() {
}

void BMP085::read() {
    unsigned short Uncompensated_Temperature;
    long P, Uncompensated_Pressure, X1, X2, X3, B3, B5, B6;
    unsigned long B4, B7;
    
    // read uncompensated Temperature
    writeRegister(BMP085_CONTROL, READ_TEMPERATURE); // say the sensor we want to read the temperature
    wait(0.005); // Wait at least 4.5ms (written in data sheet)
    char buffer[3];  // TODO: nur 2 wenn unten nicht gebraucht                                           // read 16-Bit Temperature (2 registers)
    readMultiRegister(BMP085_CONTROL_OUTPUT, buffer, 2);        
    Uncompensated_Temperature =  buffer[0] << 8 | buffer[1];               // join 8-Bit pieces to 16-bit short integer
    
    // calculate real Temperature
    X1 = ((Uncompensated_Temperature - AC6) * AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    #warning 33 is a calibration value for 3.3 degree shifft of temperature
    B5 = X1 + X2 - ((33 << 4) - 8); // TODO: I added this term because i think my sensor is about 3.3 degree off which makes a difference in absolute altitude
    Temperature = (float)((B5 + 8) >> 4)/10.0; // we want temperature in degree with digit after comma
    
    // read uncompensated Pressure
    writeRegister(BMP085_CONTROL, READ_PRESSURE + (oss << 6)); // say the sensor we want to read the pressure
    wait(0.005); // Wait at least 4.5ms (written in data sheet) TODO: oss fest, times vary and calculation of B3                         
    readMultiRegister(BMP085_CONTROL_OUTPUT, buffer, 3);                                                    // read 24-Bit Pressure (3 registers)
    Uncompensated_Pressure = (unsigned int) (buffer[0] << 16 | buffer[1] << 8 | buffer[0]) >> (8 - oss);    // join 8-Bit pieces to 24-bit integer
    
    // calculate real Pressure
    B6 = B5 - 4000;     // B5 is updated by real temperature above
    X1 = (B2 * ( (B6 * B6) >> 12 )) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((AC1 * 4 + X3) << oss) + 2) >> 2;

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ( (B6 * B6) >> 12) ) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = AC4 * ((unsigned long)X3 + 32768) >> 15;
    
    B7 = ((unsigned long)Uncompensated_Pressure - B3) * (50000 >> oss);
    
    if (B7 < 0x80000000)
        P = (B7 * 2) / B4;
    else 
        P = (B7 / B4) * 2;
    
    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;    
    P = P + ((X1 + X2 + 3791) >> 4);
    Pressure = (float)P / 100.0;
    
    // calculate height out of the pressure
    Altitude = 44330 * (1.0 - pow((Pressure / 1013.25), 1/5.25588));
}