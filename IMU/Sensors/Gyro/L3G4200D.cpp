#include "L3G4200D.h"

L3G4200D::L3G4200D(PinName sda, PinName scl) : I2C_Sensor(sda, scl, L3G4200D_I2C_ADDRESS) {
    // Turns on the L3G4200D's gyro and places it in normal mode.
    // Normal power mode, all axes enabled (for detailed info see datasheet)
    
    //writeRegister(L3G4200D_CTRL_REG2, 0x05);               // control filter
    writeRegister(L3G4200D_CTRL_REG2, 0x00);            // highpass filter disabled
    writeRegister(L3G4200D_CTRL_REG3, 0x00);
    writeRegister(L3G4200D_CTRL_REG4, 0xA0);            // sets acuracy to 2000 dps (degree per second)
    
    writeRegister(L3G4200D_REFERENCE, 0x00);
    //writeRegister(L3G4200D_STATUS_REG, 0x0F);
    
    writeRegister(L3G4200D_INT1_THS_XH, 0x2C); // TODO: WTF??
    writeRegister(L3G4200D_INT1_THS_XL, 0xA4);
    writeRegister(L3G4200D_INT1_THS_YH, 0x2C);
    writeRegister(L3G4200D_INT1_THS_YL, 0xA4);
    writeRegister(L3G4200D_INT1_THS_ZH, 0x2C);
    writeRegister(L3G4200D_INT1_THS_ZL, 0xA4);
    //writeRegister(L3G4200D_INT1_DURATION, 0x00);
    
    writeRegister(L3G4200D_CTRL_REG5, 0x00);            // deactivates the filters (only use one of these options)
    //writeRegister(L3G4200D_CTRL_REG5, 0x02);            // activates low pass filter
    //writeRegister(L3G4200D_CTRL_REG5, 0x12);          // activates both high and low pass filters
    //writeRegister(L3G4200D_CTRL_REG5, 0x01);          // activates high pass filter
    
    writeRegister(L3G4200D_CTRL_REG1, 0xBF);            // starts Gyro measurement with 400Hz ODR (don't use 800Hz ODR it has peaks in the ADC measurements!!)
    
    calibrate(50, 0.02);
}

void L3G4200D::read() {
    if (!(readRegister(L3G4200D_STATUS_REG) & 8))
        return;
        
    readraw();                                          // read raw measurement data
    
    for (int i = 0; i < 3; i++)
            data[i] = (raw[i] - offset[i])*0.07;               // subtract offset from calibration and multiply unit factor (datasheet s.10)
}

int L3G4200D::readTemp() {
    return (char) readRegister(L3G4200D_OUT_TEMP);     // read the sensors register for the temperature
}

void L3G4200D::readraw() {
    char buffer[6];                                     // 8-Bit pieces of axis data
    
    readMultiRegister(L3G4200D_OUT_X_L | (1 << 7), buffer, 6); // read axis registers using I2C   // TODO: why?!   | (1 << 7)
    
    raw[0] = (short) (buffer[1] << 8 | buffer[0]);     // join 8-Bit pieces to 16-bit short integers
    raw[1] = (short) (buffer[3] << 8 | buffer[2]);
    raw[2] = (short) (buffer[5] << 8 | buffer[4]);
}

void L3G4200D::calibrate(int times, float separation_time) {
    // calibrate sensor with an average of count samples (result of calibration stored in offset[])
    float calib[3] = {0,0,0};                           // temporary array for the sum of calibration measurement
    
    for (int i = 0; i < times; i++) {                   // read 'times' times the data in a very short time
        readraw();
        for (int j = 0; j < 3; j++)
            calib[j] += raw[j];
        wait(separation_time);
    }
    
    for (int i = 0; i < 3; i++)
        offset[i] = calib[i]/times;                     // take the average of the calibration measurements
}