// based on http://mbed.org/users/okini3939/code/BMP085/

#ifndef BMP085_H
#define BMP085_H

#include "mbed.h"
#include "I2C_Sensor.h"

#define BMP085_I2C_ADDRESS 0xEE

// register addresses
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)
#define BMP085_CONTROL           0xF4  // W   Control register 
#define BMP085_CONTROL_OUTPUT    0xF6  // R   Output registers 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB

#define BMP085_SOFTRESET         0xE0  // -- unused registers
#define BMP085_VERSION           0xD1  // ML_VERSION  pos=0 len=4 msk=0F  AL_VERSION pos=4 len=4 msk=f0
#define BMP085_CHIPID            0xD0  // pos=0 mask=FF len=8 BMP085_CHIP_ID=0x55

// BMP085 Modes
#define MODE_ULTRA_LOW_POWER    0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define MODE_STANDARD           1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define MODE_HIGHRES            2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define MODE_ULTRA_HIGHRES      3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25
                  // "Sampling rate can be increased to 128 samples per second (standard mode) for
                  // dynamic measurement.In this case it is sufficient to measure temperature only 
                  // once per second and to use this value for all pressure measurements during period."
                  // (from BMP085 datasheet Rev1.2 page 10).
// Control register
#define READ_TEMPERATURE        0x2E 
#define READ_PRESSURE           0x34 
//Other
#define MSLP                    101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)

class BMP085 : public I2C_Sensor {           
    public:
        BMP085(PinName sda, PinName scl);
        
        virtual void read();
        float Temperature, Pressure, Altitude;
         
    private:
        short AC1, AC2, AC3, B1, B2, MB, MC, MD; // calibration data
        unsigned short AC4, AC5, AC6;
        short oss;
        
        virtual void readraw();
};

#endif
