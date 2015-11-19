// by MaEtUgR (Matthias Grob) 2015

#ifndef MPU9250_H
#define MPU9250_H

#include "mbed.h"

class MPU9250 {
    public:            
        MPU9250(PinName MOSI, PinName MISO, PinName SCLK, PinName CS);          // constructor, uses SPI class

        // Device API
        uint8_t getWhoami();                                                    // get the Who am I Register (should be 0x71 = 113)
        float getTemperature();                                                 // get a temperature measurement in °C
        
        void readGyro();                                                        // read measurement data from gyrsocope
        float Gyro[3];                                                          // where gyrsocope measurement data is stored
        void readAcc();                                                         // read measurement data from accelerometer
        float Acc[3];                                                           // where accelerometer measurement data is stored
        
    //private:
        
        // SPI Inteface
        SPI spi;                                                                // SPI Bus
        DigitalOut cs;                                                          // Slave selector for SPI-Bus (needed to start and end SPI transactions)
        
        uint8_t readRegister8(uint8_t reg);                                     // expressive methods to read or write the number of bits written in the name
        uint16_t readRegister16(uint8_t reg);
        void readRegister48(uint8_t reg, int16_t *buffer);
        void writeRegister8(uint8_t reg, uint8_t buffer);
        
        void readRegister(uint8_t reg, uint8_t *buffer, int length);            // reads length bytes of the slave registers into buffer memory
        void writeRegister(uint8_t reg, uint8_t *buffer, int length);           // writes length bytes to the slave registers from buffer memory
        void select();                                                          // selects the slave for a transaction
        void deselect();                                                        // deselects the slave after transaction
};

// --------------------- Register Addresses ----------------------------------------------
//      Mnemonic                    Address Description
#define MPU9250_SELF_TEST_X_GYRO    0x00 // Gyroscope Self-Test Registers
#define MPU9250_SELF_TEST_Y_GYRO    0x01 // 
#define MPU9250_SELF_TEST_Z_GYRO    0x02 // 
#define MPU9250_SELF_TEST_X_ACCEL   0x0D // Accelerometer Self-Test Registers
#define MPU9250_SELF_TEST_Y_ACCEL   0x0E // 
#define MPU9250_SELF_TEST_Z_ACCEL   0x0F // 
#define MPU9250_XG_OFFSET_H         0x13 // Gyro Offset Registers
#define MPU9250_XG_OFFSET_L         0x14 // 
#define MPU9250_YG_OFFSET_H         0x15 // 
#define MPU9250_YG_OFFSET_L         0x16 // 
#define MPU9250_ZG_OFFSET_H         0x17 // 
#define MPU9250_ZG_OFFSET_L         0x18 // 
#define MPU9250_SMPLRT_DIV          0x19 // Sample Rate Divider
#define MPU9250_CONFIG              0x1A // Configuration
#define MPU9250_GYRO_CONFIG         0x1B // Gyroscope Configuration
#define MPU9250_ACCEL_CONFIG        0x1C // Accelerometer Configuration
#define MPU9250_ACCEL_CONFIG_2      0x1D // Accelerometer Configuration 2
#define MPU9250_LP_ACCEL_ODR        0x1E // Low Power Accelerometer ODR Control
#define MPU9250_WOM_THR             0x1F // Wake-on Motion Threshold
#define MPU9250_FIFO_EN             0x23 // FIFO Enable

#define MPU9250_I2C_MST_CTRL        0x24 // I2C Master Control
#define MPU9250_I2C_SLV0_ADDR       0x25 // I2C Slave 0 Control
#define MPU9250_I2C_SLV0_REG        0x26 // 
#define MPU9250_I2C_SLV0_CTRL       0x27 // 
#define MPU9250_I2C_SLV1_ADDR       0x28 // I2C Slave 1 Control
#define MPU9250_I2C_SLV1_REG        0x29 // 
#define MPU9250_I2C_SLV1_CTRL       0x2A // 
#define MPU9250_I2C_SLV2_ADDR       0x2B // I2C Slave 2 Control
#define MPU9250_I2C_SLV2_REG        0x2C // 
#define MPU9250_I2C_SLV2_CTRL       0x2D // 
#define MPU9250_I2C_SLV3_ADDR       0x2E // I2C Slave 3 Control
#define MPU9250_I2C_SLV3_REG        0x2F // 
#define MPU9250_I2C_SLV3_CTRL       0x30 // 
#define MPU9250_I2C_SLV4_ADDR       0x31 // I2C Slave 4 Control
#define MPU9250_I2C_SLV4_REG        0x32 // 
#define MPU9250_I2C_SLV4_DO         0x33 // 
#define MPU9250_I2C_SLV4_CTRL       0x34 // 
#define MPU9250_I2C_SLV4_DI         0x35 // 
#define MPU9250_I2C_MST_STATUS      0x36 // I2C Master Status

#define MPU9250_INT_PIN_CFG         0x37 // INT Pin / Bypass Enable Configuration
#define MPU9250_INT_ENABLE          0x38 // Interrupt Enable
#define MPU9250_INT_STATUS          0x3A // Interrupt Status

#define MPU9250_ACCEL_XOUT_H        0x3B // Accelerometer Measurements
#define MPU9250_ACCEL_XOUT_L        0x3C // 
#define MPU9250_ACCEL_YOUT_H        0x3D // 
#define MPU9250_ACCEL_YOUT_L        0x3E // 
#define MPU9250_ACCEL_ZOUT_H        0x3F // 
#define MPU9250_ACCEL_ZOUT_L        0x40 // 
#define MPU9250_TEMP_OUT_H          0x41 // Temperature Measurement
#define MPU9250_TEMP_OUT_L          0x42 // 
#define MPU9250_GYRO_XOUT_H         0x43 // Gyroscope Measurements
#define MPU9250_GYRO_XOUT_L         0x44 // 
#define MPU9250_GYRO_YOUT_H         0x45 // 
#define MPU9250_GYRO_YOUT_L         0x46 // 
#define MPU9250_GYRO_ZOUT_H         0x47 // 
#define MPU9250_GYRO_ZOUT_L         0x48 // 

#define MPU9250_EXT_SENS_DATA_00    0x49 // External Sensor Data
#define MPU9250_EXT_SENS_DATA_01    0x4A // 
#define MPU9250_EXT_SENS_DATA_02    0x4B // 
#define MPU9250_EXT_SENS_DATA_03    0x4C // 
#define MPU9250_EXT_SENS_DATA_04    0x4D // 
#define MPU9250_EXT_SENS_DATA_05    0x4E // 
#define MPU9250_EXT_SENS_DATA_06    0x4F // 
#define MPU9250_EXT_SENS_DATA_07    0x50 // 
#define MPU9250_EXT_SENS_DATA_08    0x51 // 
#define MPU9250_EXT_SENS_DATA_09    0x52 // 
#define MPU9250_EXT_SENS_DATA_10    0x53 // 
#define MPU9250_EXT_SENS_DATA_11    0x54 // 
#define MPU9250_EXT_SENS_DATA_12    0x55 // 
#define MPU9250_EXT_SENS_DATA_13    0x56 // 
#define MPU9250_EXT_SENS_DATA_14    0x57 // 
#define MPU9250_EXT_SENS_DATA_15    0x58 // 
#define MPU9250_EXT_SENS_DATA_16    0x59 // 
#define MPU9250_EXT_SENS_DATA_17    0x5A // 
#define MPU9250_EXT_SENS_DATA_18    0x5B // 
#define MPU9250_EXT_SENS_DATA_19    0x5C // 
#define MPU9250_EXT_SENS_DATA_20    0x5D // 
#define MPU9250_EXT_SENS_DATA_21    0x5E // 
#define MPU9250_EXT_SENS_DATA_22    0x5F // 
#define MPU9250_EXT_SENS_DATA_23    0x60 // 

#define MPU9250_I2C_SLV0_DO         0x63 // I2C Slave 0 Data Out
#define MPU9250_I2C_SLV1_DO         0x64 // I2C Slave 1 Data Out
#define MPU9250_I2C_SLV2_DO         0x65 // I2C Slave 2 Data Out
#define MPU9250_I2C_SLV3_DO         0x66 // I2C Slave 3 Data Out
#define MPU9250_I2C_MST_DELAY_CTRL  0x67 // I2C Master Delay Control
#define MPU9250_SIGNAL_PATH_RESET   0x68 // Signal Path Reset
#define MPU9250_MOT_DETECT_CTRL     0x69 // Accelerometer Interrupt Control
#define MPU9250_USER_CTRL           0x6A // User Control
#define MPU9250_PWR_MGMT_1          0x6B // Power Management 1
#define MPU9250_PWR_MGMT_2          0x6C // Power Management 2
#define MPU9250_FIFO_COUNTH         0x72 // FIFO Count Registers
#define MPU9250_FIFO_COUNTL         0x73 // 
#define MPU9250_FIFO_R_W            0x74 // FIFO Read Write
#define MPU9250_WHO_AM_I            0x75 // Who Am I
#define MPU9250_XA_OFFSET_H         0x77 // Accelerometer Offset Registers
#define MPU9250_XA_OFFSET_L         0x78 // 
#define MPU9250_YA_OFFSET_H         0x7A // 
#define MPU9250_YA_OFFSET_L         0x7B // 
#define MPU9250_ZA_OFFSET_H         0x7D // 
#define MPU9250_ZA_OFFSET_L         0x7E // 

#endif