#include "MPU6050.h"

MPU6050::MPU6050(PinName sda, PinName scl) : I2C_Sensor(sda, scl, MPU6050_I2C_ADDRESS)
{
    // Turns on the MPU6050's gyro and initializes it
    // register datasheet: http://www.invensense.com/mems/gyro/documents/RM-MPU-6000A.pdf
    writeRegister(MPU6050_RA_PWR_MGMT_1, 0x01);         // wake up from sleep and chooses Gyro X-Axis as Clock source (stadard sleeping and with inacurate clock is 0x40)
    /*
    last 3 Bits of   |Accelerometer(Fs=1kHz) |Gyroscope 
    MPU6050_RA_CONFIG|Bandwidth(Hz)|Delay(ms)|Bandwidth(Hz)|Delay(ms)|Fs(kHz)
    ------------------------------------------------------------------------- 
    0                |260          |0        |256          |0.98     |8 
    1                |184          |2.0      |188          |1.9      |1 
    2                |94           |3.0      |98           |2.8      |1 
    3                |44           |4.9      |42           |4.8      |1 
    4                |21           |8.5      |20           |8.3      |1 
    5                |10           |13.8     |10           |13.4     |1 
    6                |5            |19.0     |5            |18.6     |1 
    */
    writeRegister(MPU6050_RA_CONFIG, 0x03);
    writeRegister(MPU6050_RA_GYRO_CONFIG, 0x18);        // scales gyros range to +-2000dps
    writeRegister(MPU6050_RA_ACCEL_CONFIG, 0x00);       // scales accelerometers range to +-2g
}

void MPU6050::read()
{
    readraw_gyro();                                          // read raw measurement data
    readraw_acc();
    
    offset_gyro[0] = -35;                // TODO: make better calibration
    offset_gyro[1] = 3;
    offset_gyro[2] = 2;
    
    for (int i = 0; i < 3; i++)
        data_gyro[i] = (raw_gyro[i] - offset_gyro[i]) * 0.07 * 0.87; // subtract offset from calibration and multiply unit factor to get degree per second (datasheet p.10)
    
    for (int i = 0; i < 3; i++)
        data_acc[i] = raw_acc[i] - offset_acc[i];           // TODO: didn't care about units because IMU-algorithm just uses vector direction

    // I have to swich coordinates on my board to match the ones of the other sensors (clear this part if you use the raw coordinates of the sensor)
    float tmp = 0;
    tmp = data_gyro[0];
    data_gyro[0] = -data_gyro[0];
    data_gyro[1] = -data_gyro[1];
    data_gyro[2] = data_gyro[2];
    tmp = data_acc[0];
    data_acc[0] = -data_acc[0];
    data_acc[1] = -data_acc[1];
    data_acc[2] = data_acc[2];
}

int MPU6050::readTemp()
{
    char buffer[2];                                     // 8-Bit pieces of temperature data
    
    readMultiRegister(MPU6050_RA_TEMP_OUT_H, buffer, 2);     // read the sensors register for the temperature
    return (short) (buffer[0] << 8 | buffer[1]);
}

void MPU6050::readraw_gyro()
{
    char buffer[6];                                     // 8-Bit pieces of axis data
    
    if(readMultiRegister(MPU6050_RA_GYRO_XOUT_H | (1 << 7), buffer, 6) != 0) return; // read axis registers using I2C   // TODO: why?!   | (1 << 7)
    
    raw_gyro[0] = (short) (buffer[0] << 8 | buffer[1]);     // join 8-Bit pieces to 16-bit short integers
    raw_gyro[1] = (short) (buffer[2] << 8 | buffer[3]);
    raw_gyro[2] = (short) (buffer[4] << 8 | buffer[5]);
}

void MPU6050::readraw_acc()
{
    char buffer[6];                                     // 8-Bit pieces of axis data
    
    readMultiRegister(MPU6050_RA_ACCEL_XOUT_H | (1 << 7), buffer, 6); // read axis registers using I2C   // TODO: why?!   | (1 << 7)
    
    raw_acc[0] = (short) (buffer[0] << 8 | buffer[1]);     // join 8-Bit pieces to 16-bit short integers
    raw_acc[1] = (short) (buffer[2] << 8 | buffer[3]);
    raw_acc[2] = (short) (buffer[4] << 8 | buffer[5]);
}

void MPU6050::calibrate(int times, float separation_time)
{
    // calibrate sensor with an average of count samples (result of calibration stored in offset[])
    // Calibrate Gyroscope ----------------------------------
    float calib_gyro[3] = {0,0,0};                           // temporary array for the sum of calibration measurement
    
    for (int i = 0; i < times; i++) {                   // read 'times' times the data in a very short time
        readraw_gyro();
        for (int j = 0; j < 3; j++)
            calib_gyro[j] += raw_gyro[j];
        wait(separation_time);
    }
    
    for (int i = 0; i < 3; i++)
        offset_gyro[i] = calib_gyro[i]/times;                     // take the average of the calibration measurements
    
    // Calibrate Accelerometer ------------------------------- 
    float calib_acc[3] = {0,0,0};                           // temporary array for the sum of calibration measurement
    
    for (int i = 0; i < times; i++) {                   // read 'times' times the data in a very short time
        readraw_acc();
        for (int j = 0; j < 3; j++)
            calib_acc[j] += raw_acc[j];
        wait(separation_time);
    }
    
    for (int i = 0; i < 2; i++)
        offset_acc[i] = calib_acc[i]/times;                     // take the average of the calibration measurements
}