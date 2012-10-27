#include "mbed.h"
#include "I2C_Sensor.h"

// calculate the 8-Bit write/read I2C-Address from the 7-Bit adress of the device
#define GET_I2C_WRITE_ADDRESS(ADR)  (ADR << 1&0xFE) // ADR & 1111 1110
#define GET_I2C_READ_ADDRESS(ADR)   (ADR << 1|0x01) // ADR | 0000 0001

I2C_Sensor::I2C_Sensor(PinName sda, PinName scl, int8_t i2c_address) :  i2c(sda, scl), local("local")
{
    I2C_Sensor::i2c_address = i2c_address;
    //i2c.frequency(400000); // standard speed
    i2c.frequency(1500000); // ultrafast!
}

void saveCalibrationValues(float values[], int size, char * filename)
{
    FILE *fp = fopen(strcat("/local/", filename), "w");
    for(int i = 0; i < size; i++)
        fprintf(fp, "%f\r\n", values[i]);
    fclose(fp);
}

void loadCalibrationValues(float values[], int size, char * filename)
{
    FILE *fp = fopen(strcat("/local/", filename), "r");
    for(int i = 0; i < size; i++)
        fscanf(fp, "%f", &values[i]);
    fclose(fp);
}

void I2C_Sensor::writeRegister(char address, char data){ 
    char tx[2];
    tx[0] = address;
    tx[1] = data;
    i2c.write(GET_I2C_WRITE_ADDRESS(i2c_address), tx, 2);
}

void I2C_Sensor::readMultiRegister(char address, char* output, int size) {
    i2c.write (GET_I2C_WRITE_ADDRESS(i2c_address), &address, 1);      //tell it from which register to read
    i2c.read  (GET_I2C_READ_ADDRESS(i2c_address), output, size);      //tell it where to store the data read
}