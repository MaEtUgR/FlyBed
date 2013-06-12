#include "I2C_Sensor.h"

// calculate the 8-Bit write/read I2C-Address from the 7-Bit adress of the device
#define GET_I2C_WRITE_ADDRESS(ADR)  (ADR << 1&0xFE) // ADR & 1111 1110
#define GET_I2C_READ_ADDRESS(ADR)   (ADR << 1|0x01) // ADR | 0000 0001

I2C_Sensor::I2C_Sensor(PinName sda, PinName scl, char i2c_address) : i2c_init(sda, scl), i2c(sda, scl), local("local")
{
    I2C_Sensor::i2c_address = i2c_address;
    i2c_init.frequency(400000); // standard speed
    i2c.frequency(400000); // standard speed
    //i2c.frequency(1500000); // ultrafast!
}

void I2C_Sensor::saveCalibrationValues(float values[], int size, char * filename)
{
    FILE *fp = fopen(strcat("/local/", filename), "w");
    for(int i = 0; i < size; i++)
        fprintf(fp, "%f\r\n", values[i]);
    fclose(fp);
}

void I2C_Sensor::loadCalibrationValues(float values[], int size, char * filename)
{
    FILE *fp = fopen(strcat("/local/", filename), "r");
    for(int i = 0; i < size; i++)
        fscanf(fp, "%f", &values[i]);
    fclose(fp);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ATTENTION!!! there was a problem with other interrupts disturbing the i2c communication of the chip... that's why I use I2C to initialise the sonsors and MODI2C to get the data (only made with readMultiRegister)
// IT DIDN'T WORK STABLE IN OTHER COMBINATIONS (if someone has an idea why please let me know)
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

char I2C_Sensor::readRegister(char reg)
{
    char value = 0;
    
    i2c_init.write(i2c_address, &reg, 1);
    i2c_init.read(i2c_address, &value, 1);

    return value;
}

void I2C_Sensor::writeRegister(char reg, char data)
{
    char buffer[2] = {reg, data};
    i2c_init.write(i2c_address, buffer, 2);
}

void I2C_Sensor::readMultiRegister(char reg, char* output, int size)
{
    i2c.write (i2c_address, &reg, 1); // tell register address of the MSB get the sensor to do slave-transmit subaddress updating.
    i2c.read  (i2c_address, output, size); // tell it where to store the data read
}