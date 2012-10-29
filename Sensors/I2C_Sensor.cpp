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

char I2C_Sensor::readRegister(char reg)
{
    char value = 0;
    
    i2c.write(L3G4200D_I2C_ADDRESS, &reg, 1, true);
    i2c.read(L3G4200D_I2C_ADDRESS, &value, 1, true);

    return value;
}

void I2C_Sensor::writeRegister(char reg, char data)
{ 
    char buffer[2]; // TODO: Arraykonstruktion in eine Zeile
    buffer[0] = reg;
    buffer[1] = data;
    i2c.write(GET_I2C_WRITE_ADDRESS(i2c_address), tx, 2, true);
}

void I2C_Sensor::readMultiRegister(char reg, char* output, int size)
{ // ATTENTION! the I2C option repeated true is important because otherwise interrupts while bus communications cause crashes (see http://www.i2c-bus.org/repeated-start-condition/)
    i2c.write (GET_I2C_WRITE_ADDRESS(i2c_address), &reg, 1, true);      //tell it from which register to read
    i2c.read  (GET_I2C_READ_ADDRESS(i2c_address), output, size, true);      //tell it where to store the data read
}