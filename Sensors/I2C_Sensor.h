#ifndef I2C_Sensor_H
#define I2C_Sensor_H

class I2C_Sensor
{           
    public:
        I2C_Sensor(PinName sda, PinName scl, int8_t address);
        
        float data[3];
        void read();
        void calibrate();
        
    protected:
        // I2C functions
        void writeRegister(char address, char data);
        void readMultiRegister(char address, char* output, int size);
        
    private:
        I2C i2c;            // I2C-Bus
    
        int8_t i2c_address; // address
        
        // raw data and function to measure it
        int raw[3];
        void readraw();
        
        LocalFileSystem local; // file access to save calibration values
};

#endif
