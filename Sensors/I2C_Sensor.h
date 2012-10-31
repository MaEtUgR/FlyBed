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
        // Calibration value saving
        void saveCalibrationValues(float values[], int size, char * filename);
        void loadCalibrationValues(float values[], int size, char * filename);
    
        // I2C functions
        char readRegister(char reg);
        void writeRegister(char reg, char data);
        void readMultiRegister(char reg, char* output, int size);
        
        // raw data and function to measure it
        int raw[3];
        void readraw();
        
    private:
        I2C i2c;            // I2C-Bus
        int8_t i2c_address; // address
        
        LocalFileSystem local; // file access to save calibration values
};

#endif
