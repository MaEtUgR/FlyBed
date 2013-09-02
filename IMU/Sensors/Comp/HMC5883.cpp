#include "HMC5883.h"

HMC5883::HMC5883(PinName sda, PinName scl) : I2C_Sensor(sda, scl, HMC5883_I2C_ADDRESS)
{   
    #warning these three offsets are calibration values to get |MAX| = |MIN|
    offset[0] = -155; // offset calculated by hand... (min + ((max - min) / 2)
    offset[1] = -142; // TODO: make this automatic with saving to filesystem
    offset[2] = -33.5;
    
    // load calibration values
    //loadCalibrationValues(scale, 3, "COMPASS_SCALE.txt");
    //loadCalibrationValues(offset, 3, "COMPASS_OFFSET.txt");
    
    // initialize HMC5883
    writeRegister(HMC5883_CONF_REG_A, 0x78);                // 8 samples, 75Hz output, normal mode
    //writeRegister(HMC5883_CONF_REG_A, 0x19);              // 8 samples, 75Hz output, test mode! (should get constant values from measurement, see datasheet)
    writeRegister(HMC5883_CONF_REG_B, 0x20);                // Gain for +- 1.3 gauss (earth compass ~0.6 gauss)
    writeRegister(HMC5883_MODE_REG, 0x00);                  // continuous measurement-mode
}

void HMC5883::read()
{
    readraw();
    for(int i = 0; i < 3; i++)
        data[i] = (float)(raw[i]) - offset[i];
}

void HMC5883::calibrate(int s)
{
    int Min[3];                                             // values for achieved maximum and minimum amplitude in calibrating environment
    int Max[3];
    
    Timer calibrate_timer;                                  // timer to know when calibration is finished
    calibrate_timer.start();
    
    while(calibrate_timer.read() < s)                       // take measurements for s seconds
    {
        readraw();
        for(int i = 0; i < 3; i++) {
            Min[i] = Min[i] < raw[i] ? Min[i] : raw[i];      // after each measurement check if there's a new minimum or maximum
            Max[i] = Max[i] > raw[i] ? Max[i] : raw[i];
        }
    }
    
    for(int i = 0; i < 3; i++) {
        //scale[i]= 2000 / (float)(Max[i]-Min[i]);            // calculate scale and offset out of the measured maxima and minima
        //offset[i]= 1000 - (float)(Max[i]) * scale[i];       // the lower bound is -1000, the higher one 1000
    }
    
    //saveCalibrationValues(scale, 3, "COM_SCALE");   // save new scale and offset values to flash
    //saveCalibrationValues(offset, 3, "COM_OFFSE");
}

void HMC5883::readraw()
{
    char buffer[6];                                         // 8-Bit pieces of axis data
    
    readMultiRegister(HMC5883_DATA_OUT_X_MSB, buffer, 6);   // read axis registers using I2C
    
    raw[0] = (short) (buffer[0] << 8 | buffer[1]);          // join 8-Bit pieces to 16-bit short integers
    raw[1] = (short) (buffer[4] << 8 | buffer[5]);          // X, Z and Y (yes, order is stupid like this, see datasheet)
    raw[2] = (short) (buffer[2] << 8 | buffer[3]);
}

float HMC5883::get_angle()
{
    #define RAD2DEG     57.295779513082320876798154814105

    float Heading;
    
    Heading = RAD2DEG * atan2(data[0],data[1]);
    Heading += 1.367;                   // correction of the angle between geographical and magnetical north direction, called declination
                                        // if you need an east-declination += DecAngle, if you need west-declination -= DecAngle
                                        // for me in Switzerland, Bern it's ca. 1.367 degree east
                                        // see:     http://magnetic-declination.com/
                                        // for me:  http://www.swisstopo.admin.ch/internet/swisstopo/de/home/apps/calc/declination.html
    if(Heading < 0)  
        Heading += 360;                 // minimum 0 degree
        
    if(Heading > 360)   
        Heading -= 360;                 // maximum 360 degree

    return Heading;
}
    

