#include "HMC5883.h"

HMC5883::HMC5883(PinName sda, PinName scl) : I2C_Sensor(sda, scl, HMC5883_I2C_ADDRESS), local("local")
{   
    // load calibration values
    FILE *fp = fopen("/local/compass.txt", "r");
    for(int i = 0; i < 3; i++)
        fscanf(fp, "%f", &scale[i]);
    for(int i = 0; i < 3; i++)
        fscanf(fp, "%f", &offset[i]);
    fclose(fp);
    
    // initialize HMC5883 for scaling
    writeRegister(HMC5883_CONF_REG_A, 0x19); // 8 samples, 75Hz output, test mode for scaling!
    writeRegister(HMC5883_CONF_REG_B, 0x20); // Gain for +- 1.3 gauss (earth compass ~0.6 gauss)
    writeRegister(HMC5883_MODE_REG, 0x00); // continuous measurement-mode
    
    /*          (not important, just from data sheet)
    // Scaling with testmode
    for(int j = 0; j < 3; j++) // set all scales to 1 first so the measurement for scaling is not already scaled
        scale[j] = 1;
    
    int data50[3] = {0,0,0}; // to save the 50 measurements
    for(int i = 0; i < 50; i++) // measure 50 times the testmode value to get an average
    {
        read();
        for(int j = 0; j < 3; j++)
            data50[j] += data[j];
    }
    scale[0] = (1.16 * 1090)/(data50[0]/50.0); // value that it should be with selftest of 1.1 Gauss * 1090 LSB/Gauss   /   the value it is
    scale[1] = (1.16 * 1090)/(data50[1]/50.0);
    scale[2] = (1.08 * 1090)/(data50[2]/50.0);
    */
    
    // set normal mode
    writeRegister(HMC5883_CONF_REG_A, 0x78); // 8 samples, 75Hz output, normal mode
}

void HMC5883::read()
{
    readraw();
    for(int i = 0; i < 3; i++)
        data[i] = scale[i] * (float)(raw[i]) + offset[i];
}

void HMC5883::calibrate(int s)
{
    Timer calibrate_timer;
    calibrate_timer.start();
    
    while(calibrate_timer.read() < s)
    {
        readraw();
        for(int i = 0; i < 3; i++) {
            Min[i]= Min[i] < raw[i] ? Min[i] : raw[i];
            Max[i]= Max[i] > raw[i] ? Max[i] : raw[i];
    
            //Scale und Offset aus gesammelten Min Max Werten berechnen
            //Die neue Untere und obere Grenze bilden -1 und +1
            scale[i]= 2000 / (float)(Max[i]-Min[i]);
            offset[i]= 1000 - (float)(Max[i]) * scale[i];
        }
    }
    
    // save values
    FILE *fp = fopen("/local/compass.txt", "w");
    for(int i = 0; i < 3; i++)
        fprintf(fp, "%f\r\n", scale[i]);
    for(int i = 0; i < 3; i++)
        fprintf(fp, "%f\r\n", offset[i]);
    fclose(fp);
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
    Heading += 1.367;                   //bei Ost-Deklination  += DecAngle, bei West-Deklination -= DecAngle
                                        //Missweisung = Winkel zwischen geographischer und magnetischer Nordrichtung
                                        //Bern ca. 1.367 Grad Ost
                                        //http://www.swisstopo.admin.ch/internet/swisstopo/de/home/apps/calc/declination.html
    if(Heading < 0)  
        Heading += 360;                 // minimum 0 degree
        
    if(Heading > 360)   
        Heading -= 360;                 // maximum 360 degree

    return Heading;
}
    

