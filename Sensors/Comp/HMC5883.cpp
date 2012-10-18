#include "mbed.h"
#include "HMC5883.h"

HMC5883::HMC5883(PinName sda, PinName scl) :  i2c(sda, scl)
{       
    // initialize HMC5883 for scaling
    writeReg(HMC5883_CONF_REG_A, 0x19); // 8 samples, 75Hz output, test mode for scaling!
    writeReg(HMC5883_CONF_REG_B, 0x20); // Gain for +- 1.3 gauss (earth compass ~0.6 gauss)
    writeReg(HMC5883_MODE_REG, 0x00); // continuous measurement-mode
    
    // Scaling
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
    
    // set normal mode
    writeReg(HMC5883_CONF_REG_A, 0x78); // 8 samples, 75Hz output, normal mode
}

void HMC5883::read()
{
    char buffer[6];
    int dataint[3];
    
    readMultiReg(HMC5883_DATA_OUT_X_MSB, buffer, 6);
    
    // join MSB and LSB of X, Z and Y (yes, order is so stupid, see datasheet)
    dataint[0] = (short) (buffer[0] << 8 | buffer[1]);
    dataint[1] = (short) (buffer[4] << 8 | buffer[5]);
    dataint[2] = (short) (buffer[2] << 8 | buffer[3]);
    
    for(int j = 0; j < 3; j++) {
        Min[j]= Min[j] < dataint[j] ? Min[j] : dataint[j];
        Max[j]= Max[j] > dataint[j] ? Max[j] : dataint[j];
        data[j] = dataint[j]/1.090; //* scale[j];
    }
        
    heading = 57.295779513082320876798154814105*atan2(data[1], data[0]);
}

void HMC5883::writeReg(char address, char data){ 
    char tx[2];
    tx[0] = address;
    tx[1] = data;
    i2c.write(I2CADR_W(HMC5883_ADDRESS), tx, 2);
}

void HMC5883::readMultiReg(char address, char* output, int size) {
    i2c.write(I2CADR_W(HMC5883_ADDRESS), &address, 1); //tell it where to read from
    i2c.read(I2CADR_R(HMC5883_ADDRESS) , output, size); //tell it where to store the data read
}
/*
void HMC5883::Calibrate(int s)
{
    
    //Ende der Kalibrierung in ms Millisekunden berechnen
    int CalibEnd= GlobalTime.read_ms() + s*1000;
    
    while(GlobalTime.read_ms() < CalibEnd)
    {
        //Update erledigt alles
        Update();
    }
    
    AutoCalibration= AutoCalibrationBak;
}*/

// Winkel berechnen
//---------------------------------------------------------------------------------------------------------------------------------------
float HMC5883::getAngle(float x, float y)
    {
    #define Rad2Deg     57.295779513082320876798154814105
    #define PI          3.1415926535897932384626433832795

    float Heading;
    float DecAngle; 
    
    DecAngle = 1.367 / Rad2Deg;         //Missweisung = Winkel zwischen geographischer und magnetischer Nordrichtung
                                        //Bern ca. 1.367 Grad Ost
                                        //http://www.swisstopo.admin.ch/internet/swisstopo/de/home/apps/calc/declination.html
    Heading = atan2((float)y,(float)x);
    
    Heading += DecAngle;                //bei Ost-Deklination  += DecAngle, bei West-Deklination -= DecAngle
 
    if(Heading < 0)  
        Heading += 2*PI;                //korrigieren bei negativem Vorzeichen
        
    if(Heading > 2*PI)   
        Heading -= 2*PI;                 //auf 2Pi begrenzen
        
    return  (Heading * 180/PI);         //Radianten in Grad konvertieren
    }
    

