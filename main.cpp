#include "mbed.h"       // Standard Library
#include "LED.h"        // LEDs framework for blinking ;)
#include "L3G4200D.h"   // Gyro (Gyroscope)
#include "ADXL345.h"    // Acc (Accelerometer)
#include "HMC5883.h"    // Comp (Compass)
#include "BMP085.h"     // Alt (Altitude sensor)
#include "RC_Channel.h" // RemoteControl Chnnels with PPM
#include "Servo.h"      // Motor PPM
#include "PC.h"         // Serial Port via USB for debugging

#define PI             3.1415926535897932384626433832795
#define Rad2Deg        57.295779513082320876798154814105

Timer GlobalTimer; // global time to calculate processing speed
Ticker Datagetter; // timecontrolled interrupt to get data form IMU and RC

// initialisation of hardware
LED         LEDs;
PC          pc(USBTX, USBRX, 57600);        //Achtung: Treiber auf PC fuer Serial-mbed installieren. hier herunterladen https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe
L3G4200D    Gyro(p28, p27);
ADXL345     Acc(p28, p27);
HMC5883     Comp(p28, p27, GlobalTimer);
BMP085      Alt(p28, p27);
RC_Channel  RC[] = {(p10), (p11), (p12), (p13)}; // noooo p19/p20!!!!
//Servo       Motor[] = {(p15), (p16), (p17), (p18)};

// variables for loop
unsigned long   dt = 0;
unsigned long   time_loop = 0;
float           angle[3] = {0,0,0}; // angle of calculated situation
float           Gyro_data[3];
int             Acc_data[3];
float           Acc_angle[2] = {0,0};
float           Comp_angle = 0;
float           tempangle = 0;

void get_Data()
{
    // read data from sensors
    Gyro.read(Gyro_data);
    Acc.read(Acc_data);
    Comp.Update();
    Alt.Update();

    // calculate the angles for roll and pitch from accelerometer
    Acc_angle[0] = Rad2Deg * atan2((float)Acc_data[1], (float)Acc_data[2]);
    Acc_angle[1] = 3 -Rad2Deg * atan2((float)Acc_data[0], (float)Acc_data[2]); // TODO Offset accelerometer einstellen
    
    //calculate angle for yaw from compass
    Comp_angle = Comp.getAngle(Comp.Mag[0], Comp.Mag[1]);
    
    // meassure dt
    dt = GlobalTimer.read_us() - time_loop; // time in us since last loop
    time_loop = GlobalTimer.read_us();      // set new time for next measurement
    
    // calculate angles for roll, pitch an yaw
    angle[0] += (Acc_angle[0] - angle[0])/50 + Gyro_data[0] *dt/15000000.0;
    angle[1] += (Acc_angle[1] - angle[1])/50 + Gyro_data[1] *dt/15000000.0;
    tempangle += (Comp_angle - tempangle)/50 - Gyro_data[2] *dt/15000000.0;
    angle[2] -= Gyro_data[2] *dt/15000000.0;
    
    // Read RC data
    //RC[0].read() // TODO: RC daten lesen und einberechnen!
    
    LEDs.rollnext();
}


int main() {
    // init screen
    pc.locate(10,5);
    pc.printf("Flybed v0.2");
    LEDs.roll(2);
    
    //Kompass kalibrieren  --> Problem fremde Magnetfelder!
    //Comp.AutoCalibration = 1;
    short MagRawMin[3]= {-400, -400, -400};     //Gespeicherte Werte verwenden
    short MagRawMax[3]= {400, 400, 400};
    Comp.Calibrate(MagRawMin, MagRawMax);
    //Comp.Calibrate(20);
    
    Alt.oss = 0; //Oversampling des Barometers setzen
    
    GlobalTimer.start();
    Datagetter.attach(&get_Data, 0.02);     // start to get data all 10ms
    while(1) {
        // PC output
        pc.locate(10,5);
        pc.printf("dt:%dms  %6.1fm   ", dt/1000, Alt.CalcAltitude(Alt.Pressure));
        pc.locate(10,8);
        pc.printf("Roll:%6.1f Pitch:%6.1f Yaw:%6.1f    ", angle[0], angle[1], angle[2]);
        pc.locate(10,10);
        pc.printf("Debug_Yaw:  Comp:%6.1f tempangle:%6.1f  ", Comp_angle, tempangle);
        pc.locate(10,12);
        pc.printf("Comp_Raw: %6.1f %6.1f %6.1f   ", Comp.RawMag[0], Comp.RawMag[1], Comp.RawMag[2]);
        pc.locate(10,13);
        pc.printf("Comp_Mag: %6.1f %6.1f %6.1f   ", Comp.Mag[0], Comp.Mag[1], Comp.Mag[2]);
        //Motor_left = 1000 + 5*abs(angle[1]);
    }
}
