#include "mbed.h"       // Standard Library
#include "LED.h"        // LEDs framework for blinking ;)
#include "PC.h"         // Serial Port via USB for debugging in TeraTerm (driver needed: https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe)
#include "L3G4200D.h"   // Gyro (Gyroscope)
#include "ADXL345.h"    // Acc (Accelerometer)
#include "HMC5883.h"    // Comp (Compass)
#include "BMP085.h"     // Alt (Altitude sensor)
#include "RC_Channel.h" // RemoteControl Chnnels with PPM
#include "Servo.h"      // Motor PPM
#include "PID.h"        // PID Library from Aaron Berk

#define PI              3.1415926535897932384626433832795
#define Rad2Deg         57.295779513082320876798154814105
#define RATE            0.02 // speed of Ticker/PID

Timer GlobalTimer; // global time to calculate processing speed
Ticker Datagetter; // timecontrolled interrupt to get data form IMU and RC
PID controller(1.0, 0.0, 0.0, RATE); // test PID controller for throttle
PID pid(1.0, 1.0, 0.0, RATE); // test PID controller for throttle

// initialisation of hardware
LED         LEDs;
PC          pc(USBTX, USBRX, 57600);
L3G4200D    Gyro(p28, p27);
ADXL345     Acc(p28, p27);
HMC5883     Comp(p28, p27);
BMP085      Alt(p28, p27);
RC_Channel  RC[] = {(p10), (p11), (p12), (p13)}; // noooo p19/p20!!!!
Servo       Motor[] = {(p15), (p16), (p17), (p18)};

// variables for loop
unsigned long   dt = 0;
unsigned long   time_loop = 0;
float           angle[3] = {0,0,0}; // angle 0: x,roll / 1: y,pitch / 2: z,yaw
float           Comp_angle = 0;
float           tempangle = 0;
int             Motorvalue[3];

float pidtester;

void get_Data()
{
    // read data from sensors
    Gyro.read();
    Acc.read();
    Comp.read();
    Alt.Update();

    //calculate angle for yaw from compass
    //Comp_angle = Comp.getAngle(Comp.Mag[0], Comp.Mag[1]);
    
    // meassure dt
    dt = GlobalTimer.read_us() - time_loop; // time in us since last loop
    time_loop = GlobalTimer.read_us();      // set new time for next measurement
    
    // calculate angles for roll, pitch an yaw
    angle[0] += (Acc.angle[0] - angle[0])/50 + Gyro.data[0] *dt/15000000.0;
    angle[1] += (Acc.angle[1]+3 - angle[1])/50 + Gyro.data[1] *dt/15000000.0;// TODO Offset accelerometer einstellen
    tempangle += (Comp_angle - tempangle)/50 - Gyro.data[2] *dt/15000000.0;
    angle[2] += Gyro.data[2] *dt/15000000.0; // gyro only here
    
    // Read RC data
    controller.setProcessValue(RC[3 -1].read());
    for (int j = 0; j < 4; j++)
        Motorvalue[j] = controller.compute(); // throttle
    
    for (int j = 0; j < 4; j++)
        Motor[j] = 1000 + 5*abs(angle[1]);//Motorvalue[j]; // set new motorspeeds
    pid.setProcessValue(angle[0]);
    pidtester = pid.compute();
}


int main() {
    // init screen
    pc.locate(10,5);
    pc.printf("Flybed v0.2");
    LEDs.roll(2);
    
    controller.setInputLimits(1000.0, 2000.0);
    controller.setOutputLimits(1000.0, 2000.0);
    controller.setMode(AUTO_MODE);
    
    pid.setInputLimits(-90.0, 90.0);
    pid.setOutputLimits(-90.0, 90.0);
    pid.setMode(AUTO_MODE);
    pid.setSetPoint(0.0);
    
    // Start!
    GlobalTimer.start();
    Datagetter.attach(&get_Data, RATE);     // start to get data all 10ms
    while(1) {
        pc.locate(10,5); // PC output
        pc.printf("dt:%dms  %6.1fm   ", dt/1000, Alt.CalcAltitude(Alt.Pressure));
        pc.locate(10,8);
        pc.printf("Roll:%6.1f Pitch:%6.1f Yaw:%6.1f    ", angle[0], angle[1], angle[2]);
        pc.locate(10,9);
        pc.printf("ACC: Roll:%6.1f Pitch:%6.1f Yaw:%6.1f    ", Acc.angle[0], Acc.angle[1], Acc.angle[2]);
        pc.locate(10,10);
        pc.printf("Debug_Yaw:  Comp:%6.1f tempangle:%6.1f  ", Comp_angle, tempangle);
        pc.locate(10,12);
        pc.printf("Comp_data: %6.1f %6.1f %6.1f |||| %6.1f ", Comp.data[0], Comp.data[1], Comp.data[2], Comp.heading);
        pc.locate(10,13);
        pc.printf("Comp_scale: %6.4f %6.4f %6.4f   ", Comp.scale[0], Comp.scale[1], Comp.scale[2]);
        pc.locate(10,15);
        pc.printf("pidtester: %6.1f   RC: %d %d %d %d     ", pidtester, RC[0].read(), RC[1].read(), RC[2].read(), RC[3].read());
        pc.locate(10,18);
        pc.printf("Max: %d %d %d     ", Comp.Max[0], Comp.Max[1], Comp.Max[2]);
        pc.locate(10,19);
        pc.printf("Min: %d %d %d     ", Comp.Min[0], Comp.Min[1], Comp.Min[2]);
        LEDs.rollnext();
    }
}
