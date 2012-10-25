#include "mbed.h"       // Standard Library
#include "LED.h"        // LEDs framework for blinking ;)
#include "PC.h"         // Serial Port via USB by Roland Elmiger for debugging with Terminal (driver needed: https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe)
#include "L3G4200D.h"   // Gyro (Gyroscope)
#include "ADXL345.h"    // Acc (Accelerometer)
#include "HMC5883.h"    // Comp (Compass)
#include "BMP085.h"     // Alt (Altitude sensor)
#include "RC_Channel.h" // RemoteControl Chnnels with PPM
#include "Servo.h"      // Motor PPM
#include "PID.h"        // PID Library by Aaron Berk
#include "IntCtrl.h"    // Interrupt Control by Roland Elmiger

#define PI              3.1415926535897932384626433832795
#define Rad2Deg         57.295779513082320876798154814105
#define RATE            0.02 // speed of Ticker/PID

//#define COMPASSCALIBRATE decomment if you want to calibrate the Compass on start

Timer GlobalTimer; // global time to calculate processing speed
Ticker Datagetter; // timecontrolled interrupt to get data form IMU and RC
PID controller(1.0, 0.0, 0.0, RATE); // test PID controller for throttle
PID pid(1.0, 1.0, 0.0, RATE); // test PID controller for throttle
//PID P:3,0 bis 3,5 I:0,010 und 0,050 D:5 und 25 

// initialisation of hardware
LED         LEDs;
PC          pc(USBTX, USBRX, 57600);
L3G4200D    Gyro(p28, p27);
ADXL345     Acc(p28, p27);
HMC5883     Comp(p28, p27);
BMP085      Alt(p28, p27);
RC_Channel  RC[] = {(p11), (p12), (p13), (p14)}; // noooo p19/p20!!!!
Servo       Motor[] = {(p15), (p16), (p17), (p18)};

// variables for loop
unsigned long   dt_get_data = 0;
unsigned long   time_get_data = 0;
unsigned long   dt_read_sensors = 0;
unsigned long   time_read_sensors = 0;
float           angle[3] = {0,0,0}; // angle 0: x,roll / 1: y,pitch / 2: z,yaw
float           tempangle = 0;
int             Motorvalue[3];

float pidtester;

void get_Data()
{
    time_read_sensors = GlobalTimer.read_us();
    
    GPIO_IntDisable(0, 18, 2); // abschalten der Pins 11-14 mit Göttis library
    GPIO_IntDisable(0, 17, 2);
    GPIO_IntDisable(0, 16, 2);
    GPIO_IntDisable(0, 15, 2);
    //__disable_irq(); // test, deactivate all interrupts, I2C working?
    // read data from sensors
    Gyro.read(); // einzeln testen! mit LEDs
    Acc.read();
    Comp.read();
    //Alt.Update(); TODO braucht zu lange zum auslesen!
    //__enable_irq();
    GPIO_IntEnable(0, 18, 2); // schaltet enable wirklich wieder ein?? änderungsbedarf??
    GPIO_IntEnable(0, 17, 2);
    GPIO_IntEnable(0, 16, 2);
    GPIO_IntEnable(0, 15, 2);
    
    dt_read_sensors = GlobalTimer.read_us() - time_read_sensors;
    
    // meassure dt
    dt_get_data = GlobalTimer.read_us() - time_get_data; // time in us since last loop
    time_get_data = GlobalTimer.read_us();      // set new time for next measurement
    
    // calculate angles for roll, pitch an yaw
    angle[0] += (Acc.angle[0] - angle[0])/50 + Gyro.data[0] *dt_get_data/15000000.0;
    angle[1] += (Acc.angle[1]+3 - angle[1])/50 + Gyro.data[1] *dt_get_data/15000000.0;// TODO Offset accelerometer einstellen
    tempangle += (Comp.get_angle() - tempangle)/50 + Gyro.data[2] *dt_get_data/15000000.0;
    angle[2] += Gyro.data[2] *dt_get_data/15000000.0; // gyro only here
    
    // TODO Read RC data
    
    // calculate new motorspeeds
    /*
    Motor[0] = 1000 + (100 + (angle[0] * 500/90)) * (RC[1].read() - 1000) / 1000;
    Motor[1] = 1000 + (100 + (angle[1] * 500/90)) * (RC[1].read() - 1000) / 1000;
    Motor[2] = 1000 + (100 - (angle[0] * 500/90)) * (RC[1].read() - 1000) / 1000;
    Motor[3] = 1000 + (100 - (angle[1] * 500/90)) * (RC[1].read() - 1000) / 1000;
    */
}


int main() {
    //NVIC_SetPriority(TIMER3_IRQn, 2); // set priorty of tickers below hardware interrupts
    
    #ifdef COMPASSCALIBRATE
        pc.locate(10,5);
        pc.printf("CALIBRATING");
        Comp.calibrate(60);
    #endif
    
    // init screen
    pc.locate(10,5);
    pc.printf("Flybed v0.2");
    LEDs.roll(2);
    
    // Start!
    GlobalTimer.start();
    Datagetter.attach(&get_Data, RATE);     // start to get data all RATEms
    
    while(1) { 
        pc.locate(10,5); // PC output
        pc.printf("dt:%dms   dt_sensors:%dus    Altitude:%6.1fm   ", dt_get_data/1000, dt_read_sensors, Alt.CalcAltitude(Alt.Pressure));
        pc.locate(10,8);
        pc.printf("Roll:%6.1f Pitch:%6.1f Yaw:%6.1f    ", angle[0], angle[1], angle[2]);
        pc.locate(10,9);
        pc.printf("ACC: Roll:%6.1f Pitch:%6.1f Yaw:%6.1f    ", Acc.angle[0], Acc.angle[1], Acc.angle[2]);
        pc.locate(10,10);
        pc.printf("Debug_Yaw:  Comp:%6.1f tempangle:%6.1f  ", Comp.get_angle(), tempangle);
        pc.locate(10,12);
        pc.printf("Comp_data: %6.1f %6.1f %6.1f |||| %6.1f ", Comp.data[0], Comp.data[1], Comp.data[2], Comp.get_angle());
        pc.locate(10,13);
        pc.printf("Comp_scale: %6.4f %6.4f %6.4f   ", Comp.scale[0], Comp.scale[1], Comp.scale[2]);
        pc.locate(10,15);
        pc.printf("pidtester: %6.1f   RC: %d %d %d %d     ", pidtester, RC[0].read(), RC[1].read(), RC[2].read(), RC[3].read());
        
        pc.locate(10,19);
        pc.printf("RC0: %d :[", RC[0].read());
        for (int i = 0; i < (RC[0].read() - 1000)/17; i++)
            pc.printf("=");
        pc.printf("                                                       ");
        pc.locate(10,20);
        pc.printf("RC1: %d :[", RC[1].read());
        for (int i = 0; i < (RC[1].read() - 1000)/17; i++)
            pc.printf("=");
        pc.printf("                                                       ");
        pc.locate(10,21);
        pc.printf("RC2: %d :[", RC[2].read());
        for (int i = 0; i < (RC[2].read() - 1000)/17; i++)
            pc.printf("=");
        pc.printf("                                                       ");
        pc.locate(10,22);
        pc.printf("RC3: %d :[", RC[3].read());
        for (int i = 0; i < (RC[3].read() - 1000)/17; i++)
            pc.printf("=");
        pc.printf("                                                       ");
        LEDs.rollnext();
    }
}
