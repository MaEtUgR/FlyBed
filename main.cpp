#include "mbed.h"       // Standard Library
#include "LED.h"        // LEDs framework for blinking ;)
#include "PC.h"         // Serial Port via USB by Roland Elmiger for debugging with Terminal (driver needed: https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe)
#include "L3G4200D.h"   // Gyro (Gyroscope)
#include "ADXL345.h"    // Acc (Accelerometer)
#include "HMC5883.h"    // Comp (Compass)
#include "BMP085_old.h"     // Alt (Altitude sensor)
#include "RC_Channel.h" // RemoteControl Chnnels with PPM
#include "Servo_PWM.h"  // Motor PPM using PwmOut
#include "PID.h"        // PID Library by Aaron Berk

#define PI              3.1415926535897932384626433832795   // ratio of a circle's circumference to its diameter
#define RAD2DEG         57.295779513082320876798154814105   // ratio between radians and degree (360/2Pi)
#define RATE            0.02                                // speed of the interrupt for Sensors and PID

#define P_VALUE         0.1                                // viel zu tief!!!!!
#define I_VALUE         0.0                                // 
#define D_VALUE         0.0                                // 

//#define COMPASSCALIBRATE // decomment if you want to calibrate the Compass on start

Timer GlobalTimer;                      // global time to calculate processing speed
Ticker Datagetter;                      // timecontrolled interrupt to get data form IMU and RC

// initialisation of hardware (see includes for more info)
LED         LEDs;
PC          pc(USBTX, USBRX, 115200);
L3G4200D    Gyro(p28, p27);
ADXL345     Acc(p28, p27);
HMC5883     Comp(p28, p27);
BMP085_old      Alt(p28, p27);
RC_Channel  RC[] = {(p11), (p12), (p13), (p14)};    // noooo p19/p20 !
Servo_PWM   Motor[] = {(p21), (p22), (p23), (p24)}; // p21 - p26 only !

//PID         Controller[] = {(P_VALUE, I_VALUE, D_VALUE, RATE), (P_VALUE, I_VALUE, D_VALUE, RATE), (P_VALUE, I_VALUE, D_VALUE, RATE)}; // TODO: RATE != dt immer anpassen
//PID       P:3,0 bis 3,5     I:0,010 und 0,050       D:5 und 25 
PID     Controller(P_VALUE, I_VALUE, D_VALUE, RATE);


// global varibles
unsigned long   dt_get_data = 0; // TODO: dt namen
unsigned long   time_get_data = 0;
unsigned long   dt_read_sensors = 0;
unsigned long   time_read_sensors = 0;
float           angle[3] = {0,0,0}; // calculated values of the position [0: x,roll | 1: y,pitch | 2: z,yaw]
float           tempangle = 0; // temporärer winkel für yaw ohne kompass
float co; // PID test

void get_Data() // method which is called by the Ticker Datagetter every RATE seconds
{
    time_read_sensors = GlobalTimer.read_us();
    
    // read data from sensors // ATTENTION! the I2C option repeated true is important because otherwise interrupts while bus communications cause crashes
    Gyro.read();
    Acc.read(); // TODO: nicht jeder Sensor immer? höhe nicht so wichtig
    Comp.read();
    //Alt.Update(); TODO braucht zu lange zum auslesen!
    
    dt_read_sensors = GlobalTimer.read_us() - time_read_sensors;
    
    // meassure dt
    dt_get_data = GlobalTimer.read_us() - time_get_data; // time in us since last loop
    time_get_data = GlobalTimer.read_us();      // set new time for next measurement
    
    // calculate angles for roll, pitch an yaw
    angle[0] += (Acc.angle[0] - angle[0])/50 + Gyro.data[0] *dt_get_data/15000000.0;
    angle[1] += (Acc.angle[1]+3 - angle[1])/50 + Gyro.data[1] *dt_get_data/15000000.0;// TODO Offset accelerometer einstellen
    tempangle += (Comp.get_angle() - tempangle)/50 + Gyro.data[2] *dt_get_data/15000000.0;
    angle[2] += Gyro.data[2] *dt_get_data/15000000.0; // gyro only here
    
    // PID controlling
    Controller.setProcessValue(angle[1]);
    
    // calculate new motorspeeds
    co = Controller.compute() - 1000;
    if (RC[2].read() > 1100) // zu SICHERHEIT! zum testen (später ersetzen armed unarmed)
    {
        /*Motor[0] = RC[2].read()+((RC[0].read() - 1500)/10.0)+40;
        Motor[2] = RC[2].read()-((RC[0].read() - 1500)/10.0)-40;*/
        Motor[0] = RC[2].read()+co+40;
        Motor[2] = RC[2].read()-co-40;
    } else {
        Motor[0] = 1000;
        Motor[1] = 1000;
        Motor[2] = 1000;
        Motor[3] = 1000;
    }
    /*Motor[0] = 1000 + (100 - (angle[1] * 500/90)) * (RC[2].read() - 1000) / 1000; // test für erste reaktion der motoren entgegen der Auslenkung
    Motor[1] = 1000 + (100 - (angle[0] * 500/90)) * (RC[2].read() - 1000) / 1000;
    Motor[2] = 1000 + (100 + (angle[1] * 500/90)) * (RC[2].read() - 1000) / 1000;
    Motor[3] = 1000 + (100 + (angle[0] * 500/90)) * (RC[2].read() - 1000) / 1000;*/
}

int main() { // main programm only used for initialisation and debug output
    NVIC_SetPriority(TIMER3_IRQn, 1); // set priorty of tickers below hardware interrupts (standard priority is 0)
    
    //for(int i=0;i<3;i++)
        Controller.setInputLimits(-90.0, 90.0);
        Controller.setOutputLimits(0.0, 2000.0);
        Controller.setBias(1000);
        Controller.setMode(MANUAL_MODE);//AUTO_MODE);
        Controller.setSetPoint(0);
    
    #ifdef COMPASSCALIBRATE
        pc.locate(10,5);
        pc.printf("CALIBRATING");
        Comp.calibrate(60);
    #endif
    
    // init screen
    pc.locate(10,5);
    pc.printf("Flybed v0.2");
    LEDs.roll(2);
    
    // Start! TODO: Motor und RC start (armed....?)
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
        //pc.printf("Comp_scale: %6.4f %6.4f %6.4f   ", Comp.scale[0], Comp.scale[1], Comp.scale[2]); no more accessible its private
        pc.locate(10,15);
        pc.printf("PID Test: %6.1f", co);
        pc.locate(10,16);
        pc.printf("Gyro_data: X:%6.1f  Y:%6.1f  Z:%6.1f", Gyro.data[0], Gyro.data[1], Gyro.data[2]);
        pc.locate(10,17);
        pc.printf("Acc_data: X:%6.1f  Y:%6.1f  Z:%6.1f", Acc.data[0], Acc.data[1], Acc.data[2]);
        pc.locate(10,18);
        pc.printf("Comp_data: %6.1f %6.1f %6.1f |||| %6.1f ", Comp.data[0], Comp.data[1], Comp.data[2], Comp.get_angle());
        
        pc.locate(10,19);
        pc.printf("RC0: %4d :[", RC[0].read());
        for (int i = 0; i < (RC[0].read() - 1000)/17; i++)
            pc.printf("=");
        pc.printf("                                                       ");
        pc.locate(10,20);
        pc.printf("RC1: %4d :[", RC[1].read());
        for (int i = 0; i < (RC[1].read() - 1000)/17; i++)
            pc.printf("=");
        pc.printf("                                                       ");
        pc.locate(10,21);
        pc.printf("RC2: %4d :[", RC[2].read());
        for (int i = 0; i < (RC[2].read() - 1000)/17; i++)
            pc.printf("=");
        pc.printf("                                                       ");
        pc.locate(10,22);
        pc.printf("RC3: %4d :[", RC[3].read());
        for (int i = 0; i < (RC[3].read() - 1000)/17; i++)
            pc.printf("=");
        pc.printf("                                                       ");
        LEDs.rollnext();
    }
}
