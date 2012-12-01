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
#include "IMU_Filter.h" // Class to calculate position angles
#include "Mixer.h"      // Class to calculate motorspeeds from Angles, Regulation and RC-Signals

#define RATE            0.02                                // speed of the interrupt for Sensors and PID
#define MAXPITCH        40                                  // maximal angle from horizontal that the PID is aming for
#define YAWSPEED        2                                   // maximal speed of yaw rotation in degree per Rate

#define P_VALUE         0.02                                // PID values
#define I_VALUE         20
#define D_VALUE         0.004

//#define COMPASSCALIBRATE // decomment if you want to calibrate the Compass on start
#define PC_CONNECTED // decoment if you want to debug per USB and your PC

Timer GlobalTimer;                      // global time to calculate processing speed
Ticker Dutycycler;                      // timecontrolled interrupt to get data form IMU and RC

// initialisation of hardware (see includes for more info)
LED         LEDs;
#ifdef PC_CONNECTED
    PC          pc(USBTX, USBRX, 115200);
#endif
LocalFileSystem local("local");               // Create the local filesystem under the name "local"
FILE *Logger;
L3G4200D    Gyro(p28, p27);
ADXL345     Acc(p28, p27);
HMC5883     Comp(p28, p27);
BMP085_old      Alt(p28, p27);
RC_Channel  RC[] = {RC_Channel(p11,1), RC_Channel(p12,2), RC_Channel(p13,3), RC_Channel(p14,4)};    // no p19/p20 !
Servo_PWM   ESC[] = {p21, p22, p23, p24}; // p21 - p26 only because PWM!
IMU_Filter  IMU;    // don't write () after constructor for no arguments!
Mixer       MIX;      

// 0:X:Roll 1:Y:Pitch 2:Z:Yaw
PID     Controller[] = {PID(P_VALUE, I_VALUE, D_VALUE, RATE), PID(P_VALUE, I_VALUE, D_VALUE, RATE), PID(0.02, 0, 0.004, RATE)}; // TODO: RATE != dt immer anpassen

// global variables
bool            armed = false;          // this variable is for security
unsigned long   dt = 0;
unsigned long   time_for_dt = 0;
unsigned long   dt_read_sensors = 0;
unsigned long   time_read_sensors = 0;
float           tempangle = 0; // temporärer winkel für yaw mit kompass
float           controller_value[] = {0,0,0};
float           virt_angle[] = {0,0,0};
float           yawposition = 0;

void dutycycle() // method which is called by the Ticker Dutycycler every RATE seconds
{
    time_read_sensors = GlobalTimer.read_us();
    
    // read data from sensors // ATTENTION! the I2C option repeated true is important because otherwise interrupts while bus communications cause crashes
    Gyro.read();
    Acc.read(); // TODO: nicht jeder Sensor immer? höhe nicht so wichtig
    //Comp.read();
    //Alt.Update(); TODO braucht zu lange zum auslesen!
    
    dt_read_sensors = GlobalTimer.read_us() - time_read_sensors;
    
    // meassure dt
    dt = GlobalTimer.read_us() - time_for_dt; // time in us since last loop
    time_for_dt = GlobalTimer.read_us();      // set new time for next measurement
    
    IMU.compute(dt, Gyro.data, Acc.data);
    
    // Arming / disarming
    if(RC[2].read() < 20 && RC[3].read() > 850) {
        armed = true;
        #ifdef LOGGER
            if(Logger == NULL)
                Logger = fopen("/local/log.csv", "a");
        #endif
    }
    if((RC[2].read() < 30 && RC[3].read() < 30) || RC[3].read() < -10 || RC[2].read() < -10 || RC[1].read() < -10 || RC[0].read() < -10) {
        armed = false;
        #ifdef LOGGER
            if(Logger != NULL) {
                fclose(Logger);
                Logger = NULL;
            }
        #endif
    }
    
    if (armed) // for SECURITY!
    {       
            // RC controlling
            /*virt_angle[0] = IMU.angle[0] + (RC[0].read()-500)*MAXPITCH/500.0; // TODO: zuerst RC calibration
            virt_angle[1] = IMU.angle[1] + (RC[1].read()-500)*MAXPITCH/500.0;
            yawposition += (RC[3].read()-500)*YAWSPEED/500;
            virt_angle[2] = IMU.angle[2] + yawposition;*/
            
            // PID controlling
            /*if (!(RC[0].read() == -100)) { // the RC must be there to controll // alte version mit setpoint, nicht nötig? granzen bei yaw los? :)
                Controller[0].setSetPoint(-((RC[0].read()-500)*MAXPITCH/500.0));    // set angles based on RC input
                Controller[1].setSetPoint(-((RC[1].read()-500)*MAXPITCH/500.0));
                Controller[2].setSetPoint(-((RC[3].read()-500)*180.0/500.0));
            }*/
            for(int i=0;i<3;i++) {
                Controller[i].setProcessValue(virt_angle[i]); // give the controller the new measured angles that are allready controlled by RC
                controller_value[i] = Controller[i].compute() - 1000; // -1000 because controller has output from 0 to 2000
            }
            
            MIX.compute(dt, IMU.angle, RC[2].read(), controller_value); // let the Mixer compute motorspeeds based on throttle and controller output
            
            for(int i=0;i<4;i++)   // Set new motorspeeds
                ESC[i] = (int)MIX.Motor_speed[i];
            
            #ifdef LOGGER
                // Writing Log
                for(int i = 0; i < 3; i++) {
                    fprintf(Logger, "%f;", angle[i]);
                    fprintf(Logger, "%f;", controller_value[i]);
                }
                fprintf(Logger, "\r\n");
            #endif
    } else {
        for(int i=0;i<4;i++) // for security reason, set every motor to zero speed
            ESC[i] = 0;
        for(int i=0;i<3;i++)
            Controller[i].reset(); // TODO: schon ok so? anfangspeek?!
    }
}

int main() { // main programm for initialisation and debug output
    NVIC_SetPriority(TIMER3_IRQn, 1); // set priorty of tickers below hardware interrupts (standard priority is 0)(this is to prevent the RC interrupt from waiting until ticker is finished)
    
    #ifdef LOGGER
        Logger = fopen("/local/log.csv", "w"); // Prepare Logfile
        for(int i = 0; i < 3; i++) {
            fprintf(Logger, "angle[%d];", i);
            fprintf(Logger, "controller_value[%d];", i);
        }
        fprintf(Logger, "\r\n");
        fclose(Logger);
        Logger = NULL;
    #endif
    
    // Prepare PID Controllers
    for(int i=0;i<3;i++) {
        Controller[i].setInputLimits(-90.0, 90.0);
        Controller[i].setOutputLimits(0.0, 2000.0);
        Controller[i].setBias(1000);
        Controller[i].setMode(MANUAL_MODE);//AUTO_MODE);
        Controller[i].setSetPoint(0);
    }
    //Controller[2].setInputLimits(-180.0, 180.0); // yaw 360 grad TODO: Yawsteuerung mit -180 bis 180 grad
    
    #ifdef PC_CONNECTED
        #ifdef COMPASSCALIBRATE
            pc.locate(10,5);
            pc.printf("CALIBRATING");
            Comp.calibrate(60);
        #endif
        
        // init screen
        pc.locate(10,5);
        pc.printf("Flybed v0.2");
    #endif
    LEDs.roll(2);
    
    // Start!
    GlobalTimer.start();
    Dutycycler.attach(&dutycycle, RATE);     // start to process all RATEms
    
    while(1) { 
        #ifdef PC_CONNECTED
            pc.locate(30,0); // PC output
            pc.printf("dt:%dms   dt_sensors:%dus    Altitude:%6.1fm   ", dt/1000, dt_read_sensors, Alt.CalcAltitude(Alt.Pressure));
            pc.locate(5,1);
            if(armed)
                pc.printf("ARMED!!!!!!!!!!!!!");
            else
                pc.printf("DIS_ARMED            ");
            pc.locate(5,3);
            pc.printf("Roll:%6.1f   Pitch:%6.1f   Yaw:%6.1f    ", IMU.angle[0], IMU.angle[1], IMU.angle[2]);
            pc.locate(5,5);
            pc.printf("Gyro.data: X:%6.1f  Y:%6.1f  Z:%6.1f", Gyro.data[0], Gyro.data[1], Gyro.data[2]);
            pc.locate(5,8);
            pc.printf("Acc.data: X:%6d  Y:%6d  Z:%6d", Acc.data[0], Acc.data[1], Acc.data[2]); 
            pc.locate(5,11);
            pc.printf("PID Result:");
            for(int i=0;i<3;i++)
                pc.printf("  %d: %6.1f", i, controller_value[i]);
            pc.locate(5,14);
            pc.printf("RC controll: roll: %f     pitch: %f    yaw: %f    ", (RC[0].read()-500)*MAXPITCH/500.0, (RC[0].read()-500)*MAXPITCH/500.0, yawposition);
            
            pc.locate(10,15);
            pc.printf("Debug_Yaw:  Comp:%6.1f tempangle:%6.1f  ", Comp.get_angle(), tempangle);
            pc.locate(10,16);
            pc.printf("Comp_data: %6.1f %6.1f %6.1f |||| %6.1f ", Comp.data[0], Comp.data[1], Comp.data[2], Comp.get_angle());
            pc.locate(10,17);
            //pc.printf("Comp_scale: %6.4f %6.4f %6.4f   ", Comp.scale[0], Comp.scale[1], Comp.scale[2]); no more accessible its private
            pc.locate(10,18);
            pc.printf("Comp_data: %6.1f %6.1f %6.1f |||| %6.1f ", Comp.data[0], Comp.data[1], Comp.data[2], Comp.get_angle());
            
            // graphical representation for RC signal // TODO: nicht nötig, nach funktionieren der RC kalibrierung weg damit
            pc.locate(10,19);
            pc.printf("RC0: %4d :[", RC[0].read());
            for (int i = 0; i < RC[0].read()/17; i++)
                pc.printf("=");
            pc.printf("                                                       ");
            pc.locate(10,20);
            pc.printf("RC1: %4d :[", RC[1].read());
            for (int i = 0; i < RC[1].read()/17; i++)
                pc.printf("=");
            pc.printf("                                                       ");
            pc.locate(10,21);
            pc.printf("RC2: %4d :[", RC[2].read());
            for (int i = 0; i < RC[2].read()/17; i++)
                pc.printf("=");
            pc.printf("                                                       ");
            pc.locate(10,22);
            pc.printf("RC3: %4d :[", RC[3].read());
            for (int i = 0; i < RC[3].read()/17; i++)
                pc.printf("=");
            pc.printf("                                                       ");
        #endif
        if(armed){
            LEDs.rollnext();
        } else {
            for(int i=1;i<=4;i++)
                LEDs.set(i);
        }
    }
}