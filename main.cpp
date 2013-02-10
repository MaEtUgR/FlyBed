#include "mbed.h"       // Standard Library
#include "LED.h"        // LEDs framework for blinking ;)
#include "PC.h"         // Serial Port via USB by Roland Elmiger for debugging with Terminal (driver needed: https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe)
#include "L3G4200D.h"   // Gyro (Gyroscope)
#include "ADXL345.h"    // Acc (Accelerometer)
#include "HMC5883.h"    // Comp (Compass)
#include "BMP085_old.h"     // Alt (Altitude sensor)
#include "RC_Channel.h" // RemoteControl Channels with PPM
#include "Servo_PWM.h"  // Motor PPM using PwmOut
#include "PID.h"        // PID Library by Aaron Berk
#include "IMU_Filter.h" // Class to calculate position angles
#include "Mixer.h"      // Class to calculate motorspeeds from Angles, Regulation and RC-Signals

#define RATE            0.002                               // speed of the interrupt for Sensors and PID
#define PPM_FREQU       495                                 // Hz Frequency of PPM Signal for ESCs (maximum <500Hz)
#define MAXPITCH        40                                  // maximal angle from horizontal that the PID is aming for
#define YAWSPEED        2                                   // maximal speed of yaw rotation in degree per Rate

#define P               0.35                                   // PID values
#define I               0
#define D               0.5

//#define COMPASSCALIBRATE // decomment if you want to calibrate the Compass on start
#define PC_CONNECTED // decoment if you want to debug per USB/Bluetooth and your PC

Timer GlobalTimer;                      // global time to calculate processing speed
Ticker Dutycycler;                      // timecontrolled interrupt to get data form IMU and RC

// initialisation of hardware (see includes for more info)
LED         LEDs;
#ifdef PC_CONNECTED
    PC          pc(USBTX, USBRX, 115200);    // USB
    //PC          pc(p9, p10, 115200);      // Bluetooth
#endif
LocalFileSystem local("local");               // Create the local filesystem under the name "local"
//FILE *Logger;
L3G4200D    Gyro(p28, p27);
ADXL345     Acc(p28, p27);
HMC5883     Comp(p28, p27);
BMP085_old      Alt(p28, p27);
RC_Channel  RC[] = {RC_Channel(p11,1), RC_Channel(p12,2), RC_Channel(p13,4), RC_Channel(p14,3)};    // no p19/p20 !
Servo_PWM   ESC[] = {Servo_PWM(p21,PPM_FREQU), Servo_PWM(p22,PPM_FREQU), Servo_PWM(p23,PPM_FREQU), Servo_PWM(p24,PPM_FREQU)}; // p21 - p26 only because PWM needed!
IMU_Filter  IMU;    // don't write () after constructor for no arguments!
Mixer       MIX(1);      

// 0:X:Roll 1:Y:Pitch 2:Z:Yaw
PID     Controller[] = {PID(P, I, D, 1000), PID(P, I, D, 1000), PID(0.2, 0, 0.1, 1000)};

// global variables
bool            armed = false;          // this variable is for security (when false no motor rotates any more)
unsigned long   dt = 0;
unsigned long   time_for_dt = 0;
unsigned long   dt_read_sensors = 0;
unsigned long   time_read_sensors = 0;
float           tempangle = 0; // temporärer winkel für yaw mit kompass
float           controller_value[] = {0,0,0};
float           RC_angle[] = {0,0,0};  // Angle of the RC Sticks, to steer the QC

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
    if(RC[3].read() < 20 && RC[2].read() > 850) {
        armed = true;
        #ifdef LOGGER
            if(Logger == NULL)
                Logger = fopen("/local/log.csv", "a");
        #endif
    }
    if((RC[3].read() < 30 && RC[2].read() < 30) || RC[2].read() < -10 || RC[3].read() < -10 || RC[1].read() < -10 || RC[0].read() < -10) {
        armed = false;
        #ifdef LOGGER
            if(Logger != NULL) {
                fclose(Logger);
                Logger = NULL;
            }
        #endif
    }
    
    for(int i=0;i<3;i++)    // calculate new angle we want the QC to have
        RC_angle[i] = (RC[i].read()-500)*30/500.0;
    
    for(int i=0;i<3;i++) {
        Controller[i].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
        controller_value[i] = Controller[i].compute(RC_angle[i], IMU.angle[i]); // give the controller the actual angle and get his advice to correct
    }
                
    
    if (armed) // for SECURITY!
    {       
            // RC controlling
            /*for(int i=0;i<3;i++)
                AnglePosition[i] -= (RC[i].read()-500)*2/500.0;*/
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
            
            
            MIX.compute(dt, RC[3].read(), controller_value); // let the Mixer compute motorspeeds based on throttle and controller output
            
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
        //pc.printf("%f,%f,%f,%f,%f,%f\r\n", IMU.angle[0], IMU.angle[1], IMU.angle[2], controller_value[0], controller_value[1], controller_value[2]); // serialplot of IMU
        #if 1 //pc.cls();
            pc.locate(30,0); // PC output
            pc.printf("dt:%3.3fms   dt_sensors:%dus    Altitude:%6.1fm   ", dt/1000.0, dt_read_sensors, Alt.CalcAltitude(Alt.Pressure));
            pc.locate(5,1);
            if(armed)
                pc.printf("ARMED!!!!!!!!!!!!!");
            else
                pc.printf("DIS_ARMED            ");
            pc.locate(5,3);
            pc.printf("Roll:%6.1f   Pitch:%6.1f   Yaw:%6.1f    ", IMU.angle[0], IMU.angle[1], IMU.angle[2]);
            pc.locate(5,5);
            pc.printf("Gyro.data: X:%6.1f  Y:%6.1f  Z:%6.1f", Gyro.data[0], Gyro.data[1], Gyro.data[2]);
            pc.locate(5,6);
            pc.printf("Acc.data: X:%6d  Y:%6d  Z:%6d", Acc.data[0], Acc.data[1], Acc.data[2]); 
            pc.locate(5,11);
            pc.printf("PID Result:");
            for(int i=0;i<3;i++)
                pc.printf("  %d: %6.1f", i, controller_value[i]);
            pc.locate(5,14);
            pc.printf("RC angle: roll: %f     pitch: %f      yaw: %f    ", RC_angle[0], RC_angle[1], RC_angle[2]);
            pc.locate(5,16);
            pc.printf("Motor: 0:%d 1:%d 2:%d 3:%d    ", (int)MIX.Motor_speed[0], (int)MIX.Motor_speed[1], (int)MIX.Motor_speed[2], (int)MIX.Motor_speed[3]);
            
            // RC
            pc.locate(10,19);
            pc.printf("RC0: %4d   ", RC[0].read());
            pc.printf("RC1: %4d   ", RC[1].read());
            pc.printf("RC2: %4d   ", RC[2].read());
            pc.printf("RC3: %4d   ", RC[3].read());
        #endif
        if(armed){
            LEDs.rollnext();
        } else {
            for(int i=1;i<=4;i++)
                LEDs.set(i);
        }
        wait(0.05);
    }
}