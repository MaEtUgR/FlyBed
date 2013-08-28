#include "mbed.h"       // Standard Library
#include "LED.h"        // LEDs framework for blinking ;)
#include "PC.h"         // Serial Port via USB by Roland Elmiger for debugging with Terminal (driver needed: https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe)
#include "L3G4200D.h"   // Gyro (Gyroscope)
#include "ADXL345.h"    // Acc (Accelerometer)
#include "HMC5883.h"    // Comp (Compass)
#include "BMP085_old.h"     // Alt (Altitude sensor)
#include "RC_Channel.h" // RemoteControl Channels with PPM
#include "Servo_PWM.h"  // Motor PPM using PwmOut
#include "PID.h"        // PID Library (slim, self written)
#include "IMU_Filter.h" // Class to calculate position angles
#include "Mixer.h"      // Class to calculate motorspeeds from Angles, Regulation and RC-Signals

#define RATE            0.002                               // speed of the interrupt for Sensors and PID
#define PPM_FREQU       495                                 // Hz Frequency of PPM Signal for ESCs (maximum <500Hz)
#define RC_SENSITIVITY  30                                  // maximal angle from horizontal that the PID is aming for
#define YAWSPEED        0.2                                 // maximal speed of yaw rotation in degree per Rate
#define INTEGRAL_MAX    300

// RC
#define AILERON         0
#define ELEVATOR        1
#define RUDDER          2
#define THROTTLE        3
// Axes
#define ROLL            0
#define PITCH           1
#define YAW             2

#define PC_CONNECTED // decoment if you want to debug per USB/Bluetooth and your PC

// Global variables
bool    armed = false;                  // this variable is for security (when false no motor rotates any more)
bool    RC_present = false;             // this variable shows if an RC is present
float   dt = 0;
float   time_for_dt = 0;
float   dt_read_sensors = 0;
float   time_read_sensors = 0;
float   controller_value[] = {0,0,0};   // The calculated answer form the Controller
float   RC_angle[] = {0,0,0};           // Angle of the RC Sticks, to steer the QC
float   RC_yaw_adding;                  // temporary variable to take the desired yaw adjustment

float P = 4.0;                          // PID values
float I = 0;
float D = 0.1;

float PY = 0;                          // PID values for YAW
float IY = 0;
float DY = 0;

Timer GlobalTimer;                      // global time to calculate processing speed
Ticker Dutycycler;                      // timecontrolled interrupt for exact timed control loop

// Initialisation of hardware (see includes for more info)
LED         LEDs;
#ifdef PC_CONNECTED
    //PC          pc(USBTX, USBRX, 115200);    // USB
    PC          pc(p9, p10, 115200);       // Bluetooth
#endif
L3G4200D    Gyro(p28, p27);
ADXL345     Acc(p28, p27);
HMC5883     Comp(p28, p27);
BMP085_old      Alt(p28, p27);
RC_Channel  RC[] = {RC_Channel(p5,1), RC_Channel(p6,2), RC_Channel(p8,4), RC_Channel(p7,3)};                                // no p19/p20 !
Servo_PWM   ESC[] = {Servo_PWM(p21,PPM_FREQU), Servo_PWM(p22,PPM_FREQU), Servo_PWM(p23,PPM_FREQU), Servo_PWM(p24,PPM_FREQU)};   // p21 - p26 only because PWM needed!
IMU_Filter  IMU;    // (don't write () after constructor for no arguments!)
Mixer       MIX(1); // 0 for +-Formation, 1 for X-Formation 
PID     Controller[] = {PID(P, I, D, INTEGRAL_MAX), PID(P, I, D, INTEGRAL_MAX), PID(PY, IY, DY, INTEGRAL_MAX)}; // 0:X:Roll 1:Y:Pitch 2:Z:Yaw

void dutycycle() { // method which is called by the Ticker Dutycycler every RATE seconds
    time_read_sensors = GlobalTimer.read(); // start time measure for sensors
    
    // read data from sensors // ATTENTION! the I2C option repeated true is important because otherwise interrupts while bus communications cause crashes
    Gyro.read();
    Acc.read();
    //Comp.read(); // TODO: not every loop every sensor? altitude not that important
    //Alt.Update(); // TODO needs very long to read because of waits
    
    //pc.printf("%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f\r\n", Gyro.data[0], Gyro.data[1], Gyro.data[2], Acc.data[0], Acc.data[1], Acc.data[2]);
    
    
    
    // meassure dt for the filter
    dt = GlobalTimer.read() - time_for_dt; // time in us since last loop
    time_for_dt = GlobalTimer.read();      // set new time for next measurement
    
    IMU.compute(dt, Gyro.data, Acc.data);
    //pc.printf("%f,%f,%f,%3.5fs,%3.5fs\r\n", IMU.angle[0], IMU.angle[1], IMU.angle[2], dt, dt_read_sensors);

    if(RC[AILERON].read() == -100 || RC[ELEVATOR].read() == -100 || RC[RUDDER].read() == -100 || RC[THROTTLE].read() == -100)
        RC_present = false;
    else
        RC_present = true;

    // Arming / disarming
    if(RC[THROTTLE].read() < 20 && RC[RUDDER].read() > 850) {
        armed = true;
        RC_angle[YAW] = IMU.angle[YAW];
    }
    if((RC[THROTTLE].read() < 30 && RC[RUDDER].read() < 30) || !RC_present) {
        armed = false;
    }
    
    // RC Angle ROLL-PITCH-Part
    for(int i=0;i<2;i++) {    // calculate new angle we want the QC to have
        if (RC_present)
            RC_angle[i] = (RC[i].read()-500)*RC_SENSITIVITY/500.0;
        else
            RC_angle[i] = 0;
    }
    
    // RC Angle YAW-Part
    if (RC_present && RC[THROTTLE].read() > 20)
        RC_yaw_adding = (RC[RUDDER].read()-500)*YAWSPEED/500;
    else
        RC_yaw_adding = 0;
        
    while(RC_angle[YAW] + RC_yaw_adding < -180 || RC_angle[YAW] + RC_yaw_adding > 180) { // make shure it's in the cycle -180 to 180
        if(RC_angle[YAW] + RC_yaw_adding < -180)
            RC_yaw_adding += 360;
        if(RC_angle[YAW] + RC_yaw_adding > 180)
            RC_yaw_adding -= 360;
    }
    RC_angle[YAW] += RC_yaw_adding;  // for yaw angle it's integrated
    
    // PID controlling
    for(int i=0;i<2;i++) {
        Controller[i].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
        controller_value[i] = Controller[i].compute(RC_angle[i], IMU.angle[i]); // give the controller the actual angle and get his advice to correct
    }
    Controller[YAW].setIntegrate(armed); // same for YAW
    if (abs(RC_angle[YAW] - IMU.angle[YAW]) > 180)  // for YAW a special calculation because of range -180 to 180
         if (RC_angle[YAW] > IMU.angle[YAW])
            controller_value[YAW] = Controller[YAW].compute(RC_angle[YAW] - 360, IMU.angle[YAW]);
         else
            controller_value[YAW] = Controller[YAW].compute(RC_angle[YAW] + 360, IMU.angle[YAW]);
    else
        controller_value[YAW] = Controller[YAW].compute(RC_angle[YAW], IMU.angle[YAW]);
    
    if (armed) // for SECURITY!
    {       
            MIX.compute(RC[THROTTLE].read(), controller_value); // let the Mixer compute motorspeeds based on throttle and controller output
            for(int i=0;i<4;i++)   // Set new motorspeeds
                ESC[i] = (int)MIX.Motor_speed[i];
            
    } else {
        for(int i=0;i<4;i++) // for security reason, set every motor to zero speed
            ESC[i] = 0;
    }
    pc.printf("%d,%f,%f,  %f,%f,%f,  %f,%f,%f,  %f,%f,%f,  %f,%f,%f,%f\r\n",
        armed,
        dt,
        dt_read_sensors,
        IMU.angle[ROLL], 
        IMU.angle[PITCH], 
        IMU.angle[YAW], 
        RC_angle[ROLL], 
        RC_angle[PITCH], 
        RC_angle[YAW], 
        controller_value[ROLL], 
        controller_value[PITCH], 
        controller_value[YAW], 
        MIX.Motor_speed[0], 
        MIX.Motor_speed[1], 
        MIX.Motor_speed[2], 
        MIX.Motor_speed[3]);
                                    
    dt_read_sensors = GlobalTimer.read() - time_read_sensors; // stop time for loop
}

void commandexecuter(char* command) {  // take new PID values on the fly
    if (command[0] == 'p')
        if (command[1] == 'y')
            PY = atof(&command[2]);
        else
            P = atof(&command[1]);
    if (command[0] == 'i')
        if (command[1] == 'y')
            IY = atof(&command[2]);
        else
            I = atof(&command[1]);
    if (command[0] == 'd')
        if (command[1] == 'y')
            DY = atof(&command[2]);
        else
            D = atof(&command[1]);
    for(int i=0;i<2;i++) {
        Controller[i].setPID(P,I,D); // give the controller the new PID values
    }
    Controller[YAW].setPID(PY,IY,DY); // give the controller the new PID values
}

int main() { // main programm for initialisation and debug output
    NVIC_SetPriority(TIMER3_IRQn, 1); // set priorty of tickers below hardware interrupts (standard priority is 0)(this is to prevent the RC interrupt from waiting until ticker is finished)
    
    #ifdef PC_CONNECTED
        // init screen
        pc.locate(10,5);
        pc.printf("Flybed v0.2");
    #endif
    LEDs.roll(2);
    
    Gyro.calibrate(50, 0.02);
    Acc.calibrate(50, 0.02);
    
    // Start!
    GlobalTimer.start();
    Dutycycler.attach(&dutycycle, RATE);     // start to process all RATE seconds
    
    while(1) { 
        #ifdef PC_CONNECTED
            if (pc.readable())  // Get Serial input (polled because interrupts disturb I2C)
                pc.readcommand(&commandexecuter);
            //pc.printf("%f %f %f %f %f %f\r\n", IMU.angle[0], IMU.angle[1], IMU.angle[2], controller_value[0], controller_value[1], controller_value[2]); // For live plot in MATLAB of IMU
            //pc.printf("%f,%f,%f,%f,%f,%f\r\n", IMU.angle[0], IMU.angle[1], IMU.angle[2], controller_value[0], controller_value[1], controller_value[2]);
            #if 0 //pc.cls();
                pc.locate(20,0); // PC output
                pc.printf("dt:%3.5fs   dt_sensors:%3.5fs    Altitude:%6.1fm   ", dt, dt_read_sensors, Alt.CalcAltitude(Alt.Pressure));
                pc.locate(5,1);
                if(armed)
                    pc.printf("ARMED!!!!!!!!!!!!!");
                else
                    pc.printf("DIS_ARMED            ");
                pc.locate(5,3);
                pc.printf("Roll:%6.1f   Pitch:%6.1f   Yaw:%6.1f    ", IMU.angle[0], IMU.angle[1], IMU.angle[2]);
                pc.locate(5,4);
                pc.printf("q0:%6.1f   q1:%6.1f   q2:%6.1f   q3:%6.1f       ", IMU.q0, IMU.q1, IMU.q2, IMU.q3);
                pc.locate(5,5);
                pc.printf("Gyro.data: X:%6.1f  Y:%6.1f  Z:%6.1f", Gyro.data[0], Gyro.data[1], Gyro.data[2]);
                pc.locate(5,6);
                pc.printf("Acc.data:  X:%6.1f  Y:%6.1f  Z:%6.1f", Acc.data[0], Acc.data[1], Acc.data[2]);
                
                pc.locate(5,8);
                pc.printf("P :%6.1f   I :%6.1f   D :%6.1f    ", P, I, D);
                pc.locate(5,9);
                pc.printf("PY:%6.1f   IY:%6.1f   DY:%6.1f    ", PY, IY, DY);
                
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
                pc.printf("RC0: %4d   RC1: %4d   RC2: %4d   RC3: %4d   ", RC[0].read(), RC[1].read(), RC[2].read(), RC[3].read());
                
                pc.locate(10,21);
                pc.printf("Commandline: %s                                  ", pc.command);
            #endif
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