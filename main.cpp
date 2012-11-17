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

//#define RAD2DEG         57.295779513082320876798154814105   // ratio between radians and degree (360/2Pi) //TODO not needed??
#define RATE            0.02                                // speed of the interrupt for Sensors and PID

#define P_VALUE         0.05                                // PID values
#define I_VALUE         100  
#define D_VALUE         0.015

//#define COMPASSCALIBRATE // decomment if you want to calibrate the Compass on start
#define PC_CONNECTED // decoment if you want to debug per USB and your PC

Timer GlobalTimer;                      // global time to calculate processing speed
Ticker Datagetter;                      // timecontrolled interrupt to get data form IMU and RC

// initialisation of hardware (see includes for more info)
LED         LEDs;
#ifdef PC_CONNECTED
    PC          pc(USBTX, USBRX, 115200);
#endif
L3G4200D    Gyro(p28, p27);
ADXL345     Acc(p28, p27);
HMC5883     Comp(p28, p27);
BMP085_old      Alt(p28, p27);
RC_Channel  RC[] = {p11, p12, p13, p14};    // noooo p19/p20 !
Servo_PWM   Motor[] = {p21, p22, p23, p24}; // p21 - p26 only !

// 0:X:Roll 1:Y:Pitch 2:Z:Yaw
PID     Controller[] = {PID(P_VALUE, I_VALUE, D_VALUE, RATE), PID(P_VALUE, I_VALUE, D_VALUE, RATE), PID(P_VALUE, I_VALUE, D_VALUE, RATE)}; // TODO: RATE != dt immer anpassen

// global varibles
bool            armed = false;          // this variable is for security
unsigned long   dt_get_data = 0; // TODO: dt namen
unsigned long   time_get_data = 0;
unsigned long   dt_read_sensors = 0;
unsigned long   time_read_sensors = 0;
float           angle[3] = {0,0,0}; // calculated values of the position [0: x,roll | 1: y,pitch | 2: z,yaw]
float           tempangle = 0; // temporärer winkel für yaw ohne kompass
float           Gyro_angle[3] ={0,0,0};
float           controller_value[] = {0,0,0};
float           motor_value[] = {0,0,0,0};

int motor_calc(int rc_value, float contr_value)
{
    return rc_value + contr_value > 0 ? rc_value + contr_value : 0; // nicht unter 0 sinken TODO: nicht Motor halten -> langsame Reaktion
}

void get_Data() // method which is called by the Ticker Datagetter every RATE seconds
{
    time_read_sensors = GlobalTimer.read_us();
    
    // read data from sensors // ATTENTION! the I2C option repeated true is important because otherwise interrupts while bus communications cause crashes
    Gyro.read();
    Acc.read(); // TODO: nicht jeder Sensor immer? höhe nicht so wichtig
    //Comp.read();
    //Alt.Update(); TODO braucht zu lange zum auslesen!
    
    dt_read_sensors = GlobalTimer.read_us() - time_read_sensors;
    
    // meassure dt
    dt_get_data = GlobalTimer.read_us() - time_get_data; // time in us since last loop
    time_get_data = GlobalTimer.read_us();      // set new time for next measurement
    
    Gyro_angle[0] += Gyro.data[0] *dt_get_data/15000000.0;
    Gyro_angle[1] += Gyro.data[1] *dt_get_data/15000000.0;
    Gyro_angle[2] += Gyro.data[2] *dt_get_data/15000000.0;
    
    // calculate angles for roll, pitch an yaw
    angle[0] += (Acc.angle[0] - angle[0])/50 + Gyro.data[0] *dt_get_data/15000000.0;
    angle[1] += (Acc.angle[1]+3 - angle[1])/50 + Gyro.data[1] *dt_get_data/15000000.0;// TODO Offset accelerometer einstellen
    //tempangle += (Comp.get_angle() - tempangle)/50 + Gyro.data[2] *dt_get_data/15000000.0;
    angle[2] = Gyro_angle[2]; // gyro only here
    
    // PID controlling
    if (!(RC[0].read() == -100)) {
        Controller[0].setSetPoint((int)((RC[0].read()-440)/440.0*90.0));
        Controller[1].setSetPoint(-(int)((RC[1].read()-430)/430.0*90.0));
    }
    for(int i=0;i<3;i++) {
        Controller[i].setProcessValue(angle[i]);
        controller_value[i] = Controller[i].compute() - 1000;
    }
    
    // Arming / disarming
    if(RC[2].read() < 20 && RC[3].read() > 850)
        armed = true;
    if(RC[2].read() < 30 && RC[3].read() < 30)
        armed = false;
    if(RC[3].read() < -10 || RC[2].read() < -10 || RC[1].read() < -10 || RC[0].read() < -10)
        armed = false;
    
    // calculate new motorspeeds
    if (armed) // for SECURITY!
    {
            // Pitch
            motor_value[0] = motor_calc(RC[2].read(), +controller_value[1]);
            motor_value[2] = motor_calc(RC[2].read(), -controller_value[1]);
            
            // Roll
            motor_value[1] = motor_calc(RC[2].read(), +controller_value[0]);
            motor_value[3] = motor_calc(RC[2].read(), -controller_value[0]);
            
            // Yaw
            //motor_value[0] -= controller_value[2];
            //motor_value[2] -= controller_value[2];
            
            //motor_value[1] += controller_value[2];
            //motor_value[3] += controller_value[2];
            
    } else {
        for(int i=0;i<4;i++)
            motor_value[i] = 0;
    }
    // Set new motorspeeds
    for(int i=0;i<4;i++)
        Motor[i] = motor_value[i];
}

int main() { // main programm only used for initialisation and debug output
    NVIC_SetPriority(TIMER3_IRQn, 1); // set priorty of tickers below hardware interrupts (standard priority is 0)
    
    for(int i=0;i<3;i++) {
        Controller[i].setInputLimits(-90.0, 90.0);
        Controller[i].setOutputLimits(0.0, 2000.0);
        Controller[i].setBias(1000);
        Controller[i].setMode(MANUAL_MODE);//AUTO_MODE);
        Controller[i].setSetPoint(0);
    }
    
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
    Datagetter.attach(&get_Data, RATE);     // start to get data all RATEms
    
    while(1) { 
        #ifdef PC_CONNECTED
            pc.locate(30,0); // PC output
            pc.printf("dt:%dms   dt_sensors:%dus    Altitude:%6.1fm   ", dt_get_data/1000, dt_read_sensors, Alt.CalcAltitude(Alt.Pressure));
            pc.locate(5,1);
            if(armed)
                pc.printf("ARMED!!!!!!!!!!!!!");
            else
                pc.printf("DIS_ARMED            ");
            pc.locate(5,3);
            pc.printf("Roll:%6.1f   Pitch:%6.1f   Yaw:%6.1f    ", angle[0], angle[1], angle[2]);
            pc.printf("\n\r   control Roll: %d   control Pitch: %d           ", (int)((RC[0].read()-440)/440.0*90.0), (int)((RC[1].read()-430)/430.0*90.0));
            
            pc.locate(5,5);
            pc.printf("Gyro.data: X:%6.1f  Y:%6.1f  Z:%6.1f", Gyro.data[0], Gyro.data[1], Gyro.data[2]);
            pc.locate(5,6);
            pc.printf("Gyro_angle: X:%6.1f  Y:%6.1f  Z:%6.1f", Gyro_angle[0], Gyro_angle[1], Gyro_angle[2]);
            
            pc.locate(5,8);
            pc.printf("Acc.data: X:%6d  Y:%6d  Z:%6d", Acc.data[0], Acc.data[1], Acc.data[2]); 
            pc.locate(5,9);
            pc.printf("Acc.angle: Roll:%6.1f Pitch:%6.1f Yaw:%6.1f    ", Acc.angle[0], Acc.angle[1], Acc.angle[2]);
            
            pc.locate(5,11);
            pc.printf("PID Result:");
            for(int i=0;i<3;i++)
                pc.printf("  %d: %6.1f", i, controller_value[i]);
            
            
            
            pc.locate(10,15);
            pc.printf("Debug_Yaw:  Comp:%6.1f tempangle:%6.1f  ", Comp.get_angle(), tempangle);
            pc.locate(10,16);
            pc.printf("Comp_data: %6.1f %6.1f %6.1f |||| %6.1f ", Comp.data[0], Comp.data[1], Comp.data[2], Comp.get_angle());
            pc.locate(10,17);
            //pc.printf("Comp_scale: %6.4f %6.4f %6.4f   ", Comp.scale[0], Comp.scale[1], Comp.scale[2]); no more accessible its private
            pc.locate(10,18);
            pc.printf("Comp_data: %6.1f %6.1f %6.1f |||| %6.1f ", Comp.data[0], Comp.data[1], Comp.data[2], Comp.get_angle());
            
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
        wait(0.01);
        if(armed){
            LEDs.rollnext();
        } else {
            LEDs.set(1);
            LEDs.set(2);
            LEDs.set(3);
            LEDs.set(4);
        }
        
    }
}
