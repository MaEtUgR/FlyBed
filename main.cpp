#include "mbed.h"
#include "LED.h"        // LEDs framework for blinking ;)
#include "PC.h"         // Serial Port via USB by Roland Elmiger for debugging with Terminal (driver needed: https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe)

#include "IMU_10DOF.h"  // Complete IMU class for 10DOF-Board (L3G4200D, ADXL345, HMC5883, BMP085)
#include "RC_Channel.h" // RemoteControl Channels with PPM
#include "PID.h"        // PID Library (slim, self written)
#include "Servo_PWM.h"  // Motor PPM using PwmOut

#define PPM_FREQU       495     // Hz Frequency of PPM Signal for ESCs (maximum <500Hz)
#define INTEGRAL_MAX    300     // maximal output offset that can result from integrating errors
#define RC_SENSITIVITY  30      // maximal angle from horizontal that the PID is aming for
#define YAWSPEED        0.2     // maximal speed of yaw rotation in degree per Rate
#define AILERON         0       // RC
#define ELEVATOR        1
#define RUDDER          2
#define THROTTLE        3
#define CHANNEL8        4
#define CHANNEL7        5
#define CHANNEL6        6
#define ROLL            0       // Axes
#define PITCH           1
#define YAW             2

bool  armed = false;                    // this variable is for security (when false no motor rotates any more)
bool  RC_present = false;               // this variable shows if an RC is present
float P = 15, I = 8, D = 2.73;          // PID values
float PY = 5.37, IY = 0, DY = 3;           // PID values for Yaw
float RC_angle[] = {0,0,0};             // Angle of the RC Sticks, to steer the QC
float controller_value = 0;             // The calculated answer form the Controller
float Motor_speed[4] = {0,0,0,0};       // Mixed Motorspeeds, ready to send

LED         LEDs;
PC          pc(USBTX, USBRX, 921600);   // USB
//PC          pc(p9, p10, 115200);       // Bluetooth
IMU_10DOF   IMU(p28, p27);
RC_Channel  RC[] = {RC_Channel(p8,1), RC_Channel(p7,2), RC_Channel(p5,4), RC_Channel(p6,3), RC_Channel(p15,2), RC_Channel(p16,4), RC_Channel(p17,3)};                                    // no p19/p20 !
PID         Controller[] = {PID(P, I, D, INTEGRAL_MAX), PID(P, I, D, INTEGRAL_MAX), PID(PY, IY, DY, INTEGRAL_MAX)}; // 0:X:Roll 1:Y:Pitch 2:Z:Yaw
Servo_PWM   ESC[] = {Servo_PWM(p21,PPM_FREQU), Servo_PWM(p22,PPM_FREQU), Servo_PWM(p23,PPM_FREQU), Servo_PWM(p24,PPM_FREQU)};   // p21 - p26 only because PWM needed!

extern "C" void mbed_reset();

void executer() {
    char command = pc.getc();
    if (command == 'X')
        mbed_reset();
    if (command == 'A') {
        IMU.Acc.calibrate(100,0.05);
        pc.printf("\r\n***A***%.3f,%.3f,%.3f***\r\n", IMU.Acc.offset[ROLL], IMU.Acc.offset[PITCH], IMU.Acc.offset[YAW]);
        wait(10);
    }
    if (command == 'C') {
        IMU.Comp.calibrate(60);
        pc.printf("\r\n***C***%.3f,%.3f,%.3f***\r\n", IMU.Comp.offset[ROLL], IMU.Comp.offset[PITCH], IMU.Comp.offset[YAW]);
        wait(20);
    }
        
    pc.putc(command);
    LEDs.tilt(2);
}

int main() {
    pc.attach(&executer);
    while(1) {
        // IMU
        IMU.readAngles();
        //IMU.readAltitude(); // TODO: reading altitude takes much more time than the angles -> don't do this in your fast loop, Ticker?
        //pc.printf("%.1f,%.1f,%.1f,%.1f'C,%.1fhPa,%.1fmaS,%.5fs,%.5fs\r\n", IMU.angle[0], IMU.angle[1], IMU.angle[2], IMU.temperature, IMU.pressure, IMU.altitude, IMU.dt, IMU.dt_sensors); // Output for Python
        
        // Arming / disarming
        RC_present = !(RC[AILERON].read() == -100 || RC[ELEVATOR].read() == -100 || RC[RUDDER].read() == -100 || RC[THROTTLE].read() == -100); // TODO: Failsafe
        if(RC[THROTTLE].read() < 20 && RC[RUDDER].read() > 850) {
            armed = true;
            RC_angle[YAW] = IMU.angle[YAW];
        }
        if((RC[THROTTLE].read() < 30 && RC[RUDDER].read() < 30) || !RC_present) {
            armed = false;
        }
        
        // Setting PID Values from auxiliary RC channels
        if (RC[CHANNEL8].read() > 0 && RC[CHANNEL8].read() < 1000)
            P = ((float)RC[CHANNEL8].read()) * 20  / 1000;
        if (RC[CHANNEL7].read() > 0 && RC[CHANNEL7].read() < 1000)
            D = ((float)RC[CHANNEL7].read()) * 6  / 1000;
        for(int i=0;i<2;i++)
            Controller[i].setPID(P,I,D); // give the new PID values to roll and pitch controller
        Controller[YAW].setPID(PY,IY,DY);
        
        // RC Angle ROLL-PITCH-Part
        for(int i=0;i<2;i++) {    // calculate new angle we want the QC to have
            if (RC_present)
                RC_angle[i] = (RC[i].read()-500)*RC_SENSITIVITY/500.0;
            else
                RC_angle[i] = 0;
        }

        // RC Angle YAW-Part
        float   RC_yaw_adding;                  // temporary variable to take the desired yaw adjustment
        if (RC_present && RC[THROTTLE].read() > 20)
            RC_yaw_adding = -(RC[RUDDER].read()-500)*YAWSPEED/500;
        else
            RC_yaw_adding = 0;
            
        while(RC_angle[YAW] + RC_yaw_adding < -180 || RC_angle[YAW] + RC_yaw_adding > 180) { // make shure it's in the cycle -180 to 180
            if(RC_angle[YAW] + RC_yaw_adding < -180)
                RC_yaw_adding += 360;
            if(RC_angle[YAW] + RC_yaw_adding > 180)
                RC_yaw_adding -= 360;
        }
        RC_angle[YAW] += RC_yaw_adding;  // the yaw angle is integrated from stick input

        // Controlling
        for(int i=0;i<2;i++) {
            Controller[i].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
            Controller[i].compute(RC_angle[i], IMU.angle[i], IMU.Gyro.data[i]); // give the controller the actual gyro values for D and angle for P,I and get his advice to correct
        }
        Controller[YAW].setIntegrate(armed); // same for YAW
        if (abs(RC_angle[YAW] - IMU.angle[YAW]) > 180)  // for YAW a special calculation because of range -180 to 180
             if (RC_angle[YAW] > IMU.angle[YAW])
                Controller[YAW].compute(RC_angle[YAW] - 360, IMU.angle[YAW], IMU.Gyro.data[YAW]);
             else
                Controller[YAW].compute(RC_angle[YAW] + 360, IMU.angle[YAW], IMU.Gyro.data[YAW]);
        else
            Controller[YAW].compute(RC_angle[YAW], IMU.angle[YAW], IMU.Gyro.data[YAW]);
        
        // Mixing
        if (armed) // for SECURITY!
        {       
                Motor_speed[0] = RC[THROTTLE].read()   + Controller[PITCH].Value;
                Motor_speed[2] = RC[THROTTLE].read()   - Controller[PITCH].Value;
                Motor_speed[3] = RC[THROTTLE].read()   + Controller[ROLL].Value;
                Motor_speed[1] = RC[THROTTLE].read()   - Controller[ROLL].Value;
                
                Motor_speed[0] -= Controller[YAW].Value;
                Motor_speed[2] -= Controller[YAW].Value;
                Motor_speed[3] += Controller[YAW].Value;
                Motor_speed[1] += Controller[YAW].Value;
                
                
                for(int i=0;i<4;i++)   // Set new motorspeeds
                    ESC[i] = (int)Motor_speed[i];
                
        } else {
            for(int i=0;i<4;i++) // for security reason, set every motor to zero speed
                ESC[i] = 0;
        }
        
        //pc.printf("%d,%.3f,%.3f,%.3f,%.5fs,%.5fs,%4d,%4d,%4d,%4d\r\n", armed, IMU.angle[0], IMU.angle[1], IMU.angle[2], IMU.dt, IMU.dt_sensors, RC[0].read(), RC[1].read(), RC[2].read(), RC[3].read());
        pc.printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", armed, P, PY, D, IMU.angle[PITCH], controller_value, RC_angle[YAW], IMU.dt);
        //pc.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f\r\n", IMU.angle[0], IMU.angle[1], IMU.angle[2], IMU.Gyro.data[0], IMU.Gyro.data[1], IMU.Gyro.data[2], IMU.dt);
        
        //wait(0.01);

        LEDs.rollnext();
    }
}