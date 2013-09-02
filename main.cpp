#include "mbed.h"
#include "LED.h"        // LEDs framework for blinking ;)
#include "PC.h"         // Serial Port via USB by Roland Elmiger for debugging with Terminal (driver needed: https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe)

#include "IMU_10DOF.h"  // Complete IMU class for 10DOF-Board (L3G4200D, ADXL345, HMC5883, BMP085)
#include "RC_Channel.h" // RemoteControl Channels with PPM
#include "PID.h"        // PID Library (slim, self written)
#include "Servo_PWM.h"  // Motor PPM using PwmOut

#define PPM_FREQU       495     // Hz Frequency of PPM Signal for ESCs (maximum <500Hz)
#define INTEGRAL_MAX    300     // maximal output offset that can result from integrating errors
#define AILERON         0       // RC
#define ELEVATOR        1
#define RUDDER          2
#define THROTTLE        3
#define ROLL            0       // Axes
#define PITCH           1
#define YAW             2

bool  armed = false;                    // this variable is for security (when false no motor rotates any more)
float P = 1.0;                          // PID values
float I = 0;
float D = 2.0;
float controller_value = 0;             // The calculated answer form the Controller
float Motor_speed[4] = {0,0,0,0};       // Mixed Motorspeeds, ready to send

LED         LEDs;
PC          pc(USBTX, USBRX, 921600);   // USB
IMU_10DOF   IMU(p28, p27);
RC_Channel  RC[] = {RC_Channel(p5,1), RC_Channel(p6,2), RC_Channel(p8,4), RC_Channel(p7,3)};                                    // no p19/p20 !
PID         Controller(P, I, D, INTEGRAL_MAX); // X:Roll alone
Servo_PWM   ESC[] = {Servo_PWM(p21,PPM_FREQU), Servo_PWM(p22,PPM_FREQU), Servo_PWM(p23,PPM_FREQU), Servo_PWM(p24,PPM_FREQU)};   // p21 - p26 only because PWM needed!

void executer() {
    pc.putc(pc.getc());
    LEDs.tilt(2);
}

int main() {
    pc.attach(&executer);
    while(1) {
        // IMU
        IMU.readAngles();
        //IMU.readAltitude(); // reading altitude takes much more time than the angles -> don't do this in your fast loop
        //pc.printf("%.1f,%.1f,%.1f,%.1f'C,%.1fhPa,%.1fmaS,%.5fs,%.5fs\r\n", IMU.angle[0], IMU.angle[1], IMU.angle[2], IMU.temperature, IMU.pressure, IMU.altitude, IMU.dt, IMU.dt_sensors); // Output for Python
        
        // Arming / disarming
        if(RC[0].read() < 40) {
            armed = false;
        }
        if((RC[0].read() > 40 && RC[0].read() < 1100)) {
            armed = true;
        }
        
        // Controlling
        Controller.setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
        controller_value = Controller.compute(0, IMU.angle[0]); // give the controller the actual angle and get his advice to correct
        
        // Mixing
        if (armed) // for SECURITY!
        {       
                Motor_speed[0] = RC[0].read()   - controller_value;
                Motor_speed[1] = RC[0].read()   + controller_value;
                for(int i=0;i<4;i++)   // Set new motorspeeds
                    ESC[i] = (int)Motor_speed[i];
                
        } else {
            for(int i=0;i<4;i++) // for security reason, set every motor to zero speed
                ESC[i] = 0;
        }
        
        pc.printf("%d,%.3f,%.3f,%.3f,%.5fs,%.5fs,%4d,%4d,%4d,%4d\r\n", armed, IMU.angle[0], IMU.angle[1], IMU.angle[2], IMU.dt, IMU.dt_sensors, RC[0].read(), RC[1].read(), RC[2].read(), RC[3].read());
        //pc.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f\r\n", IMU.angle[0], IMU.angle[1], IMU.angle[2], IMU.Gyro.data[0], IMU.Gyro.data[1], IMU.Gyro.data[2], IMU.dt);
        
        //wait(0.01);

        LEDs.rollnext();
    }
}