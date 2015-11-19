/*   X- Configuration
        m2   m3                      --           >
          \ /                      /    \       /
          / \                            V     |
        m1   m0                                 \
                                    ROLL       PITCH */
#include "mbed.h"
#include "LED.h"        // LEDs framework for blinking ;)
#include "PC.h"         // Serial Port via USB by Roland Elmiger for debugging with Terminal (driver needed: https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe)

#include "IMU_10DOF.h"  // Complete IMU class for 10DOF-Board (L3G4200D, ADXL345, HMC5883, BMP085)
#include "RC_Channel.h" // RemoteControl Channels with PPM
#include "PID.h"        // PID Library (slim, self written)
#include "Servo.h"      // Motor PPM using any DigitalOut Pin

#define PPM_FREQU       495     // Hz Frequency of PPM Signal for ESCs (maximum <500Hz)
#define INTEGRAL_MAX    300     // maximal output offset that can result from integrating errors
#define RC_SENSITIVITY  30      // maximal angle from horizontal that the PID is aming for
#define YAWSPEED        1.0     // maximal speed of yaw rotation in degree per Rate
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

#define SQRT2           0.7071067811865

bool  armed = false;                    // is for security (when false no motor rotates any more)
bool  debug = true;                     // shows if we want output for the computer
bool  level = false;                     // switches between self leveling and acro mode
bool  RC_present = false;               // shows if an RC is present
float P_R = 2.6, I_R = 0.3, D_R = 0;      // PID values for the rate controller
float P_A = 1.9, I_A = 0.2, D_A = 0;        // PID values for the angle controller      P_A = 1.865, I_A = 1.765, D_A = 0
float PY = 2.3, IY = 0, DY = 0;         // PID values for Yaw
float RC_angle[] = {0,0,0};             // Angle of the RC Sticks, to steer the QC
float Motor_speed[4] = {0,0,0,0};       // Mixed Motorspeeds, ready to send

Timer LoopTimer;
float Times[10] = {0,0,0,0,0,0,0,0,0,0};
float control_frequency = 800;//PPM_FREQU;         // frequency for the main loop in Hz
int counter = 0;
int divider = 20;

LED         LEDs;
//PC          pc(USBTX, USBRX, 115200);   // USB
PC          pc(p9, p10, 115200);       // Bluetooth PIN: 1234
IMU_10DOF   IMU(p5, p6, p7, p19, p28, p27);
RC_Channel  RC[] = {RC_Channel(p8,1), RC_Channel(p15,2), RC_Channel(p17,4), RC_Channel(p16,3), RC_Channel(p25,2), RC_Channel(p26,4), RC_Channel(p29,3)}; // no p19/p20 !
PID         Controller_Rate[] = {PID(P_R, I_R, D_R, INTEGRAL_MAX), PID(P_R, I_R, D_R, INTEGRAL_MAX), PID(PY, IY, DY, INTEGRAL_MAX)}; // 0:X:Roll 1:Y:Pitch 2:Z:Yaw
PID         Controller_Angle[] = {PID(P_A, I_A, D_A, INTEGRAL_MAX), PID(P_A, I_A, D_A, INTEGRAL_MAX), PID(0, 0, 0, INTEGRAL_MAX)};
Servo       ESC[] = {Servo(p21,PPM_FREQU), Servo(p22,PPM_FREQU), Servo(p23,PPM_FREQU), Servo(p24,PPM_FREQU)};   // use any DigitalOit Pin

extern "C" void mbed_reset();

void loop() {
    LoopTimer.start();
    
    // IMU
    IMU.readAngles();
    Times[1] = LoopTimer.read(); // 197us
    
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
    for(int i=0;i<3;i++)
        Controller_Angle[i].setPID(P_A,I_A,D_A);
    for(int i=0;i<2;i++)
        Controller_Rate[i].setPID(P_R,I_R,D_R); // give the new PID values to roll and pitch controller
    Controller_Rate[YAW].setPID(PY,IY,DY);
    Times[2] = LoopTimer.read(); // 7us
    
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
        RC_yaw_adding = -(RC[RUDDER].read()-500)*YAWSPEED/500;  // the yaw angle is integrated from stick input
    else
        RC_yaw_adding = 0;
    
    RC_angle[YAW] = RC_angle[YAW] + RC_yaw_adding < -180 ? RC_angle[YAW] + 360 + RC_yaw_adding : RC_angle[YAW] + RC_yaw_adding; // make shure it's in the cycle -180 to 180
    RC_angle[YAW] = RC_angle[YAW] + RC_yaw_adding > 180 ? RC_angle[YAW] - 360 + RC_yaw_adding : RC_angle[YAW] + RC_yaw_adding;
    Times[3] = LoopTimer.read(); // 6us

    // Controlling
    if (level) {
        for(int i=0;i<2;i++) { // LEVEL
            Controller_Angle[i].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
            if (counter % 16 == 0)
                Controller_Angle[i].compute(RC_angle[i], IMU.angle[i]); // give the controller the actual angles and get his advice to correct
            Controller_Rate[i].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
            Controller_Rate[i].compute(-Controller_Angle[i].Value, /*IMU.mpu2.data_gyro[i]*/IMU.mpu.Gyro[i]); // give the controller the actual gyro values and get his advice to correct
            //Controller_Rate[i].compute(-Controller_Angle[i].Value, (IMU.mpu2.data_gyro[i] + IMU.mpu.Gyro[i])/2 );
        }
    } else {
        for(int i=0;i<2;i++) { // ACRO
            Controller_Rate[i].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
            Controller_Rate[i].compute((RC[i].read()-500.0)*100.0/500.0, /*IMU.mpu2.data_gyro[i]*/IMU.mpu.Gyro[i]); // give the controller the actual gyro values and get his advice to correct
            //Controller_Rate[i].compute((RC[i].read()-500.0)*100.0/500.0, (IMU.mpu2.data_gyro[i] + IMU.mpu.Gyro[i])/2 );
        }
    }
    
    Controller_Rate[2].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
    if (RC[THROTTLE].read() > 20)
        Controller_Rate[2].compute(-(RC[2].read()-500.0)*100.0/500.0, IMU.mpu.Gyro[2]); // give the controller the actual gyro values and get his advice to correct
    else
        Controller_Rate[2].compute(0, IMU.mpu.Gyro[2]); // give the controller the actual gyro values and get his advice to correct
    
    float throttle = 100 + (RC[THROTTLE].read() * 500 / 1000);
    Times[4] = LoopTimer.read(); // 53us
    
    // Mixing
    Motor_speed[0] = throttle   +SQRT2*Controller_Rate[ROLL].Value  -SQRT2*Controller_Rate[PITCH].Value;  // X Configuration
    Motor_speed[1] = throttle   -SQRT2*Controller_Rate[ROLL].Value  -SQRT2*Controller_Rate[PITCH].Value;  // 
    Motor_speed[2] = throttle   -SQRT2*Controller_Rate[ROLL].Value  +SQRT2*Controller_Rate[PITCH].Value;  // 
    Motor_speed[3] = throttle   +SQRT2*Controller_Rate[ROLL].Value  +SQRT2*Controller_Rate[PITCH].Value;  // 
    
    Motor_speed[0] -= Controller_Rate[YAW].Value;
    Motor_speed[1] += Controller_Rate[YAW].Value;
    Motor_speed[2] -= Controller_Rate[YAW].Value;
    Motor_speed[3] += Controller_Rate[YAW].Value;
    Times[5] = LoopTimer.read(); // 17us
    
    if (armed) // for SECURITY!
    {       
            debug = false;
            // PITCH
            //ESC[0] = (int)Motor_speed[0]>50 ? (int)Motor_speed[0] : 50;
            //ESC[2] = (int)Motor_speed[2]>50 ? (int)Motor_speed[2] : 50;
            // ROLL
            //ESC[1] = (int)Motor_speed[1]>50 ? (int)Motor_speed[1] : 50;
            //ESC[3] = (int)Motor_speed[3]>50 ? (int)Motor_speed[3] : 50;
            for(int i=0;i<4;i++)   // Set new motorspeeds
                ESC[i] = (int)Motor_speed[i]>100 ? (int)Motor_speed[i] : 100;
            
    } else {
        for(int i=0;i<4;i++) // for security reason, set every motor to zero speed
            ESC[i] = 0;
        debug = true;
    }
    Times[6] = LoopTimer.read(); // 6us
    
    LEDs.rollnext();
    
    /*if(counter % divider == 0) {
        pc.printf("%.3f,%.3f,%.3f\r\n", IMU.mpu.Gyro[ROLL], IMU.mpu.Gyro[PITCH], IMU.mpu.Gyro[YAW]);
    }*/
    counter++;
    
    Times[7] = LoopTimer.read(); // 7us TOTAL 297us
    while(LoopTimer.read() < 1/control_frequency); // Kill the rest of the time TODO: make a better solution so we can do misc things with these cycles
    Times[8] = LoopTimer.read();
    LoopTimer.stop();
    LoopTimer.reset();
    
    
    if (debug) {
        pc.printf("$STATE,%d,%d,%.f,%.3f,%.3f\r\n", armed, level, control_frequency, IMU.dt*1e3, IMU.dt_sensors*1e6);
        //pc.printf("$RC,%d,%d,%d,%d,%d,%d,%d\r\n", RC[AILERON].read(), RC[ELEVATOR].read(), RC[RUDDER].read(), RC[THROTTLE].read(), RC[CHANNEL6].read(), RC[CHANNEL7].read(), RC[CHANNEL8].read());
        pc.printf("$GYRO,%.3f,%.3f,%.3f\r\n", IMU.mpu.Gyro[ROLL], IMU.mpu.Gyro[PITCH], IMU.mpu.Gyro[YAW]);
        pc.printf("$GYRO2,%.3f,%.3f,%.3f\r\n", IMU.mpu2.data_gyro[ROLL], IMU.mpu2.data_gyro[PITCH], IMU.mpu2.data_gyro[YAW]);
        //pc.printf("$ACC,%.3f,%.3f,%.3f\r\n", IMU.mpu.Acc[ROLL], IMU.mpu.Acc[PITCH], IMU.mpu.Acc[YAW]);
        pc.printf("$ANG,%.3f,%.3f,%.3f\r\n", IMU.angle[ROLL], IMU.angle[PITCH], IMU.angle[YAW]);
        //pc.printf("$RCANG,%.3f,%.3f,%.3f\r\n", RC_angle[ROLL], RC_angle[PITCH], RC_angle[YAW]);
        pc.printf("$CONTR,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", Controller_Rate[ROLL].Value, Controller_Rate[PITCH].Value, Controller_Rate[YAW].Value, P_R, I_R, D_R, PY);
        pc.printf("$CONTA,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", Controller_Angle[ROLL].Value, Controller_Angle[PITCH].Value, Controller_Angle[YAW].Value, P_A, I_A, D_A);
        pc.printf("$MOT,%d,%d,%d,%d\r\n", (int)Motor_speed[0], (int)Motor_speed[1], (int)Motor_speed[2], (int)Motor_speed[3]);
        /*pc.printf("$TIMES");
        for(int i = 1; i < 10; i++)
            pc.printf(",%.3f", (Times[i]-Times[i-1])*1e6);
        pc.printf("\r\n");*/
        wait(0.1);
    }
}

void executer() {
    char command = pc.getc();
    if (command == 'X')
        mbed_reset();
    if (command == '-')
        debug = !debug;
        
    if (command == ':')
        armed = true;
    if (command == ' ')
        armed = false;
        
    if (command == 'q')
        level = true;
    if (command == 'a')
        level = false;
        
    if (command == 'w')
        P_R += 0.1;
    if (command == 's')
        P_R -= 0.1;
        
    if (command == 'e')
        I_R += 0.1;
    if (command == 'd')
        I_R -= 0.1;
        
    if (command == 'x')
        D_R += 0.001;
    if (command == 'c')
        D_R -= 0.001;
        
    if (command == 'r')
        P_A += 0.1;
    if (command == 'f')
        P_A -= 0.1;
        
    if (command == 't')
        I_A += 0.1;
    if (command == 'g')
        I_A -= 0.1;
        
    if (command == 'z')
        PY += 0.1;
    if (command == 'h')
        PY -= 0.1;
        
    if (command == 'o') {
        control_frequency += 100;
        
    }
    if (command == 'l') {
        control_frequency -= 100;
        
    }
        
        
    pc.putc(command);
    LEDs.tilt(2);
}

int main() {
    pc.attach(&executer);
    while(1) {
        loop();
    }
}