/*   X- Configuration           +-Configuration
        m0   m3                        m1               --           >
          \ /                          |              /    \       /
          / \                     m2-------m0        V            |
        m1   m2                        |                           \
                                       m3             PITCH      ROLL*/
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

//#define CONSTRAIN(VAL,LIMIT) ((VAL)<(-LIMIT)?(-LIMIT):((VAL)>(LIMIT)?(LIMIT):(VAL)))

bool  armed = false;                    // is for security (when false no motor rotates any more)
bool  debug = true;                    // shows if we want output for the computer
bool  RC_present = false;               // shows if an RC is present
float P_R = 0, I_R = 0, D_R = 0;
float P_A = 1.865, I_A = 1.765, D_A = 0;
//float P = 13.16, I = 8, D = 2.73;          // PID values
float PY = 3.2, IY = 0, DY = 0;
//float PY = 5.37, IY = 0, DY = 3;           // PID values for Yaw
float RC_angle[] = {0,0,0};             // Angle of the RC Sticks, to steer the QC
float Motor_speed[4] = {0,0,0,0};       // Mixed Motorspeeds, ready to send
//float * command_pointer = &D;           // TODO: pointer to varible that's going to be changed by UART command

/*float max[3] = {-10000,-10000,-10000};
float min[3] = {10000,10000,10000};*/

LED         LEDs;
PC          pc(USBTX, USBRX, 921600);   // USB
//PC          pc(p9, p10, 115200);       // Bluetooth
IMU_10DOF   IMU(p28, p27);
RC_Channel  RC[] = {RC_Channel(p8,1), RC_Channel(p7,2), RC_Channel(p5,4), RC_Channel(p6,3), RC_Channel(p15,2), RC_Channel(p16,4), RC_Channel(p17,3)};                                    // no p19/p20 !
PID         Controller_Rate[] = {PID(P_R, I_R, D_R, INTEGRAL_MAX), PID(P_R, I_R, D_R, INTEGRAL_MAX), PID(PY, IY, DY, INTEGRAL_MAX)}; // 0:X:Roll 1:Y:Pitch 2:Z:Yaw
PID         Controller_Angle[] = {PID(P_A, I_A, D_A, INTEGRAL_MAX), PID(P_A, I_A, D_A, INTEGRAL_MAX), PID(0, 0, 0, INTEGRAL_MAX)};
Servo       ESC[] = {Servo(p21,PPM_FREQU), Servo(p22,PPM_FREQU), Servo(p23,PPM_FREQU), Servo(p24,PPM_FREQU)};   // use any DigitalOit Pin

extern "C" void mbed_reset();

void executer() {
    char command = pc.getc();
    if (command == 'X')
        mbed_reset();
    if (command == '-')
        debug = !debug;
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
            P_R = 0 + (((float)RC[CHANNEL8].read()) * 3  / 1000);
        /*if (RC[CHANNEL7].read() > 0 && RC[CHANNEL7].read() < 1000)
            I_R = 0 + (((float)RC[CHANNEL7].read()) * 12  / 1000);*/
        for(int i=0;i<3;i++)
            Controller_Angle[i].setPID(P_A,I_A,D_A);
        for(int i=0;i<2;i++)
            Controller_Rate[i].setPID(P_R,I_R,D_R); // give the new PID values to roll and pitch controller
        Controller_Rate[YAW].setPID(PY,IY,DY);
        
        // RC Angle ROLL-PITCH-Part
        for(int i=0;i<2;i++) {    // calculate new angle we want the QC to have
            if (RC_present)
                RC_angle[i] = (RC[i].read()-500)*RC_SENSITIVITY/500.0;
            else
                RC_angle[i] = 0;
        }

        // RC Angle YAW-Part
        if (RC_present && RC[THROTTLE].read() > 20)
            RC_angle[YAW] -= (RC[RUDDER].read()-500)*YAWSPEED/500;
            
        float   RC_yaw_adding;                  // temporary variable to take the desired yaw adjustment
        if (RC_present && RC[THROTTLE].read() > 20)
            RC_yaw_adding = -(RC[RUDDER].read()-500)*YAWSPEED/500;
        else
            RC_yaw_adding = 0;
        
        RC_angle[YAW] = RC_angle[YAW] + RC_yaw_adding < -180 ? RC_angle[YAW] + 360 + RC_yaw_adding : RC_angle[YAW] + RC_yaw_adding;
        RC_angle[YAW] = RC_angle[YAW] + RC_yaw_adding > 180 ? RC_angle[YAW] - 360 + RC_yaw_adding : RC_angle[YAW] + RC_yaw_adding;
        
        /*float   RC_yaw_adding;                  // temporary variable to take the desired yaw adjustment
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
        RC_angle[YAW] += RC_yaw_adding;  // the yaw angle is integrated from stick input*/

        // Controlling
        for(int i=0;i<2;i++) {
            Controller_Rate[i].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
            Controller_Rate[i].compute((RC[i].read()-500.0)*100.0/500.0, IMU.Sensor.data_gyro[i]); // give the controller the actual gyro values and get his advice to correct
        }
        Controller_Rate[2].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
        if (RC[THROTTLE].read() > 20)
            Controller_Rate[2].compute(-(RC[2].read()-500.0)*100.0/500.0, IMU.Sensor.data_gyro[2]); // give the controller the actual gyro values and get his advice to correct
        else
            Controller_Rate[2].compute(0, IMU.Sensor.data_gyro[2]); // give the controller the actual gyro values and get his advice to correct
        /*for(int i=0;i<3;i++) {
            Controller_Angle[i].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
            Controller_Angle[i].compute(RC_angle[i], IMU.angle[i]); // give the controller the actual gyro values and get his advice to correct
            Controller_Rate[i].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
            Controller_Rate[i].compute(-Controller_Angle[i].Value, IMU.Sensor.data_gyro[i]); // give the controller the actual gyro values and get his advice to correct
        }*/
        
        // OLD Controlling
        /*for(int i=0;i<2;i++) {
            Controller[i].setIntegrate(armed); // only integrate in controller when armed, so the value is not totally odd from not flying
            Controller[i].compute(RC_angle[i], IMU.angle[i], IMU.Sensor.data_gyro[i]); // give the controller the actual gyro values for D and angle for P,I and get his advice to correct
        }
        Controller[YAW].setIntegrate(armed); // same for YAW
        if (abs(RC_angle[YAW] - IMU.angle[YAW]) > 180)  // for YAW a special calculation because of range -180 to 180
             if (RC_angle[YAW] > IMU.angle[YAW])
                Controller[YAW].compute(RC_angle[YAW] - 360, IMU.angle[YAW], IMU.Sensor.data_gyro[YAW]);
             else
                Controller[YAW].compute(RC_angle[YAW] + 360, IMU.angle[YAW], IMU.Sensor.data_gyro[YAW]);
        else
            Controller[YAW].compute(RC_angle[YAW], IMU.angle[YAW], IMU.Sensor.data_gyro[YAW]);*/
        
        // Mixing
        /*Motor_speed[2] = RC[THROTTLE].read()   + Controller_Rate[PITCH].Value;  // PITCH in direction     + Configuration
        Motor_speed[0] = RC[THROTTLE].read()   - Controller_Rate[PITCH].Value;  // PITCH against direction
        Motor_speed[1] = RC[THROTTLE].read()   + Controller_Rate[ROLL].Value;   // ROLL in direction
        Motor_speed[3] = RC[THROTTLE].read()   - Controller_Rate[ROLL].Value;   // ROLL against direction*/
        
        Motor_speed[0] = RC[THROTTLE].read()   +SQRT2*Controller_Rate[PITCH].Value +SQRT2*Controller_Rate[ROLL].Value;  // PITCH in direction       X Configuration
        Motor_speed[1] = RC[THROTTLE].read()   +SQRT2*Controller_Rate[PITCH].Value -SQRT2*Controller_Rate[ROLL].Value;  // PITCH against direction
        Motor_speed[2] = RC[THROTTLE].read()   -SQRT2*Controller_Rate[PITCH].Value -SQRT2*Controller_Rate[ROLL].Value;  // ROLL in direction
        Motor_speed[3] = RC[THROTTLE].read()   -SQRT2*Controller_Rate[PITCH].Value +SQRT2*Controller_Rate[ROLL].Value;  // ROLL against direction
        
        Motor_speed[0] -= Controller_Rate[YAW].Value;
        Motor_speed[2] -= Controller_Rate[YAW].Value;
        Motor_speed[3] += Controller_Rate[YAW].Value;
        Motor_speed[1] += Controller_Rate[YAW].Value;
        
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
                    ESC[i] = (int)Motor_speed[i]>50 ? (int)Motor_speed[i] : 50;
                
        } else {
            for(int i=0;i<4;i++) // for security reason, set every motor to zero speed
                ESC[i] = 0;
        }
        
        if (debug) {
        //pc.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", IMU.Acc.data[0], IMU.Acc.data[1], IMU.Acc.data[2], D, IMU.angle[PITCH], Controller[PITCH].Value, RC_angle[YAW], IMU.dt);
        //MAIN OUTPUT pc.printf("%d,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.2f,%.2f\r\n", armed, IMU.angle[ROLL], IMU.angle[PITCH], IMU.angle[YAW], Controller[ROLL].Value, Controller[PITCH].Value, Controller[YAW].Value, P, D); // RC[0].read(), RC[1].read(), RC[2].read(), RC[3].read()
        //pc.printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", armed, P, PY, D, IMU.angle[PITCH], Controller[PITCH].Value, RC_angle[YAW], IMU.dt);
        //pc.printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", armed, P, PY, D, IMU.angle[PITCH], Controller[PITCH].Value, RC_angle[YAW], IMU.dt);
        //pc.printf("%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%.5f\r\n", IMU.angle[0], IMU.angle[1], IMU.angle[2], IMU.Sensor.data_gyro[0], IMU.Sensor.data_gyro[1], IMU.Sensor.data_gyro[2], IMU.dt);
            pc.printf("$STATE,%d,%.3f\r\n", armed, IMU.dt);
            pc.printf("$RC,%d,%d,%d,%d,%d,%d,%d\r\n", RC[AILERON].read(), RC[ELEVATOR].read(), RC[RUDDER].read(), RC[THROTTLE].read(), RC[CHANNEL6].read(), RC[CHANNEL7].read(), RC[CHANNEL8].read());
            pc.printf("$GYRO,%.3f,%.3f,%.3f\r\n", IMU.Sensor.data_gyro[ROLL], IMU.Sensor.data_gyro[PITCH], IMU.Sensor.data_gyro[YAW]);
            pc.printf("$ACC,%.3f,%.3f,%.3f\r\n", IMU.Sensor.data_acc[ROLL], IMU.Sensor.data_acc[PITCH], IMU.Sensor.data_acc[YAW]);
            pc.printf("$ANG,%.3f,%.3f,%.3f\r\n", IMU.angle[ROLL], IMU.angle[PITCH], IMU.angle[YAW]);
            pc.printf("$RCANG,%.3f,%.3f,%.3f\r\n", RC_angle[ROLL], RC_angle[PITCH], RC_angle[YAW]);
            pc.printf("$CONT,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", Controller_Rate[ROLL].Value, Controller_Rate[PITCH].Value, Controller_Rate[YAW].Value, P_R, I_R, D_R);
            pc.printf("$MOT,%d,%d,%d,%d\r\n", (int)Motor_speed[0], (int)Motor_speed[1], (int)Motor_speed[2], (int)Motor_speed[3]);
            /*for (int i=0;i<3;i++) {
                min[i] = IMU.Sensor.data_gyro[i]<min[i] ? IMU.Sensor.data_gyro[i] : min[i];
                max[i] = IMU.Sensor.data_gyro[i]>max[i] ? IMU.Sensor.data_gyro[i] : max[i];
            }*/
            //pc.printf("%.5f\r\n", IMU.dt);
            //pc.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", IMU.Sensor.raw_gyro[ROLL], IMU.Sensor.raw_gyro[PITCH], IMU.Sensor.raw_gyro[YAW], min[0], min[1], min[2], max[0], max[1], max[2]);
            //pc.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", IMU.Sensor.data_gyro[ROLL], IMU.Sensor.data_gyro[PITCH], IMU.Sensor.data_gyro[YAW], min[0], min[1], min[2], max[0], max[1], max[2]);
            //pc.printf("%.3f,%.3f,%.3f\r\n", IMU.Sensor.data_gyro[ROLL], IMU.Sensor.data_gyro[PITCH], IMU.Sensor.data_gyro[YAW]);
            
            // SimPlot output
            /*int16_t sendvalue[4];   //Buffer to hold the packet, note it is 16bit data type
            sendvalue[0] = (int16_t) IMU.Sensor.data_gyro[ROLL];    //Channel 1 data. 16bit signed integer
            sendvalue[1] = (int16_t) IMU.Sensor.data_gyro[PITCH];    //Channel 2 data. 16bit signed integer
            sendvalue[2] = (int16_t) IMU.Sensor.data_gyro[YAW];    //Channel 3 data. 16bit signed integer
            sendvalue[3] = (int16_t) 0;    //Channel 4 data. 16bit signed integer

            pc.putc(0xAB); // header
            pc.putc(0xCD);
            pc.putc(0x08); // size LSB
            pc.putc(0x00); // size MSB
            for(int i=0; i<4; i++) {
                pc.putc((char)sendvalue[i]);        // LSB
                pc.putc((char)(sendvalue[i] >> 8)); // MSB
            }*/

            wait(0.04);
        }

        LEDs.rollnext();
    }
}