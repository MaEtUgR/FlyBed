#include "mbed.h"
#include "PC.h"         // Serial Port via USB by Roland Elmiger for debugging with Terminal (driver needed: https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe)
#include "RC_Channel.h" // RemoteControl Channels with PPM
#include "Servo_PWM.h"  // Motor PPM using PwmOut
#include "MODI2C.h"

// Defines
#define PPM_FREQU       495                                 // Hz Frequency of PPM Signal for ESCs (maximum <500Hz)
#define RC_SENSITIVITY  0.3
#define YAW_SENSITIVITY 0.4
// RC
#define AILERON         0
#define ELEVATOR        1
#define RUDDER          2
#define THROTTLE        3
// Axes
#define ROLL            0
#define PITCH           1
#define YAW             2
// Motors
#define FRONT           0
#define RIGHT           1
#define BACK            2
#define LEFT            3

// Gyro
#define L3G4200D_I2C_ADDRESS    0xD0
#define L3G4200D_CTRL_REG1      0x20
#define L3G4200D_CTRL_REG2      0x21
#define L3G4200D_CTRL_REG3      0x22
#define L3G4200D_CTRL_REG4      0x23
#define L3G4200D_CTRL_REG5      0x24
#define L3G4200D_REFERENCE      0x25
#define L3G4200D_OUT_X_L        0x28

// Hardware connections
DigitalOut loopLED(LED1);
DigitalOut armedLED(LED2);
DigitalOut FBlowLED(LED3);
DigitalOut RLlowLED(LED4);
PC          pc(USBTX, USBRX, 38400);    // USB
I2C i2cinit(p28, p27);            // I2C-Bus for sensors-initialisation
MODI2C i2c(p28, p27);            // I2C-Bus for sensors-data
RC_Channel  RC[] = {RC_Channel(p5,1), RC_Channel(p6,2), RC_Channel(p8,4), RC_Channel(p7,3)};                                // no p19/p20 !
Servo_PWM   ESC[] = {Servo_PWM(p23,PPM_FREQU), Servo_PWM(p22,PPM_FREQU), Servo_PWM(p21,PPM_FREQU), Servo_PWM(p24,PPM_FREQU)};   // p21 - p26 only because PWM needed!

// Global variables
bool    armed = false;
float   ESC_value[4] = {0,0,0,0};
float   Error[3] = {0,0,0};
float   P[3] = {1.45,1.45,1.45};
// Timing
float   dt = 0;
float   time_for_dt = 0;
Timer   GlobalTimer;
// IMU
float   Gyro[3] = {0,0,0};
float   Gyro_sum[3] = {0,0,0};
float   Gyro_history[3][5] = {{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
int     Gyro_history_index = 0;
float   offset[3] = {0,0,0};

void writeRegister(char reg, char data)
{
    char buffer[2] = {reg, data};
    i2cinit.write(L3G4200D_I2C_ADDRESS, buffer, 2, true);
}

void readMultiRegister(char reg, char* output, int size)
{
    i2c.write (L3G4200D_I2C_ADDRESS, &reg, 1, true); // tell register address of the MSB get the sensor to do slave-transmit subaddress updating.
    i2c.read  (L3G4200D_I2C_ADDRESS, output, size, true); // tell it where to store the data read
}

void readGyro()
{
    char buffer[6];                                     // 8-Bit pieces of axis data
    int raw[3];
    readMultiRegister(L3G4200D_OUT_X_L | (1 << 7), buffer, 6); // read axis registers using I2C   // TODO: why?!   | (1 << 7)
    raw[0] = (short) (buffer[1] << 8 | buffer[0]);     // join 8-Bit pieces to 16-bit short integers
    raw[1] = (short) (buffer[3] << 8 | buffer[2]);
    raw[2] = (short) (buffer[5] << 8 | buffer[4]);
    for(int i=0;i<3;i++)
        Gyro[i] = raw[i] - offset[i];// * 0.07;
}

void setup() {
    // init screen -------------------------------------------------------------------------------------------------
    pc.locate(10,5);
    pc.printf("Flybed Light");
    pc.locate(10,7);
    pc.printf("Init...");
    
    // init Gyro ---------------------------------------------------------------------------------------------------
    i2cinit.frequency(400000);
    i2c.frequency(400000);
    
    writeRegister(L3G4200D_CTRL_REG1, 0x8F);            // starts Gyro measurement
    //writeRegister(L3G4200D_CTRL_REG2, 0x00);            // highpass filter disabled
    //writeRegister(L3G4200D_CTRL_REG3, 0x00);
    writeRegister(L3G4200D_CTRL_REG4, 0x20);            // sets acuracy to 2000 dps (degree per second)
    writeRegister(L3G4200D_CTRL_REG5, 0x02);
    //writeRegister(L3G4200D_REFERENCE, 0x00);
    
    // calibrate Gyro ----------------------------------------------------------------------------------------------
    int calib[3] = {0,0,0};                           // temporary array for the sum of calibration measurement
    const int times = 50;  
    for (int i = 0; i < times; i++) {                   // read 'times' times the data in a very short time
        readGyro();
        for (int j = 0; j < 3; j++)
            calib[j] += Gyro[j];
        wait(0.05);
    }
    for (int i = 0; i < 3; i++)
        offset[i] = (float)calib[i]/(float)times;                     // take the average of the calibration measurements
    
    GlobalTimer.start();                            // Start Timemeasurement for first loop
    
    pc.locate(10,7);
    pc.printf("READY!!");
}

void loop() {
    // meassure dt
    dt = GlobalTimer.read() - time_for_dt; // time in us since last loop
    time_for_dt = GlobalTimer.read();      // set new time for next measurement
    
    // Arming / disarming
    if(RC[THROTTLE].read() < 20 && RC[RUDDER].read() < 30) {
        armed = true;
    }
    if((RC[THROTTLE].read() < 30 && RC[RUDDER].read() > 920) || RC[2].read() < -10 || RC[3].read() < -10 || RC[1].read() < -10 || RC[0].read() < -10) {
        armed = false;
    }
    
    // get sensor data ----------------------------------------------------------------------------------------------------------
    readGyro();
    for (int i = 0; i < 3; i++) {
        Gyro[i] *= 0.07;                            // make result degree per second
        
        // fill ringbuffer
        /*
        Gyro_history[i][Gyro_history_index] = Gyro[i]; // save newest value to ringbuffer
        if (Gyro_history_index < 5-1)
            Gyro_history_index++;
        else
            Gyro_history_index = 0;
        
        // calculate average of ringbuffer
        float sum = 0;
        for (int j = 0; j < 5; j++) {
            sum += Gyro_history[i][j];
        }
        Gyro[i] = sum/5;
        */
        Gyro_sum[i] += Gyro[i]*dt;      // integrate speed to get angle
    }
    
    // calculate ESC -------------------------------------------------------------------------------------------------------------
    
    if (RC[ROLL].read() != -100) {  // only count RC when it's available
        Error[ROLL] = ((RC[ROLL].read() - 500)*RC_SENSITIVITY  -  Gyro[ROLL]) * P[ROLL];
        Error[PITCH] = ((RC[PITCH].read() - 500)*RC_SENSITIVITY  -  Gyro[PITCH]) * P[PITCH];
        Error[YAW] = ((RC[YAW].read() - 500)*YAW_SENSITIVITY  -  Gyro[YAW]) * P[YAW];
    } else
        for (int i = 0; i < 3; i++)
            Error[i] = (-  Gyro[i]) * P[i];
    
    for (int i = 0; i < 4; i++)
        ESC_value[i] = RC[THROTTLE].read();
    
    ESC_value[RIGHT] -= Error[ROLL];
    ESC_value[LEFT] += Error[ROLL];
    
    ESC_value[FRONT] -= Error[PITCH];
    ESC_value[BACK] += Error[PITCH];
    
    ESC_value[FRONT] -= Error[YAW];
    ESC_value[BACK] -= Error[YAW];
    ESC_value[RIGHT] += Error[YAW];
    ESC_value[LEFT] += Error[YAW];
    
    
    const int minimum_throttle = 140;
    FBlowLED = ESC_value[FRONT] < minimum_throttle || ESC_value[BACK] < minimum_throttle;
    RLlowLED = ESC_value[RIGHT] < minimum_throttle || ESC_value[LEFT] < minimum_throttle;
    for (int i = 0; i < 4; i++) {    // make shure there are no too low or too high ESC_values
            if (ESC_value[i] < minimum_throttle) {
                /*switch (i){
                    case FRONT: ESC_value[BACK] -= ESC_value[i]+minimum_throttle; break;
                    case BACK: ESC_value[FRONT] -= ESC_value[i]+minimum_throttle; break;
                    case LEFT: ESC_value[RIGHT] -= ESC_value[i]+minimum_throttle; break;
                    case RIGHT: ESC_value[LEFT] -= ESC_value[i]+minimum_throttle; break; //default: 
                }*/
                
                ESC_value[i] = minimum_throttle;
            }
            if (ESC_value[i] > 1000) {
                /*switch (i){
                    case FRONT: ESC_value[BACK] -= ESC_value[i] - 1000; break;
                    case BACK: ESC_value[FRONT] -= ESC_value[i] - 1000; break;
                    case LEFT: ESC_value[RIGHT] -= ESC_value[i] - 1000; break;
                    case RIGHT: ESC_value[LEFT] -= ESC_value[i] - 1000; break; //default: 
                }*/
                ESC_value[i] = 1000;
            }
    }
    
    // set ESC -------------------------------------------------------------------------------------------------------------------
    if (armed) {
        ESC[FRONT] = (int)ESC_value[FRONT];
        ESC[BACK] = (int)ESC_value[BACK];
        ESC[LEFT] = (int)ESC_value[LEFT];
        ESC[RIGHT] = (int)ESC_value[RIGHT];
        /*for (int i = 0; i < 4; i++)
            ESC[i] = (int)ESC_value[i];*/
    } else
        for (int i = 0; i < 4; i++)
            ESC[i] = 0;
    
    // display --------------------------------------------------------------------------------------------------------------------
    //pc.printf("%6.3f %6.3f %6.3f\n\r", Gyro_sum[ROLL], Gyro_sum[PITCH], Gyro_sum[YAW]);
    wait(0.01);
    #if 0 
        pc.locate(10,7);
        pc.printf("Gyro: X:%6.3f  Y:%6.3f  Z:%6.3f                     ", Gyro[ROLL], Gyro[PITCH], Gyro[YAW]);
        pc.locate(10,8);
        pc.printf("Gyro_sum: X:%6.1f  Y:%6.1f  Z:%6.1f       dt: %1.4f                ", Gyro_sum[ROLL], Gyro_sum[PITCH], Gyro_sum[YAW], dt);
        pc.locate(10,10);
        pc.printf("RC Aileron: %4d   Elevator: %4d   Rudder: %4d   Throttle: %4d     ", RC[AILERON].read(), RC[ELEVATOR].read(), RC[RUDDER].read(), RC[THROTTLE].read());
        pc.locate(10,17);
        pc.printf("Error: Roll:%6.1f  Pitch:%6.1f  Yaw:%6.1f     ", Error[ROLL], Error[PITCH], Error[YAW]);
        pc.locate(10,19);
        pc.printf("ESC: Front:%6.1f  Right:%6.1f  Back:%6.1f  Left:%6.1f    ", ESC_value[FRONT], ESC_value[RIGHT], ESC_value[BACK], ESC_value[LEFT]);
    #endif
    
    // LED
    armedLED = armed;
    loopLED = 1;
    //wait(0.0005);
    loopLED = 0;
    //wait(0.001);
}

int main() {
    setup();
    while(1) {
        loop();
    }
}