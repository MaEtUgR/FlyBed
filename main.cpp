#include "mbed.h"       // Standard Library
#include "LED.h"        // LEDs framework for blinking ;)
#include "L3G4200D.h"   // Gyro (Gyroscope)
#include "ADXL345.h"    // Acc (Accelerometer)
#include "HMC5883.h"    // Comp (Compass)
#include "BMP085.h"     // Alt (Altitude sensor)
#include "RC_Channel.h" // RemoteControl Chnnels with PPM
#include "Servo.h"      // Motor PPM
#include "PC.h"         // Serial Port via USB for debugging

Timer GlobalTimer;

// initialisation of hardware
LED         LEDs;
L3G4200D    Gyro(p28, p27);
ADXL345     Acc(p28, p27);
HMC5883     Comp(p28, p27, GlobalTimer);
BMP085      Alt(p28, p27);
//Servo       Motor_front(p18);
Servo       Motor_left(p17);
Servo       Motor_right(p19);
Servo       Motor_back(p20);

PC          pc(USBTX, USBRX);        //Achtung: Treiber auf PC fuer Serial-mbed installieren. hier herunterladen https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe

#define PI             3.1415926535897932384626433832795
#define Rad2Deg        57.295779513082320876798154814105

int main() {
    pc.baud(57600);
    pc.cls();
    // init screen
    pc.locate(10,5);
    pc.printf("Flybed v0.2");
    LEDs.roll(2);
    
    // variables for loop
    float           Gyro_data[3];
    int             Acc_data[3];
    unsigned long   dt = 0;
    unsigned long   time_loop = 0;
    float angle[3] = {0,0,0};
    float Acc_angle[2] = {0,0};
    float Comp_angle = 0;
    float tempangle = 0;
    
    Motor_left.initialize();
    Motor_right.initialize();
    Motor_back.initialize();
    //Kompass kalibrieren  --> Problem fremde Magnetfelder!
    //Comp.AutoCalibration = 1;
    short MagRawMin[3]= {-400, -400, -400};     //Gespeicherte Werte verwenden
    short MagRawMax[3]= {400, 400, 400};
    Comp.Calibrate(MagRawMin, MagRawMax);
    //Comp.Calibrate(20);
    
    //Oversampling des Barometers setzen
    Alt.oss = 0;
    
    GlobalTimer.start();
    while(1) {
        // read data from sensors
        Gyro.read(Gyro_data);
        Acc.read(Acc_data);
        Comp.Update();
        Alt.Update();

        // calculate the angles for roll and pitch from accelerometer
        Acc_angle[0] = Rad2Deg * atan2((float)Acc_data[1], (float)Acc_data[2]);
        Acc_angle[1] = 3 -Rad2Deg * atan2((float)Acc_data[0], (float)Acc_data[2]); // TODO Offset accelerometer einstellen
        
        //calculate angle for yaw from compass
        Comp_angle = Comp.getAngle(Comp.Mag[0], Comp.Mag[1]);
        
        // meassure dt
        dt = GlobalTimer.read_us() - time_loop; // time in us since last loop
        time_loop = GlobalTimer.read_us();      // set new time for next measurement
        
        // calculate angles for roll, pitch an yaw
        angle[0] += (Acc_angle[0] - angle[0])/50 + Gyro_data[0] *dt/15000000.0;
        angle[1] += (Acc_angle[1] - angle[1])/50 + Gyro_data[1] *dt/15000000.0;
        angle[2] += (Comp_angle - angle[2])/50 - Gyro_data[2] *dt/15000000.0;
        tempangle -= Gyro_data[2] *dt/15000000.0;
        
        // LCD output
        pc.locate(10,5);
        pc.printf("dt:%d %6.1fm   ", dt, Alt.CalcAltitude(Alt.Pressure));
        pc.locate(10,8);
        pc.printf("Roll:%6.1f Pitch:%6.1f Yaw:%6.1f    ", angle[0], angle[1], angle[2]);
        pc.locate(10,10);
        pc.printf("Debug_Yaw:  Comp:%6.1f Gyro:%6.1f  ", Comp_angle, tempangle);
        pc.locate(10,12);
        pc.printf("Comp_Raw: %6.1f %6.1f %6.1f   ", Comp.RawMag[0], Comp.RawMag[1], Comp.RawMag[2]);
        pc.locate(10,13);
        pc.printf("Comp_Mag: %6.1f %6.1f %6.1f   ", Comp.Mag[0], Comp.Mag[1], Comp.Mag[2]);
        
        // Read RC data
        //RC[0].read() // TODO: RC daten lesen und einberechnen!
        
        // PID Regelung
        
        // set new motor speeds
        Motor_left = 1000 + 5*abs(angle[1]);
        Motor_right = 1000 + 5*abs(angle[1]);
        Motor_back = 1000 + 5*abs(angle[1]);
        
        LEDs.rollnext();
        wait(0.1);
    }
}
