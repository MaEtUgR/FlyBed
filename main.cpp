#include "mbed.h" // Standar Library
#include "LED.h" // LEDs
#include "L3G4200D.h" // Gyro (Gyroscope)
#include "ADXL345.h" // Acc (Accelerometer)
#include "HMC5883.h" // Comp (Compass)
#include "BMP085.h" // Alt (Altitude sensor)
#include "Servo.h" // Motor
#include "terminal.h"

Timer GlobalTimer;

// initialisation of hardware
LED         LEDs;
L3G4200D    Gyro(p28, p27);
ADXL345     Acc(p28, p27);
HMC5883     Comp(p28, p27, GlobalTimer);
BMP085      Alt(p28, p27);
Servo     Motor(p20);

terminal       pc(USBTX, USBRX);        //Achtung: Treiber auf PC fuer Serial-mbed installieren. hier herunterladen https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe

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
    
    Motor.initialize();
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
        dt = GlobalTimer.read_us() - time_loop; // Zeit in us seit letzter loop
        time_loop = GlobalTimer.read_us(); // setze aktuelle zeit f�r n�chste messung
        
        // calculate angles for roll, pitch an yaw
        angle[0] += (Acc_angle[0] - angle[0])/50 + Gyro_data[0] *dt/15000000.0;
        angle[1] += (Acc_angle[1] - angle[1])/50 + Gyro_data[1] *dt/15000000.0;
        angle[2] += (Comp_angle - angle[2])/50 - Gyro_data[2] *dt/15000000.0;
        tempangle -= Gyro_data[2] *dt/15000000.0;
        
        // LCD output
        pc.locate(10,5); // first line
        pc.printf("Y:%2.1f %2.1fm   ", angle[2], Alt.CalcAltitude(Alt.Pressure));
        //LCD.printf("%2.1f %2.1f %2.1f   ", Comp.RawMag[0], Comp.RawMag[1], Comp.RawMag[2]);
        pc.locate(10,8); // second line
        pc.printf("Roll:%6.1f Pitch:%6.1f Yaw:%6.1f    ", angle[0], angle[1], angle[2]);
        //LCD.printf("R:%2.1f P:%2.1f  ", Comp_angle, tempangle);
        //LCD.printf("%2.1f %2.1f %2.1f   ", Comp.Mag[0], Comp.Mag[1], Comp.Mag[2]);
        
        Motor = 1000 + 5*abs(angle[1]); // set new motor speed
        
        //LED hin und her
        int ledset = 0;
        if (angle[0] < 0) ledset += 1; else ledset += 8; 
        if (angle[1] < 0) ledset += 2; else ledset += 4;
        LEDs = ledset;
        
        //LEDs.rollnext();
        //wait(0.1);
    }
}
