#include "mbed.h" // Standar Library
#include "LCD.h" // Display
#include "LED.h" // LEDs
#include "L3G4200D.h" // Gyro
#include "ADXL345.h" // Acc
#include "Servo.h" // Motor

// initialisation
LED LEDs;
TextLCD LCD(p5, p6, p7, p8, p9, p10, p11, TextLCD::LCD16x2); // RS, RW, E, D0, D1, D2, D3, Typ
L3G4200D Gyro(p28, p27);
ADXL345 Acc(p28, p27);
//Servo Motor(p12);

Timer GlobalTimer;

#define PI             3.1415926535897932384626433832795
#define Rad2Deg        57.295779513082320876798154814105

int main() {
    // LCD/LED init
    LCD.cls(); // Display löschen
    LCD.printf("FlyBed v0.2");
    LEDs.roll(2);
    LEDs = 15;
    
    float Gyro_data[3];
    int Acc_data[3];
    //int Gyro_angle[3] = {0,0,0};
    unsigned long dt = 0;
    unsigned long time_loop = 0;
    
    //Motor.initialize();
    
    float angle[3] = {0,0,0};
    float Acc_angle[2] = {0,0};
    
    GlobalTimer.start();
    while(1) {
        Gyro.read(Gyro_data);
        Acc.read(Acc_data);

        // Acc data angle
        //float Acc_abs = sqrt(pow((float)Acc_data[0],2) + pow((float)Acc_data[1],2) + pow((float)Acc_data[2],2));
        //float Acc_angle = Rad2Deg * acos((float)Acc_data[2]/Acc_abs);
        Acc_angle[0] = Rad2Deg * atan2((float)Acc_data[1], (float)Acc_data[2]);
        Acc_angle[1] = -Rad2Deg * atan2((float)Acc_data[0], (float)Acc_data[2]);
        
        //dt berechnen
        dt = GlobalTimer.read_us() - time_loop;
        time_loop = GlobalTimer.read_us();
        
        angle[0] += (Acc_angle[0] - angle[0])/50 + Gyro_data[0] *dt/15000000.0;
        angle[1] += (Acc_angle[1] - angle[1])/50 + Gyro_data[1] *dt/15000000.0;
        angle[2] += /*(Acc_angle[1] - angle[1])/50 +*/ Gyro_data[2] *dt/15000000.0;
        //Gyro_angle[0] += (Gyro_data[0]) * 0.01;
        LCD.locate(0,0);
        //LCD.printf("%2.1f  %2.1f %2.1f", Gyro_data[0],Gyro_data[1],Gyro_data[2]);
        //LCD.printf("%d %d |%2.1f   ",Acc_data[1],Acc_data[2] ,Acc_angle); //roll(x) pitch(y) yaw(z)
        LCD.printf("     |%2.1f   ",Acc_data[2]/20.0);
        
        LCD.locate(1,0);
        //LCD.printf("%d %d %d %2.1f  ", Acc_data[0],Acc_data[1],Acc_data[2]);
        //LCD.printf("%2.1f %2.1f %2.2f %2.1f", Acc_angle,Acc_angle,dt/10000.0, angle);
        LCD.printf("%2.1f %2.1f %2.1f   ", angle[0], angle[1], angle[2]);
        
        //Motor = 1000 + abs(Acc_data[1]); // Motorwert anpassen
        
        //LED hin und her
        int ledset = 0;
        if (Acc_angle < 0)
            ledset += 1;
        else
            ledset += 8; 
        if (angle < 0)
            ledset += 2;
        else
            ledset += 4;
        //wait(0.1);
        LEDs = ledset;
        
        //LEDs.rollnext();
    }
}
