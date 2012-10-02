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
Servo Motor(p12);

Timer GlobalTimer;

#define PI             3.1415926535897932384626433832795
#define Rad2Deg        57.295779513082320876798154814105

int main() {
    // LCD/LED init
    LCD.cls(); // Display löschen
    LCD.printf("FlyBed v0.2");
    LEDs.roll(2);
    //LEDs = 15;
    
    float Gyro_data[3];
    int Acc_data[3];
    int Gyro_angle[3] = {0,0,0};
    unsigned long dt = 0;
    unsigned long time_loop = 0;
    
    Motor.initialize();
    
    float angle = 0;//TEMP
    int j = 0;
    //float drift = 0;
    GlobalTimer.start();
    while(1) {
        j++;
        
        Gyro.read(Gyro_data);
        Acc.read(Acc_data);
        
        // Acc data angle
        float Acc_abs = sqrt(pow((float)Acc_data[0],2) + pow((float)Acc_data[1],2) + pow((float)Acc_data[2],2));
        //float Acc_angle = Rad2Deg * acos((float)Acc_data[2]/Acc_abs);
        float Acc_angle = 0.9 * Rad2Deg * atan((float)Acc_data[1]/(float)Acc_data[2]);
        
        //angle = (0.98)*(angle + Gyro_data[0] * 0.1) + (0.02)*(Acc_angle);
        
        /*float messfrequenz = 10;
        float geschwindigkeit = Gyro_data[0] - drift; 
        drift += (geschwindigkeit * messfrequenz * 0.3); 
        angle += (geschwindigkeit * messfrequenz); 
        angle += ((Acc_angle - angle) * messfrequenz * 0.1);*/
        
        //for (int i= 0; i < 3;i++)
            //drift[i] += (Gyro_data[i]-drift[i])* 0.1;
            
        //for (int i= 0; i < 3;i++)
            //Gyro_angle[i] += (Gyro_data[i]/*-drift[i]*/);
        
        //dt berechnen
        dt = GlobalTimer.read_us() - time_loop;
        time_loop = GlobalTimer.read_us();
        
        angle += (Acc_angle - angle)/30 + Gyro_data[0] * 0.01;
        //Gyro_angle[0] += (Gyro_data[0]) * 0.01;
        
        LCD.locate(0,0);
        LCD.printf(" |%2.1f   ",Acc_angle); //roll(x) pitch(y) yaw(z)
        LCD.locate(1,0);
        //LCD.printf("%d %d %d %2.1f  ", Acc_data[0],Acc_data[1],Acc_data[2], Acc_angle);
        LCD.printf("%d %d %2.4f %2.1f  ", Gyro_angle[0],Gyro_angle[1],dt/10000000.0, angle);
        
        Motor = 1000 + abs(Acc_data[1]); // Motorwert anpassen
        
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
            
        LEDs = ledset;
        
        //LEDs.rollnext();
        wait(0.1);
    }
}
