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

int main() {
    // LCD/LED init
    LCD.cls(); // Display löschen
    LCD.printf("FlyBed v0.1");
    LEDs.roll(2);
    //LEDs = 15;
    
    int Gyro_data[3];
    int Acc_data[3];
    
    while(1) {
        
        Gyro.read(Gyro_data);
        Acc.getOutput(Acc_data);
        
        LCD.locate(0,0);
        LCD.printf("%d %d %d |%d   ", Gyro_data[0],Gyro_data[1],Gyro_data[2],Gyro.readTemp()); //roll(x) pitch(y) yaw(z)
        LCD.locate(1,0);
        LCD.printf("%d %d %d %d   ", (int16_t)Acc_data[0],(int16_t)Acc_data[1],(int16_t)Acc_data[2], 1000 + abs((int16_t)Acc_data[1]));
        
        if(abs((int16_t)Acc_data[0]) > 200)
            Motor.initialize();
        
        Motor = 1000 + abs((int16_t)Acc_data[1]); // Motorwert anpassen
        
        LEDs.rollnext();
        wait(0.1);
        
    }
}
