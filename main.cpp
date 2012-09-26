#include "mbed.h"
#include "LCD.h" // Display
#include "LED.h" // LEDs
#include "L3G4200D.h" // Gyro
#include "ADXL345.h" // Acc

PinName kl[4] = {LED1, LED2, LED3, LED4};
LED Led();
TextLCD LCD(p5, p6, p7, p8, p9, p10, p11, TextLCD::LCD16x2); // RS, RW, E, D0, D1, D2, D3, Typ

// Sensor initialisation
Gyro Gyro(p28, p27);
ADXL345 Acc(p28, p27);

int main() {

    LCD.printf("FlyBed v0.1");
    
    wait(2);
    int j = -1;
    int Gyro_data[3];
    int Acc_data[3];
    
    Acc.setPowerControl(0x00);
    Acc.setDataFormatControl(0x0B);
    Acc.setDataRate(ADXL345_3200HZ);
    Acc.setPowerControl(MeasurementMode);
    
    while(1) {
        
        Gyro.read(Gyro_data);
        Acc.getOutput(Acc_data);
        
        //LCD.cls(); // Display löschen
        LCD.locate(0,0);
        LCD.printf("%d %d %d |%d   ", Gyro_data[0],Gyro_data[1],Gyro_data[2],Gyro.readTemp()); //roll(x) pitch(y) yaw(z)
        LCD.locate(1,0);
        //LCD.printf("%d|%d|%d %d", gmitt[0]/j,gmitt[1]/j,gmitt[2]/j,gyro.readTemp()); //roll pitch yaw
        LCD.locate(1,0);
        LCD.printf("%d %d %d       ", (int16_t)Acc_data[0],(int16_t)Acc_data[1],(int16_t)Acc_data[2]);
        
        j++;
        
        //Led[j%4] = !Led[j%4];
        
        wait(0.1);
        
    }
}
