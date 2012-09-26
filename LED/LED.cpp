#include "LED.h"
#include "mbed.h"
    
LED::LED() : Led(LED1, LED2, LED3, LED4){ }

void LED::shownumber(int number) {
    Led = number;
}

void LED::roll(int times = 1) {
    for (int j = 0; j < times; j++) {
        for(int i=0; i<4; i++) {
                Led = 1 << i;
                wait(0.25);
        }
    }
}

void LED::tilt(int index) {
    Led = Led^(1 << (index-1));
}