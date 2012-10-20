#include "LED.h"
#include "mbed.h"
    
LED::LED() : Led(LED1, LED2, LED3, LED4){
    roller = 0;
}

void LED::shownumber(int number) {
    Led = number;
}

void LED::ride(int times = 1) {
    Led = 0;
    for (int j = 0; j < times; j++) {
        for(int i=0; i < 4; i++) {
                Led = 1 << i;
                wait(0.2);
        }
    }
    Led = 0;
}

void LED::roll(int times = 1) {
    Led = 0;
    for (int j = 0; j < (times*2); j++) {
        for(int roller = 1; roller <= 4; roller++) {
                tilt(roller);
                wait(0.1);
        }
    }
    roller = 0;
    Led = 0;
}

void LED::rollnext() {
    if (roller >= 4)
        roller = 0;
    roller++;
    tilt(roller);
}

void LED::tilt(int index) {
    Led = Led ^ (1 << (index-1)); //XOR
}

void LED::set(int index) {
    Led = Led | (1 << (index-1)); //OR
}

void LED::reset(int index) {
    Led = Led & ~(1 << (index-1)); //OR
}

int LED::check(int index) {
    return Led & (1 << (index-1));
}

void LED::operator=(int value) {
    Led = value;
}