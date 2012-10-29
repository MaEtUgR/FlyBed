#include "Servo_PWM.h"
#include "mbed.h"

Servo_PWM::Servo_PWM(PinName Pin) : ServoPin(Pin) {
    ServoPin.period(0.020);
    ServoPin = 0;
    initialize(); // TODO: Works?
}

void Servo_PWM::initialize() {
    // initialize ESC
    SetPosition(1000); // full throttle
    wait(0.01);         // for 0.01 secs
    SetPosition(1000);  // low throttle
}

void Servo_PWM::SetPosition(int position) {
    ServoPin.pulsewidth(position/1000000.0);
}

void Servo_PWM::operator=(int position) {
    SetPosition(position);
}