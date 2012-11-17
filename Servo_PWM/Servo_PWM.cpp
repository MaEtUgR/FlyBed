#include "Servo_PWM.h"
#include "mbed.h"

Servo_PWM::Servo_PWM(PinName Pin) : ServoPin(Pin) {
    ServoPin.period(0.020);
    ServoPin = 0;
    initialize();
}

void Servo_PWM::initialize() {
    // initialize ESC
    SetPosition(0);  // zero throttle
}

void Servo_PWM::SetPosition(int position) {
    ServoPin.pulsewidth_us(position+1000);
}

void Servo_PWM::operator=(int position) {
    SetPosition(position);
}