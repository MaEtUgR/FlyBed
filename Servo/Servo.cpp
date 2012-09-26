#include "Servo.h"
#include "mbed.h"

Servo::Servo(PinName Pin) : ServoPin(Pin) {
    // initialize ESC
    Enable(2000,20000);   // full throttle
    wait(2);    // for 2 secs
    SetPosition(1000);    // low throttle
}

void Servo::SetPosition(int Pos) {
    Position = Pos;
}

void Servo::StartPulse() {
    ServoPin = 1;
    PulseStop.attach_us(this, &Servo::EndPulse, Position);
}

void Servo::EndPulse() {
    ServoPin = 0;
}

void Servo::Enable(int StartPos, int Period) {
    Position = StartPos;
    Pulse.attach_us(this, &Servo::StartPulse, Period);
}

void Servo::Disable() {
    Pulse.detach();
}

void Servo::operator=(int position) {
    SetPosition(position);
}