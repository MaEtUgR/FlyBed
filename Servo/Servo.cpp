#include "Servo.h"
#include "mbed.h"

Servo::Servo(PinName Pin, int frequency) : ServoPin(Pin) {
    Enable(1000,1000000/frequency); // low throttle 50Hz TODO: Frequency modify
}

void Servo::SetFrequency(int frequency) {
    Pulse.detach();
    Pulse.attach_us(this, &Servo::StartPulse, 1/frequency);
}

void Servo::SetPosition(int Pos) {
    if (Pos > 1000)
        Pos = 1000;
    if (Pos < 0)
        Pos = 0;
    Position = Pos+1000;
}

void Servo::StartPulse() {
    ServoPin = 1;
    PulseStop.attach_us(this, &Servo::EndPulse, Position);
}

void Servo::EndPulse() {
    // my change
    PulseStop.detach();
    // my change
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