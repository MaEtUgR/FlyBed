#include "RC_Channel.h"

#include "mbed.h"

RC_Channel::RC_Channel(PinName pin) : _interrupt(pin) {
    _time = -100; // start value to see if there was any value yet

    _interrupt.rise(this, &RC_Channel::rise);	// attach interrupts such that the driver starts processing the signal
    _interrupt.fall(this, &RC_Channel::fall);
    _timeoutchecker.attach(this, &RC_Channel::timeoutcheck, 1);
}

int RC_Channel::read() {
    return _time;
}

void RC_Channel::rise() {
    _timer.start();
}

void RC_Channel::fall() {
    _timer.stop();
    int tester = _timer.read_us();
    if(tester >= 1000 && tester <= 2000)
        _time = tester-1000;  // we want only the signal from 1000 - 2000 as 0 - 1000 for easier scaling
    _timer.reset();
    _timer.start();
}

void RC_Channel::timeoutcheck() {
    if (_timer.read() > 0.3)
        _time = -100;
}
