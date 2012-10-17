#include "RC_Channel.h"
#include "mbed.h"

RC_Channel::RC_Channel(PinName mypin) : myinterrupt(mypin)
{
    time = -1; // start value to see if there was any value yet
    myinterrupt.rise(this, &RC_Channel::rise);
    myinterrupt.fall(this, &RC_Channel::fall);
    timeoutchecker.attach(this, &RC_Channel::timeoutcheck, 1);
}

int RC_Channel::read()
{
    return time;
}

void RC_Channel::rise()
{
    timer.start();
}

void RC_Channel::fall()
{
    timer.stop();
    time = timer.read_us();
    timer.reset();
    timer.start();
}

void RC_Channel::timeoutcheck()
{
    if (timer.read() > 0.3)
        time = 0;
}