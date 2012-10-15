#include "RC_Channel.h"
#include "mbed.h"

RC_Channel::RC_Channel(PinName mypin) : myinterrupt(mypin)
{
    myinterrupt.rise(this, &RC_Channel::rise);
    myinterrupt.fall(this, &RC_Channel::fall);
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
}