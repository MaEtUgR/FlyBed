#include "RC_Channel.h"
#include "mbed.h"

RC_Channel::RC_Channel(PinName mypin) : myinterrupt(mypin)
{
    time = -100; // start value to see if there was any value yet
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
    int tester = timer.read_us();
    if(tester >= 1000 && tester <=2000)
        time = (tester-70)-1000;  // TODO: skalierung mit calibrierung (speichern....)
    timer.reset();
    timer.start();
}

void RC_Channel::timeoutcheck()
{
    if (timer.read() > 0.3)
        time = -100;
}