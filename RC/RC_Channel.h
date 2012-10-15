#ifndef RC_CHANNEL_H
#define RC_CHANNEL_H

#include "mbed.h"

class RC_Channel
{
    public:
        RC_Channel(PinName mypin); // noooo p19/p20!!!!
        int read();
       
    private:
        InterruptIn myinterrupt;
        void rise();
        void fall();
        Timer timer;
        int time;
};

#endif