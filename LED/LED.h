// by MaEtUgR

#ifndef __LED_H
#define __LED_H

#include "mbed.h"

class LED {

    public:
        LED();
        void shownumber(int number);
        void ride(int times);
        void roll(int times);
        void rollnext();
        void tilt(int index);
        void operator=(int value);
    
    private:
        BusOut Led;
        int roller;
};

#endif