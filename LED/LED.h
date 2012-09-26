// by MaEtUgR

#ifndef __LED_H
#define __LED_H

#include "mbed.h"

class LED {

public:
    LED();
    void shownumber(int number);
    void roll(int times);
    void tilt(int index);

private:
    BusOut Led;
};

#endif