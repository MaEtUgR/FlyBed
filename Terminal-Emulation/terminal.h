
#include "mbed.h"

#ifndef MBED_TERMINAL_H
#define MBED_TERMINAL_H

class terminal : public Serial 
    {
    public:
        terminal(PinName tx, PinName rx);
        void cls();
        void locate(int column, int row);
    };
#endif
