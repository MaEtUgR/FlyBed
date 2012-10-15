#include "mbed.h"

#ifndef PC_H
#define PC_H

class PC : public Serial 
{
    public:
        PC(PinName tx, PinName rx);
        void cls();
        void locate(int column, int row);
};
#endif
