#ifndef RC_CHANNEL_H
#define RC_CHANNEL_H

#include "mbed.h"

class RC_Channel
{
    public:
        RC_Channel(PinName mypin);	// NO p19/p20!!!!, they don't support InterruptIn
        int read();					// read the last measured data
       
    //private:
        int _time;					// last measurement data
        float _scale;				// calibration values
        float _offset;
        
        InterruptIn _interrupt;		// interrupt on the pin to react when signal falls or rises
        void rise();				// start the time measurement when signal rises
        void fall();				// stop the time measurement and save the value when signal falls
        Timer _timer;				// timer to measure the up time of the signal and if the signal timed out
        
        Ticker _timeoutchecker;		// Ticker to see if signal broke down
        void timeoutcheck();		// to check for timeout, checked every second
};

#endif
