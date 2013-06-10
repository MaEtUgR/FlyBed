#ifndef RC_CHANNEL_H
#define RC_CHANNEL_H

#include "mbed.h"

class RC_Channel
{
    public:
        RC_Channel(PinName mypin, int index); // NO p19/p20!!!!, they don't support InterruptIn
        int read(); // read the last measured data
       
    private:
        int index; // to know which channel of the RC an instance has (only for calibrations savings)
        int time; // last measurement data
        float scale; // calibration values
        float offset;
        
        InterruptIn myinterrupt; // interrupt on the pin to react when signal falls or rises
        void rise(); // start the time measurement when signal rises
        void fall(); // stop the time mesurement and save the value when signal falls
        Timer timer; // timer to measure the up time of the signal and if the signal timed out
        
        Ticker timeoutchecker; // Ticker to see if signal broke down
        void timeoutcheck(); // to check for timeout, checked every second
        
        // Calibration value saving
        void saveCalibrationValue(float * value, char * fileextension);
        void loadCalibrationValue(float * value, char * fileextension);
};

#endif