#include "RC_Channel.h"
#include "mbed.h"

RC_Channel::RC_Channel(PinName mypin, int index) : myinterrupt(mypin)
{
    LocalFileSystem local("local");                                  // If theres no local yet
    RC_Channel::index = index;
    time = -100; // start value to see if there was any value yet
    
    loadCalibrationValue(&scale, "SCALE");
    loadCalibrationValue(&offset, "OFFSE");
    
    myinterrupt.rise(this, &RC_Channel::rise);
    myinterrupt.fall(this, &RC_Channel::fall);
    timeoutchecker.attach(this, &RC_Channel::timeoutcheck, 1);
}

int RC_Channel::read()
{
    if(time == -100)
        return time;
    float result = scale * (float)(time) + offset; // calibration of the readings
    return result;
    //return (result > 490 && result < 510) ? 500 : result; // make the middle of the stickpositions non drifting
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
        time = tester-1000;  // we want only the signal from 1000 - 2000 as 0 - 1000 for easier scaling
    timer.reset();
    timer.start();
}

void RC_Channel::timeoutcheck()
{
    if (timer.read() > 0.3)
        time = -100;
}

void RC_Channel::saveCalibrationValue(float * value, char * fileextension)
{
    char path[40];
    sprintf(path, "/local/FlyBed/RC_%d_%s", index, fileextension);
    FILE *fp = fopen(path, "w");
    if (fp != NULL) {
        fprintf(fp, "%f", *value);
        fclose(fp);
    } else
        value = 0;
}

void RC_Channel::loadCalibrationValue(float * value, char * fileextension)
{
    char path[40];
    sprintf(path, "/local/RC%d%s", index, fileextension);
    FILE *fp = fopen(path, "r");
    if (fp != NULL) {
        fscanf(fp, "%f", value);
        fclose(fp);
    } else
        value = 0;
}