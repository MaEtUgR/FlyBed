// by MaEtUgR

#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include "mbed.h"

/** Class to control a servo by using PWM
 */

class Servo_PWM {

public:
    /** Create a new Servo object on any PWM pin
     *
     * @param Pin Pin on mbed to connect servo to
     */
    Servo_PWM(PinName Pin, int frequency);
    
    void SetFrequency(int frequency);
    
    /** Change the position of the servo. Position in us
     *
     * @param position The new value of the servos position between 0 and 1000 (gets 1000-2000) (us)
     */
    void SetPosition(int position);
    
    /** Operator for confortable positioning
     *
     * @param position see SetPosition
     */
    void operator=(int position);

private:
    PwmOut ServoPin;
    
};

#endif