// based on http://mbed.org/users/jdenkers/code/Servo/

#ifndef SERVO_H
#define SERVO_H

#include "mbed.h"

/** Class to control a servo on any pin, without using pwm
 *
 * Example:
 * @code
 * // Keep sweeping servo from left to right
 * #include "mbed.h"
 * #include "Servo.h"
 * 
 * Servo Servo1(p20);
 *
 * Servo1.Enable(1500,20000);
 *
 * while(1) {
 *     for (int pos = 1000; pos < 2000; pos += 25) {
 *         Servo1.SetPosition(pos);  
 *         wait_ms(20);
 *     }
 *     for (int pos = 2000; pos > 1000; pos -= 25) {
 *         Servo1.SetPosition(pos); 
 *         wait_ms(20); 
 *     }
 * }
 * @endcode
 */

class Servo {

public:
    /** Create a new Servo object on any mbed pin
     *
     * @param Pin Pin on mbed to connect servo to
     */
    Servo(PinName Pin, int frequency);
    void SetFrequency(int frequency);
    
    /** Change the position of the servo. Position in us
     *
     * @param NewPos The new value of the servos position (us)
     */
    void SetPosition(int NewPos);
    
    /** Enable the servo. Without enabling the servo won't be running. Startposition and period both in us.
     *
     * @param StartPos The position of the servo to start (us) 
     * @param Period The time between every pulse. 20000 us = 50 Hz(standard) (us)
     */
    void Enable(int StartPos, int Period);
    
    //Disable the servo. After disabling the servo won't get any signal anymore
    void Disable();
    
    //operator for confortable positioning
    void operator=(int position);
    

private:
    void StartPulse();
    void EndPulse();

    int Position;
    DigitalOut ServoPin;
    Ticker Pulse;
    Timeout PulseStop;
};

#endif