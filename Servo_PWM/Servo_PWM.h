// by MaEtUgR

#ifndef SERVO_PWM_H
#define SERVO_PWM_H

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

class Servo_PWM {

public:
    /** Create a new Servo object on any mbed pin
     *
     * @param Pin Pin on mbed to connect servo to
     */
    Servo_PWM(PinName Pin);
    
    /** Change the position of the servo. Position in us
     *
     * @param NewPos The new value of the servos position (us)
     */
    void SetPosition(int position);
    
    //operator for confortable positioning
    void operator=(int position);
    
    // initialize ESC
    void initialize();

private:
    PwmOut ServoPin;
    
};

#endif