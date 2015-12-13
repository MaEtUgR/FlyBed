/*
 * RemoteControl.h
 *
 *  Created on: 04.12.2015
 *      Author: MaEtUgR
 */

#ifndef REMOTECONTROL_H_
#define REMOTECONTROL_H_

#include "mbed.h"
#include "RC_Channel.h"
#include "ParameterSystem.h"

#define CHANNELS 7	// TODO: make more general
#define RC_SENSITIVITY  30      // maximal angle from horizontal that the PID is aming for
#define YAWSPEED        1.0f    // maximal speed of yaw rotation in degree per Rate

#define AILERON         0       // RC
#define ELEVATOR        1		// TODO: enum
#define RUDDER          2
#define THROTTLE        3
#define CHANNEL8        4
#define CHANNEL7        5
#define CHANNEL6        6

class RemoteControl {
public:
	RemoteControl();

	inline bool		armed() {return _armed;}							// for safety (when true the QC is DANGEROUS!! Take this SERIOUS!!)
	bool			present();											// shows if remote control is working	TODO: Failsafe
	float			getValue(int i);									// get the Value from a specific channel of the remote control
	inline float	operator[](int i) {return getValue(i);}

	void			run(float IMU_yaw);									// needs to be called in the main loop to get everything updated
	void			enableStickCentering() {_stickCentring = true;}		// values near the middle position get pulled to exactly the middle value (so there's no drift when you let go of the sticks)
	void			calibrate(int seconds);								// calibrate the remote control (move the sticks to all the limits during the seconds)

//private:
	RC_Channel			_channels[CHANNELS];							// driver to get values for the channels
	bool				_armed;											// safety flag
	ParameterSystem		_params;										// calibration parameters which also get saved to the flash
	bool				_stickCentring;									// centering feature flag
	float 				_angle[3];										// set pount angle of the RC Sticks, to steer the QC in level mode
};

#endif
