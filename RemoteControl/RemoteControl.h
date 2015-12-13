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

	void run();
	inline bool armed() {return _armed;}
	float getValue(int i);
	inline float operator[](int i) {return getValue(i);}
	inline bool present() {return (_channels[AILERON].present() && _channels[ELEVATOR].present() && _channels[RUDDER].present() && _channels[THROTTLE].present());};  // shows if remote control is working	TODO: Failsafe
	void enableStickCentering() {_stickCentring = true;}
	void calibrate(int seconds);
//private:
	RC_Channel			_channels[CHANNELS];
	bool				_armed;  // is for safety (when false no motor should rotate any more!)
	ParameterSystem		_params;
	bool				_stickCentring;
};

#endif
