/*
 * RemoteControl.h
 *
 *  Created on: 04.12.2015
 *      Author: MaEtUgR
 */

#ifndef REMOTECONTROL_H_
#define REMOTECONTROL_H_

#include <vector>
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

	float getValueCalibrated(int i);
	inline float operator[](int i) {return getValueCalibrated(i);}
	void enableStickCentering() {_stickCentring = true;}
	void calibrate(int seconds);
//private:
	RC_Channel			_channels[CHANNELS];
	ParameterSystem		_params;
	bool				_stickCentring;
};

#endif
