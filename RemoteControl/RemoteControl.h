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

class RemoteControl {
public:
	RemoteControl();

	float getValueCalibrated(int i);
	inline float operator[](int i) {return getValueCalibrated(i);}
//private:
	ParameterSystem		_params;
	RC_Channel			_channels[7];
};

#endif
