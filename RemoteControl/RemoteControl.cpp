/*
 * RemoteControl.cpp
 *
 *  Created on: 04.12.2015
 *      Author: MaEtUgR
 */

#include "RemoteControl.h"

RemoteControl::RemoteControl() :
	_channels({RC_Channel(p8), RC_Channel(p15), RC_Channel(p17), RC_Channel(p16), RC_Channel(p25), RC_Channel(p26), RC_Channel(p29)}),
	_params("RC")
{
	_armed = true;
	_stickCentring = true;
	for(int i = 0; i < 3; i++)
		_angle[i] = 0;

	_params.readASCIIFile();
	if (_params.size() != CHANNELS * 2) {
		_params.resize(CHANNELS * 2);

		for(int i = 0; i < CHANNELS; i++) // set all offsets to zero
			_params.setParameter(i, 0);

		for(int i = CHANNELS; i < CHANNELS*2; i++) // set all scales to 1
			_params.setParameter(i, 1);
	}
}

bool RemoteControl::present() {
	return (_channels[AILERON].present() && _channels[ELEVATOR].present() && _channels[RUDDER].present() && _channels[THROTTLE].present());
}

float RemoteControl::getValue(int i) {
	float calibratedValue = (_channels[i].read() + _params[i]) * _params[CHANNELS + i];

	if(_stickCentring && i != THROTTLE && 485 < calibratedValue && calibratedValue < 515)
		calibratedValue = 500;

	return _channels[i].present() ? calibratedValue : -100;
}

void RemoteControl::run(float IMU_yaw) {
	if(getValue(THROTTLE) < 20 && getValue(RUDDER) > 850) {					// arming / disarming
		_armed = true;
		_angle[2] = IMU_yaw;
	}
	if((getValue(THROTTLE) < 30 && getValue(RUDDER) < 30) || !present())
		_armed = false;

	for(int i=0;i<2;i++)													// RC Angle ROLL-PITCH-Part
		if (present())
			_angle[i] = (getValue(i)-500)*RC_SENSITIVITY/500.0;
		else
			_angle[i] = 0;

	float yaw_adding;                  										// RC Angle YAW-Part
	if (present() && getValue(THROTTLE) > 20)
		yaw_adding = -(getValue(RUDDER)-500)*YAWSPEED/500;					// the yaw angle is integrated from stick input		TODO: make this independent of loop frequency
	else
		yaw_adding = 0;

	_angle[2] = _angle[2] + yaw_adding < -180 ? _angle[2] + 360 + yaw_adding : _angle[2] + yaw_adding; // make shure it's in the cycle -180 to 180
	_angle[2] = _angle[2] + yaw_adding > 180 ? _angle[2] - 360 + yaw_adding : _angle[2] + yaw_adding;
}

void RemoteControl::calibrate(int seconds) {
	int mins[CHANNELS];
	int maxs[CHANNELS];
	fill(mins, mins+CHANNELS-1, 1000);
	fill(maxs, maxs+CHANNELS-1, 0);
	Timer calibrationTimer;
	calibrationTimer.start();

	while(calibrationTimer.read() < seconds)
		for(int i = 0; i < CHANNELS; i++) {
			int value = _channels[i].read();
			printf("%d ", value);
			if(i == CHANNELS-1)
				printf("\r\n");
			mins[i] = fminf(mins[i], value);
			maxs[i] = fmaxf(maxs[i], value);
		}

	for(int i = 0; i < CHANNELS; i++) {
		_params.setParameter(i, -mins[i]); // get the offset	TODO: setParameter with operator overloading params[i] = v;
		_params.setParameter(CHANNELS + i, 1000/(float)(maxs[i]-mins[i])); // get the scale
	}

	_params.writeASCIIFile(); // safe the calibration
}
