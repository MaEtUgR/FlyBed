/*
 * RemoteControl.cpp
 *
 *  Created on: 04.12.2015
 *      Author: MaEtUgR
 */

#include "../RemoteControl/RemoteControl.h"

RemoteControl::RemoteControl() :
	_params("RemoteControl"),
	_channels({RC_Channel(p8), RC_Channel(p15), RC_Channel(p17), RC_Channel(p16), RC_Channel(p25), RC_Channel(p26), RC_Channel(p29)})
{
	_stickCentring = false;

	_params.readASCIIFile();
	if (_params.size() != CHANNELS * 2) {
		_params.resize(CHANNELS * 2);

		for(int i = 0; i < CHANNELS; i++) // set all offsets to zero
			_params[i] = 0;

		for(int i = CHANNELS; i < CHANNELS*2; i++) // set all scales to 1
			_params[i] = 1;
	}
}

float RemoteControl::getValueCalibrated(int i) {
	float calibratedValue = (float)(_channels[i].read() + _params[i]) * _params[CHANNELS + i];
	if(_stickCentring && i != THROTTLE && 490 < calibratedValue && calibratedValue < 510)
		calibratedValue = 500;
	return calibratedValue;
}

void RemoteControl::calibrate(int seconds) {
	int min[CHANNELS];
	int max[CHANNELS];
	Timer calibrationTimer;
	calibrationTimer.start();

	while(calibrationTimer.read() < seconds)
		for(int i = 0; i < CHANNELS; i++) {
			int value = _channels[i].read();
			min[i] = value < min[i] ? value : min[i];
			max[i] = value > max[i] ? value : max[i];
		}

	for(int i = 0; i < CHANNELS; i++) {
		_params[i] = -min[i]; // get the offset
		_params[CHANNELS + i] = 1000/(float)(max[i]-min[i]); // get the scale
	}

	_params.writeASCIIFile(); // safe the calibration
}
