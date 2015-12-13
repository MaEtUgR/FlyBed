/*
 * RemoteControl.cpp
 *
 *  Created on: 04.12.2015
 *      Author: MaEtUgR
 */

#include "../RemoteControl/RemoteControl.h"

RemoteControl::RemoteControl() :
	_channels({RC_Channel(p8), RC_Channel(p15), RC_Channel(p17), RC_Channel(p16), RC_Channel(p25), RC_Channel(p26), RC_Channel(p29)}),
	_params("RC")
{
	_stickCentring = false;

	_params.readASCIIFile();
	if (_params.size() != CHANNELS * 2) {
		_params.resize(CHANNELS * 2);

		for(int i = 0; i < CHANNELS; i++) // set all offsets to zero
			_params.setParameter(i, 0);

		for(int i = CHANNELS; i < CHANNELS*2; i++) // set all scales to 1
			_params.setParameter(i, 1);
	}
}

float RemoteControl::getValueCalibrated(int i) {
	float calibratedValue = (_channels[i].read() + _params[i]) * _params[CHANNELS + i];
	if(_stickCentring && i != THROTTLE && 490 < calibratedValue && calibratedValue < 510)
		calibratedValue = 500;
	return calibratedValue;
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
			mins[i] = min(mins[i], value);
			maxs[i] = max(maxs[i], value);
		}

	for(int i = 0; i < CHANNELS; i++) {
		_params.setParameter(i, -mins[i]); // get the offset	TODO: setParameter with operator overloading params[i] = v;
		_params.setParameter(CHANNELS + i, 1000/(float)(maxs[i]-mins[i])); // get the scale
	}

	_params.writeASCIIFile(); // safe the calibration
}
