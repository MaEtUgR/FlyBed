/*
 * RemoteControl.cpp
 *
 *  Created on: 04.12.2015
 *      Author: MaEtUgR
 */

#include "../RemoteControl/RemoteControl.h"

RemoteControl::RemoteControl() :
	_params("RemoteControl"),
	_channels({RC_Channel(p8 ,1), RC_Channel(p15,2), RC_Channel(p17,4), RC_Channel(p16,3), RC_Channel(p25,2), RC_Channel(p26,4), RC_Channel(p29,3)}){
}

float RemoteControl::getValueCalibrated(int i) {
	return (float)_channels[i].read();
}
