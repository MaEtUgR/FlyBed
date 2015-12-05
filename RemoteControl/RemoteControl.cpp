/*
 * RemoteControl.cpp
 *
 *  Created on: 04.12.2015
 *      Author: MaEtUgR
 */

#include "../RemoteControl/RemoteControl.h"

RemoteControl::RemoteControl() {
	_channels.reserve(9);
	_channels[0] = RC_Channel(p8 ,1);
	_channels[1] = RC_Channel(p15,2);
	_channels[2] = RC_Channel(p17,4);
	_channels[3] = RC_Channel(p16,3);
	_channels[4] = RC_Channel(p25,2);
	_channels[5] = RC_Channel(p26,4);
	_channels[6] = RC_Channel(p29,3);
}

