/*
 * RemoteControl.h
 *
 *  Created on: 04.12.2015
 *      Author: MaEtUgR
 */

#ifndef REMOTECONTROL_H_
#define REMOTECONTROL_H_

#include "RC_Channel.h"
#include <vector>

class RemoteControl {
public:
	RemoteControl();
protected:
	vector<RC_Channel> _channels;
};

#endif
