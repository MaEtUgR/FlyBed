/*
 * ParameterSystem.h
 *
 *  Created on: 29.11.2015
 *      Author: MaEtUgR
 */

#ifndef PARAMETERSYSTEM_H_
#define PARAMETERSYSTEM_H_

#include "mbed.h"
#include <vector>

class ParameterSystem {
public:
	ParameterSystem();
	virtual ~ParameterSystem();

	void writeParametersToFile();
	void readParametersFromFile();
	inline float getParameter(int i) {return _parameters[i];}
	inline float operator[](int i) {return getParameter(i);}
	void setParameter(int i, float v) {_parameters[i] = v;}
protected:
	LocalFileSystem local;
	vector<float> _parameters;
};

#endif
