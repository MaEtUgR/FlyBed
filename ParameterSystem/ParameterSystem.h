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
#include <string>

class ParameterSystem {
public:
	ParameterSystem(string filename);
	virtual ~ParameterSystem();

	void writeBinaryFile();
	void readBinaryFile();
	void writeASCIIFile();
	void readASCIIFile();
	inline float getParameter(int i) {return _parameters[i];}
	inline float operator[](int i) {return getParameter(i);}
	inline void setParameter(int i, float v) {_parameters[i] = v;}
protected:
	LocalFileSystem _localFileSystem;
	string _filename;
	vector<float> _parameters;
};

#endif
