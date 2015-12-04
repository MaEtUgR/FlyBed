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
	virtual ~ParameterSystem(){}

	void writeBinaryFile();											// saving or loading all the values (overwrites)
	void readBinaryFile();
	void writeASCIIFile();
	void readASCIIFile();

	inline float getParameter(int i) {return _parameters[i];}		// using parameters from RAM
	inline float operator[](int i) {return getParameter(i);}

	inline void setParameter(int i, float v) {_parameters[i] = v;}	// changing parameters in RAM
	inline int size() {return _parameters.size();}
protected:
	LocalFileSystem _localFileSystem;								// object needed to get the mbed boards flash memory "mounted"
	string _filename;												// for binary directly the filename for ASCII filename.txt
	vector<float> _parameters;										// parameter data in RAM
};

#endif
