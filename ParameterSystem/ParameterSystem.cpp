/*
 * ParameterSystem.cpp
 *
 *  Created on: 29.11.2015
 *      Author: MaEtUgR
 */

#include "ParameterSystem.h"

ParameterSystem::ParameterSystem() : local("local") {
	// TODO Auto-generated constructor stub

}

ParameterSystem::~ParameterSystem() {
	// TODO Auto-generated destructor stub
}

void ParameterSystem::writeParametersToFile() {
	FILE *fp = fopen("/local/param", "w");
	if(!fp)
		return;
	for(vector<float>::iterator it = _parameters.begin(); it != _parameters.end(); it++)
		fwrite(&*it, sizeof(float), 1, fp);
	fclose(fp);
}

void ParameterSystem::readParametersFromFile() {
	FILE *fp = fopen("/local/param", "r");
	if(!fp)
		return;
	float input;
	while(fread(&input, sizeof(input), 1, fp))
		_parameters.push_back(input);
	fclose(fp);
}
