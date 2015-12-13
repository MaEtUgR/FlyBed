/*
 * ParameterSystem.cpp
 *
 *  Created on: 29.11.2015
 *      Author: MaEtUgR
 */

#include "ParameterSystem.h"

ParameterSystem::ParameterSystem(string filename) :
	_localFileSystem("local"),
	_filename(filename) {
}

void ParameterSystem::writeBinaryFile() {
	FILE *fp = fopen(("/local/" + _filename).c_str(), "w");
	if(!fp) return; // TODO: better error handling
	for(vector<float>::iterator it = _parameters.begin(); it != _parameters.end(); it++)
		fwrite(&*it, sizeof(float), 1, fp);
	fclose(fp);
}

void ParameterSystem::readBinaryFile() {
	FILE *fp = fopen(("/local/" + _filename).c_str(), "r");
	if(!fp) return; // TODO: better error handling
	_parameters.clear();
	float input;
	while(fread(&input, sizeof(input), 1, fp))
		_parameters.push_back(input);
	fclose(fp);
}

void ParameterSystem::writeASCIIFile() {
	FILE *fp = fopen(("/local/" + _filename + ".txt").c_str(), "w");
	if(!fp) return; // TODO: better error handling
	for(vector<float>::iterator it = _parameters.begin(); it != _parameters.end(); it++)
		fprintf(fp, "%f\r\n", *it);
	fclose(fp);
}

void ParameterSystem::readASCIIFile() {
	FILE *fp = fopen(("/local/" + _filename + ".txt").c_str(), "r");
	if(!fp) return; // TODO: better error handling
	_parameters.clear();
	float input;
	while(fscanf(fp, "%f", &input) > 0)
		_parameters.push_back(input);
	fclose(fp);
}
