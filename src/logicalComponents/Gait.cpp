/*
 * Gait.cpp
 *
 *  Created on: 19 may. 2018
 *      Author: ziash
 */

#include <logicalComponents/Gait.h>

Gait::Gait(GaitEnum gaitEnum, const signed  char* gaitPointer) {
	this->gaitEnum = gaitEnum;
	this->gaitPointer = gaitPointer;
}

Gait::GaitEnum Gait::getGaitEnum(){
	return gaitEnum;
}
