/*
 * GaitController.h
 *
 *  Created on: 12 may. 2018
 *      Author: ziash
 */

#ifndef SRC_CONTROLLERS_GAITCONTROLLER_H_
#define SRC_CONTROLLERS_GAITCONTROLLER_H_

#include <logicalComponents/Gait.h>
#include <logicalComponents/GaitModes.h>
#include "Arduino.h"


#define DEFAULT_MAX_GAIT_SIZE 	274
#define GAIT_VECTOR_SIZE 		8

class GaitController {
public:
	GaitController();
	Gait* getGait(Gait::GaitEnum gaitEnum);
private:
	static Gait* gaitVector[GAIT_VECTOR_SIZE];
};

#endif /* SRC_CONTROLLERS_GAITCONTROLLER_H_ */
