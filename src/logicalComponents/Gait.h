/*
 * Gait.h
 *
 *  Created on: 19 may. 2018
 *      Author: ziash
 */

#ifndef SRC_CONTROLLERS_GAIT_H_
#define SRC_CONTROLLERS_GAIT_H_

class Gait {
public:
	enum GaitEnum{
					Bound = 0,
					Crawl,
					CrawlRight,
					CrawlLeft,
					TrotLeft,
					TrotRight,
					Walk,
					WalkLeft,
					WalkRight,
					Back,
					BackLeft,
					BackRight,
					StepOnSpot,
					ButtonUp,
					Dropped,
					Sit,
					Calibration,
					Pee,
					Balance,
					Lifted,
					Rest
	};
	Gait();
	Gait(GaitEnum gaitEnum, const signed  char* gaitPointer);
	Gait::GaitEnum getGaitEnum();
private:
	GaitEnum gaitEnum;
	const signed  char* gaitPointer;
};

#endif /* SRC_CONTROLLERS_GAIT_H_ */
