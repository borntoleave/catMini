/*
 * GaitController.cpp
 *
 *  Created on: 12 may. 2018
 *      Author: ziash
 */

#include "GaitController.h"

Gait* GaitController::gaitVector[GAIT_VECTOR_SIZE] = {
		new Gait(Gait::GaitEnum::Bound		, bound),
		new Gait(Gait::GaitEnum::Crawl		, crawl),
		new Gait(Gait::GaitEnum::CrawlRight	, crawlRight),
		new Gait(Gait::GaitEnum::CrawlLeft	, crawlLeft),
		new Gait(Gait::GaitEnum::TrotLeft	, trotLeft),
		new Gait(Gait::GaitEnum::TrotRight	, trotRight),
		new Gait(Gait::GaitEnum::Walk		, walk)
		};

GaitController::GaitController() {}

Gait* GaitController::getGait(Gait::GaitEnum gaitEnum){
	for(int i=0; i<GAIT_VECTOR_SIZE; i++){
		if(gaitVector[i]->getGaitEnum() == gaitEnum){
			return gaitVector[i];
		}
	};
	return NULL;
}
