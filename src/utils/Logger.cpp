/*
 * Logger.cpp
 *
 *  Created on: 30 mar. 2018
 *      Author: diego
 */

#include "Logger.h"

Logger::LogLevelEnum Logger::logLevel = Debug;

void Logger::setLogLevel(LogLevelEnum newLogLevel){
	logLevel = newLogLevel;
}

void Logger::log(LogLevelEnum newLogLevel, const char logText[]){
	if(logLevel <= newLogLevel){
		Serial.println(logText);
	}
}

void Logger::log(LogLevelEnum newLogLevel,const String &logText){
	if(logLevel <= newLogLevel){
		Serial.println(logText);
	}
}

