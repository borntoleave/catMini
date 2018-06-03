/*
 * Logger.h
 *
 *  Created on: 30 mar. 2018
 *      Author: diego
 */

#ifndef SRC_UTILS_LOGGER_H_
#define SRC_UTILS_LOGGER_H_

#include <Arduino.h>

class Logger {
public:
	enum LogLevelEnum{
				Debug = 0,
				Info,
				Error,
				Fatal
			};

	static void log(LogLevelEnum logLevel,const char logText[]);
	static void log(LogLevelEnum newLogLevel,const String &logText);
	static void setLogLevel(LogLevelEnum logLevel);

private:
	static LogLevelEnum logLevel;
};

#endif /* SRC_UTILS_LOGGER_H_ */
