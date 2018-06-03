/*
 * SerialCommunicationController.h
 *
 *  Created on: 12 may. 2018
 *      Author: ziash
 */

#ifndef SRC_CONTROLLERS_SERIALCOMMUNICATIONCONTROLLER_H_
#define SRC_CONTROLLERS_SERIALCOMMUNICATIONCONTROLLER_H_

#include "Arduino.h"
#include "utils/Logger.h"

#define SERIAL_TERMINATOR_CHARACTER '\r'

class SerialCommunicationController {
public:
	static SerialCommunicationController* serialCommunicationController;

	enum SerialCommandStateEnum{
						noState = 0,
						help ,
						calibrateServos,
						shutDownServos,
						reset
					};

	SerialCommunicationController();
	static SerialCommunicationController* getInstance();
	static void emptyBuffer();
	int  parseCommand(String command);
	void threadTask();

private:
	SerialCommandStateEnum serialCommandState = noState;

};

#endif /* SRC_CONTROLLERS_SERIALCOMMUNICATIONCONTROLLER_H_ */
