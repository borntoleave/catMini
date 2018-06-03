/*
 * EEPROM.cpp
 *
 *  Created on: 30 mar. 2018
 *      Author: ziash
 */

#include "EEPROMMemory.h"

EEPROMMemory::EEPROMMemory() {}

void EEPROMMemory::writeInt(int p_address, int p_value){
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);
  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

int EEPROMMemory::readInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

int8_t EEPROMMemory::readInt8t(byte address)
{
  return (int8_t)EEPROM.read(address);
}

byte EEPROMMemory::readByte(byte address)
{
  return (byte)EEPROM.read(address);
}

