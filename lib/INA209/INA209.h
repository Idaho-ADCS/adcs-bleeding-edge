/*
  INA209.h - Library for Texas Instruments INA209
  Created 10 Aug. 2012 (Rev. 1.0)
  Copyright (c) 2012 Antonio Dambrosio <info@adpslab.com>.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#ifndef INA209_H
#define INA209_H

#include <Arduino.h>
#include <Wire.h>

class INA209 {
	int i2c_addr;
	int p_addr;
private:
	word readWord();
	void writeWord(int p_address, word wordToW);
	void pointReg(int p_address);
public:
	INA209(void){}
	INA209(int address);
	~INA209();
	word readCfgReg();
	void writeCfgReg(word CfgReg);
	word statusReg();
	word readSMBusCtrlReg();
	void writeSMBusCtrlReg(word SMBusCtrlReg);
	int shuntVol();
	int busVol();
	int power();
	int current();
	int readShuntVolPpk();
	void writeShuntVolPpk(word ShuntVolPpk);
	int readShuntVolNpk();
	void writeShuntVolNpk(word ShuntVolNpk);
	int readBusVolMaxPk();
	void writeBusVolMaxPk(word BusVolMaxPk);
	int readBusVolMinPk();
	void writeBusVolMinPk(word BusVolMinPk);
	word readPowerPk();
	void writePowerPk(word PowerPk);
	int readShuntVolPwrn();
	void writeShuntVolPwrn(word ShuntVolPwrn);
	int readShuntVolNwrn();
	void writeShuntVolNwrn(word ShuntVolNwrn);
	word readPowerWrn();
	void writePowerWrn(word PowerWrn);
	word readBusOVwrn();
	void writeBusOVwrn(word BusOVwrn);
	word readBusUVwrn();
	void writeBusUVwrn(word BusUVwrn);
	word readPowerOL();
	void writePowerOL(word PowerOL);
	word readBusOVOL();
	void writeBusOVOL(word BusOVOL);
	word readBusUVOL();
	void writeBusUVOL(word BusUVOL);
	word readCrShuntPV();
	void writeCrShuntPV(word CrShuntPV);
	word readCrShuntNV();
	void writeCrShuntNV(word CrShuntNV);
	word readCal();
	void writeCal(word cal);
};

#endif
