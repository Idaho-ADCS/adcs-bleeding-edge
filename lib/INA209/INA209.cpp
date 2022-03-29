/*
  INA209.cpp - Library for Texas Instruments INA209
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

#include "INA209.h"

//<<constructor>> 
INA209::INA209(int address){
	i2c_addr = address;	
}
//<<destructor>>
INA209::~INA209(){/*nothing to destruct*/}

// positioning on register pointer address
void INA209::pointReg(int p_address) {			
	Wire.beginTransmission(i2c_addr);      
	Wire.write(p_address);                  
	Wire.endTransmission();
}
// read a word from the register pointed
word INA209::readWord() {			
	Wire.requestFrom(i2c_addr, 2);    // read 2 bytes from register	
	byte MSB = Wire.read();    
	byte LSB = Wire.read();    
	return word(MSB,LSB);
}
// write a word into the register pointed
void INA209::writeWord(int p_address, word wordToW) {	
	Wire.beginTransmission(i2c_addr);	
	Wire.write(p_address);
	Wire.write(highByte(wordToW)); 
	Wire.write(lowByte(wordToW));  
	Wire.endTransmission();
}

// ----------------------- Config/Status REGISTERS ----------------------------------------
// Configuration Register (All-register reset, settings for bus, voltage range, PGA Gain, ADC, resolution/averaging).
word INA209::readCfgReg(){
	pointReg(0x00);     
	return readWord();
}
void INA209::writeCfgReg(word CfgReg){
	writeWord(0x00,CfgReg);
}
// read Status Register (Status flags for warnings,over-/under-limits, conversion ready,math overflow, and SMBus Alert).).
word INA209::statusReg(){
	pointReg(0x01);
	return readWord();
}
// SMBus Alert Mask/Enable Control Register(Enables/disables flags in the Status Register).
word INA209::readSMBusCtrlReg(){
	pointReg(0x02);
	return readWord();
}
void INA209::writeSMBusCtrlReg(word SMBusCtrlReg){
	writeWord(0x02,SMBusCtrlReg);
}

// ----------------------- DATA OUTPUT REGISTERS ----------------------------------------
// Shunt Voltage (Shunt voltage measurement data).
int INA209::shuntVol(){
	pointReg(0x03);
	return int(readWord()); // returns integer value LSB=10uV
}
// Bus Voltage (Bus Voltage 12bit measurement data;
//				; at full-scale range = 32V (decimal = 8000, hex = 1F40), and LSB = 4mV
//				; at full-scale range = 16V (decimal = 4000, hex = 0FA0), and LSB = 4mV).
int INA209::busVol(){
	pointReg(0x04);
	return int(readWord() >> 1);   // returns mV integer value
}
// Power (Power measurement data).
// 		Full-scale range and LSB depend on the value entered in the Calibration Register (see the datasheet). 
int INA209::power(){
	pointReg(0x05);
	return int(readWord());
}
// Current (Contains the value of the current flowing through the shunt resistor).
// 		Full-scale range and LSB depend on the value entered in the Calibration Register (see the datasheet). 
int INA209::current(){
	pointReg(0x06);
	return int(readWord());
}

// ----------------------- PEAK-HOLD REGISTERS ----------------------------------------
// Note: All peak-hold registers are cleared and reset to POR values by writing a '1' into the respective D0 bits.

// Shunt Voltage Positive Peak (Contains most positive voltage reading of Shunt Voltage Register).
int INA209::readShuntVolPpk(){
	pointReg(0x07);
	return int(readWord());
}
void INA209::writeShuntVolPpk(word ShuntVolPpk){
	writeWord(0x07,ShuntVolPpk);
}
// Shunt Voltage Negative Peak (Contains most negative voltage reading of Shunt Voltage Register).
int INA209::readShuntVolNpk(){
	pointReg(0x08);
	return int(readWord());
}
void INA209::writeShuntVolNpk(word ShuntVolNpk){
	writeWord(0x08,ShuntVolNpk);
}
// Bus Voltage Maximum Peak (Contains highest voltage reading of Bus Voltage Register).
int INA209::readBusVolMaxPk(){
	pointReg(0x09);
	return int(readWord() >> 1);
}
void INA209::writeBusVolMaxPk(word BusVolMaxPk){
	writeWord(0x09,BusVolMaxPk);
}
// Bus Voltage Minimum Peak (Contains lowest voltage reading of Bus Voltage Register).
int INA209::readBusVolMinPk(){
	pointReg(0x0A);
	return int(readWord() >> 1);
}
void INA209::writeBusVolMinPk(word BusVolMinPk ){
	writeWord(0x0A,BusVolMinPk);
}
// Power Peak (Contains highest power reading of Power Register).
word INA209::readPowerPk(){
	pointReg(0x0B);
	return readWord();
}
void INA209::writePowerPk(word PowerPk){
	writeWord(0x0B,PowerPk);
}

// ----------------------- WARNING WATCHDOG REGISTERS ----------------------------------------
// These registers set warning limits that trigger flags in the Status Register and activate the Warning pin.

// Shunt Voltage Positive Warning (Warning watchdog register. Sets positive shunt voltage limit that triggers a warning flag in the Status Register, and activates Warning pin).
int INA209::readShuntVolPwrn(){
	pointReg(0x0C);
	return int(readWord());
}
void INA209::writeShuntVolPwrn(word ShuntVolPwrn){
	writeWord(0x0C,ShuntVolPwrn);
}
// Shunt Voltage Negative Warning (Warning watchdog register. Sets negative shunt voltage limit that triggers a warning flag in the Status Register, and activates Warning pin).
int INA209::readShuntVolNwrn(){
	pointReg(0x0D);
	return int(readWord());
}
void INA209::writeShuntVolNwrn(word ShuntVolNwrn){
	writeWord(0x0D,ShuntVolNwrn);
}
// Power Warning (Warning watchdog register. Sets power limit that triggers a warning flag in the Status Register, and activates Warning pin).
word INA209::readPowerWrn(){
	pointReg(0x0E);
	return readWord();
}
void INA209::writePowerWrn(word writePowerWrn){
	writeWord(0x0E,writePowerWrn);
}
// Bus Over-Voltage Warning (Warning watchdog register. Sets high Bus voltage limit that triggers a warning flag in the Status Register, and activates Warning pin. Also contains bits to set Warning pin polarity and latch feature).
word INA209::readBusOVwrn(){
	pointReg(0x0F);
	return readWord();
}
void INA209::writeBusOVwrn(word BusOVwrn){
	writeWord(0x0F,BusOVwrn);
}
// Bus Under-Voltage Warning (Warning watchdog register. Sets low Bus voltage limit that triggers a warning flag in the Status Register, and activates Warning pin).
word INA209::readBusUVwrn(){
	pointReg(0x10);
	return readWord();
}
void INA209::writeBusUVwrn(word BusUVwrn){
	writeWord(0x10,BusUVwrn);
}

// ----------------------- OVER-LIMIT/CRITICAL WATCHDOG REGISTERS ----------------------------------------
// These registers set the over-limit and critical DAC limits that trigger flags to be set in the Status Register and activate the Overlimit pin or the Critical pin.

// Power Over-Limit (Over-limit watchdog register. Sets power limit that triggers an over-limit flag in the Status Register, and activates the Overlimit pin).
word INA209::readPowerOL(){
	pointReg(0x11);
	return readWord();
}
void INA209::writePowerOL(word PowerOL){
	writeWord(0x11,PowerOL);
}
// Bus Over-Voltage Over-Limit(Over-limit watchdog register. Sets Bus over-voltage limit that triggers an over-limit flag in the Status Register, and activates the Overlimit pin. Also contains bits to set Overlimit pin polarity and latch feature).
word INA209::readBusOVOL(){	
	pointReg(0x12);
	return readWord();
}
void INA209::writeBusOVOL(word BusOVOL){
	writeWord(0x12,BusOVOL);
}
// Bus Under-Voltage Over-Limit(Over-limit watchdog register. Sets Bus under-voltage limit that triggers an over-limit flag in the Status Register, and activates the Overlimit pin).
word INA209::readBusUVOL(){	
	pointReg(0x13);
	return readWord();
}
void INA209::writeBusUVOL(word BusUVOL){
	writeWord(0x13,BusUVOL);
}
// Critical Shunt Positive Voltage(Sets a positive limit for internal Critical DAC+. Contains bits for GPIO pin status and mode of operation, Critical Comparator latch feature and hysteresis).
word INA209::readCrShuntPV(){	
	pointReg(0x14);
	return readWord();
}
void INA209::writeCrShuntPV(word CrShuntPV){
	writeWord(0x14,CrShuntPV);
}
// Critical Shunt Negative Voltage(Sets a negative limit for internal Critical DAC-. Contains bits for Warning pin delay, and Critical Comparator output filter configuration).
word INA209::readCrShuntNV(){	
	pointReg(0x15);
	return readWord();
}
void INA209::writeCrShuntNV(word CrShuntNV){
	writeWord(0x15,CrShuntNV);
}
// Calibration(Sets full-scale range and LSB of current and power measurements. Overall system calibration.).
word INA209::readCal(){	
	pointReg(0x16);
	return readWord();
}
void INA209::writeCal(word cal){
	writeWord(0x16,cal);
}











