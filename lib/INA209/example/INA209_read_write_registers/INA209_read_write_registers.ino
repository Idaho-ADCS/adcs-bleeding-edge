/*
  Texas Instruments INA209 Library
  Created 10 Aug. 2012 (Rev. 1.0)
  Copyright (c) 2012 Antonio Dambrosio <info@adpslab.com>.  All right reserved.
  
  This example demonstrates the I2C communication with an INA209 chip
  (High-Side Measurement, Bi-Directional Current/Power Monitor)
  The INA209 library works only with the Texas Instruments INA209 chips.
  
    The circuit:
      INA209 PIN 1 (Vin+)  <->  to positive side of 0,1 Ohm shunt resistor
      INA209 PIN 2 (Vin-)  <->  to negative side of 0,1 Ohm shunt resistor
      INA209 PIN 4 (GND)   <->  to power supply ground
      INA209 PIN 5 (Vs+)   <->  to power supply 3V to 5,5V 
      INA209 PIN 10 (Vs+)  <->  to power supply 3V to 5,5V
      INA209 PIN 11 (GND)  <->  to power supply ground
      INA209 PIN 12 (SCL)  <->  ANALOG PIN 5 for ARDUINO UNO
      INA209 PIN 13 (SDA)  <->  ANALOG PIN 4 for ARDUINO UNO
      INA209 PIN 14 (A0)   <->  to power supply ground for set slave address to 0x40
      INA209 PIN 15 (A1)   <->  to power supply ground for set slave address to 0x40

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

// include the library code:
#include <Wire.h>        //I2C library 
#include <INA209.h>

INA209 ina209a(0x40);    // creation of an instance of the INA209 class called ina209a with I2C slave address = 0x40

void setup() {
  Serial.begin(9600);    //  setup serial
  Wire.begin();         // join i2c bus
  
  // ------------- General Configuration ---------  
  /* Configuration Register 00h (Read/Write)
     Default settings -> writeCfgReg(14751)
        Bus Voltage Range = 32V
        PGA (Shunt Voltage Only) = +/- 320mV
        BADC Bus ADC Resolution/Averaging = 12bit
        SADC Shunt ADC Resolution/Averaging = 12bit
        Operating Mode = Shunt and Bus, Continuos
  */
  ina209a.writeCfgReg(14751);       //  Default settings 
  
  // -------------- Calibration ------------------
  /* Calibration Register 16h (Read/Write)
    This register sets the current that corresponds to a full-scale drop across the shunt. 
    Full-scale range and the LSB of the current and power measurement depend on the value entered in this register.
    See the Programming the INA209 Power Measurement Engine section of INA209 datasheet.         
    In this example:
            Vbus_max = 32V
            Vshunt_max = 0,32V
            Rshunt = 0,1 Ohm
        ->  Max Expected Current = +/- 3A
            Current LSB = 100uA
            Calibration Reg = 4096
            Power LSB = 2mW
            Mas Power = 102,4W 
  */
  ina209a.writeCal(4096);
    
  // ---------- WARNING WATCHDOG REGISTERS -------
  /* These registers set warning limits that trigger flags in the Status Register 
     and activate the Warning pin.
  */
  ina209a.writeShuntVolPwrn(30000);        // set Shunt Voltage Positive Warning limit to 30000 (= 300mV) 
  ina209a.writeShuntVolNwrn(-500);         // set Shunt Voltage Negative Warning limit to -500 (= -5mV) 
  ina209a.writePowerWrn(10000);            // set Power Warning limit
  ina209a.writeBusOVwrn(52000);            // set Bus Over-Voltage Warning limit, if you want a limit of 26V you have to set 26000*2 = 52000    
  ina209a.writeBusUVwrn(10000);            // set Bus Under-Voltage Warning limit, if you want a limit of 5V you have to set 5000*2 = 10000 
  
  // ---------- OVER-LIMIT/CRITICAL WATCHDOG REGISTERS ----------------
  /* These registers set the over-limit and critical DAC limits that trigger flags to be set in the Status Register 
     and activate the Overlimit pin or the Critical pin.
  */
  ina209a.writePowerOL(12000);             // set Power Over-Limit
  ina209a.writeBusOVOL(60000);             // set Bus Over-Voltage Over-Limit, if you want a limit of 30V you have to set 30000*2 = 60000 (see the datasheet for other functions)
  ina209a.writeBusUVOL(6000);              // set Bus Under-Voltage Over-Limit, if you want a limit of 3V you have to set 3000*2 = 6000
  ina209a.writeCrShuntPV(51200);           // set Critical DAC+ Register (Critical Shunt Positive Voltage). No sign bit (sets a positive limit only). At full-scale range = 255mV; LSB = 1mV; 8-bit.
                                           // if you want a limit of +200mV you have to set 200*256 = 51200;
                                           // this register control GPIO PIN, see the datasheet.
  ina209a.writeCrShuntNV(51200);           // set Critical DAC– Register (Critical Shunt Negative Voltage). No sign bit (sets negative limit only). At full-scale range = –255mV; LSB = 1mV; 8-bit.
                                           // if you want a limit of -200mV you have to set 200*256 = 51200;
                                           // this register control DAC Comparator output filter, see the datasheet.
  }

void loop() {

  // ----------------------- CONFIG AND STATUS REGISTERS ---------------------------------- 
  Serial.println(" Configuration Register ");
  Serial.println(ina209a.readCfgReg());
  Serial.println(" Status Register ");
  Serial.println(ina209a.statusReg());
  Serial.println(" SMBus Alert ");
  Serial.println(ina209a.readSMBusCtrlReg());
 
  // ----------------------- DATA OUTPUT REGISTERS ---------------------------------------- 
  Serial.println(" Bus Voltage ");
  Serial.println(ina209a.busVol());            // returns integer value Bus Voltage measurement in mV (Bus Voltage LSB=4mV)
  Serial.println(" Shunt Voltage ");
  Serial.println(ina209a.shuntVol());          // returns integer value Shunt Voltage measurement data with LSB=10uV
  Serial.println(" Current ");
  Serial.println(ina209a.current());          // returns integer value of the Current flowing through the shunt resistor with LSB=100uA (= ShuntVoltage * CalReg / 4096) 
  Serial.println(" Power ");
  Serial.println(ina209a.power() << 1);       // returns integer value Power measurement in mW (in this example = register value * Power LSB=2mW)
 
  // ----------------------- PEAK-HOLD REGISTERS -----------------------------------------
  // Note: All peak-hold registers are cleared and reset to POR values by writing a '1' into the respective D0 bits.  
  ina209a.writeShuntVolPpk(0x01);            // clear Shunt Voltage Positive Peak Register
  ina209a.writeShuntVolNpk(0x01);
  ina209a.writeBusVolMaxPk(0x01);
  ina209a.writeBusVolMinPk(0x01);
  ina209a.writePowerPk(0x01);
  Serial.println(" Shunt Voltage Positive Peak  ");
  Serial.println(ina209a.readShuntVolPpk());            // mirrors highest voltage reading of the Shunt Voltage Register
  Serial.println(" Shunt Voltage Negative Peak ");
  Serial.println(ina209a.readShuntVolNpk());
  Serial.println(" Bus Voltage Maximum Peak  ");
  Serial.println(ina209a.readBusVolMaxPk());
  Serial.println(" Bus Voltage Minimum Peak  ");
  Serial.println(ina209a.readBusVolMinPk());
  Serial.println(" Power Peak ");
  Serial.println(ina209a.readPowerPk());      // returns integer value Power Peak in mW (in this example = register value * Power LSB=2mW)
  
  // ----------------------- WARNING WATCHDOG REGISTERS ----------------------------------
  // These registers set warning limits that trigger flags in the Status Register and activate the Warning pin.
  Serial.println(" Shunt Voltage Positive Warning ");
  Serial.println(ina209a.readShuntVolPwrn());
  Serial.println(" Shunt Voltage Negative Warning ");
  Serial.println(ina209a.readShuntVolNwrn());
  Serial.println(" Power Warning ");
  Serial.println(ina209a.readPowerWrn());
  Serial.println(" Bus Over-Voltage Warning ");
  Serial.println(ina209a.readBusOVwrn());
  Serial.println(" Bus Under-Voltage Warning ");
  Serial.println(ina209a.readBusUVwrn());
  
  // ----------------------- OVER-LIMIT/CRITICAL WATCHDOG REGISTERS ----------------------
  // These registers set the over-limit and critical DAC limits that trigger flags to be set in the Status Register and
  // activate the Overlimit pin or the Critical pin.
  Serial.println(" Power Over-Limit ");
  Serial.println(ina209a.readPowerOL());
  Serial.println(" Bus Over-Voltage Over-Limit ");
  Serial.println(ina209a.readBusOVOL());
  Serial.println(" Bus Under-Voltage Over-Limit ");
  Serial.println(ina209a.readBusUVOL());
  Serial.println(" Critical Shunt Positive Voltage ");
  Serial.println(ina209a.readCrShuntPV());
  Serial.println(" Critical Shunt Negative Voltage ");
  Serial.println(ina209a.readCrShuntNV());
   
}
