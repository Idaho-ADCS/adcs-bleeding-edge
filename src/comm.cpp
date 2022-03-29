#include "comm.h"

/* TEScommand METHODS ======================================================= */

TEScommand::TEScommand()
{
	clear();
}

void TEScommand::addByte(uint8_t b)
{
	_data[_bytes_received] = b;
	_bytes_received++;

	if (_bytes_received >= COMMAND_LEN)
	{
		_full = true;
		_bytes_received = 0;
	}
	else
	{
		_full = false;
	}
}

bool TEScommand::isFull()
{
	return _full;
}

uint8_t TEScommand::getCommand()
{
	return _command;
}

bool TEScommand::checkCRC()
{
	CRC16 crcGen;
	crcGen.add(_data, COMMAND_LEN-2);

#if 0
	char crc_str[8];
	sprintf(crc_str, "0x%02x", crcGen.getCRC());
	SERCOM_USB.write("TES CRC: ");
	SERCOM_USB.write(crc_str);
	SERCOM_USB.write("\r\n");
#endif

	if (crcGen.getCRC() == _crc)
		return true;
	else
		return false;
}

void TEScommand::clear()
{
	for (int i = 0; i < COMMAND_LEN; i++)
		_data[i] = 0;

	_bytes_received = 0;
}

/* ADCSdata METHODS ========================================================= */

ADCSdata::ADCSdata()
{
	clear();
}

void ADCSdata::setStatus(uint8_t s)
{
	_status = s;
}

void ADCSdata::setINAdata(float v, float i)
{
	_voltage = floatToFixed(v);
	_current = (int8_t)(i / 10);
}

void ADCSdata::setIMUdata(float mx, float my, float mz, float gx, float gy, float gz)
{
	_magX = (int8_t)mx;
	_magY = (int8_t)my;
	_magZ = (int8_t)mz;

	_gyroX = floatToFixed(gx);
	_gyroY = floatToFixed(gy);
	_gyroZ = floatToFixed(gz);
}

void ADCSdata::computeCRC()
{
	CRC16 crcGen;
	crcGen.add(_data, PACKET_LEN-2);
	_crc = crcGen.getCRC();

#if 0
	char crc_str[8];
	sprintf(crc_str, "0x%02x", crcGen.getCRC());
	SERCOM_USB.write("CRC: ");
	SERCOM_USB.write(crc_str);
	SERCOM_USB.write("\r\n");
#endif
}

void ADCSdata::clear()
{
	for (int i = 0; i < PACKET_LEN; i++)
		_data[i] = 0;
}

void ADCSdata::send()
{
	SERCOM_UART.write((char*)_data, PACKET_LEN);
}

/* HELPER FUNCTIONS ========================================================= */

/**
 * @brief
 * Converts a floating-point number to a fixed-point number with 5 bits for the
 * integer part and 3 bits for the fraction part. Resulting data cannot be
 * properly interpreted until converted back into a float.
 * 
 * @param[in] f  Float to convert
 * 
 * @return Fixed-point conversion
 */
fixed5_3_t floatToFixed(float f)
{
    fixed5_3_t fix;
    fix = (uint8_t)(f * (1 << 3));
    return fix;
}

/**
 * @brief
 * Converts a fixed-point number with 5 integer bits and 3 fraction bits to a
 * floating-point number. Resulting data can be properly interpreted with no
 * extra conversions.
 * 
 * @param[in] fix  Fixed-point number to convert
 * 
 * @return Float conversion
 */
float fixedToFloat(fixed5_3_t fix)
{
    float f;
    f = ((float)fix) / (1 << 3);
    return f;
}