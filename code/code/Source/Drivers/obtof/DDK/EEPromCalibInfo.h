#pragma once

const int TOF_EEPROM_SN_ADDR = 0x0000;
const int TOF_EEPROM_SN_SIZE = 14;
const int TOF_EEPROM_CALIB_SIZE_ADDR = 0x0024;
const int TOF_EEPROM_DATA_LENGTH_SIZE = 4;
const int TOF_EEPROM_CALIB_MD5_ADDR = 0x0028;
const int TOF_EEPROM_MD5_SIZE = 16;
const int TOF_EEPROM_CALIB_DATA_ADDR = 0x0040;
const int TOF_EEPROM_CALIB_DATA_SIZE = 830;

const int TOF_EEPROM_D2C_SIZE_ADDR = 0x03ac;
const int TOF_EEPROM_D2C_MD5_ADDR = 0x03b0;
const int TOF_EEPROM_D2C_DATA_ADDR = 0x03c0;
const int TOF_EEPROM_D2C_DATA_SIZE = 2180;


void md5Calac(void *data, int dataSize, uint8_t *md5RawBuf) {
	uint8_t md5HexBuf[MD5_STRING_SIZE] = { 0 };
	md5::md5_t md5_;
	md5_.process((const void*)data, dataSize);
	md5_.finish(md5RawBuf);
	md5::sig_to_string((const void*)md5RawBuf, (char*)md5HexBuf, MD5_STRING_SIZE);
}

void checkReleaseBuffer(void *buf) {
	if (NULL != buf) {
		delete[] buf;
		buf = NULL;
	}
}