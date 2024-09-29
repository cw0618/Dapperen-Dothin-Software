#pragma once
#ifndef SERIALPORT_H
#define SERIALPORT_H


typedef struct SerialParams_
{
	const char* portName;
	int ioBuffSize;
	int baudRate;
	int byteSize;  /* Number of bits/byte, 4-8        */
	int parity;    /* 0-4=None,Odd,Even,Mark,Space    */
	int stopBits;  /* 0,1,2 = 1, 1.5, 2               */
	bool is_sync;
	      
	SerialParams_() {
		portName = "COM1";
		ioBuffSize = 1024;
		baudRate = 9600;
		byteSize = 8;
		parity = 0;
		stopBits = 0; 
		is_sync = true;
	}
}SerialParams;



class SerialPort
{
public:
	SerialPort();
	~SerialPort();

	bool open(SerialParams parms);
	void close();

	bool send(const char* cmd, unsigned long size); 
	bool recevive(char* recStr,unsigned long &realSize);

	bool isOpen() { return m_is_open; }

private:

	void* m_handle;
	bool m_sync;
	bool m_is_open;

private:
	bool createFile(const char* portName);
	bool setupComm(int iBuffSize,int oBuffSize);
	bool setTimeouts(int readInterval, int readMulti, int readConst, int writeMulti, int writeConst);
	bool setCommState(int baudRate, int byteSize, int parity, int stopBits);

	bool readFile(char* data, unsigned long size, unsigned long *realSize);

	bool writeFile(char* data, unsigned long size, unsigned long *realSize);



};

#endif