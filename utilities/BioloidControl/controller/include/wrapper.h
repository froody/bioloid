#ifndef __WRAPPER

#define __WRAPPER

#include "types.h"

// Comm-Protocol, CMD based
// ------------------------
#define COMM	255
enum 
{
	COMM_NONE,
	COMM_RFC, 	//read from console
	COMM_WTC,	//write to console
	COMM_RFAX,	//read from ax
	COMM_WTAX,	//write to ax
	COMM_TIME,	//get time
	COMM_RBS,	//read button state
	COMM_RLS,	//read led state
	COMM_WLS,	//write led state
	COMM_RSAX,	//run scheduled ax
	COMM_RFAAX,	//read from ALL ax
	COMM_WTAAX  // write to ALL ax
};

class CWrapper
{
	public:
	unsigned long readTimer();
	
    byte readButtonState(byte id);
	byte readLedState(byte id);
	void writeLedState(byte id, byte state);
	
	void readFromAx(byte id, byte addr, void *buffer, byte len);
	void writeToAx(byte id, byte addr, void *data, byte len, byte schedule);
	
	bool readFromAllAx(byte addr, void *buffer, int len);
	bool writeToAllAx(byte addr, void *data, int len);
	
	void runScheduledAx(void);
	
	int  readFromConsole(byte* buffer);
	void writeToConsole(byte* buffer, int len);
};

void *copymem(void * dest,const void *src, int count);
void readData(byte *bData, int len);
void writeData(byte *bData, int len);
bool readComm(byte comm, byte* param, int paramlen, byte* buffer, int len);
bool writeComm(byte comm, byte* param, int paramlen);


extern class CWrapper Wrapper;

#endif
