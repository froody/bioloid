#include <stdlib.h>
#include <string.h>

#include "..\include\wrapper.h"
#include "..\include\constants.h"
#include "..\include\timer.h"
#include "..\include\communication.h"

// global wrapper object
CWrapper Wrapper;

// simple bytewise memory copy
void *copymem(void * dest,const void *src, int count)
{
	char *tmp = (char *) dest, *s = (char *) src;

	while (count--)
		*tmp++ = *s++;

	return dest;
}

// read len bytes from serial
void readData(byte *bData, int len)
{
	while (len-- > 0)
	{
		*bData = RxD8();
		bData++;
	}
}

// write len bytes to serial
void writeData(byte *bData, int len)
{
	while (len-- > 0)
	{
		TxD8(*bData++);
	}
}

// 
unsigned long CWrapper::readTimer()
{
	unsigned long time;
	TxD8(COMM);TxD8(COMM_TIME);
	readData((byte*)time, 4);
	//readComm(COMM_TIME,0,0,(byte*)&time, 4);
}
	
byte CWrapper::readButtonState(byte id)
{
	TxD8(COMM);TxD8(COMM_RBS);
	TxD8(id);
	return RxD8();
	/*byte tmp;
	readComm(COMM_RBS, &id, 1, &tmp, 1);
	return tmp;*/
}
	
byte CWrapper::readLedState(byte id)
{
	TxD8(COMM);TxD8(COMM_RLS);
	TxD8(id);
	return RxD8();
	/*byte tmp;
	readComm(COMM_RLS, &id, 1, &tmp, 1);
	return tmp;*/
}
	
void CWrapper::writeLedState(byte id, byte state)
{
	TxD8(COMM);TxD8(COMM_WLS);
	TxD8(id);
	TxD8(state);
	/*byte tmp[2];
	tmp[0]=id;
	tmp[1]=state;
	writeComm(COMM_WLS, tmp, 2);*/
}

	
void CWrapper::readFromAx(byte id, byte addr, void *buffer, byte len)
{
	TxD8(COMM);TxD8(COMM_RFAX);
	TxD8(id);
	TxD8(addr);
	TxD8(len);
	readData((byte*)buffer, len);
	/*byte tmp[2];
	tmp[0] = id;
	tmp[1] = addr;
	readComm(COMM_RFAX, tmp, 2, buffer, len);*/
}
	
void CWrapper::writeToAx(byte id, byte addr, void *data, byte len, byte schedule)
{
	TxD8(COMM);TxD8(COMM_WTAX);
	TxD8(id);
	TxD8(addr);
	TxD8(len);
	TxD8(schedule);
	writeData((byte*)data, len);
	
	/*byte buffer[255];
	buffer[0] = id;
	buffer[1] = addr;
	buffer[2] = schedule;
	buffer[3] = len;
	copymem(&buffer[4], data, len); */
}
	
void CWrapper::runScheduledAx(void)
{
	//writeComm(COMM_RSAX, 0, 0);
	TxD8(COMM);TxD8(COMM_RSAX);
}
	
int CWrapper::readFromConsole(byte* buffer)
{
	char tmp;
	//cin.get(tmp);
	return (byte)tmp;
}

void CWrapper::writeToConsole(byte* buffer, int len)
{
	//cout.put((char) character);
}
