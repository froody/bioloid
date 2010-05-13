#ifndef __CMD

#define __CMD

#include "types.h"

#define CMD_LIST_LEN 10
#define PARAMCOUNT 10

typedef struct _Cmd
{
	byte type;
	char **param;
	byte  param_count;
} Cmd;

class Console
{
	private:
	// linked list better?
	Cmd  list[CMD_LIST_LEN];
	byte list_len;
	
	public:
	Console();
	
	void executeCmd(int index, int type);
	void executeCmds();
	void createCmd(char **param, int param_count);
	void processWord(char *word);
	void processLine(char *line, int len);
};

#endif
