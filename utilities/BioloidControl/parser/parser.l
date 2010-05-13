%{
/**************************************************************************
    
    Copyright 2007, 2008 Rainer Jäkel <rainer.jaekel@googlemail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
**************************************************************************/

#include <stdio.h>
#include <cstring>
#include <iostream>
#include "../include/y.tab.h"
#include "../include/vars.h"
#include "../include/parser.h"
#include "../include/platform.h"

extern YYSTYPE yylval;

#define YY_INPUT(buf,result,max_size) \
    result = -1; \
    if ( global.parser.filename!= "") {CPlatform::readFromFile(global.parser.filename, (byte*)buf, result);global.parser.filename="";} \
    if (result <= 0) \
	{  \
     \
       strcpy(buf, "sleep(10);"); \
       result = YY_NULL; \
    }
%}

letter [A-Za-z]
digit  [0-9]
name   {letter}({letter}|{digit})*
op     "+"|"-"|"*"|"/"|"("|")"|";"|"<"|">"
ws     [ \t\n\r]+
other  .

%%
{ws}        ;
"if"        { return IF;}
"then"      { return THEN;}
"else"      { return ELSE;}
"while"     { return WHILE;}
"do"        { return DO; }
"print"     { return PRINT;}
"endif"     { return ENDIF;}
"endwhile"  { return ENDWHILE;}
">="        { return GE;}
"<="        { return LE;}
"!="        { return UE;}
"arml"      { return ARML; }
"armr"      { return ARMR; }
"legl"      { return LEGL; }
"legr"      { return LEGR; }
"pos"       { return POS; }
"moveto"    {return MOVETO; }
"torque"    { return TORQUE;}
"abs"       { return ABS;}
"sin"       { return SIN;}
"cos"       { return COS;}
"asin"       { return ASIN;}
"acos"       { return ACOS;}
"tan"       { return TAN;}
"atan"       { return ATAN;}
"sleep"     { return SLEEP;}
"sensor"    { return SENSOR;}
"stop"      { return STOP; }
"load"      { return LOAD; }
"wait"	    { return WAIT; }
"motion"    { return MOTION;}
"play"      { return PLAY;}
"run"       { return RUN;}
"script"    { return SCRIPT;}
"time"	    { return TIME; }
"collision" { return COLLISION; }
"hold"      { return HOLD; }
"sync"      { return SYNC; }
"endsync"   { return ENDSYNC; }
"get"       { return GET; }
"set"       { return SET; }

(("legl"|"legr"|"arml"|"armr")"."("x"|"y"|"z"|"rotx"|"roty"|"rotz")) {  
                yylval.ival =  global.parser.buffer.addString(yytext);
                return POSROT; }


\"({letter}|{digit}|{op}|\\|"/"|" "|"!"|"?"|":"|"_"|"."|","|"=")*\" {yytext[strlen(yytext)-1] = '\0'; yylval.ival = global.parser.buffer.addString(&yytext[1]); return STR;}
              
"\\n"        { return NEWLINE; }
{name}      { yylval.ival = global.parser.buffer.addVariable(yytext,0); return ID; }
{op}        { return yytext[0]; }

(([0-9]+(\.[0-9]*)?)|([0-9]*\.[0-9]+)) { yylval.dval = atof(yytext); yylval.ival = global.parser.buffer.addVariable(yytext, yylval.dval); return NUM;}

"=="        { return EQEQ; }
"="    	    { return ASSIGN; }
.           { return yytext[0]; }

%%

int yywrap()
{ 
    return 1; 
}



