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
#include <stdlib.h>
#include <strings.h>
#include <iostream>
#include "../include/vars.h"
#include "../include/parser.h"
#include "../include/util.h"

void yyerror(char *msg);
void yyrestart(FILE *f);

int yyparse();
int yylex();
int yywrap();

int temp;
%}

%union { double dval; int ival; }
%token <ival> NUM
%token <ival> STR
   
%token GE
%token LE
%token UE
%token ID
%token EQEQ
%token ASSIGN
%token PRINT
%token IF
%token DO
%token NEWLINE
%token THEN
%token ELSE
%token WHILE
%token ENDIF
%token ENDWHILE
%token RUN
%token POSROT
%token SCRIPT
%token TORQUE
%token ABS
%token SIN
%token COS
%token ASIN
%token ACOS
%token TAN
%token ATAN
%token SLEEP
%token SENSOR
%token STOP
%token WAIT
%token MOTION
%token PLAY
%token LOAD
%token POS
%token ARML
%token ARMR
%token LEGL
%token LEGR
%token MOVETO
%token COLLISION
%token TIME
%token SYNC
%token ENDSYNC
%token HOLD
%token GET
%token SET
%type <ival> stmtlist
%type <ival> stmt
%type <ival> numlist
%type <ival> numl
%type <ival> expr
%type <ival> NEWLINE
%type <ival> ID
%token <ival> POSROT
   
%left '|' '&'
%left '+' '-' '<' '>' GE LE EQEQ 
%left '*' '/'
%left UMINUS

%start stmtlist

%%
stmtlist: stmtlist  stmt  { $$ = global.parser.buffer.addItem(';',$1,$2); }
| stmt  { $$= $1;}
;

numlist: expr { $$ = global.parser.buffer.addItem('k', $1, -1); }
| '[' expr ',' numl { $$ = global.parser.buffer.addItem('k', $2, $4); }
;
numl: expr ']' { $$ = global.parser.buffer.addItem('k', $1, -1); }
| expr ',' numl { $$ = global.parser.buffer.addItem('k',$1, $3); }

stmt: ID ASSIGN expr ';' { $$ = global.parser.buffer.addItem('=',$1,$3); }
| PRINT '(' expr ')' ';' { $$ = global.parser.buffer.addItem('P',$3,-1); }
| SET '(' expr ',' expr ',' expr ')' ';' { global.parser.buffer.addItem('v',$3,$5);
					$$ = global.parser.buffer.addItem('v',$7, -1); }
| PRINT '(' STR ')' ';'  { $$ =  global.parser.buffer.addItem('p',$3,-1);}
| PRINT '(' NEWLINE ')' ';' { $$ = global.parser.buffer.addItem('q',$3,-1);}
| SLEEP '(' expr ')' ';' { $$ = global.parser.buffer.addItem('s',$3,-1);}
| SCRIPT '(' STR ')' ';' { $$ = global.parser.buffer.addItem('c',$3,-1);}
| WAIT '(' ')' ';' { $$ = global.parser.buffer.addItem('w',-1,-1);}
| STOP '(' ')' ';' { $$ = global.parser.buffer.addItem('D',-1,-1);}
| HOLD '(' ')' ';' { $$ = global.parser.buffer.addItem('h',-1,-1);}
| MOTION '(' STR ',' expr ')' ';' { $$ = global.parser.buffer.addItem('M',$3,$5);}
| PLAY '(' expr ',' expr ',' expr ')' ';' { global.parser.buffer.addItem('m',$3,$5);
					    $$ = global.parser.buffer.addItem('m',$7,-1);}
| RUN '(' numlist ',' expr ',' expr ')' ';' { global.parser.buffer.addItem('n',$3,$5);
					    $$ = global.parser.buffer.addItem('n',$7,-1);}
| SYNC stmtlist ENDSYNC ';' { $$ = global.parser.buffer.addItem('y',$2, -1);}
| LOAD '(' expr ',' STR')' ';' { $$ = global.parser.buffer.addItem('l',$3,$5); }
| ARML '(' expr ',' expr ',' expr ',' expr ')' ';' { 
						global.parser.buffer.addItem('1',$3,$5);
                                            $$ = global.parser.buffer.addItem('1',$7,$9); }
| ARMR '(' expr ',' expr ',' expr ',' expr ')' ';' { global.parser.buffer.addItem('2',$3,$5);
                                            $$ = global.parser.buffer.addItem('2',$7,$9); }
| LEGL '(' expr ',' expr ',' expr ',' expr ',' expr ',' expr ',' expr  ')' ';' { 
					    global.parser.buffer.addItem('3',$3,$5);
                                            global.parser.buffer.addItem('3',$7,$9);
					    global.parser.buffer.addItem('3',$11,$13); 
					    $$ = global.parser.buffer.addItem('3',$15,-1);}
| LEGR '(' expr ',' expr ',' expr ',' expr ',' expr ',' expr ',' expr  ')' ';' { 
					    global.parser.buffer.addItem('4',$3,$5);
                                            global.parser.buffer.addItem('4',$7,$9);
					    global.parser.buffer.addItem('4',$11,$13);
					    $$ = global.parser.buffer.addItem('4',$15,-1); }
| MOVETO '(' expr ',' expr ',' expr ',' expr ',' expr ')' ';' { 
						global.parser.buffer.addItem('5',$3,$5);
                                            	global.parser.buffer.addItem('5',$7,$9);
						$$ = global.parser.buffer.addItem('5',$11,-1); }
| MOVETO '(' expr ',' expr ',' expr ',' expr ',' expr ',' expr ',' expr ',' expr  ')' ';' { 
					    global.parser.buffer.addItem('6',$3,$5);
                                            global.parser.buffer.addItem('6',$7,$9);
					    global.parser.buffer.addItem('6',$11,$13); 
					    $$ = global.parser.buffer.addItem('6',$15,$17);}
| IF  expr   THEN stmtlist  ELSE  stmtlist ENDIF ';' { global.parser.buffer.addItem('i',$2,-1);
					               $$ = global.parser.buffer.addItem('i',$4,$6); } 
| IF  expr   THEN stmtlist  ENDIF ';' { $$=global.parser.buffer.addItem('I',$2,$4); } 
| WHILE  expr  DO  stmtlist ENDWHILE ';' { $$=global.parser.buffer.addItem('d',$2,$4);}
   ;
expr : expr '+' expr  { $$ =global.parser.buffer.addItem('+',$1,$3);}
| expr '-' expr { $$ =global.parser.buffer.addItem('-',$1,$3); }
| '-' expr { $$ =global.parser.buffer.addItem('_',$2,-1); }
| expr '/' expr  { $$ =global.parser.buffer.addItem('/',$1,$3);}
| expr '*' expr { $$ = global.parser.buffer.addItem('*',$1,$3); }
| '(' expr ')' { $$ =  $2;}
| COLLISION '(' ')' { $$ = global.parser.buffer.addItem('C', -1,-1); }
| TIME '(' ')' { $$ = global.parser.buffer.addItem('t', -1,-1); }
| TORQUE '(' expr ')' { $$ = global.parser.buffer.addItem('T',$3,-1);}
| ABS '(' expr ')' { $$ = global.parser.buffer.addItem('A',$3,-1);}
| SIN '(' expr ')' { $$ = global.parser.buffer.addItem('B',$3,-1);}
| ASIN '(' expr ')' { $$ = global.parser.buffer.addItem('b',$3,-1);}
| COS '(' expr ')' { $$ = global.parser.buffer.addItem('X',$3,-1);}
| ACOS '(' expr ')' { $$ = global.parser.buffer.addItem('x',$3,-1);}
| TAN  '(' expr ')' { $$ = global.parser.buffer.addItem('Z',$3,-1);}
| ATAN '(' expr ',' expr ')' { $$ = global.parser.buffer.addItem('z',$3,$5);}
| SENSOR '(' STR ')' { $$ = global.parser.buffer.addItem('S',$3,-1);}
| POSROT { $$ = global.parser.buffer.addItem('O',$1,-1);}
| ID { $$ = global.parser.buffer.addItem('L',yylval.ival,-1);}
| NUM {$$ = global.parser.buffer.addItem('L',yylval.ival,-1);}
| expr '>' expr  { $$ =global.parser.buffer.addItem('>',$1,$3);}
| expr '<' expr  { $$ =global.parser.buffer.addItem('<',$1,$3);}
| expr EQEQ expr  { $$ = global.parser.buffer.addItem('e',$1,$3);}
| expr UE expr  { $$ = global.parser.buffer.addItem('u',$1,$3);}
| expr GE expr  { $$ = global.parser.buffer.addItem('g',$1,$3);}	
| expr LE expr  { $$ = global.parser.buffer.addItem('G',$1,$3);}	
| expr '|' '|' expr  { $$ = global.parser.buffer.addItem('|',$1,$4);}	
| expr '&' '&' expr  { $$ = global.parser.buffer.addItem('&',$1,$4);}
| POS '(' expr ',' STR ')'{ $$ = global.parser.buffer.addItem('9',$3,$5);}
| GET '(' expr ')' { $$ = global.parser.buffer.addItem('V',$3,-1); }
       ;

%%


void yyerror(char *msg)
{
 char s[255];
 sprintf(s, "%s\n", msg);
 CUtil::cout(s, TEXT_ERROR);
}









