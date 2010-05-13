Note: bison.exe and flex.exe are win32 versions of bison (gnu) and flex(gnu).

To compile the parser on a win32 platform do the following:
flex parser.l
bison -d parser.y

Rename parser_tab.c to y.tab.c
Rename parser_tab.h to y.tab.h

Copy lex.yy.c and y.tab.c to main/src
Copy y.tab.h to main/include