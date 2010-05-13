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


#ifndef _PARSER
#define _PARSER

/*! \brief Variable used in scripting language
 */
class CVariable
{
public:
  char name[16];
  double value;
};

/*! \brief Node item used in parser tree
 */
class CParserItem
{
public:
  char id; ///< Type of Node
  int first; ///< Left Successor
  int second; ///< Right Successor
};

// parser data
#define STRINGS_MAX 1000 // max number of strings
#define STRINGS_LEN 255 // max size of each string
#define VARIABLES_MAX 1000 // max number of variables
#define ITEMS_MAX 2000 // max number of nodes in parse tree

/*! \brief Image of inner parser state
 */
class CParserBufferTimestamp
{
public:
  int strings_count;
  int variables_count;
  int items_count;
};

/*! \brief Parser Tree, Variable and String Storage
 */
class CParserBuffer
{
private:
  double execute(int id);

public:
  char strings_buffer[STRINGS_MAX][STRINGS_LEN];
  int strings_count;

  CVariable variables_buffer[VARIABLES_MAX];
  int variables_count;

  CParserItem items[ITEMS_MAX];
  int items_count;

  int  addVariable(char *name, double val);
  int  addString(char *text);
  int  addItem(char id, int first, int second);

  void save(CParserBufferTimestamp &timestamp); ///< Stores current state of parser
  void restore(CParserBufferTimestamp &timestamp); ///< Restores state of parser

  void execute();

};

/*! \brief Parser

   Stores data associated with the parsing process, runs the parser and holds
   implementations of all parser functions.
 */
class CParser
{
public:
  char*filename; ///< Current file to parse, will be used by lexx parser
  char*params; ///< Parameters associated with current file

  CParserBuffer buffer; ///< Memory storage for lexx and yacc parser

  /*! \brief Parses and executes a scripting file
   */
  int    parseFile(char*filename, float *argv = NULL, int argc = 0);

  double print(char*text);
  double torque(double number);
  double sleep(double number);
  double play(double id, double delay, double kid);
  double run(int *ids, int len, int delay, int ipo);
  double motion(char*name, double count);
  double sensor(char*name);
  double fabs(double number);
  double get(int id);
  double set(int id, double angle, int time);
  double moveto(int id, double x, double y, double z, int time);
  double moveto(int id, double x, double y, double z, double rotx, double roty, double rotz, int time);

  double pos(int id, char*param);

  double script(char*filename, char*params);
  double load(double id, char*params);
  double rotation(char*kin, char*xyz);
  double collision();
  double time();
  double sync_start();
  double sync_end();
  double hold();
  double stop();
  double wait();

  void   finish();

};

int yyparse();
void yyrestart(FILE *f);

#endif
