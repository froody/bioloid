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

#ifndef __CMD

#define __CMD

#include "types.h"
#include "vecmath.h"

// directories
#ifdef WIN32
 #define DIR_SEPARATOR "\\"
#else
 #define DIR_SEPARATOR "/"
#endif
#define DIR_SCRIPTS "scripts"
#define DIR_MOTIONS "motions"

#define CONSOLE_CMD_SEPARATOR ','
#define CONSOLE_PARAM_SEPARATOR ' '
#define CONSOLE_PARAM_LEN 255

#define LINESCOUNT 50 // max number of entries in log
#define INPUTCOUNT 50 // max number of inputs in history
#define CMD_LIST_LEN 10 // max number of cmds in one input
#define PARAMCOUNT 10 // max number of parameters per cmd

enum TEXT_TYPES
{
  TEXT_BOLD      =  10,
  TEXT_NORMAL    =   3,
  TEXT_ERROR     =  11,
  TEXT_EXECUTE   =  14,
  TEXT_COLLISION =   1,
  TEXT_DEBUG     =  12
};

/*! \brief Command classes

    Configuration, Movement or Hidden.
 */
enum COMMAND_CLASSES
{
  CT_CFG,
  CT_MOV,
  CT_HIDE
};


/*! \brief Command Types
 */
enum COMMAND_TYPES
{
  CMD_MOVETO,
  CMD_SET,
  CMD_GET,
  CMD_TORQUE,
  CMD_SPEED,
  CMD_CAPTURE,
  CMD_PLAY,
  CMD_INFO,
  CMD_HOLD,
  CMD_T,
  CMD_JOIN,
  CMD_LOAD,
  CMD_TEST,
  CMD_STOP,
  CMD_WAIT,
  CMD_RESYNC,
  CMD_LOOP,
  CMD_GRAB,
  CMD_ON,
  CMD_OFF,
  CMD_RON,
  CMD_ROFF,
  CMD_QUIT,
  CMD_M,
  CMD_SCRIPT,
  CMD_PARSE,
  CMD_S,
  CMD_L,
  CMD_DH,
  CMD_PLAYSHOW,
  CMD_CAM,
  CMD_SETPARAM,
  CMD_SETROBOT,
  CMD_SETPAUSE,
  CMD_HELP,
  CMD_LOADDH,
  CMD_RECHARGE,
  CMD_TCONTROL,
  CMD_SCONTROL,
  CMD_DEBUG,
  CMD_COLLISION,

  CMD_ARML,
  CMD_ARMR,
  CMD_LEGL,
  CMD_LEGR,
  CONSOLE_CMD_COUNT
};


/*! \brief Console Command
 */
typedef struct _Command
{
  const char*name;
  byte type;
  const char *helpText;
  int paramMin;
  const char*errorText;
} Command;

/*! \brief Console Commands

   Syntax: command, command-type, help text, min. parameters, parameter-error-text
 */
const Command console_cmds[CONSOLE_CMD_COUNT] = {
  {"MOVETO", CT_MOV, "MOVETO i x y z t: \tMove i'th Kinematic Chain To (x y z) In (t) Milliseconds", 4, "Example: moveto 1 180 0 0"},
  {"SET", CT_MOV, "SET id value delay: \tSet Targetangle Of Ax12 (id = [1..19, 255]) To (value) With Delay (delay)", 3, "Example: set 15 100 1000"},
  {"GET", CT_MOV, "GET id: \t\tGet Current Values Of Ax12 (id = [1..19, 255])", 1, "Example: get 15"},
  {"TORQUE", CT_CFG, "TORQUE id value: \tSet Max. Torque Of Ax12 (id = [1..19, 255]) To (value)", 2, "Example: torque 15 1023"},
  {"SPEED", CT_CFG, "SPEED id value: \tSet Max. Speed To Of Ax12 (id = [1..19, 255]) To (value)", 2, "Example: speed 15 1023"},
  {"CAPTURE", CT_CFG, "CAPTURE x: \t\tCapture Movement (Continous) And Store To Slot (x)", 1, "Example: capture 0"},
  {"PLAY", CT_MOV, "PLAY x1 .. xn d ipo: \tRun Captured Movement In Slots (x_i) With A\n\t\t\tDelay of (d) Milliseconds, Interpolation-Type\n\t\t\t(ipo): 0 Linear (js), 1 Spline (js), 2 Bezier (js),\n\t\t\t3 Custom Bezier (js), 4 PTP (js), 5 Sinoidal PTP (js), \n\t\t\t6 Linear (cs), 7 Spline (cs), 8 Bezier (cs),\n\t\t\t9 Custom Bezier (cs), 10 PTP (cs), 11 Sinoidal PTP (cs),", 3, "Example: play 0 1000 0"},
  {"INFO", CT_CFG, "INFO: \t\t\tShow Status Information", 0, ""},
  {"HOLD", CT_MOV, "HOLD: \t\t\tHold Current Position", 0, ""},
  {"T", CT_MOV, "T x: \t\t\tTorque on/off", 1, "Example: t on, t off"},
  {"JOIN", CT_CFG, "JOIN to x1 .. xn: \tJoin Motions in Slot (x1) To (xn) And Store\n\t\t\tResult in Slot (to)", 2, "Example: join 10 1 2 3"},
  {"LOAD", CT_CFG, "LOAD filename: \t\tLoad mtn-File", 1, "Example: load motion.mtn"},
  {"TEST", CT_MOV, "TEST: \t\t\tRun Test-Programm", 0, ""},
  {"STOP", CT_MOV, "STOP: \t\t\tStop Current MotionSequence", 0, ""},
  {"WAIT", CT_HIDE, "WAIT: \t\t\tWait Until Current MotionSequence Has Finished", 0, ""},
  {"RESYNC", CT_HIDE, "RESYNC: \t\tResync Program and Robot", 0, ""},
  {"LOOP", CT_MOV, "LOOP x1 .. xn d ipo c: \tRuns (c) times 'play x1 .. xn d ipo'"},
  {"GRAB", CT_CFG, "GRAB x: \t\tCapture Movement (Discrete) And Store To Slot (x)", 1, "Example: grab 0"},
  {"ON", CT_CFG, "ON: \t\t\tEnable Writing To Robot", 0, ""},
  {"OFF", CT_CFG, "OFF: \t\t\tDisable Writing To Robot", 0, ""},
  {"RON", CT_CFG, "RON: \t\t\tEnable Reading From Robot", 0, ""},
  {"ROFF", CT_CFG, "ROFF: \t\t\tDisable Reading From Robot", 0, ""},
  {"QUIT", CT_CFG, "QUIT: \t\t\tExit Program", 0, ""},
  {"M", CT_MOV, "M cmd parameter: \tExecute Movement Command", 1, "Example: m walk"},
  {"SCRIPT", CT_MOV, "SCRIPT filename: \tExecute Script Stored In File 'filename' ", 1, "Example: script stand.s"},
  {"PARSE", CT_HIDE, "PARSE: \t\t\tScript Editor", 0, ""},
  {"S", CT_CFG, "S id filename: \t\tSave Motion 'id' To File 'filename'", 2, "Example: s 0 test.mv"},
  {"L", CT_CFG, "L id filename: \t\tLoad Motion 'id' From File 'filename'", 2, "Example: l 0 test.mv"},
  {"DH", CT_CFG, "DH i index rz tz rx tx: Set DH Parameters Of i'th Kinematic Chain", 1, "Example: dh 1"},
  {"PLAYSHOW", CT_MOV, "PLAYSHOW: \t\tStarts Show Program, BE CAREFULL: HUMANOID_INVERT_HIP ONLY!", 0, ""},
  {"CAM", CT_CFG, "CAM x: \t\t\tCenter Camera On Frame x", 0, "Example: Center on 6th frame in kinematic chain legl: cam legl:6"},
  {"SETPARAM", CT_CFG, "SETPARAM id x: \t\tDisplay/Set User Parameter id to x", 0, ""},
  {"SETROBOT", CT_CFG, "SETROBOT id x: \t\tSet Parameter id On Robot (uC) to x", 2, "0: #Keyframes 1: Pause 2: Ipo 3: VMAX 4: BMAX"},
  {"SETPAUSE", CT_CFG, "SETPAUSE x: \t\tSets the interpolation pause to x milliseconds\n\t\t\tATTENTION: To change the interpolation pause on the robot, use setrobot 6 (x*10)", 1, "Example: setpause 80"},
  {"HELP", CT_CFG, "HELP: \t\t\tDisplay Help Text", 0, ""},
  {"LOADDH", CT_HIDE, "LOADDH file: \t\tLoad Model from file", 1, "Example: loaddh dh.xml"},
  {"RECHARGE", CT_CFG, "RECHARGE: \t\t\tRecharge Batteries of Bioloid", 0, ""},

  {"TCONTROL", CT_MOV, "TCONTROL x: \t\tTorque Control on/off", 1, "Example: tcontrol on, tcontrol off"},
  {"SCONTROL", CT_MOV, "SCONTROL x: \t\tSpeed Control on/off", 1, "Example: scontrol on, scontrol off"},
  {"DEBUG", CT_MOV, "DEBUG x: \t\tDisplay Debug Messages on/off", 1, "Example: debug on, debug off"},
  {"COLLISION", CT_MOV, "COLLISION x: \t\tCollision Avoidance on/off", 1, "Example: collision on, collision off"},

  {"ARML", CT_MOV, "ARML x y z t: \t\tMove Left Hand To (x y z) In (t) Milliseconds", 3, "Example: arml 180 0 0"},
  {"ARMR", CT_MOV, "ARMR x y z t: \t\tMove Right Hand To (x y z) In (t) Milliseconds", 3, "Example: armr 180 0 0"},
  {"LEGL", CT_MOV, "LEGL x y z a b c t: \tMove Left Leg To (x y z a b c) In (t) Milliseconds", 3, "Example: legl 180 0 0"},
  {"LEGR", CT_MOV, "LEGR x y z a b c t: \tMove Right Leg To (x y z a b c) In (t) Milliseconds", 3, "Example: legr 180 0 0"}
};

// help command: header strings
#define CMD_HELP_TEXT_HEADER "Console: Commands:\n------------------\n"
#define CMD_HELP_TEXT_CTMOV  "Movement:\n---------\n"
#define CMD_HELP_TEXT_CTCFG  "Config:\n-------\n"

/*! \brief Console Input Line
 */
typedef struct _Cmd
{
  byte type;
  char **param;
  byte param_count;
} Cmd;

/*! \brief Console

    Parses text input from console and runs commands.
 */
class CConsole
{
private:
  char buffer[LINESCOUNT][255]; ///< Text lines in log
  int bufferType[LINESCOUNT]; ///< Type of text lines in log, \see TEXT_TYPES
  char inputBuffer[INPUTCOUNT][255]; ///< Input line history
  char input[255]; ///< Current input line
  int currentInput; ///< Index of current input line in history
  int totalInput; ///< Number of input lines in history
  int currentLine; ///< Index of last text line in log
  int totalLines; ///< Number of text lines in log

  Cmd list[CMD_LIST_LEN]; ///< Command and parameter buffer
  int list_len; ///< Number of commands in buffer

public:
  CConsole();

  void init(); ///< Initialisation

  void refresh();
  void clearLines();

  /*! \brief Adds new text line to log. Color of text and background depends on type.
   */
  void addLine(char*line, int type = TEXT_NORMAL);

  /*! \brief Prints text in log on screen.
   */
  void printLog();

  /*! \brief Prints information about limbs on screen.

     \param positionAndOrientation Two times number of limbs vectors storing Position and Orientation of every limb.
   */
  void printInfo(CVec*positionAndOrientation);

  /*! \brief Prints user input on screen and adjusts cursorposition.
   */
  void setInput(char*userinput, int cursorposition = -1);

  /*! \brief Reads one character from user input.
   */
  int  readChar();

  /*! \brief Executes command at \a index in \a list.

      \param index Index of command.
      \param type Type of command.
      \param doWrite If set to true starts write cycle after execution of all commands is complete.
   */
  int  executeCmd(int index, int type, byte &doWrite);

  /*! \brief Executes buffered commands in a specific order (configuration commands first)
   */
  int  executeCmds();

  /*! \brief Creates command from array of strings
   */
  void createCmd(char **param, int param_count);

  /*! \brief Processes one word (text containing one command and several parameters)

     Creates a command object and adds it to the \a list.
   */
  void processWord(char *word);

  /*! \brief Splits one line of user input in several words (commands and parameters)
   */
  void processLine(char *line, int len);

  /*! \brief User input loop. Processes one line of text at a time.
   */
  void process();

};

#endif
