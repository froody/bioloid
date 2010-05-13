/**************************************************************************

    Copyright 2007, 2008 Rainer J�kel <rainer.jaekel@googlemail.com>

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

#include <iostream>
#include <cstring>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "../include/platform.h"
#include "../include/cmd.h"
#include "../include/robot.h"
#include "../include/util.h"
#include "../include/vecmath.h"
#include "../include/motion.h"
#include "../include/parser.h"
#include "../include/commands.h"
#include "../include/interpolation.h"
#include "../include/vars.h"
#include "../include/test.h"

#ifndef WIN32
 #define _SIMPLECONSOLE

 #ifndef SIMPLECONSOLE
  #include <curses.h>
  #include <signal.h>
  #define ESCDELAY 0
void resizeHandler(int);

WINDOW *winInput = NULL;
WINDOW *winLog   = NULL;
WINDOW *winInfo  = NULL;

  #define TEXT_COLORS_COUNT 7
const int TEXT_COLORS[TEXT_COLORS_COUNT][2] = {
  {COLOR_RED, COLOR_WHITE},
  {COLOR_YELLOW, COLOR_WHITE},
  {COLOR_BLACK, COLOR_WHITE},
  {COLOR_WHITE, COLOR_BLUE},
  {COLOR_RED, COLOR_WHITE},
  {COLOR_WHITE, COLOR_RED},
  {COLOR_BLACK, COLOR_WHITE}
};
 #endif

 #define ATTRON(win, x) if(x >= TEXT_BOLD) {wattron(win, COLOR_PAIR(x - TEXT_BOLD) | A_BOLD);} else {wattron(win, COLOR_PAIR(x) );}
 #define ATTROFF(win, x) if(x >= TEXT_BOLD) {wattroff(win, COLOR_PAIR(x - TEXT_BOLD) | A_BOLD);} else {wattroff(win, COLOR_PAIR(x) );}

#endif

#ifdef WIN32
 #define SIMPLECONSOLE

 #include <windows.h>
 #ifndef SIMPLECONSOLE
  #include <conio.h>
 #endif
 #define _CUSTOMCONSOLE
#endif

#define ROUND(x) (float)( (int)(x*100) )/100.0
#define KINCOUNT (global.robotInput.kinematicChains.length)

using namespace std;

// helper function: get index of command by name
int getIndexByName(char *name) {
  for(int i = 0; i < CONSOLE_CMD_COUNT; i++) {
    if(strcasecmp(console_cmds[i].name, name) == 0) {
      return i;
    }
  }
  return -1;
}

// waits until return key is pressed
void CUtil::waitForReturn() {
  char bChar[255];

  // only one cin handler at a time
  ML(MUTEXCIN);

#ifdef SIMPLECONSOLE
  std::cin.getline(bChar, 255);
#else
 #ifdef WIN32
  std::cin.getline(bChar, 255);
 #else
  int ch;
  do {
    ch = wgetch(winInput);
  } while(ch != '\n' && ch != KEY_ENTER);
 #endif

#endif
  MU(MUTEXCIN);
}

// init console object
CConsole::CConsole() {
  list_len = currentInput = totalInput = currentLine = totalLines = 0;
}

// clear console buffer
void CConsole::clearLines() {
  ML(MUTEXCOUT);
  currentLine = 0;
  totalLines = 0;
  MU(MUTEXCOUT);
}

// read character from std input
int CConsole::readChar() {
  int result = 0;

  ML(MUTEXCIN);

#ifdef SIMPLECONSOLE
  char buffer[255];
  std::cin.getline(buffer, 255);
  if( (int)buffer[0] == 0) {
    result = (int)'\n';
  }
  else {
    result = (int)buffer[0];
  }
#else
 #ifdef WIN32
  result = (int)getch();
 #else
  result = wgetch(winInput);
 #endif
#endif

  MU(MUTEXCIN);
  return result;
}

/* set content of input window and display cursor
   example: user typed help -> display

   enter cmd: help|

 */
void CConsole::setInput(char*str, int cursor) {
#ifndef SIMPLECONSOLE
  if(!global.initialisationComplete) {
    return;
  }

  // no further console updates during operation
  ML(MUTEXCOUT);

  // store input
  strcpy(input, str);

 #ifdef WIN32
  // not implemented yet
 #else
  // erase input window & display "enter cmd: " + supplied command
  werase(winInput);
  wmove(winInput, 1, 1);
  waddstr(winInput, "Enter Cmd: ");

  // add command char per char, highlight cursor
  for(int i=0; i<strlen(input); i++) {
    if(i == cursor) {
      waddch(winInput, input[i] | A_STANDOUT);
    }
    else {
      waddch(winInput, input[i]);
    }
  }

  // cursor at the end of input -> add dummy char to display cursor
  if( (cursor == -1) || (cursor == strlen(input) ) ) {
    waddch(winInput, ' ' | A_STANDOUT);
  }

  // make changes visible
  wrefresh(winInput);
 #endif
  MU(MUTEXCOUT);
#endif
}

// add line to log
void CConsole::addLine(char*str, int type) {
#ifdef SIMPLECONSOLE
  if(global.showDebugMessages || (type != TEXT_DEBUG) ) {
    printf(str);
  }
#endif

  // no further console updates during operation
  ML(MUTEXCOUT);

  // log is a circular list, currentline = index of first free spot
  strcpy(buffer[currentLine], str);
  bufferType[currentLine] = type;

  currentLine = (currentLine + 1);
  if(currentLine >= LINESCOUNT) {
    currentLine -= LINESCOUNT;
  }

  if(totalLines < LINESCOUNT) {
    totalLines++;
  }

  MU(MUTEXCOUT);
}

// print hand and feet positions on screen
void CConsole::printInfo(CVec*positionAndOrientation) {
#ifndef SIMPLECONSOLE
  if(!global.initialisationComplete) {
    return;
  }

  ML(MUTEXCOUT);
 #ifdef WIN32
  // not implemented yet
 #else
  werase(winInfo);
  wmove(winInfo, 0, 0);

  char str[255];

  // init
  CVec limb[2];

  for(int i=0; i<global.robotInput.kinematicChains.length; i++) {
    if(positionAndOrientation != NULL) {
      limb[0] = positionAndOrientation[i*2];
      limb[1] = positionAndOrientation[i*2 + 1];
    }

    float rotx, roty, rotz;
    rotx = roty = rotz = 0.0;
    // tweak
    switch(global.robotInput.id)
    {
    case ROBOT_HUMANOID:
    case ROBOT_HUMANOID_INVERSE_HIP:
      rotx = 90.0;
      roty = 90.0;
      break;
    }
    sprintf(str, "Limb %d: (%7.2f, %7.2f, %7.2f) (%3.0f, %3.0f, %3.0f)                 \n", i+1, limb[0].x, limb[0].y, limb[0].z,
            (-rotx + (180.0 / M_PI) * limb[1].x),
            (-roty + (180.0 / M_PI)  * limb[1].y),
            (-rotz + (180.0 / M_PI) * limb[1].z) );
    waddstr(winInfo, str);
  }

  wrefresh(winInfo);
 #endif
  MU(MUTEXCOUT);
#endif
}

// print log (= text added per addline)
void CConsole::printLog()
{
#ifndef SIMPLECONSOLE
  if(!global.initialisationComplete) {
    return;
  }

  // no further console updates during operation
  ML(MUTEXCOUT);
  int i = 0;
  int len = 0;
  int c = (currentLine - totalLines);
  if(c < 0) {
    c += LINESCOUNT;
  }

 #ifdef WIN32
  CPlatform::clearScreen();
  printf("Enter Cmd: %s\n\n", input);

  printf("Log:\n----\n");

  while(i++ < totalLines)
  {
    if(global.showDebugMessages || (bufferType[c] != TEXT_DEBUG) ) {
      printf(buffer[c]);
    }

    c++;
    if(c >= LINESCOUNT) {
      c -= LINESCOUNT;
    }
  }

  HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
  COORD coordScreen = { 11 + strlen(input), 0 };
  SetConsoleCursorPosition(hConsole, coordScreen);
 #endif

 #ifndef WIN32

  // hide cursor (input window recvs input )
  curs_set(0);
  werase(winLog);
  wmove(winLog, 0, 0);

  while(i++ < totalLines)
  {
    if(global.showDebugMessages || (bufferType[c] != TEXT_DEBUG) ) {
      ATTRON(winLog, bufferType[c]);

      waddstr(winLog, buffer[c]);

      // cmd type execute -> fill line with blue characters
      if(bufferType[c] == TEXT_EXECUTE) {
        int nh, nw;
        getmaxyx(stdscr, nh, nw);

        for(int i=0; i<nw - strlen(buffer[c]); i++) {
          waddch(winLog, ' ');
        }

        waddstr(winLog, "\n");
      }

      ATTROFF(winLog, bufferType[c]);
    }

    c++;
    if(c >= LINESCOUNT) {
      c -= LINESCOUNT;
    }
  }

  wmove(winInput, 1, 1 + 11 + strlen(input) );

  wrefresh(winLog);

 #endif

  MU(MUTEXCOUT);
#endif
}

#ifndef WIN32
 #ifndef SIMPLECONSOLE
// react on change in console size
bool isSizing = false;
void resizeHandler(int sig)
{
  // console is updated asynchronously, prevent it while changing screen size
  ML(MUTEXCOUT);

  isSizing = true;

  int nh, nw;
  getmaxyx(stdscr, nh, nw);  /* get the new screen size */

  // check for minimal size
  if( (nw < 8) || (nh < 8) ) {
    MU(MUTEXCOUT);
    endwin();
    initscr();
    return;
  }
  endwin();
  initscr();

  // create windows with new size or resize and move'em
  if(winInput == NULL) {
    winInput   = newwin(3, nw, nh-3, 0);
    winLog     = newwin(nh-3-KINCOUNT, nw, 0, 0);
    winInfo    = newwin(KINCOUNT, nw, nh-3-KINCOUNT, 0);

    scrollok(winLog, TRUE);
    keypad(winInput, TRUE);
    bkgd(COLOR_PAIR(TEXT_NORMAL) | A_BOLD);
    wbkgd(winInput, COLOR_PAIR(7) );
    wbkgd(winLog,   COLOR_PAIR(3) );
    wbkgd(winInfo,  COLOR_PAIR(6) | A_BOLD);
  }
  else {
    wresize(winInput, 3, nw);
    wresize(winLog, nh-3-KINCOUNT, nw);
    wresize(winInfo, KINCOUNT, nw);
    mvwin(winInput, nh-3, 0);
    mvwin(winLog, 0, 0);
    mvwin(winInfo, nh-3-KINCOUNT, 0);
  }

  isSizing = false;
  MU(MUTEXCOUT);
  global.console.refresh();

}

 #endif
#endif

// refresh display (mainly after resizing the console)
void CConsole::refresh()
{
  printLog();
  // dummy position information (0) during resizing
  printInfo(NULL);
  setInput(input);
}

void CConsole::init() {
  // current position in history
  currentInput = 0;
  // total commands in history <= INPUTCOUNT
  totalInput = 0;

#ifndef WIN32
 #ifndef SIMPLECONSOLE
  initscr();
  cbreak();
  noecho();

  // init console colorsystem
  start_color();

  if(has_colors() ) {
    start_color();

    for(int i=0; i< TEXT_COLORS_COUNT; i++) {
      init_pair(i+1, TEXT_COLORS[i][0], TEXT_COLORS[i][1]);
    }
  }
  // init windows
  resizeHandler(0);

  //react on screen size changes
  signal(SIGWINCH, resizeHandler);
 #endif
  // clear input
  setInput( (char *)"");
#endif
}

// console thread function, processes input
void CConsole::process()
{
  char buffer[255];

  // cursor position
  int curpos = 0;
  // input length
  int len = 0;
  // used to cycle through history (historyindex = currentInput - cInput )
  int cInput = 0;

  // misc
  char str[255];
  char c;
  int j;

  // clear everything
  clearLines();

  // init is now complete = ready to display console information
  global.initialisationComplete = true;
  CUtil::cout("Initialisation complete.\n", TEXT_NORMAL);

#ifdef SIMPLECONSOLE
  while(true)
  {
    CPlatform::sleep(200);

    CUtil::cout("\nEnter Cmd:\n");

    ML(MUTEXCIN);
    cin.getline(buffer, 255);
    MU(MUTEXCIN);

    cout << "\nCmd: " << buffer << endl;

    global.console.processLine(buffer, strlen(buffer)+1);
    if( (bool)global.console.executeCmds() == false) {
      break;
    }
  }
#else

  refresh();
  // main console input loop
  while(1)
  {
    CPlatform::sleep(1);

 #ifdef WIN32
    // read input
    ML(MUTEXCIN);
    cin.getline(buffer, 255);
    MU(MUTEXCIN);

    // write commands to log, clear input window
    sprintf(str, "Executing: %s\n", buffer);
    setInput("");
    CUtil::cout(str, TEXT_EXECUTE);
    curpos = 0;

    // process commands
    processLine(buffer, strlen(buffer)+1);
    if( (bool)executeCmds() == false) {
      goto cmd_end;
    }

    // refresh console
    CPlatform::clearScreen();
    printLog();
 #endif

 #ifndef WIN32

    // read input (one character at a time)
    int ch;
    ML(MUTEXCIN);
    ch = wgetch(winInput);
    MU(MUTEXCIN);

    switch(ch)
    {
      // escape -> emergency stop
    case 27:
      global.motion.stop();
      CUtil::cout("Emergency Stop.\n", TEXT_ERROR);
      break;

      // left arrow -> cursorposition - 1 in input text
    case KEY_LEFT:
      if(curpos > 0) {
        curpos--;
      }
      setInput(buffer, curpos);
      break;

      // right arrow -> cursorposition + 1 in input text
    case KEY_RIGHT:
      if(curpos < strlen(buffer) ) {
        curpos++;
      }
      setInput(buffer, curpos);
      break;

      // delete key -> delete character at cursor position
    case KEY_DC:
      if(curpos < len) {
        // shift left characters
        for(int k=curpos; k<len-1; k++) {
          buffer[k] = buffer[k+1];
        }

        if(len > 0) {
          len--;
        }

        buffer[len]   = 0;

        setInput(buffer, curpos);
      }
      break;

      // backspace key -> delete character at cursorposition-1
    case KEY_BACKSPACE:
      if(curpos > 0) {
        // shift left characters
        for(int k=curpos-1; k<len-1; k++) {
          buffer[k] = buffer[k+1];
        }

        if(len > 0) {
          len--;
        }
        buffer[len]   = 0;
        curpos--;
        setInput(buffer, curpos);
      }
      break;

      // up arrow -> go backward in input history
    case KEY_UP:
      if(cInput < totalInput) {
        cInput++;
      }

      j = currentInput - cInput;
      if(j < 0) {
        j += INPUTCOUNT;
      }
      strcpy(buffer, inputBuffer[j]);

      curpos  = len = strlen(buffer);
      setInput(buffer, curpos);
      break;

      // down arrow -> go forward in input history
    case KEY_DOWN:
      if(cInput > 0) {
        cInput--;
      }

      j = currentInput - cInput;
      if(j < 0) {
        j += INPUTCOUNT;
      }

      strcpy(buffer, inputBuffer[j]);
      curpos = len = strlen(buffer);
      setInput(buffer, curpos);
      break;

      // return/newsline = user pressed return button -> execute command
    case '\n':
    case KEY_ENTER:
      buffer[len] = 0;
      sprintf(str, "\nExecuting: %s", buffer);
      CUtil::cout(str, TEXT_EXECUTE);
      strcpy(inputBuffer[currentInput], buffer);

      // add command to history
      if(currentInput < INPUTCOUNT-1) {
        currentInput++;
      }
      else {
        currentInput = 0;
      }

      if(totalInput < INPUTCOUNT) {
        totalInput++;
      }

      // clear current history place =
      // you will get a fresh line pressing the down button (cycling to end of history)
      inputBuffer[currentInput][0] = 0;
      setInput( (char *)"");
      curpos = 0;
      len = 0;
      cInput = 0;

      // process commands
      processLine(buffer, strlen(buffer)+1);

      // execute commands
      if( (bool)executeCmds() == false) {
        goto cmd_end;
      }

      break;
    }
    // user entered text -> update input text
    if( (ch >= ' ') && (ch <= '~') ) {
      // insert character at cursor position

      // shift right (characters right of cursor position)
      for(int k=len; k>curpos; k--) {
        buffer[k] = buffer[k-1];
      }

      buffer[curpos] = (char)ch;
      len++;

      curpos++;

      buffer[len]   = 0;
      setInput(buffer, curpos);
      printLog();
    }

 #endif
  }

cmd_end:
  global.initialisationComplete = false;

 #ifndef WIN32
  // clean windows
  delwin(winInput);
  delwin(winLog);
  delwin(winInfo);
  clrtoeol();
  refresh();
  endwin();
 #endif
#endif
}

// executes a single command
int CConsole::executeCmd(int index, int type, byte &doWrite)
{
  /* alotta variablas */
  int i, j, k, len, id, oldInputState;
  float params[100];
  float angles[AX12_COUNT];
  char buffer[1024];

  int result = true;

  // check for parameter count, display error message on fault, 255 = -1 in byte
  if( (list[index].type != 255) && (list[index].type < CONSOLE_CMD_COUNT) ) {
    if(list[index].param_count <= console_cmds[list[index].type].paramMin) {
      if(console_cmds[list[index].type].paramMin == 1) {
        sprintf(buffer, "%s expects at least 1 parameter.\n", list[index].param[0]);
      }
      else {
        sprintf(buffer, "%s expects at least %d parameters.\n", list[index].param[0], console_cmds[list[index].type].paramMin);
      }
      CUtil::cout(buffer);

      if(console_cmds[list[index].type].errorText != "") {
        CUtil::cout(console_cmds[list[index].type].errorText);
      }

      return result;
    }
  }

  // big big command switch
  switch(list[index].type)
  {
    // quit program
  case CMD_QUIT:
    return false;

  case CMD_LOADDH:
    // todo, load dh files online
    break;

    // resync serial communication
  case CMD_RESYNC:
    CPlatform::initSerial(global.config.getString("Software.ComPort") );
    CPlatform::sleep(100);
    ML(MUTEXCOMM);
    CUtil::resync();
    MU(MUTEXCOMM);
    break;

    // simple script editor: finish with ! + return in a newline
    // script is stored in tempscript.s and executed immediately
    // TODO: doesnt work on linux
  case CMD_PARSE:
    char charBuffer[255];
    int charLen;
    charLen = 0;
    CUtil::cout("Enter Code, finish with RETURN + '!' :\n");
    CUtil::cout("-----------------------------------\n");
    while(charLen < 255)
    {
      charBuffer[charLen++] = (char)readChar();

      if( (charLen >= 2) &&
          (charBuffer[charLen-2] == '\n') &&
          (charBuffer[charLen-1] == '!') ) {
        charLen -= 1;
        break;
      }
    }
    CPlatform::writeToFile("tempscript.s", (byte*)charBuffer, charLen);
    global.parser.parseFile( (char *)"tempscript.s");
    break;

  case CMD_SETPAUSE:
    if(list[index].param_count >= 2) {
      id = (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
      if(id >= 0) {
        global.motion.ipoPause = id;
        global.motion.capturePause = id;
        sprintf(buffer, "Interpolation and Capture pause set to %d milliseconds.\n", id);
        CUtil::cout(buffer);
      }
    }
    break;

    // set walking parameters (for quick testing)
  case CMD_SETPARAM:
    if(list[index].param_count >= 3) {
      id = (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
      if( (id >= 0) && (id < CONSOLE_PARAMETER_COUNT) ) {
        global.parameters[id] = (float)CUtil::strtodouble(list[index].param[2], strlen(list[index].param[2]) );
      }

    }
    {
      CUtil::cout("Walking Parameters:\n-------------------");
      for(id = 0; id < CONSOLE_PARAMETER_COUNT; id++) {
        if(id % 4 == 0) {
          CUtil::cout("\n");
        }
        sprintf(buffer, "%d: %f\t", id, global.parameters[id]);
        CUtil::cout(buffer);
      }
      CUtil::cout("\n");
    }
    break;

    // center camera on left|right foot or center of body
  case CMD_CAM:
    if(list[index].param_count >= 2) {
      global.viewType = global.robotInput.getFrameByName(list[index].param[1]);
    }
    break;

    // parse and execute script
  case CMD_SCRIPT:
    sprintf(buffer, "%s%s%s%s", global.programPath, DIR_SCRIPTS, DIR_SEPARATOR, list[index].param[1]);

    id = list[index].param_count-1 < 15 ? list[index].param_count-2 : 15;
    {
      for(i=0; i<id; i++) {
        params[i] = CUtil::strtodouble(list[index].param[2+i], strlen(list[index].param[2+i]) );
      }
    }

    global.parser.parseFile(buffer, params, id);
    break;

    // turn writing position data to robot on/off
  case CMD_ON:
  case CMD_OFF:
    global.writeToRobot = (list[index].type == CMD_ON);
    if(global.writeToRobot) {
      CUtil::cout("Writing To Robot Is Now Active.\n");
    }
    else {
      CUtil::cout("Writing To Robot Is Now Disabled.\n");
    }
    break;

    // turn reading position data from robot on/off
  case CMD_RON:
  case CMD_ROFF:
    global.readFromRobot = (list[index].type == CMD_RON);

    for(i=0; i<AX12_COUNT; i++) {
      global.robotWrapper.Ax12s[i].useCurrentAngle = global.readFromRobot;
    }

    if(global.readFromRobot) {
      CUtil::cout("Reading From Robot Is Now Active.\n");
    }
    else {
      CUtil::cout("Reading From Robot Is Now Disabled.\n");
    }
    break;

    // show and modify dh parameters
  case CMD_DH:
    i = (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
    i--;
    if( (i >= 0) && (i<= global.robotInput.kinematicChains.length) ) {

      sprintf(buffer, "DH Parameters of %s:\n", global.robotInput.kinematicChains.chain[i].name);
      CUtil::cout(buffer);
      CUtil::cout("--------------------\n");
      CUtil::cout("Syntax: dh i id rotz transz rotx transx\n\n");

      CUtil::cout("id\trotz\ttransz\trotx\ttransx\n");
      ML(MUTEXCOMM);
      if(list[index].param_count >= 6) {
        id = (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
        params[0] =CUtil::strtodouble(list[index].param[2], strlen(list[index].param[2]) );
        params[1] =CUtil::strtodouble(list[index].param[3], strlen(list[index].param[3]) );
        params[2] =CUtil::strtodouble(list[index].param[4], strlen(list[index].param[4]) );
        params[3] =CUtil::strtodouble(list[index].param[5], strlen(list[index].param[5]) );
        if(id < global.robotInput.kinematicChains.chain[i].length) {
          global.robotInput.kinematicChains.chain[i].dhParameters[id].set(params[0]*M_PI/180.0, params[1], params[2]*M_PI/180.0, params[3]);
        }
      }
      global.robotInput.updateDhParameters();
      global.robotInput.calcForwardKinematics();

      for(j=0; j<global.robotInput.kinematicChains.chain[i].length; j++) {
        sprintf(buffer, "%d\t%g\t%g\t%g\t%g\n", j,
                global.robotInput.kinematicChains.chain[i].dhParameters[j].rot_z*180.0/M_PI,
                global.robotInput.kinematicChains.chain[i].dhParameters[j].trans_z,
                global.robotInput.kinematicChains.chain[i].dhParameters[j].rot_x*180.0/M_PI,
                global.robotInput.kinematicChains.chain[i].dhParameters[j].trans_x);
        CUtil::cout(buffer);
      }
      MU(MUTEXCOMM);
    }
    break;

    // execute test-function located in util.cpp
  case CMD_TEST:
    id = list[index].param_count < 15 ? list[index].param_count-1 : 15;
    {
      for(i=0; i<id; i++) {
        params[i] = CUtil::strtodouble(list[index].param[1+i], strlen(list[index].param[1+i]) );
      }

      global.test.run(params, id);
    }
    break;

  case CMD_RECHARGE:
    CUtil::cout("DO THIS ON YOUR OWN RISK!!!\nBattery recharging will be stopped when battery voltage remains unchanged for 200 seconds (see controller program)\n");
    CUtil::cout("\nPress R + Return to proceed.\n");
    CUtil::cout("Press Return to cancel.\n");
    // read user input
    int bChar;
    do {
      bChar = global.console.readChar();
    } while(bChar != '\n' && bChar != 'r');

    // != r -> cancel
    if(bChar != 'r') {
      break;
    }

    oldInputState = global.motion.enableIpo(false);
    CUtil::cout("Recharging Batteries of Bioloid:\n");
    global.wrapper->rechargeBatteries();
    global.motion.enableIpo(true);
    break;

  case CMD_SETROBOT:
    oldInputState = global.motion.enableIpo(false);
    byte param[2];
    id = (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
    // mdda : double CUtil::strtodouble(char *text, int len, bool* error)
    i = (int)CUtil::strtodouble(list[index].param[2], strlen(list[index].param[2]) );
    param[0] =  i & 0xFF;
    param[1] =  (i & 0xFF00) >> 8;
    global.wrapper->sendControlData(id, param, 2);
    global.motion.enableIpo(true);
    break;

    // enable/disable torque -> move motors by hand
  case CMD_T:
    global.robotWrapper.enableTorque(list[index].param_count < 2 || strcasecmp(list[index].param[1], "on") == 0);
    break;

    // enable/disable torque control
  case CMD_TCONTROL:
    global.controlTorque = list[index].param_count < 2 || strcasecmp(list[index].param[1], "on") == 0;

    if(global.controlTorque) {
      CUtil::cout("Torque Control is enabled.\n");
    }
    else {
      CUtil::cout("Torque Control is disabled.\n");
    }
    break;

    // enable/disable speed control
  case CMD_SCONTROL:
    global.controlSpeed = list[index].param_count < 2 || strcasecmp(list[index].param[1], "on") == 0;

    if(global.controlSpeed) {
      CUtil::cout("Speed Control is enabled.\n");
    }
    else {
      CUtil::cout("Speed Control is disabled.\n");
    }
    break;

    // enable/disable debug mode
  case CMD_DEBUG:
    global.showDebugMessages = list[index].param_count < 2 || strcasecmp(list[index].param[1], "on") == 0;

    if(global.showDebugMessages) {
      CUtil::cout("Debug mode is enabled.\n");
    }
    else {
      CUtil::cout("Debug mode is disabled.\n");
    }
    break;

    // enable/disable collision avoidance
  case CMD_COLLISION:
    global.stopOnCollisionRead = list[index].param_count < 2 || strcasecmp(list[index].param[1], "on") == 0;
    global.stopOnCollisionWrite = global.stopOnCollisionRead;
    global.addMotionsWithCollision = !global.stopOnCollisionRead;

    if(global.stopOnCollisionRead) {
      CUtil::cout("Collision avoidance is enabled.\n");
    }
    else {
      CUtil::cout("Collision avoidance is disabled.\n");
    }
    break;

    // capture movement -> continously(capture) oder at discrete timestamps (grab)
  case CMD_CAPTURE:
  case CMD_GRAB:
    id = (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );

    if( (id < 0) ){//|| (id >= MOTION_MAXUSER) ) {
      sprintf(buffer, "Please choose a slot number in the range of 0 .. %d.\n", MOTION_MAXUSER);
      CUtil::cout(buffer);
    }
    else {
      CUtil::cout("Press Return To Start Capturing:\n");
      CUtil::cout("---------------------------------\n");

      CUtil::waitForReturn();

      if(list[index].type == CMD_GRAB) {
        global.motion.capture(id, CMotionContainer::CAPTURE_KEY);
      }
      else {
        CUtil::cout("Started.\nPress Return To Finish Capturing.\n\n");
        global.motion.capture(id, CMotionContainer::CAPTURE_TIME);

        CUtil::waitForReturn();

        global.motion.stopCapture();
      }
      CUtil::cout("Capturing Done.\n\n");
    }
    break;

    // execute motion commands, f.e. m walk
    // !! load motion.mtn first !!
  case CMD_M:
    for (i=2; i<list[index].param_count; i++)
        params[i-2] = CUtil::strtodouble(list[index].param[i], strlen(list[index].param[i]) );
  
    // add command to command list for execution
    global.motion.addCommand(list[index].param[1], params, list[index].param_count - 2);
    break;

    // play motion sequence (generated by capture/grab)
  case CMD_PLAY:
  case CMD_LOOP:
  {
    //local variables
    // len = id count, i = loop count
    if(list[index].type == CMD_LOOP) {
      // # of motions ids: play 0 1 2 3 100 4 -> #=4
      len = list[index].param_count-4;
      // get loop count
      i = (int)CUtil::strtodouble(list[index].param[list[index].param_count-1], strlen(list[index].param[list[index].param_count-1]) );
    }
    else {
      // # of motions ids: loop 0 1 2 3 100 4 5 -> #=4
      len = list[index].param_count-3;
      i = 1; // CMD_PLAY -> loop count = 1
    }

    // malloc integer array to store ids
    int *intptr = (int*)malloc(len * sizeof(int) );
    if(intptr == NULL) {
      CUtil::cout("CMD_PLAY: malloc() failed.\n", TEXT_ERROR);
      break;
    }

    // read ids
    j = 0;
    for(id=0; id<len; id++) {
      intptr[id] = (int)CUtil::strtodouble(list[index].param[1+id], strlen(list[index].param[1+id]) );
      if( (intptr[id] < 0) ) {// || (intptr[id] >= MOTION_MAXUSER) ) {
        sprintf(buffer, "Please choose a slot number in the range of 0 .. %d.\n", MOTION_MAXUSER);
        CUtil::cout(buffer);
        j = 1;
        break;
      }
    }

    if(j == 0) {
      // read delay
      j = (int)CUtil::strtodouble(list[index].param[1+len], strlen(list[index].param[1+len]) );

      // read ipo type
      k = (int)CUtil::strtodouble(list[index].param[1+len+1], strlen(list[index].param[1+len+1]) );

      // schedule sequence for playing
      global.motion.playSequence(intptr,
                                 j,
                                 k,
                                 i,
                                 len);
    }
    free(intptr);
    intptr = NULL;
  }
  break;

    // stop movement of robot = stop all sequences, clear all motion lists
  case CMD_STOP:
  case CMD_WAIT:
    global.motion.stopSequence(list[index].type == CMD_WAIT);

    global.motion.stop();
    break;

    // hold current position (removes beep) by setting target positions to current positions
  case CMD_HOLD:
    global.robotWrapper.holdPosition();
    break;

    // show current position and orientation of hands and feet
  case CMD_INFO:
  { // local variables

    // create local robot object
    /* CRobot robot;
    robot.init(global.dhFilename);

    // read robot configuration from wrapper
    ML(MUTEXCOMM);
    robot.updateDhParameters(true);
    MU(MUTEXCOMM);
    // calc hands and feet position
    robot.calcForwardKinematics(false);*/
    
    // use global robot
    CRobot &robot = global.robotInput;

    robot.getAnglesFromDh(angles);

    // display as arml(), armr(), legl() and legr() functions with parametes describing
    // current positions -> use in scripts
    CUtil::cout("Current Angles:\n");
    CUtil::cout("------------------\n");
    for(i=0; i<AX12_COUNT; i++) {
      //sprintf(buffer, "%d: %g\n", i+1, global.robotWrapper.Ax12s[i].getCurrentAngle() );
      sprintf(buffer, "%d: %g\n", i+1, angles[i]);
      CUtil::cout(buffer);
    }

    CUtil::cout("Current Position (Copy And Paste):\n");
    CUtil::cout("------------------\n");

    for(i=0; i<robot.kinematicChains.length; i++) {
      CVec tmp1, tmp2;
      CMatrix p = robot.kinematicChains.chain[i].getRelativeToBase();

      /*
         for (id=0;id<3;id++)
         printf("%g ", 180.0/M_PI*robot.kinematicChains.chain[i].dhParameters[id].getAngle());
         printf("\n");
       */
      CMathLib::getOrientation(p, tmp1, tmp2);

      float rotx, roty, rotz;
    rotx = roty = rotz = 0.0;
    // tweak
    switch(global.robotInput.id)
    {
    case ROBOT_HUMANOID:
    case ROBOT_HUMANOID_INVERSE_HIP:
      rotx = 90.0;
      roty = 90.0;
      break;
    }
      sprintf(buffer, "moveto(%d, %g, %g, %g, %g, %g, %g, t);\n", i+1, p[3].x,
              p[3].y,
              p[3].z,
              -rotx + tmp1.x * 180.0/M_PI,
              -roty + tmp1.y * 180.0/M_PI,
              -rotz + tmp1.z * 180.0/M_PI);
      CUtil::cout(buffer);

    }
  }
  break;

    // display help text (commands, syntac, description)
  case CMD_HELP:
    // header first
    CUtil::cout(CMD_HELP_TEXT_HEADER);

    // then movement commands
    CUtil::cout(CMD_HELP_TEXT_CTMOV);
    for(i=0; i<CONSOLE_CMD_COUNT; i++) {
      if(console_cmds[i].type == CT_MOV) {
        sprintf(buffer, "%s\n", console_cmds[i].helpText);
        CUtil::cout(buffer);
      }
    }

    // then config commands
    CUtil::cout("\n");
    CUtil::cout(CMD_HELP_TEXT_CTCFG);
    for(i=0; i<CONSOLE_CMD_COUNT; i++) {
      if(console_cmds[i].type == CT_CFG) {
        sprintf(buffer, "%s\n", console_cmds[i].helpText);
        CUtil::cout(buffer);
      }
    }

    CUtil::cout("\n");
    break;

    // set speed and torque of single (1<=id<=AX12_COUNT) or all servos (id = 255)
  case CMD_SET:
  case CMD_SPEED:
  case CMD_TORQUE:
  {
    id = (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
    params[0] = CUtil::strtodouble(list[index].param[2], strlen(list[index].param[2]) );
    if(list[index].param_count >= 4) {
      params[1] = CUtil::strtodouble(list[index].param[3], strlen(list[index].param[3]) );
    }
    else {
      params[1] = 1000;
    }

    if(list[index].type == CMD_SET) {
      global.parser.set(id, params[0], (int)params[1]);
      break;
    }

    oldInputState = global.motion.enableIpo(false);

    ML(MUTEXCOMM);
    // internal representation: ax12 with id 1 = ax12[0]
    id--;

    if( (id >= 0) && (id < AX12_COUNT) ) {
      switch(list[index].type)
      {
      case CMD_SET:
        global.robotWrapper.Ax12s[id].setTargetAngle(params[0]);
        doWrite = 1;
        break;

      case CMD_TORQUE:
        global.robotWrapper.Ax12s[id].setTorqueLimit( (unsigned int)params[0]);
        doWrite = 1;
        break;

      case CMD_SPEED:
        global.robotWrapper.Ax12s[id].setSpeedLimit( (unsigned int)params[0]);
        doWrite = 1;
        break;
      }
    }
    else {
      switch(list[index].type)
      {
      case CMD_SET:
        doWrite = 1;
        for(id=0; id<AX12_COUNT; id++) {
          global.robotWrapper.Ax12s[id].setTargetAngle(params[0]);
        }
        break;

      case CMD_TORQUE:
        doWrite = 1;
        for(id=0; id<AX12_COUNT; id++) {
          global.robotWrapper.Ax12s[id].setTorqueLimit( (unsigned int)params[0]);
        }
        break;

      case CMD_SPEED:
        doWrite = 1;
        for(id=0; id<AX12_COUNT; id++) {
          global.robotWrapper.Ax12s[id].setSpeedLimit( (unsigned int)params[0]);
        }
        break;
      }
    }

    global.robotWrapper.writeAx12s();
    MU(MUTEXCOMM);

    global.motion.enableIpo(true);
  }
  break;

    // read speed and torque of single (1<=id<=AX12_COUNT) or all servos (id = 255)
  case CMD_GET:
  {
    id = (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
    id--;

    if( (id >= 0) && (id < AX12_COUNT) ) {
      ML(MUTEXCOMM);
      sprintf(buffer, "Ax12[%d]:\nCurrent Angle: %g \tTarget: %g\nCurrent Speed: %d \tMax: %d\nCurrent Torque: %d \tMax: %d\n\n",
              id+1, global.robotWrapper.Ax12s[id].getCurrentAngle(), global.robotWrapper.Ax12s[id].getTargetAngle(),
              global.robotWrapper.Ax12s[id].getCurrentSpeed(), global.robotWrapper.Ax12s[id].getSpeedLimit(),
              global.robotWrapper.Ax12s[id].getCurrentTorque(), global.robotWrapper.Ax12s[id].getTorqueLimit() );
      MU(MUTEXCOMM);
      CUtil::cout(buffer);
    }
  }
  break;

    // movement commands: add desired position and orientation to trajectory list
    // robot will try to reach the position in 1000ms
  case CMD_ARML:
  case CMD_ARMR:
  case CMD_LEGL:
  case CMD_LEGR:
    CUtil::cout("Inverse Kinematics:\n\n");
    CUtil::cout(" z\n");
    CUtil::cout(" ^\n");
    CUtil::cout(" |\n");
    CUtil::cout(" |\n");
    CUtil::cout("  ---> x\n");

    params[0] = CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
    params[1] = CUtil::strtodouble(list[index].param[2], strlen(list[index].param[2]) );
    params[2] = CUtil::strtodouble(list[index].param[3], strlen(list[index].param[3]) );


    if(list[index].param_count < 7) {
      params[3] =   0.0;
      params[4] =   0.0;
      params[5] =   0.0;
    }
    else {
      params[3] = CUtil::strtodouble(list[index].param[4], strlen(list[index].param[4]) );
      params[4] = CUtil::strtodouble(list[index].param[5], strlen(list[index].param[5]) );
      params[5] = CUtil::strtodouble(list[index].param[6], strlen(list[index].param[6]) );
    }

    sprintf(buffer, "Target-Position: (%g, %g, %g)\n", params[0], params[1], params[2]);
    CUtil::cout(buffer);

    // duration of motion, default value = 1000ms
    id = 1000;

    i = 0;
    switch(list[index].type)
    {
    case CMD_ARML:
      i = 2;
      if(list[index].param_count > 4) {
        id = (int)CUtil::strtodouble(list[index].param[4], strlen(list[index].param[4]) );
      }
      break;

    case CMD_ARMR:
      i = 3;
      if(list[index].param_count > 4) {
        id = (int)CUtil::strtodouble(list[index].param[4], strlen(list[index].param[4]) );
      }
      break;

    case CMD_LEGL:
      i = 0;
      if(list[index].param_count > 7) {
        id = (int)CUtil::strtodouble(list[index].param[7], strlen(list[index].param[7]) );
      }
      break;

    case CMD_LEGR:
      i = 1;
      if(list[index].param_count > 7) {
        id = (int)CUtil::strtodouble(list[index].param[7], strlen(list[index].param[7]) );
      }
      break;
    }
    global.motion.moveLimb(i, params[0], params[1], params[2], params[3], params[4], params[5], id);
    break;

    // movement commands: add desired position and orientation to trajectory list
    // robot will try to reach the position in 1000ms
  case CMD_MOVETO:
    i = (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
    i--;

    params[0] = CUtil::strtodouble(list[index].param[2], strlen(list[index].param[2]) );
    params[1] = CUtil::strtodouble(list[index].param[3], strlen(list[index].param[3]) );
    params[2] = CUtil::strtodouble(list[index].param[4], strlen(list[index].param[4]) );


    if(list[index].param_count < 8) {
      params[3] =   0.0;
      params[4] =   0.0;
      params[5] =   0.0;
    }
    else {
      params[3] = CUtil::strtodouble(list[index].param[5], strlen(list[index].param[5]) );
      params[4] = CUtil::strtodouble(list[index].param[6], strlen(list[index].param[6]) );
      params[5] = CUtil::strtodouble(list[index].param[7], strlen(list[index].param[7]) );
    }

    sprintf(buffer, "Target-Position: (%g, %g, %g)\n", params[0], params[1], params[2]);
    CUtil::cout(buffer);

    // duration of motion, default value = 1000ms
    id = 1000;
    if(list[index].param_count > 8) {
      id = (int)CUtil::strtodouble(list[index].param[8], strlen(list[index].param[8]) );
    }

    global.motion.moveLimb(i, params[0], params[1], params[2], params[3], params[4], params[5], id);
    break;

    // combine multiple motions and store it in extra slot
  case CMD_JOIN:
  { //local variables
    int *intptr;
    intptr = (int*)malloc( (list[index].param_count-2) * sizeof(int) );
    if(intptr == NULL) {
      CUtil::cout("CMD_PLAY: malloc() failed.\n", TEXT_ERROR);
      break;
    }

    for(id=1; id<list[index].param_count-1; id++) {
      intptr[id-1] = (int)CUtil::strtodouble(list[index].param[1+id], strlen(list[index].param[1+id]) );
    }

    global.motion.joinSequence( (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) ), intptr, list[index].param_count-2);

    free(intptr);
    intptr = NULL;
  }
  break;

    // store motion in file
  case CMD_S:
    i = (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
    if( (i<0) || (i>=MOTION_COUNT) ) {
      sprintf(buffer, "Error: Id must be between 0 and %d.\n", MOTION_COUNT-1);
      CUtil::cout(buffer);
      break;
    }
    sprintf(buffer, "%s%s%s%s", global.programPath, DIR_MOTIONS, DIR_SEPARATOR, list[index].param[2]);

    global.motion.motions[i].saveToFile(buffer);
    sprintf(buffer, "Stored Motion %d to File %s.\n", i, list[index].param[2]);
    CUtil::cout(buffer);
    break;

    // loads a motion from a file
  case CMD_L:
    i = (int)CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
    if( (i<0) || (i>=MOTION_COUNT) ) {
      sprintf(buffer, "Error: Id must be between 0 and %d.\n", MOTION_COUNT-1);
      CUtil::cout(buffer);
      break;
    }
    sprintf(buffer, "%s%s%s%s", global.programPath, DIR_MOTIONS, DIR_SEPARATOR, list[index].param[2]);

    global.motion.motions[i].loadFromFile(buffer);
    sprintf(buffer, "Loaded Motion %d (%d Keyframes) from File %s.\n", i, global.motion.motions[i].count(), list[index].param[2]);
    CUtil::cout(buffer);
    break;

    // play show
    // obsolete: replaced by "script playshow.s"
  case CMD_PLAYSHOW:
    char *path;
#ifdef WIN32
    path = (char *)"show\\";
#else
    path = (char *)"show/";
#endif
    char str[255];
    sprintf(str, "%s%splayshow.s", global.programPath, path);
    //global.parser.parseFile(str,"");
    sprintf(str, "%s%sbow.test", global.programPath, path); global.motion.motions[0].loadFromFile(str);
    sprintf(str, "%s%sswing.test", global.programPath, path); global.motion.motions[1].loadFromFile(str);
    sprintf(str, "%s%sswing0.test", global.programPath, path); global.motion.motions[2].loadFromFile(str);
    sprintf(str, "%s%sstand.test", global.programPath, path); global.motion.motions[3].loadFromFile(str);
    sprintf(str, "%s%sclap0.test", global.programPath, path); global.motion.motions[4].loadFromFile(str);
    sprintf(str, "%s%sclap1.test", global.programPath, path); global.motion.motions[5].loadFromFile(str);
    sprintf(str, "%s%sclap2.test", global.programPath, path); global.motion.motions[6].loadFromFile(str);
    sprintf(str, "%s%sclap3.test", global.programPath, path); global.motion.motions[7].loadFromFile(str);
    sprintf(str, "%s%sfight0.test", global.programPath, path); global.motion.motions[8].loadFromFile(str);
    sprintf(str, "%s%sfight1.test", global.programPath, path); global.motion.motions[9].loadFromFile(str);
    sprintf(str, "%s%sfight2.test", global.programPath, path); global.motion.motions[10].loadFromFile(str);
    sprintf(str, "%s%sfight3.test", global.programPath, path); global.motion.motions[11].loadFromFile(str);
    sprintf(str, "%s%sfight4.test", global.programPath, path); global.motion.motions[12].loadFromFile(str);

    params[0] = 1.0;

    if(list[index].param_count > 1) {
      params[0] = CUtil::strtodouble(list[index].param[1], strlen(list[index].param[1]) );
    }

    int motionids[10];
    motionids[0] = 0;
    global.motion.playSequence(motionids, (int)(1200.0*params[0]), IPO_NOCOLLISION + IPO_SPLINE, 1, 1);

    motionids[0] = 1; motionids[1] = 1; motionids[2] = 1; motionids[3] = 2;
    global.motion.playSequence(motionids, (int)(500.0*params[0]), IPO_NOCOLLISION + IPO_SPLINE, 1, 4);
    
    motionids[0] = 3; motionids[1] = 4;
    global.motion.playSequence(motionids, (int)(1000.0*params[0]), IPO_NOCOLLISION + IPO_SPLINE, 1, 2);

    motionids[0] = 5; motionids[1] = 6;
    motionids[2] = 6; motionids[3] = 6;
    motionids[4] = 6; motionids[5] = 6;

    global.motion.playSequence(motionids, (int)(200.0*params[0]), IPO_NOCOLLISION + IPO_SPLINE, 1, 6);

    motionids[0] = 7;
    global.motion.playSequence(motionids, (int)(1000.0*params[0]), IPO_NOCOLLISION + IPO_SPLINE, 1, 1);
    
    motionids[0] = 8;
    global.motion.playSequence(motionids, (int)(2000.0*params[0]), IPO_NOCOLLISION + IPO_SPLINE, 1, 1);

    motionids[0] = 9; motionids[1] = 10;
    motionids[2] = 9; motionids[3] = 10;
    motionids[4] = 9; motionids[5] = 10;
    global.motion.playSequence(motionids, (int)(200.0*params[0]), IPO_NOCOLLISION + IPO_SPLINE, 1, 6);

    motionids[0] = 9; motionids[1] = 12;
    motionids[2] = 2;
    global.motion.playSequence(motionids, (int)(1000.0*params[0]), IPO_NOCOLLISION + IPO_SPLINE, 1, 3);
    break;

    // load motion file (.mtn) into sequence buffer
  case CMD_LOAD:
    i = 0; j = 110;
    global.motion.loadMotionFromFile(0, list[index].param[1], i, j);
    break;
  }
  
  return result;
}

// run configuration commands first, movement commands afterwards
int CConsole::executeCmds()
{
  int i;
  bool result = true;
  byte doWrite = 0;

  // cfg cmds first
  for(i=0; i<list_len; i++) {
    if(list[i].type == 255) { // 255 = -1 in byte
      char buffer[255];
      sprintf(buffer, "Invalid Command: %s\n", list[i].param[0]);
      CUtil::cout(buffer);
    }
    else if(console_cmds[list[i].type].type == CT_HIDE) {
      CUtil::cout("This command is disabled.\n");
    }
    else if(console_cmds[list[i].type].type == CT_CFG) {
      result = result & executeCmd(i, CT_CFG, doWrite);
    }
  }

  // movement afterwards
  for(i=0; i<list_len; i++) {
    if(list[i].type != 255) {
      if(console_cmds[list[i].type].type == CT_MOV) {
        result = result & executeCmd(i, CT_MOV, doWrite);
      }
    }
  }

  return (int)result;
}

// creates a command object
void CConsole::createCmd(char **param, int param_count)
{
  if( (param_count > 0) && (list_len < CMD_LIST_LEN) ) {
    list[list_len].type = getIndexByName(param[0]);
    list[list_len].param = (char**)malloc(param_count * sizeof(char*) );

    if(list[list_len].param == NULL) {
      CUtil::cout("createCmd: malloc failed().\n", TEXT_ERROR);
      return;
    }

    for(int i=0; i<param_count; i++) {
      list[list_len].param[i] = param[i];
    }

    list[list_len].param_count = param_count;
    list_len++;
  }
}

// processes a textual command, f.e. arml 180 0 0
void CConsole::processWord(char *word)
{
  char *param[PARAMCOUNT];
  int param_count = 0;
  int len;
  int i;
  char *pch;

  //trim left
  word = CUtil::strtrim(word);
  len = strlen(word);
  pch = word;

  // strtok non-recursive, strtokr unavailable on atmega
  for(i=0; i<len; i++) {
    if(word[i] == CONSOLE_PARAM_SEPARATOR) {
      word[i] = 0;
      param[param_count++] = pch;

      for(i++; i<len && word[i] == CONSOLE_PARAM_SEPARATOR; i++) {
        ;
      }
      pch = &word[i];
    }
  }

  // dont forget last parameter
  if(*pch != 0) {
    param[param_count++] = pch;
  }

  // create command object
  createCmd(param, param_count);
}

// processes a whole line of input, f.e arml 180 0 0, armr -180 0 0, play 0 150 0
// len includes '\0'
void CConsole::processLine(char *line, int len)
{
  char buffer[255];

  // clean up old commands
  for(int i=0; i<list_len; i++) {
    //for (int j=0; j<list[i].param_count; j++)
    //    free(list[i].param[j]);

    free(list[i].param);
  }
  list_len = 0;

  char *pch = line;
  // overwrite '\0' with ',', makes parsing more easy
  line[len-1] = CONSOLE_CMD_SEPARATOR;

  // strtok non-recursive, strtokr unavailable on atmega
  // -> use own mechanism, better readability, speed doesnt matter
  int i;
  for(i=0; i<len; i++) {
    if(line[i] == CONSOLE_CMD_SEPARATOR) {
      line[i] = 0;
      processWord(pch);

      for(i++; i<len && line[i] == CONSOLE_CMD_SEPARATOR; i++) {
        ;
      }
      pch = &line[i];
    }
  }
}

