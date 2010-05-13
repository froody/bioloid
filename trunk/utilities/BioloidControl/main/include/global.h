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


#ifndef __GLOBAL

#define __GLOBAL

#include <cstdlib>
#include <string.h>
#include <vector>

#include "cmd.h"
#include "robot.h"
#include "platform.h"
#include "constants.h"
#include "global.h"
#include "tinyxml.h"
#include "wrapper.h"
#include "parser.h"
#include "test.h"
#include "configuration.h"

#define CONSOLE_PARAMETER_COUNT 200 // number of parameters, see setparam console command
#define CONFIG_FILE "config.xml" // default config file
#define DH_FILE "dh.xml" // default robot config file
#define CONFIG_ROOTNODE "Configuration" // root node of config file
#define PARAMETERSFILE "parameters.txt"
#define MOTIONSFILE "motion.mtn"

/*! \brief Global Objects container

   Stores global variables, objects. Manages initialisation, finalisation and execution of main program loop.
 */
class CGlobalContainer
{
public:
  static float parameters[CONSOLE_PARAMETER_COUNT];
  CConfiguration config;

  // global variables
  CWrapper           *wrapper; ///< Communication wrapper
  CRobotWrapper robotWrapper; ///< Hardware wrapper (ax12, axs1, buttons, leds, ..)
  CRobot robotInput;   ///< Stores the current robot data, use MUTEXINPUT to access
  CRobot robotCalc; ///< Robot structure to do calculations (inverse kinematics, ...)
  CMotionContainer motion;    ///< Motion container
  CConsole console;  ///< Console object, processes input
  CParser parser; ///< Lexx, yacc parser
  CTest test; ///< Test Object
  bool initialisationComplete; ///< True if initialisation is complete
  bool writeToRobot;   ///< Turns writing data to robot off/on
  bool readFromRobot; ///< Turns reading data from robot off/on
  bool showDebugMessages; ///< Show debug text in console
  int ax12ToKinematicChain[AX12_COUNT]; ///< Mapping from ax12 servo ids to kinematic chain
  float collisionAnglesBackup[AX12_COUNT]; ///< Stores robot angles if a collision occurred. Used to reduce the amount of generated collision warning.
  int viewType; ///< Index of camera frame in robot.frames (3d viewer)
  bool controlTorque; ///< Enables Torque PID controller
  bool controlSpeed; ///< Enables Speed PID controller
  bool stopOnCollisionRead;
  bool stopOnCollisionWrite;
  bool addMotionsWithCollision;
  char configFilename[255];
  char dhFilename[255];
  char programPath[255];
  char motionsFile[255];
  bool storeParameters;

  bool useTcp;
  bool useSerial;

  char tcpHostname[255];
  unsigned short tcpPortIn;
  unsigned short tcpPortOut;


  CGlobalContainer();
  ~CGlobalContainer();

  void loadParameters(const char*filename = PARAMETERSFILE);
  void saveParameters(const char*filename = PARAMETERSFILE);

  bool init(int argc, char *argv[]); ///< Initialisation
  void run(); ///< Inits and starts threads and runs main program loop.
  void clear(); ///< Finalisation

  void threadMotion(); ///< Thread function
  void threadCmd(); ///< Thread function

};

#endif

