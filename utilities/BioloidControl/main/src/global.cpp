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


#include <cstdio>
#include <iostream>
#include <cstring>
#include <fstream>

using namespace std;

#include "../include/cmd.h"
#include "../include/robot.h"
#include "../include/platform.h"
#include "../include/constants.h"
#include "../include/global.h"
#include "../include/tinyxml.h"

#ifdef WIN32
 #include <windows.h>
#else
 #include <curses.h>
 #include <signal.h>
#endif

CGlobalContainer global;

// will be loaded from parameters.txt and stored if program is properly shut down
float CGlobalContainer::parameters[CONSOLE_PARAMETER_COUNT] = {
  0.2, 0.08, 2, 0.35,
  2.3, 3, 0.1, 0.2,
  2, 3.5, 0.5, 0.5,
  0.125, 3, 0.025, 0.5,
  0, 0.5, 2, 0.15, 2,
  0.25, 0.5, 0.75, 0.02,
  0.08, 0.04, 2, 0.7,
  0.01, 0.1, 0.875, 0.9,
  6, 4.5, -5, 0,
  80, 24, 0, 10,
  0, 0.5, 150, 20,
  0.05, 0.3, 100, 0.95,
  0, 80, 20, 70,
  0, 20, 20, 10,
  0, 0, -9, 10,
  25, -15, -45, -5,
  0, 5, 0, 15,
  -90
};

CGlobalContainer::CGlobalContainer()
{
  initialisationComplete = false; // can be used to check if initialisation is complete
  writeToRobot = true;   // global switch to turn off/on writing position data to robot
  showDebugMessages = false;
  viewType = 0;
  useTcp = false;
  useSerial = true;
  sprintf(tcpHostname, "localhost");
  tcpPortIn = 22615;
  tcpPortOut = 22614;

  for(int i=0; i<AX12_COUNT; i++) {
    ax12ToKinematicChain[i] = -1;
    collisionAnglesBackup[i] = 0.0;
  }
}

CGlobalContainer::~CGlobalContainer()
{
}

bool CGlobalContainer::init(int argc, char *argv[])
{
  initialisationComplete = false;

  sprintf(programPath, "");
  if(argc > 0) {
#ifdef WIN32
    char buffer[1023];
    GetModuleFileName(NULL, buffer, 1023);
    CPlatform::getPathFromFilename(buffer, programPath);
#else
    CPlatform::getPathFromFilename(argv[0], programPath);
#endif
  }

  sprintf(dhFilename, "%s%s", programPath, DH_FILE);
  sprintf(configFilename, "%s%s", programPath, CONFIG_FILE);
  sprintf(motionsFile, "%s%s", programPath, MOTIONSFILE);

  CUtil::cout("Program path set to: ");
  CUtil::cout(programPath);
  CUtil::cout("\n");

  char buffer[255];

  for(int i=0; i<(argc-1); i++) { // mdda - (argc-1) since we need the parameter after too
    if(strcasecmp("-config", argv[i]) == 0) {
      sprintf(configFilename, "%s%s", programPath, argv[i+1]);
    }
  }

  sprintf(buffer, "Init: Loading config-file: %s.\n", configFilename);
  CUtil::cout(buffer);

  sprintf(config.rootNode, CONFIG_ROOTNODE);
  if(!config.load(configFilename) ) {
    CUtil::cout("Error: Loading config-file.\n", TEXT_ERROR);
    return false;
  }

  if(config.getBoolean("Software.UseCM5", true) ) {
    CUtil::cout("Init: Packets are compressed and send to CM-5.\n");
    wrapper = new CWrapperCM5();
  }
  else {
    CUtil::cout("Init: Packets will be send directly to the Dynamixel Bus.\n");
    wrapper = new CWrapperDynamixelBus();
  }

  // tcp server
  useTcp = config.getBoolean("Software.TcpEnabled", false);
  tcpPortIn = (unsigned short)config.getInteger("Software.TcpPortIn", 22615);
  tcpPortOut = (unsigned short)config.getInteger("Software.TcpPortOut", 22614);
  sprintf(tcpHostname, config.getString("Software.TcpHostname", "localhost") );

  global.showDebugMessages = config.getBoolean("Software.Debug", false);
  stopOnCollisionRead = config.getBoolean("Software.stopOnCollisionRead", false);
  stopOnCollisionWrite = config.getBoolean("Software.stopOnCollisionWrite", false);
  addMotionsWithCollision = config.getBoolean("Software.addMotionsWithCollision", false);

  storeParameters = config.getBoolean("Software.storeParams", true);
  loadParameters();

  sprintf(dhFilename, "%s%s", programPath, config.getString("Software.RobotConfigFile", DH_FILE) );
  sprintf(motionsFile, "%s%s", programPath, config.getString("Software.MotionsFile", MOTIONSFILE) );

  CUtil::cout("Init: CPlatform::init().\n");
  // init platform dependend stuff (like mutexes)
  CPlatform::init();

  CUtil::cout("Init: servo data.\n");

  // init servo data
  for(int j=0; j<AX12_COUNT; j++) {
    robotWrapper.Ax12s[j].setTargetAngleMinMax(-150.0, 150.0);
    robotWrapper.Ax12s[j].setSpeedLimit(INITSPEED);
    robotWrapper.Ax12s[j].setTorqueLimit(INITTORQUE);
    robotWrapper.Ax12s[j].setTargetAngle(0.0);
    robotWrapper.Ax12s[j].setCurrentAngle(0.0);
  }

  if(config.getBoolean("Software.RobotIsPresent", true) ) {
    // writing of positiondata is by default enabled
    writeToRobot = config.getBoolean("Software.WriteToRobot");
    if( (argc >= 2) && (strcasecmp(argv[1], "off") == 0) ) {
      writeToRobot = false;
    }
    readFromRobot = config.getBoolean("Software.ReadFromRobot");
  }
  else {
    writeToRobot = false;
    readFromRobot = false;
  }

  for(int i=0; i<AX12_COUNT; i++) {
    global.robotWrapper.Ax12s[i].useCurrentAngle = readFromRobot;
  }

  CUtil::cout("Init: robotInput.init().\n");

  // init robotInput structure (kinematics, geometry)
  robotInput.init(global.dhFilename);
  robotCalc.init(global.dhFilename);

  for(int i=0; i<robotInput.kinematicChains.length; i++) {
    for(int j=0; j<robotInput.kinematicChains.chain[i].length; j++) {
      if(robotInput.kinematicChains.chain[i].dhParameters[j].id >= 1) {
        global.ax12ToKinematicChain[robotInput.kinematicChains.chain[i].dhParameters[j].id-1] = i;
        robotWrapper.Ax12s[robotInput.kinematicChains.chain[i].dhParameters[j].id-1].angleSign = robotInput.kinematicChains.chain[i].dhParameters[j].sgn;
      }
    }
  }

  motion.init();

  // init random number generator
  srand(CPlatform::getTickCount() );

  CUtil::cout("Init: CPlatform::initMemoryMappedFile().\n");
  // init shared memory to communicate with the 3d viewer
  CPlatform::openMemoryMappedFile();

  // init serial communcation device
  CUtil::cout("Init: CPlatform::initSerial().\n");
  useSerial = config.getBoolean("Software.ComEnabled", true);
//    if (config.getBoolean("Software.RobotIsPresent", true)) {
  if(useSerial && config.getBoolean("Software.RobotIsPresent", true) ) { // added by mdda
    if(!CPlatform::initSerial(config.getString("Software.ComPort", "COM4"), (unsigned int)config.getInteger("Software.ComSpeed", 115200) ) ) {
      CUtil::cout("Init: Initialisation of serial port failed. Check your configuration in config.xml.\n");
      CUtil::cout("If you want to use the program without a robot set RobotIsPresent in config.xml to false.\n");
      return false;
    }
  }

  // init servo data
  if(config.getBoolean("Hardware.WriteToRobot", false) ) {
    CUtil::cout("Init: servo parameters.\n");

    TiXmlElement*ax12Node = config.findNode("Hardware.Ax12");
    if(ax12Node != NULL) {
      robotWrapper.loadAx12FromXml(ax12Node);
    }
  }

//    CUtil::cout("Pausing - after loadAx12FromXml...\n"); getchar();

  // init pid controllers
  controlTorque = config.getBoolean("Hardware.ControlTorque", false);
  if(controlTorque) {
    CUtil::cout("Torque PID Controller is active.\n");
  }
  else {
    CUtil::cout("Torque PID Controller is inactive.\n");
  }

  controlSpeed = config.getBoolean("Hardware.ControlSpeed", false);
  if(controlSpeed) {
    CUtil::cout("Speed PID Controller is active.\n");
  }
  else {
    CUtil::cout("Speed PID Controller is inactive.\n");
  }

  TiXmlElement*controllerNode = config.findNode("Hardware.Controller");
  if(controllerNode != NULL) {
    robotWrapper.loadPidFromXml(controllerNode);
  }

//    CUtil::cout("Pausing - after loadPidFromXml - no comms with Robot yet...\n"); getchar();

  CUtil::cout("Init: read ax12s.\n");
  // read current servo positions - try to do it for (up to) 10 iterations of 100ms
  if(writeToRobot) {
    int i = 0;
    while(!robotWrapper.readAx12s() && i < 10) {
      i++;
      CPlatform::sleep(100);
    }
  }
//    CUtil::cout("Pausing - after readAx12s - 18 READs have been performed...\n"); getchar();

  // set target positions to current position (should be consistent)
  for(int j=0; j<AX12_COUNT; j++) {
    robotWrapper.Ax12s[j].setTargetAngle(robotWrapper.Ax12s[j].getCurrentAngle() );
  }

//    CUtil::cout("Pausing - after setTargetAngle...\n"); getchar();

  CUtil::cout("Init: CPlatform::initThreads().\n");

  // Load first AUTOSAVENUM motions from last session
#define AUTOSAVENUM 30
  char str[255];
  for(int i=0; i<AUTOSAVENUM; i++) {
    sprintf(str, "Loading autosave/%d\n", i);
    CUtil::cout(str);

    sprintf(str, "%sautosave/%d", global.programPath, i);
    motion.motions[i].loadFromFile(str);
  }
  
  sprintf(str, "Loading System Motions %d to %d from file: %s\n", 0, 110, motionsFile);
  CUtil::cout(str);
  global.motion.loadMotionFromFile(MOTION_MAXSYSTEM, motionsFile, 0, 110);

  // Set viewpoint of 3d viewer to frame "cam" (if defined in configuration file)
  global.viewType = global.robotInput.getFrameByName( (char *)"cam");

//    CUtil::cout("Pausing - after capturePause setting...\n"); getchar();
  console.init();


  // post process commands
  for(int i=0; i<argc-1; i++) { // mdda - use up to (argc-1) since we're interested in params after
    if(strcasecmp("-tcp", argv[i]) == 0) {
      useTcp = strcasecmp(argv[i+1], "on") == 0;
    }

    if(strcasecmp("-serial", argv[i]) == 0) {
      useSerial = !(strcasecmp(argv[i+1], "off") == 0);
    }
  }
//    CUtil::cout("Pausing - after post process commands ...\n"); getchar();

  sprintf(buffer, "Init: Communication Devices: TCP(%s), COM(%s)\n",
          useTcp ? "on" : "off",  useSerial ? "on" : "off");
  CUtil::cout(buffer);

  // static system initialized, threads left -> call global.run()
  return true;
}

void CGlobalContainer::run() {
  // init and run threads
  CPlatform::initThreads(false);

  /*
      main loop runs here
   */
}

void CGlobalContainer::clear() {
  // store paramters
  if(storeParameters) {
    saveParameters();
  }

  // autosave motions
  char str[255];
  for(int i=0; i<AUTOSAVENUM; i++) {
    sprintf(str, "Saving autosave/%d\n", i);
    CUtil::cout(str);

    sprintf(str, "%sautosave/%d", global.programPath, i);
    motion.motions[i].saveToFile(str);
  }

  // pause everything
  motion.stop();
  motion.enableIpo(false);
  // clean up threads
  CUtil::cout("Free: CPlatform::closeThreads().\n");
  CPlatform::closeThreads();
  // clean up platform dependend stuff
  CUtil::cout("Free: CPlatform::clean().\n");
  CPlatform::clean();

  initialisationComplete = false;

  // store configuration
  if(config.getBoolean("General.SaveOnExit") ) {
    config.save(CONFIG_FILE);
  }

  // clean up shared memory
}

// motion thread, processes motion commands
void CGlobalContainer::threadMotion() {
  motion.process();
}

// command thread, processes console commands and scripts
void CGlobalContainer::threadCmd() {
  console.process();
}

void CGlobalContainer::loadParameters(const char*filename) {
  string line;
  ifstream input;
  input.open(filename);

  if(!input.is_open() ) {
    CUtil::cout("Info: Couldn't open parameters file, loading default values.\n");
    return;
  }

  int linesCounter = 0;
  while(!input.eof() && linesCounter < CONSOLE_PARAMETER_COUNT) {
    getline(input, line);
    parameters[linesCounter] = atof(line.c_str() );
    linesCounter++;
  }

  input.close();
}

void CGlobalContainer::saveParameters(const char*filename) {
  string line;
  ofstream output;
  output.open(filename);

  if(!output.is_open() ) {
    CUtil::cout("Error: Couldn't write to parameters file.\n", TEXT_ERROR);
    return;
  }

  for(int i=0; i<CONSOLE_PARAMETER_COUNT; i++) {
    output << parameters[i] << std::endl;
  }

  output.close();
}

