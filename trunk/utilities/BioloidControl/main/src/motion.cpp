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
#include <cstdlib>
#include <cstring>


#include "../include/platform.h"
#include "../include/wrapper.h"
#include "../include/robot.h"
#include "../include/constants.h"
#include "../include/types.h"
#include "../include/motion.h"
#include "../include/util.h"
#include "../include/interpolation.h"
#include "../include/cmd.h"
#include "../include/tinyxml.h"
#include "../include/commands.h"
#include "../include/vars.h"
#include "../include/test.h"

// tiny xml
#if defined (WIN32) && defined (TUNE)
 #include <crtdbg.h>
_CrtMemState startMemState;
_CrtMemState endMemState;
#endif

#ifdef WIN32
 #include <windows.h>
#endif

// use current (alternative: target) position of servos as a starting point
// if MOTION_USECURRENTANGLE is NOT defined, uses target position only if (target - current) < MOTION_CURRENTANGLE_DIFF (see motion.h)
#define MOTION_USECURRENTANGLE

// use spline/linear ipo in trajectory mode
#define SPLINE

// prints motion commands (arml(), armr(), legl() and legr())
// representing key frames when playing sequences
#define _CREATEMOTIONCOMMANDS

// use xml to store motions
#define XML_MOTION_FILES

#ifdef WIN32
 #define AUTOSAVE "autosave\\"
#else
 #define AUTOSAVE "autosave/"
#endif


using namespace std;

const char*CMotionContainer::command_names[CMotionContainer::COMMAND_COUNT] = {
  "STAND", "WALK", "WALKSTAND", "TURNRIGHT", "TURNLEFT", "STOP", "OMNIWALK"
};

int CMotionContainer::ipoPause = MOTION_IPO_PAUSE;
int CMotionContainer::capturePause = MOTION_CAPTURE_PAUSE;
int CMotionContainer::statePause = MOTION_STATE_PAUSE;
int CMotionContainer::trajectoryPause = MOTION_TRAJ_PAUSE;
int CMotionContainer::trajectoryLookAhead = TRAJ_LOOKAHEAD;
int CMotionContainer::interpolationLookAhead = IPO_LOOKAHEAD;
int CMotionContainer::collisionLookAhead = COLLISION_LOOKAHEAD;
int CMotionContainer::ipoMinPause = MOTION_IPO_MINPAUSE;

CMotionData::CMotionData()
{
  pose = new CMatrix[global.robotInput.kinematicChains.length];
}

CMotionData::~CMotionData()
{
  if(pose != NULL) {
    delete[] pose;

    pose = NULL;
  }
}

/* Motion Sequence Class */
CMotionSequence::CMotionSequence()
{
  // nothing
}

// store sequence in a file
void CMotionSequence::saveToFile(char*filename)
{
  int i, j, k;
  CListIterator iter = getIterator();
  CMotionData *item;
  char str[255];

#ifdef XML_MOTION_FILES
  // xml stuff

  // append .xml to filename
  char xmlFilename[255];
  sprintf(xmlFilename, "%s.xml", filename);

 #if defined (WIN32) && defined (TUNE)
  _CrtMemCheckpoint(&startMemState);
 #endif

  // create xml document & xml header
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "", "");
  doc.LinkEndChild(decl);

  // crate root node
  TiXmlElement topNode(XML_MOTION_ROOT);

  // i counts number of stored frames
  i = 0;

  // cycle through key frames
  while(iter.hasNext() )
  {
    item = (CMotionData*)iter.next();
    i++;

    // create frame node, store pause as attribute
    // TODO: use pause value while playing motions
    TiXmlElement frameNode(XML_MOTION_CHILD);
    item->pause = 80;
    sprintf(str, "%d", item->pause);
    frameNode.SetAttribute(XML_MOTION_ATTR, str);

    // store angle-values, f.e. <AX1>90</AX1>
    for(j=0; j<AX12_COUNT; j++) {
      // create <AX*></AX*> node
      sprintf(str, "%s%d", XML_MOTION_ANGLE, j+1);
      TiXmlElement ax12Node(str);

      // add angle value as text node
      sprintf(str, "%f", item->angles[j]);
      TiXmlText valueNode(str);

      // build node
      ax12Node.InsertEndChild(valueNode);
      frameNode.InsertEndChild(ax12Node);
    }
    // append to root
    topNode.InsertEndChild(frameNode);
  }
  // add root node to document
  doc.InsertEndChild(topNode);

  // save finally
  doc.SaveFile(xmlFilename);

  // tiny xml optimizations
 #if defined (WIN32) && defined (TUNE)
  _CrtMemCheckpoint(&endMemState);
  _CrtMemState diffMemState;
  _CrtMemDifference(&diffMemState, &startMemState, &endMemState);
  _CrtMemDumpStatistics(&diffMemState);
 #endif
  // xml stuff end

#else
  // store in binary mode as sequence of word[AX12_COUNT]
  byte buffer[MOTIONBUFFERMAX];  // 15min


  int len = count();

  // store length as 2 bytes (<= 32.767 or 65535)
  buffer[0] = (byte)(len / 256);
  buffer[1] = (byte)(len % 256);

  // store only angles as 2 bytes each
  i = j = 0;
  k = 2;
  float angle_in_degree;
  word tmp;
  while(iter.hasNext() && k < MOTIONBUFFERMAX - AX12_COUNT * 2)
  {
    item = (CMotionData*)iter.next();
    i++;

    for(j=0; j<AX12_COUNT; j++) {
      // use same representation as bioloid robot
      angle_in_degree = 512.0 - item->angles[j] * 512.0 / 150.0 + 0.5;
      tmp = (word)angle_in_degree;

      buffer[k]   = (byte)(tmp & 0x00FF);
      buffer[k+1] = (byte)(tmp >> 8);
      k += 2;
    }
  }

  // write buffer to file
  CPlatform::writeToFile(filename, buffer, k);
#endif

  sprintf(str, "Stored %d Key Frames.\n", i);
  CUtil::cout(str);
}

// load sequence from file
void CMotionSequence::loadFromFile(char*filename)
{
  // clear current sequence
  clear();

  // we need a robot to calculate forward kinematics
  CRobot rbt;
  rbt.init(global.dhFilename);

  // read position values and calculate position and orientation with inverse kinematics
  CMotionData *item;

  // misc
  char str[255];
  int i, j, k, pause;
  bool success;

  // xml stuff
#ifdef XML_MOTION_FILES
  int frameId = 0;

  // append .xml to filename
  char xmlFilename[255];
  sprintf(xmlFilename, "%s.xml", filename);

 #if defined (WIN32) && defined (TUNE)
  _CrtMemCheckpoint(&startMemState);
 #endif

  // create and load xml file
  TiXmlDocument doc(xmlFilename);
  bool loadOkay = doc.LoadFile();

  // misc
  float ax12Values[AX12_COUNT];
  bool ax12Read[AX12_COUNT];

  // file loaded -> load keyframes
  if(loadOkay) {
    TiXmlNode*frameNode;
    TiXmlNode*ax12Node;
    TiXmlNode*valueNode;
    TiXmlElement*topElement;
    TiXmlElement*frameElement;
    TiXmlElement*ax12Element;
    TiXmlText*valueElement;

    // get root node
    TiXmlNode *topNode = doc.FirstChild(XML_MOTION_ROOT);
    if(topNode != NULL) {
      i = 0;

      // get first frame node
      topElement = topNode->ToElement();
      frameNode = topElement->FirstChildElement();

      // cycle through frame nodes
      while(frameNode != NULL)
      {
        frameId++;
        pause = XML_MOTION_ATTR_VALUE;

        success = false;
        for(i=0; i<AX12_COUNT; i++) {
          ax12Read[i] = false;
        }

        // convert to tixmlelement
        frameElement = frameNode->ToElement();

        // load attribute value
        bool error;
        char*tmpstr = (char*)frameElement->Attribute(XML_MOTION_ATTR);
        double pausef;

        if(tmpstr != NULL) {
          pausef = CUtil::strtodouble(tmpstr, strlen(tmpstr) &error);
          if(!error) {
            pause = (int)pausef;
          }
        }

        // cycle through <AX*></AX*> nodes
        for(ax12Node = frameElement->FirstChild(); ax12Node; ax12Node = ax12Node->NextSibling() ) {
          // extract number <AX14> -> 14
          j = strlen(ax12Node->Value() );
          if(j < 3) {
            continue;
          }

          for(i=2; i<j; i++) {
            str[i-2] = ax12Node->Value()[i];
          }

          k = ( (int)CUtil::strtodouble(str, j-2) - 1);
          if( (k >= 0) && (k < AX12_COUNT) ) {
            // retrieve angle
            valueNode = ax12Node->FirstChild();
            valueElement = valueNode->ToText();

            char *anglestr = (char*)valueElement->Value();
            double tmpval = CUtil::strtodouble(anglestr, strlen(anglestr), &error);

            if(!error) {
              ax12Values[k] = tmpval;
              ax12Read[k] = true;
            }
            else {
              sprintf(xmlFilename, "Xml: Couldn't parse: %s. Skipped Frame #%d.\n", anglestr, frameId);
              break;
            }
          }
        }

        // read AX12_COUNT angles?
        success = true;
        for(i=0; i<AX12_COUNT; i++) {
          if(!ax12Read[i]) {
            success = false;
            break;
          }
        }

        // yes -> store as motiondata
        if(success) {
          item = new CMotionData();
          if(item == NULL) {
            CUtil::cout("loadFromFile: malloc failed().\n", TEXT_ERROR);
            return;
          }

          for(j=0; j<AX12_COUNT; j++) {
            item->angles[j] = ax12Values[j];
          }

          // calc forward kinematics = position and orientation of hands and feet
          rbt.setDhFromAngles(item->angles);
          rbt.calcForwardKinematics(false);

          for(j=0; j<rbt.kinematicChains.length; j++) {
            item->pose[j] = rbt.kinematicChains.chain[j].getRelativeToBase();
          }

          item->pause = pause;

          // add motiondata to motions[slot]
          add(item);
        }

        frameNode = frameNode->NextSibling();
      }
    }
  }
 #if defined (WIN32) && defined (TUNE)
  _CrtMemCheckpoint(&endMemState);
  //_CrtMemDumpStatistics( &endMemState );

  _CrtMemState diffMemState;
  _CrtMemDifference(&diffMemState, &startMemState, &endMemState);
  _CrtMemDumpStatistics(&diffMemState);
 #endif

  // xml stuff end
#endif

  if(count() == 0) {
    byte buffer[MOTIONBUFFERMAX];
    buffer[0] = buffer[1] = 0;
    int read = MOTIONBUFFERMAX;

    // read file in binary mode
    CPlatform::readFromFile(filename, buffer, read);

    if(read <= 0) {
      CUtil::cout("Loaded 0 Frames.\n");
      return;
    }

    // first 2 bytes = number of keyframes
    int len = 256 * buffer[0] + buffer[1];
    i = 0;
    k = 2;
    float angle_in_degree;
    word tmp;

    // load keyframes
    for(i=0; i<len; i++) {
      item = new CMotionData();
      if(item == NULL) {
        CUtil::cout("loadFromFile: malloc failed().\n", TEXT_ERROR);
        return;
      }

      // load angles
      for(j=0; j<AX12_COUNT; j++) {
        angle_in_degree = (512.0 - (float)(buffer[k]+256*buffer[k+1]) ) * 150.0 / 512.0;

        k += 2;
        item->angles[j] = angle_in_degree;
      }

      // calc forward kinematics = position and orientation of hands and feet
      rbt.setDhFromAngles(item->angles);
      rbt.calcForwardKinematics(false);

      for(j=0; j<rbt.kinematicChains.length; j++) {
        item->pose[j] = rbt.kinematicChains.chain[j].getRelativeToBase();
      }

      // add motiondata to motions[slot]
      add(item);
    }
  }
  // print result
  if(count() == 1) {
    sprintf(str, "Loaded 1 Frame.\n");
  }
  else {
    sprintf(str, "Loaded %d Frames.\n", count() );
  }

  CUtil::cout(str);
}

void CMotionSequence::loadFromMotionEditorFile(char*filename, unsigned int firstId, unsigned int lastId)
{
  unsigned int tmp[1024];

  int len = lastId - firstId + 1;
  if(len > 1024) {
    len = 1024;
    CUtil::cout("CMotionSequence::loadFromMotionEditorFile: Too many key frames. Truncated to 1024.", TEXT_ERROR);
  }

  for(unsigned int i = 0; i < len; i++) {
    tmp[i] = i + firstId;
  }

  loadFromMotionEditorFile(filename, tmp, len);
}

void CMotionSequence::loadFromMotionEditorFile(char*filename, unsigned int*ids, unsigned int length)
{
#define MOTIONPAGES 128
  struct _PAGE pages[MOTIONPAGES];
  int len = MOTIONPAGES * sizeof(struct _PAGE);
  int read = len;
  // read file, pages struct is mapped on file content
  CPlatform::readFromFile(filename, (byte*)pages, read);

  if(len != read) {
    CUtil::cout("Couldn't load motion file.\n", TEXT_ERROR);
    return;
  }

  // we need a robot to calculate forward kinematics
  CRobot robot;
  robot.init(global.dhFilename);

  // OldMtnFiles (19 Ax12s) -> exclude first ax12 in upper body (non-existent in 18 dof version)
  int idOffset = 1;
  if(!global.config.getBoolean("Software.OldMtnFiles", true) ) {
    idOffset = 0;
  }

  for(int id=0; id<length; id++) {
    for(int j=0; j<7; j++) {
      //create motiondata
      CMotionData*newMotionData = new CMotionData();
      if(newMotionData == NULL) {
        CUtil::cout("loadMotionFromFile: malloc failed().\n", TEXT_ERROR);
        return;
      }

      // get angles
      for(int k=0; k<AX12_COUNT; k++) {
        newMotionData->angles[k] = CUtil::wordToAngle(pages[ids[id]].rec[j].posData[(k + idOffset) * 2], pages[ids[id]].rec[j].posData[(k + idOffset) * 2 + 1]);
      }

      // calc pose of feet and hands
      robot.setDhFromAngles(newMotionData->angles);
      robot.calcForwardKinematics(false);

      for(int i=0; i<global.robotInput.kinematicChains.length; i++) {
        robot.kinematicChains.chain[i].getPose(newMotionData->pose[i]);
      }

      // store delay
      newMotionData->pause = (int)pages[ids[id]].rec[j].delay;

      // store motion data
      add(newMotionData);
    }
  }
}

// remove last key frame
void CMotionSequence::remove() {
  motionData.removeLast(true);
}

// return amount of sequences stored
int CMotionSequence::count() {
  return motionData.count();
}

// return listiterator
CListIterator CMotionSequence::getIterator() {
  return motionData.getIterator();
}

// add keyframe to end of sequence
void CMotionSequence::add(CMotionData *item) {
  motionData.add(item);
}

// return keyframe at position i in sequence (0 <= i < count())
CMotionData*CMotionSequence::operator [](int i) {
  CListIterator iter = motionData.getIterator();
  CMotionData *result = NULL;

  while(iter.hasNext() ) {
    result = (CMotionData*)iter.next();
    if(i==0) {
      return result;
    }
    i--;
  }
  return NULL;
}

// remove all keyframes and free up memory
void CMotionSequence::clear() {
  motionData.clear();
}

/*
   Motion Container Class
   ----------------------
   runs in own thread ->
   state driven:
   state = MOTION_NONE, MOTION_PLAY, MOTION_STOP, MOTION_CAPTURE, MOTION_STOPCAPTURE
   sequences: store, play, capture, grab
   trajectory: command-, trajectory- and interpolation list
               processed by one thread each
 */

// initialisation
CMotionContainer::CMotionContainer() {
  state = MOTION_NONE;
  stateParam[0] = 0;
  ipoState = IPO_ON;
}

void CMotionContainer::init() {
  trajectory = new CList[global.robotInput.kinematicChains.length];
  char buffer[255];
  sprintf(buffer, "Trajectories(%d) set.\n", global.robotInput.kinematicChains.length);
  CUtil::cout(buffer, TEXT_DEBUG);
}

// join multiple sequences and in an extra slot
void CMotionContainer::joinSequence(int id, int ids[], int len) {
  int i, j;
  char str[255];
  for(i=0; i<len; i++) {
    if( (ids[i] < 0) || (ids[i] >= MOTION_COUNT) ) {
      sprintf(str, "Ids must be in range 0 .. %d.\n", MOTION_COUNT-1);
      CUtil::cout(str);
      return;
    }
  }

  if( (id < 0) || (id >= MOTION_COUNT) ) {
    sprintf(str, "Ids must be in range 0 .. %d.\n", MOTION_COUNT-1);
    CUtil::cout(str);
    return;
  }

  for(i=0; i<len; i++) {
    if(ids[i] == id) {
      CUtil::cout("The storage id shouldn't be in the sequence.\n");
      return;
    }
  }

  // clear storage slot
  motions[id].clear();

  // cycle through slots to join
  for(i=0; i<len; i++) {
    CListIterator iter = motions[ids[i]].getIterator();
    CMotionData *item, *newItem;

    // cycle through motiondata
    while(iter.hasNext() ) {
      item = (CMotionData*)iter.next();

      // copy motiondata
      newItem = new CMotionData();
      if(newItem == NULL) {
        CUtil::cout("joinSequence: malloc failed().\n", TEXT_ERROR);
        return;
      }

      for(j=0; j<AX12_COUNT; j++) {
        newItem->angles[j] = item->angles[j];
      }

      for(j=0; j<global.robotInput.kinematicChains.length; j++) {
        newItem->pose[j] = item->pose[j];
      }

      // add new motiondata to storage slot
      motions[id].add(newItem);
    }
  }

  sprintf(str, "Joined %d Keyframes.\n", motions[id].count() );
  CUtil::cout(str);
}

int CMotionContainer::getIpoState() {
  return ipoState;
}

// enables / disables interpolation thread via simple handshake (waits until disabled)
int CMotionContainer::enableIpo(bool enable) {
  int old, rtn;

  // lock state variable
//    CUtil::cout("Interpolation Thread - wait for lock.\n", TEXT_DEBUG);
  ML(MUTEXMOTION);
//    CUtil::cout("Interpolation Thread - got lock.\n", TEXT_DEBUG);

  old = ipoState;


  if (old == IPO_OFF && enable)
    CPlatform::sleep(1000);

  CPlatform::clearLine();
/*
       if (enable) {
                                // enable ipo thread, no handshake necessary
                                ipoState = IPO_ON;
                                MU(MUTEXMOTION);
                } else
 */
  {
    // disable ipo thread
    ipoState = enable ? IPO_ON : IPO_OFF;

    // turn off -> handshake necessary
    if(old != ipoState) {
      // handshake variable == 1 -> ack
      ipoReturn = 0;

      MU(MUTEXMOTION);

      rtn = 0;
      while(rtn == 0) {
        CUtil::cout("Interpolation Thread - waiting in loop\n", TEXT_DEBUG); // mdda
        ML(MUTEXMOTION);
        rtn = ipoReturn;
        MU(MUTEXMOTION);
        CPlatform::sleep(ipoPause);
      }
    }
    else {
      MU(MUTEXMOTION);
    }
  }



  if(!enable) {
    CUtil::cout("Interpolation Thread stopped.\n", TEXT_DEBUG);
  }
  else {
    CUtil::cout("Interpolation Thread continued.\n", TEXT_DEBUG);
  }

  return old;
}

// sets current state
void CMotionContainer::setState(int newstate) {
  ML(MUTEXMOTION);
  state = newstate;
  MU(MUTEXMOTION);
}

// returns current state
int CMotionContainer::getState() {
  int tmp;
  ML(MUTEXMOTION);
  tmp = state;
  MU(MUTEXMOTION);
  return tmp;
}

// motion container thread function, processes different states
void CMotionContainer::process() {
  volatile int current, id, type;

  while(true) {
    switch(current = getState() ) {
      // play sequences
    case MOTION_PLAY:
      play();
      break;

      // capture movement (grab or capture)
    case MOTION_CAPTURE:
      // store in slot id
      id = stateParam[0];
      // capture type (grab, capture = discrete, continous)
      type = stateParam[1];
      captureMotion(id, type);
      break;

    default:
      setState(MOTION_NONE);
      break;
    }
    // check every MOTION_STATE_PAUSE milliseconds
    CPlatform::sleep(CMotionContainer::statePause);
  }
}

// captures movement (kinaesthetic programming) continuous or discrete
void CMotionContainer::captureMotion(int id, int type) {
  int current, oldInputState, k, iter;
  bool discrete;
  bool bRead;
  CMotionData *newMotionData;

  // we need a robot for collision checking and visualisation
  CRobot&robot = global.robotInput;

  // continous or discrete?
  discrete = (type == CAPTURE_KEY);

  // should have been checked earlier -> slot id okay?
  if( (id < 0) || (id >= MOTION_COUNT) ) {
    return;
  }

  // clear stored key frames
  ML(MUTEXMOTION);
  motions[id].clear();
  MU(MUTEXMOTION);


  // put input thread asleep
  oldInputState = enableIpo(false);

  // disable servo torque -> we want to move the limbs by hand
  global.robotWrapper.enableTorque(false);

  unsigned long time, oldTime, diff;
  iter = 0;

  // get current time in milliseconds
  time = oldTime = CPlatform::getTickCount();

  CUtil::cout("Capturing started.\n");

  // capture
  while(discrete ||     // until: discrete - user presses space bar, see below
        ( (current = getState() ) == MOTION_CAPTURE) )  // or continous - stopCapture() is called
  {
    // discrete -> process user input
    if(discrete) {
      CUtil::cout("Press 'Return' To Capture Frame ('Space' To Finish, 'r' To Remove Last Frame):\n");

      // read user input
      int bChar;
      do {
        bChar = global.console.readChar();
      } while(bChar != '\n' && bChar != 32 && bChar != 'r');

      // space = finish
      if(bChar == ' ') {
        break;
      }

      // r = remove last frame, display last pose and continue
      if(bChar == 'r') {
        // remove last frame
        ML(MUTEXMOTION);
        k = motions[id].count() - 1;
        motions[id].remove();
        newMotionData = motions[id][k-1];
        MU(MUTEXMOTION);

        // display last valid pose
        if(newMotionData != NULL) {
          robot.setDhFromAngles(newMotionData->angles);
          robot.calcForwardKinematics(true);
        }
        CUtil::cout("Removed last frame.\n");
        continue;
      }

      // else grab frame
      char str[255];
      sprintf(str, "Frame grabbed at: %ds.\n", time / 1000);
      CUtil::cout(str);
    }

    // grab frame
    bRead = false;

    ML(MUTEXCOMM);
    bRead = global.robotWrapper.readAx12s();

    // no read error? -> store frame
    if(bRead) {
      newMotionData = new CMotionData();
      if(newMotionData == NULL) {
        CUtil::cout("captureMotion: malloc failed().\n", TEXT_ERROR);
        return;
      }

      for(k=0; k<AX12_COUNT; k++) {
        newMotionData->angles[k] = global.robotWrapper.Ax12s[k].getCurrentAngle();
      }

      // update robot object with current angle values
      robot.updateDhParameters();
    }
    MU(MUTEXCOMM);

    // no read error -> update 3d display
    if(bRead) {
      robot.calcForwardKinematics(true);

      // print current limb positions
      CVec *tmp = new CVec[2 * global.robotInput.kinematicChains.length];
      robot.getTCPs(tmp);
      global.console.printInfo(tmp);
      delete tmp;
      tmp = NULL;

      // store limb poses in motionData-Objecto
      for(k=0; k<global.robotInput.kinematicChains.length; k++) {
        robot.kinematicChains.chain[k].getPose(newMotionData->pose[k]);
      }

      // collisiontest yes -> user can accept current frame or remove it
      if(robot.checkCollisions(true) ) {;}
      // nothing

      // store frame in storage slot
      ML(MUTEXMOTION);
      motions[id].add(newMotionData);
      MU(MUTEXMOTION);
    }
    else {
      CUtil::cout("Motion: Couldn't Read From Robot. Skipped Frame.\n");
    }

    // continous -> try to make a pause of ipoPause ms
    oldTime = time;
    time = CPlatform::getTickCount();

    if(!discrete) {
      k = CMotionContainer::capturePause - (int)(time - oldTime);

      if(k <= 0) {
        k = 0;
      }

      CPlatform::sleep(k);
    }
  }

  // capturing done
  CUtil::cout("Capturing finished.\n");

  // autosave captured movement in autosave/id
  char str[255];
  sprintf(str, "%s%s%d", global.programPath, AUTOSAVE, id);
  ML(MUTEXMOTION);
  motions[id].saveToFile(str);
  MU(MUTEXMOTION);

  char str2[255];
  sprintf(str2, "Motion stored in file: %s. Reload with 'l id %s'.\n", str, str);
  CUtil::cout(str2);

  // important: set target positions to current positions
  // trajectory based movement: current robot pose OR (if near) target robot pose
  // is added to beginning of trajectory movement: so this is for savety reasons
  // (no small jumps in servo positions)
  ML(MUTEXCOMM);
  global.robotWrapper.readAx12s();
  for(int i=0; i<AX12_COUNT; i++) {
    global.robotWrapper.Ax12s[i].setTargetAngle(
      global.robotWrapper.Ax12s[i].getCurrentAngle() );
  }
  MU(MUTEXCOMM);

  // hmm
  CPlatform::sleep(100);

  // enable torque
  global.robotWrapper.enableTorque(true);

  // wake up input thread
  enableIpo(oldInputState == IPO_ON);

  // reset state
  setState(MOTION_NONE);
}

// plays motion sequences with different interpolation types:
// see ipo_* in motion.h
// dirty function - needs revision
void CMotionContainer::play() {
  // use global robot
  CRobot *robot = &global.robotInput;

  // misc
  int frames;
  int ipoCycle = CMotionContainer::capturePause;             // in ms
  int ipoCount = 1;

  int i, k, j, id, seqLen, pause, loopCounter, ipo, ipoType, motionListLen, timeDiff;
  double x, y, z;
  CMatrix rot;
  bool checkCollision = true;
  int    *count;
  int    *seq;
  CMotion *current;

  float angles[AX12_COUNT];
  float anglesOld[AX12_COUNT];
  CVec*tcpBuffer = new CVec[2*robot->kinematicChains.length];
  CMotionData *currentMotion;
  unsigned long time, writeTime;

  // interpolation objects
  CInterpolationWrapper*ipoWrapper = NULL;
  CInterpolationData ipoIn, ipoOut;
  ipoIn.malloc();
  ipoOut.malloc();

  // initialise
  count = NULL;
  seq = NULL;

  CUtil::cout("Playing started.\n");

  // put ipo thread asleep
  int oldIpo = enableIpo(false);
  
  writeTime = CPlatform::getTickCount();
  bool wroteData = false;
  while(true)    {
    CUtil::cout("Initialization loop...\n");
    // motion scheduler empty -> finish
    ML(MUTEXMOTION);
    motionListLen = motionList.count();
    MU(MUTEXMOTION);

    if(motionListLen <= 0) {
      break;
    }

    // read current motion sequence
    ML(MUTEXMOTION);
    current     = (CMotion*)motionList.getFirst()->data;
    seqLen      = current->count < MOTION_SEQUENCE_MAX ? current->count : MOTION_SEQUENCE_MAX;
    loopCounter = current->loop;
    pause       = current->pause;
    ipoType     = current->ipo;

    if(seqLen == 0) {
      motionList.removeFirst(true);
      MU(MUTEXMOTION);
      continue;
    }

    // ipoTye >= IPO_NOCOLLISION indicates we want NO COLLISION CHECKING
    checkCollision = true;
    if(ipoType >= IPO_NOCOLLISION) {
      checkCollision = false;
      ipoType -= IPO_NOCOLLISION;;
    } else if (!global.stopOnCollisionRead)
      checkCollision = false;
    
    if (!checkCollision)
      CUtil::cout("Collision-Checking deactivated.\n");

    // enable or disable range check in servos
    for(id=0; id<AX12_COUNT; id++) {
      global.robotWrapper.Ax12s[id].checkRange = checkCollision;
    }

    // store ids of sequences in array seq: play 0 1 2 3 100 0 -> seq = [0, 1, 2, 3]
    seq = (int*)CUtil::realloc(seq, seqLen * sizeof(int) );
    if(seq == NULL) {
      MU(MUTEXMOTION);
      goto LABEL_MALLOC;
    }

    count = (int*)CUtil::realloc(count, seqLen * sizeof(int) );
    if(count == NULL) {
      MU(MUTEXMOTION);
      goto LABEL_MALLOC;
    }

    for(id=0; id<seqLen; id++) {
      seq[id] = current->ids[id];
    }

    // remove current play command from schedule
    motionList.removeFirst(true);
    MU(MUTEXMOTION);

    // calculate total amount of frames
    frames = getSequenceLength(seq, seqLen);

    // no frames -> process next motion command
    if(frames <= 0) {
      CUtil::cout("Motion sequence must contain at least 1 frame.\n");
      continue;
    }

    int tmpFrames = frames;
    if(frames == 1) {
      CUtil::cout("Motion sequence contains only 1 frame. Doubled it.\n", TEXT_DEBUG);
      tmpFrames++;
    }

    // init interpolation objects:
    ipoWrapper = CInterpolationFactory::getWrapper(ipoType, robot);
    ipoWrapper->setLength(frames);
    ipoWrapper->setTime(pause);

    // calculate number of interpolation points between keyframes
    double tmp = (double)pause / (double)ipoCycle;
    tmp = (double)frames * tmp;
    ipoCount = (int)tmp;

    // at least two interpolation points
    if(ipoCount < 2) {
      ipoCount = 2;
    }

//								CUtil::cout("Preparing Interpolation\n");  // mdda

    // interpolation: preparation
    for(id=0; id<seqLen; id++) {
      // count[i] = number of keyframes referenced by seq[0] .. seq[i]
      if(id > 0) {
        count[id] = count[id-1];
      }
      else {
        count[id] = 0;
      }

      // skip bad ids
      if( (seq[id] < 0) || (seq[id] >= MOTION_COUNT) ) {
        continue;
      }

      // read servo positions and cartesian poses
      ML(MUTEXMOTION);
      CListIterator iter = motions[seq[id]].getIterator();

      while(iter.hasNext() ) {
        currentMotion = (CMotionData*)iter.next();

        ipoIn.angles_position = (double)count[id] / (double)(tmpFrames - 1);

        for(i=0; i<AX12_COUNT; i++) {
          ipoIn.angles[i] = currentMotion->angles[i];
        }

        for(i=0; i<global.robotInput.kinematicChains.length; i++) {
          ipoIn.pose_position[i] = ipoIn.angles_position;
          ipoIn.pose[i] = currentMotion->pose[i];
        }

        ipoWrapper->setPoint(count[id], &ipoIn);
        count[id]++;
      }
      MU(MUTEXMOTION);
    }

    // further interpolation type specific preprocessing
    ipoWrapper->start();

    /*
        !!! Initialisation complete !!!
     */
//								CUtil::cout("Initialisation complete !!!\n");  // mdda

    // current relative position 0 <= x <= 1
    double currentPosition;

    // main processing loop
    while(loopCounter-- != 0) {   // != -> -1 = infinite
      // reset interpolation objects
      ipoWrapper->reset();

      
      // save starttime
      time = CPlatform::getTickCount();

      // cycle through frames
      for(id=0; id<ipoCount; id++) {
        if(0) {
          char str[255];
          sprintf(str, "Cycling through frames - id : %d\n", id);
          CUtil::cout(str);
        }

        // react on stop command -> leave
        if(getState() == MOTION_STOP) {
          ML(MUTEXMOTION);
          motionList.clear(true);
          MU(MUTEXMOTION);
          goto LOOP_END;
        }

        // display time left in seconds
        if( ( (ipoCount - id)*ipoCycle) % 1000 < ipoCycle) {
          char str[255];
          sprintf(str, "Play: %ds left.\n", ( (ipoCount - id)*ipoCycle) / 1000);
          CUtil::cout(str);
        }

        // current relative position
        currentPosition = (double)id / (double)(ipoCount-1);
        ipoOut.angles_position = currentPosition;
        for(k=0; k<global.robotInput.kinematicChains.length; k++) {
          ipoOut.pose_position[k] = currentPosition;
        }

        // get interpolated point
        ipoWrapper->getPoint(currentPosition, &ipoOut);

	/*
        if(global.readFromRobot) {
          ML(MUTEXCOMM);
          global.robotWrapper.readAx12s();
          robot->updateDhParameters(true);
          MU(MUTEXCOMM);
	  } else */
	robot->setDhFromAngles(ipoOut.angles);
        
        // 3d display of current pose
        robot->calcForwardKinematics(true);

//                                                                      CUtil::cout("Have read from robot\n");  // mdda

        // write calculated angles to robot
        for(k=0; k<AX12_COUNT; k++) {
          // simple speed check to limit servo speed
          if(global.controlSpeed) {
            global.robotWrapper.Ax12s[k].controlSpeed(ipoOut.angles[k], (float)ipoCycle);
          }

          if(global.controlTorque) {
            global.robotWrapper.Ax12s[k].controlTorque(ipoOut.angles[k], (float)ipoCycle);
          }

          // update wrapper structures
          global.robotWrapper.Ax12s[k].setTargetAngle(ipoOut.angles[k]);

          if(!global.readFromRobot) {
            global.robotWrapper.Ax12s[k].setCurrentAngle(ipoOut.angles[k]);
          }
        }

        wroteData = false;
      
        timeDiff = (int)(CPlatform::getTickCount() - writeTime);

        while (true) 
        {
	  CPlatform::sleep(1);
            timeDiff = (int)(CPlatform::getTickCount() - writeTime);      
            if (timeDiff >= ipoPause)
               break;

        }

	//printf("time: %d\n", timeDiff);
//                                                                      CUtil::cout("Calculated Kinematics\n");  // mdda 
        float tmpf = 0.0;
        // write angles to robot
        if(global.writeToRobot) {
          if(!checkCollision || !robot->checkCollisions() ) {
            
            ML(MUTEXCOMM);
//                                                                              CUtil::cout("Got Comms MUTEX\n");  // mdda
	    //printf("sending\n");
            writeTime = CPlatform::getTickCount();
            global.robotWrapper.writeAx12s();
            wroteData = true;
            MU(MUTEXCOMM);
          }
          else {
            CUtil::cout("aborting.\n");
            stop();
            goto LOOP_END;
          }
        }

//                                                                      CUtil::cout("Written to robot\n");  // mdda

        if (!wroteData)
           writeTime = CPlatform::getTickCount();
        // print current limb position and orientation
        robot->getTCPs(tcpBuffer);
        global.console.printInfo(tcpBuffer);
        // pause between frames should be ipoPause
        /*k = ipoCycle - (int)(CPlatform::getTickCount() - (time + (unsigned long)id * ipoCycle) );

        if(k <= 0) {
          k = 1;
        }

        CPlatform::sleep(k);*/
      }
    }
    // motion command processed
    // clean up memory
    CUtil::free( (void**)&count);
    CUtil::free( (void**)&seq);

    if(ipoWrapper != NULL) {                          // should be
      delete ipoWrapper;
      ipoWrapper = NULL;
    }
  }
  goto LOOP_END;


LABEL_MALLOC:
  CUtil::cout("play: malloc failed().\n", TEXT_ERROR);

LOOP_END:
  // clean up memory
  if(tcpBuffer != NULL) {
    delete[] tcpBuffer;
  }

  ipoIn.free();
  ipoOut.free();

  CUtil::free( (void**)&count);
  CUtil::free( (void**)&seq);

  if(ipoWrapper != NULL) {              // should not be
    delete ipoWrapper;
    ipoWrapper = NULL;
  }

  // enable servo angle range check
  for(id=0; id<AX12_COUNT; id++) {
    global.robotWrapper.Ax12s[id].checkRange = true;
  }

  // hmm
  CPlatform::sleep(1000);

  // wakeup ipo thread
  enableIpo(oldIpo == IPO_ON);

  // do nothing
  // TODO: not really threadsafe
  setState(MOTION_NONE);
  CUtil::cout("Playing finished.\n");
}

// calculate motion transforming the current robot pose into the target robot pose
CMotion*CMotionContainer::getIntermediateMotion(CMotionData*target, int id, int ipoType, int pause)
{
  int i;

  // allocate memory
  CMotion*newMotion = new CMotion();
  if(newMotion == NULL) {
    CUtil::cout("getIntermediateMotion: malloc failed().\n", TEXT_ERROR);
    return NULL;
  }

  // one time, no loop, linear interpolation in joint space, INTERMEDIATE_PAUSE delay
  // motion will be store in slot INTERMEDIATE_MOTION (not accessible through l,grab,capture)
  newMotion->count = 1;
  newMotion->loop  = 1;
  newMotion->ipo   = ipoType;
  newMotion->pause = pause;
  newMotion->ids[0] = id;

  motions[id].clear();

  // store current pose as motiondata
  CMotionData*item = new CMotionData();
  if(item == NULL) {
    CUtil::cout("loadFromFile: malloc failed().\n", TEXT_ERROR);
    return NULL;
  }

  ML(MUTEXCOMM);
  for(i=0; i<AX12_COUNT; i++) {
    item->angles[i] = global.robotWrapper.Ax12s[i].getCurrentAngle();
  }
  MU(MUTEXCOMM);

  ML(MUTEXINPUT);
  global.robotCalc.setDhFromAngles(item->angles);
  global.robotCalc.calcForwardKinematics(false);
  for(int i=0; i<global.robotCalc.kinematicChains.length; i++) {
    item->pose[i] = global.robotCalc.kinematicChains.chain[i].getRelativeToBase();
  }
  MU(MUTEXINPUT);

  motions[id].add(item);

  // store target pose as motiondata
  CMotionData*tItem = new CMotionData();
  if(tItem == NULL) {
    CUtil::cout("loadFromFile: malloc failed().\n", TEXT_ERROR);
    motions[id].clear();
    return NULL;
  }

  for(i=0; i<AX12_COUNT; i++) {
    tItem->angles[i] = target->angles[i];
  }

  motions[id].add(tItem);

  // calculate max distance between current angle and target angle
  float dist = 0.0;
  for(i=0; i<AX12_COUNT; i++) {
    if(fabsf(tItem->angles[i] - item->angles[i]) > dist) {
      dist = fabsf(tItem->angles[i] - item->angles[i]);
    }
  }

  // no real distance -> abort
  if(dist < INTERMEDIATE_DIFFERENCE) {
    motions[id].clear();
    return NULL;
  }
  else { return newMotion;}
}

// calculate motion transforming the current robot pose into the target robot pose
CMotion*CMotionContainer::getIntermediateMotion(CMotion*target, int id, int ipoType, int pause)
{
  // get first id in target sequence list with >0 frames
  int tmpId = -1;
  for(int i=0; i<target->count; i++) {
    if( (target->ids[i] >= 0) && (target->ids[i] < MOTION_MAXUSER) ) {
      if(motions[target->ids[i]].count() > 0) {
        tmpId = target->ids[i];
        break;
      }
    }
  }

  // doesnt exit -> abort
  if(tmpId < 0) {
    return NULL;
  }

  CListIterator iter = motions[tmpId].getIterator();
  CMotionData*t = (CMotionData*)iter.next();

  return getIntermediateMotion(t, id, ipoType, pause);
}

// send signal to play motion sequence
void CMotionContainer::playSequence(int id[], int pause, int ipo, int loop, int num)
{
  int i, innerstate = getState();

  // motion processor busy?
  if( (innerstate == MOTION_NONE) || (innerstate == MOTION_PLAY) ) {
    CMotion*newMotion = new CMotion();
    if(newMotion == NULL) {
      CUtil::cout("playSequence: malloc failed().\n", TEXT_ERROR);
      return;
    }

    // fill motion-command-object
    if(num > MOTION_SEQUENCE_MAX) {
      newMotion->count = MOTION_SEQUENCE_MAX;
    }
    else { newMotion->count = num;}

    newMotion->loop = loop;
    newMotion->ipo  = ipo;
    newMotion->pause = pause;

    for(i=0; i<newMotion->count; i++) {
      newMotion->ids[i] = id[i];
    }

    // add motion-command to execution list
    // list empty ->
    // add intermediate motion from current pose to start pose of motion
    ML(MUTEXMOTION);
    i = motionList.count();
    MU(MUTEXMOTION);

    if(ipo < IPO_NOCOLLISION) {
      if(i <= 0) {
        CMotion*tmpMotion = getIntermediateMotion(newMotion);
        if(tmpMotion != NULL) {
          ML(MUTEXMOTION);
          motionList.add(tmpMotion);
          MU(MUTEXMOTION);
        }
      }
    }

    // add and set motion processor in play-mode
    ML(MUTEXMOTION);
    motionList.add(newMotion);
    state = MOTION_PLAY;
    MU(MUTEXMOTION);
  }
}

// send signal to stop motion currently executed
void CMotionContainer::stopSequence(bool wait)
{
  ML(MUTEXMOTION);
  if(state == MOTION_PLAY) {
    state = MOTION_STOP;
  }
  MU(MUTEXMOTION);
}

// send signal to start continous capturing
void CMotionContainer::capture(int id, byte input)
{
  ML(MUTEXMOTION);
  if(state == MOTION_NONE) {
    if(input == CAPTURE_KEY) {
      MU(MUTEXMOTION);
      captureMotion(id, CAPTURE_KEY);
    }
    else {
      stateParam[0] = id;
      stateParam[1] = CAPTURE_TIME;
      state =         MOTION_CAPTURE;
      MU(MUTEXMOTION);
    }
  }
  else {
    MU(MUTEXMOTION);
  }
}

// send signal to stop continous capturing
void CMotionContainer::stopCapture()
{
  ML(MUTEXMOTION);
  if(state == MOTION_CAPTURE) {
    state = MOTION_STOPCAPTURE;
  }
  MU(MUTEXMOTION);
}

// load .mtn file into sequence buffer, thanks to Bullit from Tribotix-forums
void CMotionContainer::loadMotionFromFile(int startid, char *filename, int from, int to)
{
#define MOTIONPAGES 128
  struct _PAGE pages[MOTIONPAGES];
  int len = MOTIONPAGES * sizeof(struct _PAGE);
  int read = len;
  // read file, pages struct is mapped on file content
  CPlatform::readFromFile(filename, (byte*)pages, read);

  if(len != read) {
    CUtil::cout("Couldn't load motion file.\n", TEXT_ERROR);
    return;
  }

  // we need a robot to calculate forward kinematics
  CRobot robot;
  robot.init(global.dhFilename);

  // OldMtnFiles (19 Ax12s) -> exclude first ax12 in upper body (non-existent in 18 dof version)
  int idOffset = 1;
  if(!global.config.getBoolean("Software.OldMtnFiles", true) ) {
    idOffset = 0;
  }

  if(to > MOTIONPAGES) {
    to = MOTIONPAGES;
  }

  for(int id=from; id<to; id++) {
    // clear storage id
    ML(MUTEXMOTION);
    motions[startid].clear();
    MU(MUTEXMOTION);

    for(int j=0; j<7; j++) {
      //create motiondata
      CMotionData*newMotionData = new CMotionData();
      if(newMotionData == NULL) {
        CUtil::cout("loadMotionFromFile: malloc failed().\n", TEXT_ERROR);
        return;
      }

      // get angles
      for(int k=0; k<AX12_COUNT; k++) {
        newMotionData->angles[k] = CUtil::wordToAngle(pages[id].rec[j].posData[(k + idOffset) * 2], pages[id].rec[j].posData[(k + idOffset) * 2 + 1]);
      }

      // calc pose of feet and hands
      robot.setDhFromAngles(newMotionData->angles);
      robot.calcForwardKinematics(false);

      for(int i=0; i<global.robotInput.kinematicChains.length; i++) {
        robot.kinematicChains.chain[i].getPose(newMotionData->pose[i]);
      }

      // store delay
      newMotionData->pause = (int)pages[id].rec[j].delay;

      // store motion data in storage id
      ML(MUTEXMOTION);
      motions[startid].add(newMotionData);
      MU(MUTEXMOTION);
    }

    startid++;
  }
}

/* add motion sequence (grab, capture) to point-to-point movement (arml, armr, legl, legr) system
   select which limbs to include via kid:
   1 = arml
   2 = armr
   4 = legl
   8 = legr
   kid = or combination of 1 2 4 8
   f.e. left arm and right leg -> kid = 1 + 8 = 9
 */
void CMotionContainer::addSequenceToTrajectory(int ids[], int len, int pause, bool checkCollision, int kId)
{
  CPoint **points = (CPoint**)malloc(global.robotInput.kinematicChains.length*sizeof(CPoint*) );
  CMotionData *motionData;

  int i, j, k;

  const int kIds[4] = {1, 2, 4, 8};

  for(k=0; k<len; k++) {
    CListIterator iter = motions[ids[k]].getIterator();
    while(iter.hasNext() )
    {
      motionData = (CMotionData*)iter.next();

      for(i=0; i<global.robotInput.kinematicChains.length; i++) {
        // limb selected? -> create new trajectory point from motion pose
        if( (kId & kIds[i]) == kIds[i]) {
          points[i] = new CPoint();
          if(points[i] == NULL) {
            CUtil::cout("addSequenceToTrajectory: malloc failed().\n", TEXT_ERROR);
            return;
          }

          points[i]->pose = motionData->pose[i];

          // get angles associated with kinematicchain
          for(j=0; j<AX12_COUNT; j++) {
            if(global.ax12ToKinematicChain[j] == i) {
              points[i]->angles[j] = motionData->angles[j];
            }
            else {
              points[i]->angles[j] = AX12_INVALID;
            }
          }

          points[i]->duration = pause;
          points[i]->current = 0;
          points[i]->checkCollision = checkCollision;
        }
      }

      // add all create trajectory points at the same time
      // avoids asynchronous behaviour
      ML(MUTEXTRAJECTORY);
      for(i=0; i<global.robotInput.kinematicChains.length; i++) {
        if( (kId & kIds[i]) == kIds[i]) {
          trajectory[i].add(points[i]);
        }
      }
      MU(MUTEXTRAJECTORY);
    }
  }

  free(points);
  points = NULL;
}

void CMotionContainer::addControlCommand(char*command, int params[], int paramCount)
{
  int i, cmdId = -1;

  for(i = 0; i < CONTROL_COMMAND_COUNT; i++) {
    if(strcasecmp(command_names[i], command) == 0) {
      cmdId = i;
      break;
    }
  }

  if(cmdId == -1) {
    return;
  }

  // atm: only two parameters, convenience
  int p[2];
  if(paramCount >= 2) {
    p[0] = ( (int*)params)[0];
    p[1] = ( (int*)params)[1];
  }
  else {
    // dirty
    p[0] = 1;
    p[1] = 100;
  }

  CCommand*cmd;

  switch(cmdId)
  {
  case CONTROL_COMMAND_STOP:
  {
    CCommandSequence *c = new CCommandStop(1, true, this);
    c->setParam(0, p[0]);
    c->setParam(1, p[1]);
    c->addImmediateMotion = true;
    c->setId(0, 65);
    cmd = c;
    break;
  }
  }
  ML(MUTEXCMDS);
  cmdControl.add(cmd);
  MU(MUTEXCMDS);
}

// add motion command (f.e. m walk, m turnright) to execution list
void CMotionContainer::addCommand(char*cmd, int param1, int param2)
{
  float *params = (float*)malloc(2*sizeof(float) );
  if(params == NULL) {
    CUtil::cout("addCommand: malloc failed().\n", TEXT_ERROR);
    return;
  }

  params[0] = (float)param1;
  params[1] = (float)param2;

  addCommand(cmd, params, 2);
  
  free(params);
}

// add motion command (f.e. m walk, m turnright) to execution list
// - includes a CCommand-Factory
// - add new CCommands here
void CMotionContainer::addCommand(char*command, float params[], int paramCount)
{
  int i, cmdId = -1;

  for(i = 0; i < COMMAND_COUNT; i++) {
    if(strcasecmp(command_names[i], command) == 0) {
      cmdId = i;
      break;
    }
  }

  if(cmdId == -1) {
    return;
  }

  CCommand*cmd;

  // atm: only two parameters, convenience
  int p[10];
  if(paramCount >= 2) {
    p[0] = (int) params[0];
    p[1] = (int) params[1];
  }
  else {
    // std
    p[0] = MOTION_DEFAULT_NUM;
    p[1] = MOTION_DEFAULT_PAUSE;
  }


  // init commands:
  // CSequence: load motion.mtn first (humanoid.mtn)
  switch(cmdId)
  {
  case COMMAND_TURNRIGHT:
  {
    CCommandSequence *c = new CCommandSequence(4, true, this);
    c->setParam(0, p[0]);
    c->setParam(1, p[1]);
    c->setId(0, MOTION_MAXSYSTEM+89); c->setId(1, MOTION_MAXSYSTEM+90); c->setId(2, MOTION_MAXSYSTEM+91); c->setId(3, MOTION_MAXSYSTEM+92);
    cmd = c;
  }
  break;

   case COMMAND_WALK:
        {
             CCommandContainer *container = new CCommandContainer(3, this);
             CCommandSequence* c[3];
             for (i=0;i<3;i++)
             {
                 c[i] = new CCommandSequence(4, false, this);
                 container->add(i, c[i]);
             }   
             
             // startwalking
             c[0]->setParam(0, 1);
             c[0]->setId(0, MOTION_MAXSYSTEM+60);c[0]->setId(1, MOTION_MAXSYSTEM+63);c[0]->setId(2, MOTION_MAXSYSTEM+64);c[0]->setId(3, MOTION_MAXSYSTEM+65);
             // loopwalking
             c[1]->setParam(0, p[0]);
             c[1]->setId(0, MOTION_MAXSYSTEM+62);c[1]->setId(1, MOTION_MAXSYSTEM+63);c[1]->setId(2, MOTION_MAXSYSTEM+64);c[1]->setId(3, MOTION_MAXSYSTEM+65);
             // stopwalking
             c[2]->setParam(0, 1);
             c[2]->setId(0, MOTION_MAXSYSTEM+62);c[2]->setId(1, MOTION_MAXSYSTEM+63);c[2]->setId(2, MOTION_MAXSYSTEM+64);c[2]->setId(3, MOTION_MAXSYSTEM+83);
      
             // ipo delay identical
             for (i=0;i<3;i++)
                 c[i]->setParam(1, p[1]);
                 
             cmd = container;
             break;
         }
         break;
  
  
  case COMMAND_TURNLEFT:
  {
    CCommandSequence *c = new CCommandSequence(4, true, this);
    c->setParam(0, p[0]);
    c->setParam(1, p[1]);
    c->setId(0, 98); c->setId(1, MOTION_MAXSYSTEM+99); c->setId(2, MOTION_MAXSYSTEM+100); c->setId(3, MOTION_MAXSYSTEM+101);
    cmd = c;
  }
  break;

  case COMMAND_OMNIWALK:
  {
       CVec speed;
       if(paramCount >= 1)
       {
          p[0] = (int) params[0]; 
       }
       speed.x = paramCount >= 2 ? params[1] : 0.0;
       speed.y = paramCount >= 3 ? params[2] : 0.05;
       speed.z = paramCount >= 4 ? params[3] : 0.0;
       
    cmd = new CCommandOmniWalk(p[0], speed.x, speed.y, speed.z, paramCount >= 5 ? fabsf(params[4]) > 0.5 : false);
    break;
  }

  case COMMAND_STOP:
  {
    CCommandSequence *c = new CCommandStop(1, true, this);
    c->setParam(0, p[0]);
    c->setParam(1, p[1]);
    c->addImmediateMotion = true;
    c->setId(0, 65);
    cmd = c;
    break;
  }
  }
  // add to command list -> motion command thread will execute it
  ML(MUTEXCMDS);
  switch(cmdId)
  {
  case COMMAND_STOP:
    cmdControl.add(cmd);
    break;

  default:
    cmds.add(cmd);
    break;
  }
  MU(MUTEXCMDS);
}

// processes motion commands and adds trajectory points to
// trajectory list of left arm, right arm, left leg and right leg
// the interpolation thread (::interpolate()) will interpolate
// between these points

#define GETMEM(x, y, z) x = (y*)malloc(z * sizeof(y) ); \
  if(x == NULL) \
  {\
    CUtil::cout("interpolate: malloc failed().\n", TEXT_ERROR); \
    return; \
  }

void CMotionContainer::calcTrajectory()
{
  CMotionData *motionData;
  CCommand *cmd, *control;

  int time;
  int i, j, k, len;
  int ids[10];

  while(true)
  {
    cmd = NULL;
    control = NULL;

    // get and execute next control command
    ML(MUTEXCMDS);
    CListIterator iter1 = cmdControl.getIterator();
    if(iter1.hasNext() ) {
      control = (CCommand*)iter1.next();

    }
    MU(MUTEXCMDS);

    if(control != NULL) {
      control->execute();
    }


    // get next motion command
    ML(MUTEXCMDS);
    CListIterator iter = cmds.getIterator();
    if(iter.hasNext() ) {
      cmd = (CCommand*)iter.next();
    }
    MU(MUTEXCMDS);

    // execute motion command, remove if finished
    // (execute can be called several times)
    // time = approx. time left until command.hasFinished = true
    // execution can depend on control command
    if(cmd != NULL) {
      time = cmd->execute(control);

      if(cmd->hasFinished() ) {
        ML(MUTEXCMDS);
        cmds.removeFirst();
        MU(MUTEXCMDS);
      }
      else {
        // sleep(time) -> best performance but not very responsive
        CPlatform::sleep(time < CMotionContainer::trajectoryPause ? time : CMotionContainer::trajectoryPause);
      }
    }
    // control command hasFinished -> remove
    if( (control != NULL) && control->hasFinished() ) {
      ML(MUTEXCMDS);
      cmdControl.removeFirst();
      MU(MUTEXCMDS);
    }

    // sleep until too few trajectory points left (in one of the four lists)
    bool getNext = false;
    while(true)
    {
      ML(MUTEXTRAJECTORY);
      for(i=0; i<global.robotInput.kinematicChains.length; i++) {
        getNext |= trajectory[i].count() <=trajectoryLookAhead;
      }
      MU(MUTEXTRAJECTORY);

      if(!getNext) {
        CPlatform::sleep(CMotionContainer::trajectoryPause);
      }
      else {
        break;
      }
    };
  }
}

// interpolate trajectory points & angle
// zero: init target angles with current angles
// first: calc interpolated hand/foot-pose and overwrite
//        the corresponding target angles
// second: calc interpolated angles (joints[]-list) and
//         overwrite corresponding target angles
// -> direct specification of an angle overwrites cartesian interpolated angle

// TODO: messy, clean up

// WARNING: use only local interpolation types (like linear)
// local = adding a point wont change the whole interpolated function
// not local -> interpolation can be sloppy if points are added during execution
//              (the currently executed part of the trajectory can change)

/*
   void CMotionContainer::interpolate()
   {
     // misc
     int i,j,k;
     int kc = global.robotInput.kinematicChains.length;

     CListItem* currentListItem;
     CListItem tmpListItem;

     float anglesOld[AX12_COUNT], anglesTmp[AX12_COUNT];
     double angles[AX12_COUNT][LOOKAHEAD];
     CIpoData *newMotionData;
     int *timePassed;
     GETMEM(timePassed, int, global.robotInput.kinematicChains.length);

     // cartesian space
     CPoint **limbPoints;
     GETMEM(limbPoints, CPoint*, LOOKAHEAD*global.robotInput.kinematicChains.length);

     int *limbDuration;
     GETMEM(limbDuration, int, LOOKAHEAD*global.robotInput.kinematicChains.length);

     int *limbTotalTime;
     GETMEM(limbTotalTime, int, global.robotInput.kinematicChains.length);
     int *limbCurrentTime;
     GETMEM(limbCurrentTime, int, global.robotInput.kinematicChains.length);
     bool noLimbs;
     bool *noLimb;
     GETMEM(noLimb, bool, global.robotInput.kinematicChains.length);
     // joint space
     CAnglePoint *jointPoints[AX12_COUNT][LOOKAHEAD];
     int jointTotalTime[AX12_COUNT], jointCurrentTime[AX12_COUNT];
     int jointDuration[AX12_COUNT][LOOKAHEAD];
     bool noJoints;
     bool noJoint[AX12_COUNT];

     // we need a robot to calc forward kinematics (collison test)
     CRobot innerRobot;
     innerRobot.init(global.dhFilename);

     // interpolation objects, initialization in loop (dynamic choice of ipo type)
     CInterpolationBase** limbIpo;
     GETMEM(limbIpo, CInterpolationBase*, global.robotInput.kinematicChains.length);

     CInterpolationBase *jointIpo[AX12_COUNT];
     CInterpolationData ipoIn, ipoOut;
     ipoIn.malloc();
     ipoOut.malloc();
     CLinearQuaternionData quaterData;


     // init ipo objects
     for (i=0;i<global.robotInput.kinematicChains.length;i++)
         limbIpo[i] = NULL;
     for (i=0;i<AX12_COUNT;i++)
         jointIpo[i] = NULL;

     // main loop
     while (true)
     {
         // save current time
         unsigned long timestamp = CPlatform::getTickCount();

         // lock trajectory lists during execution
         ML(MUTEXTRAJECTORY);

         // helpers
         noJoints = true;
         for (i=0;i<AX12_COUNT;i++)
         {
             noJoints &= (joints[i].count() == 0);
             noJoint[i] = (joints[i].count() == 0);
         }
         noLimbs = true;
         for (i=0;i<global.robotInput.kinematicChains.length;i++)
         {
             noLimb[i] = (trajectory[i].count() == 0);
             noLimbs &= noLimb[i];
         }

         // lists empty? -> sleep and recheck
         if ( noLimbs && noJoints)
         {
               MU(MUTEXTRAJECTORY);
               CPlatform::sleep(ipoPause);
               continue;
         }

         // update robot object with current joint angles
         ML(MUTEXCOMM);
         innerRobot.updateDhParameters(false);
         innerRobot.getAnglesFromDh(anglesOld);
         MU(MUTEXCOMM);

         // read LOOKAHEAD next trajectory points into limbPoints[][]
         for (i=0;i<global.robotInput.kinematicChains.length;i++)
             if (!noLimb[i])
             {
                 currentListItem = trajectory[i].getFirst();

                 for (k=0;k<LOOKAHEAD;k++)
                 {
                     limbPoints[i * LOOKAHEAD + k] = (CPoint*)(currentListItem->data);

                     // = NULL -> duplicate current trajectory point
                     if (currentListItem->next != NULL)
                         currentListItem=currentListItem->next;
                 }
             }

         for (i=0;i<global.robotInput.kinematicChains.length;i++)
             if (!noLimb[i])
             {
                 // store duration in duration array
                 // point       0    1    2    4
                 // pause       0    500  1000 500
                 // ->
                 // duration    500  1000 500  0

                 // calc total execution time of every limb
                 limbTotalTime[i] = 0;
                 for (j=0;j<LOOKAHEAD-1;j++)
                 {
                     limbDuration[i * LOOKAHEAD +j] = limbPoints[i * LOOKAHEAD + j+1]->duration;
                     if (limbDuration[i * LOOKAHEAD +j] < ipoPause)
                         limbDuration[i * LOOKAHEAD +j] = ipoPause;

                     limbTotalTime[i] += limbDuration[i * LOOKAHEAD +j];
                 }
                 limbDuration[i * LOOKAHEAD + LOOKAHEAD-1] = 0;
                 limbCurrentTime[i] = limbPoints[i * LOOKAHEAD + 0]->current;

                 // create interpolation objects
                 limbIpo[i] = new CLinearQuaternionLinear();
                 limbIpo[i]->setLength(LOOKAHEAD);

                 timePassed[i] = 0;
             }

         // add trajectory points
         for (k=0; k<global.robotInput.kinematicChains.length; k++)
             if (!noLimb[k])
             {
                 // add points
                 for (i=0; i<LOOKAHEAD; i++)
                 {
                     quaterData.position = (double) timePassed[k] / (double)limbTotalTime[k];
                     quaterData.pose = &(limbPoints[k * LOOKAHEAD + i]->pose);

                     limbIpo[k]->setPoint(i, &quaterData);
                     timePassed[k] += limbDuration[k * LOOKAHEAD + i];
                 }

                 // init interpolation-object
                 limbIpo[k]->start();
                 limbIpo[k]->reset();
             }

         // jointAngles:
         if (!noJoints)
         {
             // read LOOKAHEAD next trajectory points into anglePoints[][]
             for (i=0;i<AX12_COUNT;i++)
             if (!noJoint[i])
             {
                 currentListItem = this->joints[i].getFirst();
                 if (currentListItem != NULL)
                 {
                     jointTotalTime[i] = 0;

                     // create interpolation objects
                     jointIpo[i] = new CLinear();
                     jointIpo[i]->setLength(LOOKAHEAD);

                     for (k=0;k<LOOKAHEAD;k++)
                     {
                         jointPoints[i][k] = (CAnglePoint*)(currentListItem->data);

                         if (k>0)
                         {
                             jointTotalTime[i] += jointPoints[i][k]->duration;
                         }

                         if (currentListItem->next != NULL)
                             currentListItem=currentListItem->next;
                     }
                     jointCurrentTime[i] = jointPoints[i][0]->current;
                     jointDuration[i][LOOKAHEAD-1] = 0;

                     // add points
                     double tmp[2];
                     int current = 0;
                     for (k=0;k<LOOKAHEAD;k++)
                     {
                         tmp[0] = (double) current / (double) jointTotalTime[i];
                         tmp[1] =  jointPoints[i][k]->angle;

                         if (k<LOOKAHEAD-1)
                             current += jointPoints[i][k+1]->duration;

                         jointIpo[i]->setPoint(k, tmp);

                         if (k<LOOKAHEAD-1)
                          jointDuration[i][k] = jointPoints[i][k+1]->duration;
                     }

                     // init interpolation objects
                     jointIpo[i]->start();
                     jointIpo[i]->reset();
                 }
             }
         }

         // interpolation starts here
         ML(MUTEXINTERPOLATION);
         int ipos = 0;
         bool done = false;
         while ((ipos++ < CMotionContainer::interpolationLookAhead) && !done)
         {
             newMotionData = new CIpoData();

             // cartesian space
             for (k=0;k<global.robotInput.kinematicChains.length;k++)
             if (!noLimb[k])
             {
                 CMatrix limbMatrix;
                 // calc interpolated point
                 limbIpo[k]->getPoint((double)limbCurrentTime[k]/(double)limbTotalTime[k], &limbMatrix);

                 // calc inverse kinematics
                 innerRobot.calcInverseKinematics(k, limbMatrix);
                 innerRobot.getAnglesFromDh(anglesTmp);

                 // overwrite angles belonging to kinematic chain
                 for (i=0; i<AX12_COUNT; i++)
                     if (global.ax12ToKinematicChain[i] == k)
                         anglesOld[i] = anglesTmp[i];

                 // update elapsed time -> stop when limbTotalTime reached
                 limbCurrentTime[k] += ipoPause;

                 if (limbCurrentTime[k] >= limbTotalTime[k])
                    done = true;
             }

             // add angles from jointangles
             for (k=0;k<AX12_COUNT;k++)
             if (!noJoint[k])
             if (jointCurrentTime[k] < jointTotalTime[k])
             {
                double tmp;
                jointIpo[k]->getPoint((double)jointCurrentTime[k]/(double)jointTotalTime[k], &tmp);
                anglesOld[k] = (float) tmp;

                jointCurrentTime[k] += ipoPause;
             }

             // collision check
             for (k=0; k<AX12_COUNT; k++)
                 newMotionData->angles[k] = anglesOld[k];

             innerRobot.setDhFromAngles(newMotionData->angles);
             innerRobot.calcForwardKinematics(false);
             newMotionData->collision = innerRobot.checkCollisions(false, false) ? CIpoData::COLLISION_YES : CIpoData::COLLISION_NO;

             newMotionData->checkCollision = noLimbs;
             for (k=0;k<global.robotInput.kinematicChains.length;k++)
             if (!noLimb[k])
                 newMotionData->checkCollision |= limbPoints[k * LOOKAHEAD + limbIpo[k]->getCurrentPosition()]->checkCollision;

             // add interpolated point to interpolated-list
             // will be processed by updaterobot()-thread
             interpolated.add(newMotionData);
         }
         MU(MUTEXINTERPOLATION);

         // update elapsed time of trajectory points
         // remove all reached points
         for (i=0;i<AX12_COUNT;i++)
         if (!noJoint[i])
         {
             for (j=0;j<LOOKAHEAD;j++)
             {
                 if (jointCurrentTime[i] < jointDuration[i][j])
                 {
                    jointPoints[i][j]->current = jointCurrentTime[i];
                    break;
                 } else
                 {
                    jointCurrentTime[i] -= jointDuration[i][j];
                 }
             }

             // j = index of first unfinished trajectory point -> remove all < j
             if (j>0)
             {
                joints[i].removeFirst(true);
                for (k=1;(k<j) && (jointPoints[i][k] != jointPoints[i][k-1]);k++)
                    joints[i].removeFirst(true);
             }
         }

         // update elapsed time of trajectory points
         // remove all points in the past
         for (i=0;i<global.robotInput.kinematicChains.length;i++)
         if (!noLimb[i])
         {
             for (j=0;j<LOOKAHEAD;j++)
             {
                 if (limbCurrentTime[i] < limbDuration[i * LOOKAHEAD + j])
                 {
                    limbPoints[i * LOOKAHEAD + j]->current = limbCurrentTime[i];
                    break;
                 } else
                 {
                    limbCurrentTime[i] -= limbDuration[i * LOOKAHEAD + j];
                 }
             }

             // j = index of first unfinished trajectory point -> remove all < j
             if (j>0)
             {
                trajectory[i].removeFirst(true);
                for (k=1;(k<j) && (limbPoints[i * LOOKAHEAD + k] != limbPoints[i * LOOKAHEAD + k-1]);k++)
                    trajectory[i].removeFirst(true);
             }
         }
         MU(MUTEXTRAJECTORY);

         ipoIn.free();
         ipoOut.free();

         // free interpolation object
         for (k=0;k<AX12_COUNT;k++)
         if (jointIpo[k] != NULL)
         {
             delete jointIpo[k];
             jointIpo[k] = NULL;
         }

         for (k=0;k<global.robotInput.kinematicChains.length;k++)
         if (limbIpo[k] != NULL)
         {
             delete limbIpo[k];
             limbIpo[k] = NULL;
         }

         // wait until too few interpolation points are left
         bool getNext = false;
         do
         {
            CPlatform::sleep(ipoPause);
            ML(MUTEXINTERPOLATION);
            getNext = interpolated.count() < 3;
            MU(MUTEXINTERPOLATION);
         } while (!getNext);
         }
   }
 */
void CMotionContainer::interpolate()
{
  // misc
  int i, j, k;
  int kc = global.robotInput.kinematicChains.length;

  CListItem*currentListItem;
  CListItem tmpListItem;

  float anglesOld[AX12_COUNT], anglesTmp[AX12_COUNT];
  double angles[AX12_COUNT][LOOKAHEAD];
  CIpoData *newMotionData;
  int *timePassed;
  GETMEM(timePassed, int, global.robotInput.kinematicChains.length);

  // cartesian space
  CPoint **limbPoints;
  GETMEM(limbPoints, CPoint*, LOOKAHEAD*global.robotInput.kinematicChains.length);

  int *limbDuration;
  GETMEM(limbDuration, int, LOOKAHEAD*global.robotInput.kinematicChains.length);

  int *limbTotalTime;
  GETMEM(limbTotalTime, int, global.robotInput.kinematicChains.length);
  int *limbCurrentTime;
  GETMEM(limbCurrentTime, int, global.robotInput.kinematicChains.length);
  bool noLimbs;
  bool *noLimb;
  GETMEM(noLimb, bool, global.robotInput.kinematicChains.length);
  // joint space
  CAnglePoint *jointPoints[AX12_COUNT][LOOKAHEAD];
  int jointTotalTime[AX12_COUNT], jointCurrentTime[AX12_COUNT];
  int jointDuration[AX12_COUNT][LOOKAHEAD];
  bool noJoints;
  bool noJoint[AX12_COUNT];

  // we need a robot to calc forward kinematics (collison test)
  CRobot innerRobot;
  innerRobot.init(global.dhFilename);

  // interpolation objects, initialization in loop (dynamic choice of ipo type)
  CInterpolationBase**limbIpo;
  GETMEM(limbIpo, CInterpolationBase*, global.robotInput.kinematicChains.length);

  CInterpolationBase *jointIpo[AX12_COUNT];
  CInterpolationData ipoIn, ipoOut;
  ipoIn.malloc();
  ipoOut.malloc();
  CLinearQuaternionData quaterData;


  // init ipo objects
  for(i=0; i<global.robotInput.kinematicChains.length; i++) {
    limbIpo[i] = NULL;
  }
  for(i=0; i<AX12_COUNT; i++) {
    jointIpo[i] = NULL;
  }

  // main loop
  while(true)
  {
    // save current time
    unsigned long timestamp = CPlatform::getTickCount();

    // lock trajectory lists during execution
    ML(MUTEXTRAJECTORY);

    // helpers
    noJoints = true;
    for(i=0; i<AX12_COUNT; i++) {
      noJoints &= (joints[i].count() == 0);
      noJoint[i] = (joints[i].count() == 0);
    }
    noLimbs = true;
    for(i=0; i<global.robotInput.kinematicChains.length; i++) {
      noLimb[i] = (trajectory[i].count() == 0);
      noLimbs &= noLimb[i];
    }

    // lists empty? -> sleep and recheck
    if(noLimbs && noJoints) {
      MU(MUTEXTRAJECTORY);
      CPlatform::sleep(ipoPause);
      continue;
    }

    // update robot object with current joint angles
    ML(MUTEXCOMM);
    innerRobot.updateDhParameters(false);
    innerRobot.getAnglesFromDh(anglesOld);
    MU(MUTEXCOMM);

    // read LOOKAHEAD next trajectory points into limbPoints[][]
    for(i=0; i<global.robotInput.kinematicChains.length; i++) {
      if(!noLimb[i]) {
        currentListItem = trajectory[i].getFirst();

        for(k=0; k<LOOKAHEAD; k++) {
          limbPoints[i * LOOKAHEAD + k] = (CPoint*)(currentListItem->data);

          // = NULL -> duplicate current trajectory point
          if(currentListItem->next != NULL) {
            currentListItem=currentListItem->next;
          }
        }
      }
    }

    for(i=0; i<global.robotInput.kinematicChains.length; i++) {
      if(!noLimb[i]) {
        // store duration in duration array
        // point       0    1    2    4
        // pause       0    500  1000 500
        // ->
        // duration    500  1000 500  0

        // calc total execution time of every limb
        limbTotalTime[i] = 0;
        for(j=0; j<LOOKAHEAD-1; j++) {
          limbDuration[i * LOOKAHEAD +j] = limbPoints[i * LOOKAHEAD + j+1]->duration;
          if(limbDuration[i * LOOKAHEAD +j] < ipoPause) {
            limbDuration[i * LOOKAHEAD +j] = ipoPause;
          }

          limbTotalTime[i] += limbDuration[i * LOOKAHEAD +j];
        }
        limbDuration[i * LOOKAHEAD + LOOKAHEAD-1] = 0;
        limbCurrentTime[i] = limbPoints[i * LOOKAHEAD + 0]->current;

        // create interpolation objects
        limbIpo[i] = new CLinearQuaternionLinear();
        limbIpo[i]->setLength(LOOKAHEAD);

        timePassed[i] = 0;
      }
    }

    // add trajectory points
    for(k=0; k<global.robotInput.kinematicChains.length; k++) {
      if(!noLimb[k]) {
        // add points
        for(i=0; i<LOOKAHEAD; i++) {
          quaterData.position = (double)timePassed[k] / (double)limbTotalTime[k];
          quaterData.pose = &(limbPoints[k * LOOKAHEAD + i]->pose);

          limbIpo[k]->setPoint(i, &quaterData);
          timePassed[k] += limbDuration[k * LOOKAHEAD + i];
        }

        // init interpolation-object
        limbIpo[k]->start();
        limbIpo[k]->reset();
      }
    }

    // jointAngles:
    if(!noJoints) {
      // read LOOKAHEAD next trajectory points into anglePoints[][]
      for(i=0; i<AX12_COUNT; i++) {
        if(!noJoint[i]) {
          currentListItem = this->joints[i].getFirst();
          if(currentListItem != NULL) {
            jointTotalTime[i] = 0;

            // create interpolation objects
            jointIpo[i] = new CLinear();
            jointIpo[i]->setLength(LOOKAHEAD);

            for(k=0; k<LOOKAHEAD; k++) {
              jointPoints[i][k] = (CAnglePoint*)(currentListItem->data);

              if(k>0) {
                jointTotalTime[i] += jointPoints[i][k]->duration;
              }

              if(currentListItem->next != NULL) {
                currentListItem=currentListItem->next;
              }
            }
            jointCurrentTime[i] = jointPoints[i][0]->current;
            jointDuration[i][LOOKAHEAD-1] = 0;

            // add points
            double tmp[2];
            int current = 0;
            for(k=0; k<LOOKAHEAD; k++) {
              tmp[0] = (double)current / (double)jointTotalTime[i];
              tmp[1] =  jointPoints[i][k]->angle;

              if(k<LOOKAHEAD-1) {
                current += jointPoints[i][k+1]->duration;
              }

              jointIpo[i]->setPoint(k, tmp);

              if(k<LOOKAHEAD-1) {
                jointDuration[i][k] = jointPoints[i][k+1]->duration;
              }
            }

            // init interpolation objects
            jointIpo[i]->start();
            jointIpo[i]->reset();
          }
        }
      }
    }

    // interpolation starts here
    ML(MUTEXINTERPOLATION);
    int ipos = 0;
    bool done = false;
    while( (ipos++ < CMotionContainer::interpolationLookAhead) && !done) {
      newMotionData = new CIpoData();

      // cartesian space
      for(k=0; k<global.robotInput.kinematicChains.length; k++) {
        if(!noLimb[k]) {
          CMatrix limbMatrix;
          // calc interpolated point
          limbIpo[k]->getPoint( (double)limbCurrentTime[k]/(double)limbTotalTime[k], &limbMatrix);

          // calc inverse kinematics
          innerRobot.calcInverseKinematics(k, limbMatrix);
          innerRobot.getAnglesFromDh(anglesTmp);

          // overwrite angles belonging to kinematic chain
          for(i=0; i<AX12_COUNT; i++) {
            if(global.ax12ToKinematicChain[i] == k) {
              anglesOld[i] = anglesTmp[i];
            }
          }

          // update elapsed time -> stop when limbTotalTime reached
          limbCurrentTime[k] += ipoPause;

          if(limbCurrentTime[k] >= limbTotalTime[k]) {
            done = true;
          }
        }
      }

      // add angles from jointangles
      for(k=0; k<AX12_COUNT; k++) {
        if(!noJoint[k]) {
          if(jointCurrentTime[k] < jointTotalTime[k]) {
            double tmp;
            jointIpo[k]->getPoint( (double)jointCurrentTime[k]/(double)jointTotalTime[k], &tmp);
            anglesOld[k] = (float)tmp;

            jointCurrentTime[k] += ipoPause;
          }
        }
      }
      // collision check
      for(k=0; k<AX12_COUNT; k++) {
        newMotionData->angles[k] = anglesOld[k];
      }

      innerRobot.setDhFromAngles(newMotionData->angles);
      innerRobot.calcForwardKinematics(false);
      newMotionData->collision = innerRobot.checkCollisions(false, false) ? CIpoData::COLLISION_YES : CIpoData::COLLISION_NO;

      newMotionData->checkCollision = noLimbs;
      for(k=0; k<global.robotInput.kinematicChains.length; k++) {
        if(!noLimb[k]) {
          newMotionData->checkCollision |= limbPoints[k * LOOKAHEAD + limbIpo[k]->getCurrentPosition()]->checkCollision;
        }
      }

      // add interpolated point to interpolated-list
      // will be processed by updaterobot()-thread
      interpolated.add(newMotionData);
    }
    MU(MUTEXINTERPOLATION);

    // update elapsed time of trajectory points
    // remove all reached points
    for(i=0; i<AX12_COUNT; i++) {
      if(!noJoint[i]) {
        for(j=0; j<LOOKAHEAD; j++) {
          if(jointCurrentTime[i] < jointDuration[i][j]) {
            jointPoints[i][j]->current = jointCurrentTime[i];
            break;
          }
          else {
            jointCurrentTime[i] -= jointDuration[i][j];
          }
        }

        // j = index of first unfinished trajectory point -> remove all < j
        if(j>0) {
          joints[i].removeFirst(true);
          for(k=1; (k<j) && (jointPoints[i][k] != jointPoints[i][k-1]); k++) {
            joints[i].removeFirst(true);
          }
        }
      }
    }

    // update elapsed time of trajectory points
    // remove all points in the past
    for(i=0; i<global.robotInput.kinematicChains.length; i++) {
      if(!noLimb[i]) {
        for(j=0; j<LOOKAHEAD; j++) {
          if(limbCurrentTime[i] < limbDuration[i * LOOKAHEAD + j]) {
            limbPoints[i * LOOKAHEAD + j]->current = limbCurrentTime[i];
            break;
          }
          else {
            limbCurrentTime[i] -= limbDuration[i * LOOKAHEAD + j];
          }
        }

        // j = index of first unfinished trajectory point -> remove all < j
        if(j>0) {
          trajectory[i].removeFirst(true);
          for(k=1; (k<j) && (limbPoints[i * LOOKAHEAD + k] != limbPoints[i * LOOKAHEAD + k-1]); k++) {
            trajectory[i].removeFirst(true);
          }
        }
      }
    }
    MU(MUTEXTRAJECTORY);

    ipoIn.free();
    ipoOut.free();

    // free interpolation object
    for(k=0; k<AX12_COUNT; k++) {
      if(jointIpo[k] != NULL) {
        delete jointIpo[k];
        jointIpo[k] = NULL;
      }
    }

    for(k=0; k<global.robotInput.kinematicChains.length; k++) {
      if(limbIpo[k] != NULL) {
        delete limbIpo[k];
        limbIpo[k] = NULL;
      }
    }

    // wait until too few interpolation points are left
    bool getNext = false;
    do {
      CPlatform::sleep(ipoPause);
      ML(MUTEXINTERPOLATION);
      getNext = interpolated.count() < 3;
      MU(MUTEXINTERPOLATION);
    } while(!getNext);
  }
}

// reads servo and sensor data from robot (every ipoPause ms)
// processes interpolated trajectory points in interpolated-list
// sends target-angles to robot
// collision check on read/target values
// speed check
#define SLEEP CPlatform::sleep(1) ;
void CMotionContainer::updateRobot() {
  float collisionangles[AX12_COUNT];
  float angles[AX12_COUNT];
  float targetangles[AX12_COUNT];

  CRobot *robot = &global.robotInput;

  CIpoData *motionData;
  int i, j, timeDiff;

  unsigned long time, writeTime;
  int timediff, timediffold;
  int counter = 0;
  int state = ipoState;
  bool collision = false;
  CVec*tmpVec = new CVec[2 * AX12_COUNT];             // * global.robotInput.kinematicChains.length*sizeof(CVec));

  for(i=0; i<AX12_COUNT; i++) {
    targetangles[i] = global.robotWrapper.Ax12s[i].getCurrentAngle();
  }
  
  writeTime = CPlatform::getTickCount();
  bool wroteData = false;
  //ipoState = IPO_OFF;
  //return ;
  while(true) {
              
    wroteData = false;
    // if ipoState = IPO_OFF -> pause thread
    ML(MUTEXMOTION);
    state = ipoState;
    ipoReturn = 1;
    MU(MUTEXMOTION);

    if(state == IPO_OFF) {
      CPlatform::sleep(ipoPause);
      continue;
    }

    // store starttime
    time = CPlatform::getTickCount();

    // get next interpolated trajectory point
    ML(MUTEXINTERPOLATION);
    motionData = (CIpoData*)interpolated.removeFirst(false);
    MU(MUTEXINTERPOLATION);
    

    if(global.readFromRobot) {
      // counter = 0 -> read servos
      // counter = 1 -> read sensors
      // ( serial communication too slow atm -> better read servos and sensors every frame )
      if(counter == 0) {
        // read servos and update robot-object
        ML(MUTEXCOMM);
        global.robotWrapper.readAx12s();
        robot->updateDhParameters();
        MU(MUTEXCOMM);
        SLEEP;

        // calc limb positions, display in 3d-viewer
        // if writing to robot is enabled or no new target-position is available
        robot->calcForwardKinematics( (motionData == NULL) );
        //global.writeToRobot || (motionData == NULL));

        // print current position and orientation of all limbs
        robot->getTCPs(tmpVec);
        global.console.printInfo(tmpVec);

        // collision detected
        // store current angle values -> compare with previous collision angles
        // major distance -> stop all movement, print error message
        if(robot->checkCollisions(true, true) ) {
          bool majorDist = false;
          for(i=0; i<AX12_COUNT; i++) {
            if(fabs(global.robotWrapper.Ax12s[i].getCurrentAngle() - collisionangles[i]) > 2.0) {
              majorDist = true;
              break;
            }
          }
          if(majorDist) {
            for(i=0; i<AX12_COUNT; i++) {
              collisionangles[i] = global.robotWrapper.Ax12s[i].getCurrentAngle();
            }

            if(global.stopOnCollisionRead) {
              CUtil::cout("stopping...\n");
              stop();
            }
          }
        }
        // next time check sensors
        if(AXS1_COUNT > 0) {
          counter = 1;
        }
      }
      else                                              //uncomment in order to check only servos OR sensor at a time
      if(counter == 1) {
        //  read sensors
        ML(MUTEXCOMM);
        global.robotWrapper.readAxs1s();
        MU(MUTEXCOMM);
        SLEEP;
        // next time check servos
        counter = 0;
      }
    }

    if(motionData == NULL) {
      for(i=0; i<AX12_COUNT; i++) {
        global.robotWrapper.Ax12s[i].torquePID.esum = 0.0;
        global.robotWrapper.Ax12s[i].speedPID.esum = 0.0;
      }
    }
    if(motionData != NULL) {
      for(i=0; i<AX12_COUNT; i++) {
        angles[i] = motionData->angles[i];
      }

      ML(MUTEXCOMM);

      // set target angles
      for(i=0; i<AX12_COUNT; i++) {
        // speed check -> limit speed value
        if(global.controlSpeed) {
          global.robotWrapper.Ax12s[i].controlSpeed(angles[i], (float)ipoPause, SPEED_MAX);
        }

        // torque check -> limit torque value
        if(global.controlTorque) {
          global.robotWrapper.Ax12s[i].controlTorque(angles[i], (float)ipoPause, TORQUE_MAX);
        }

        collisionangles[i] = global.robotWrapper.Ax12s[i].getCurrentAngle();

        // set target angle
        global.robotWrapper.Ax12s[i].setTargetAngle(angles[i]);

        // as long as writing is deactivated use target values as current values
        if(!global.writeToRobot) {
          global.robotWrapper.Ax12s[i].setCurrentAngle(angles[i]);
        }

        // store new target position
        targetangles[i] = angles[i];
      }

      // writing to robot disabled? -> display target pose in 3d-viewer
      if(!global.writeToRobot) {
        robot->setDhFromAngles(angles);
        robot->calcForwardKinematics(true);
      }
      MU(MUTEXCOMM);

      // print current position and orientation of all limbs
      if(!global.readFromRobot) {
        robot->getTCPs(tmpVec);
        global.console.printInfo(tmpVec);
      }

      // will target pose produce a collsion? -> check
      collision = false;

      // collisions have been calculated before

      if( (motionData != NULL) && motionData->checkCollision) {
        if(motionData->collision == CIpoData::COLLISION_YES) {
          collision = true;
        }
        else {
          // check next COLLISION_LOOKAHEAD ipoframes
          // collision within COLLISION_LOOKAHEAD * ipoPause = 5 * 30 = 150ms -> stop
          ML(MUTEXINTERPOLATION);
          CListIterator iter = interpolated.getIterator();
          CIpoData*data;
          int counter = CMotionContainer::collisionLookAhead;
          while( (counter-- > 0) &&
                 iter.hasNext() &&
                 !collision)
          {
            data = (CIpoData*)iter.next();
            if(data->collision == CIpoData::COLLISION_YES) {
              char str[255];
              sprintf(str, "Warning: Collision could occur in %d milliseconds.\n", (1 +  CMotionContainer::collisionLookAhead - counter) * ipoPause);
              CUtil::cout(str, TEXT_ERROR);
              collision = true;
            }
          }
          MU(MUTEXINTERPOLATION);
        }
      }

      if(collision) {
        // collision will occur!
        if(global.writeToRobot && global.stopOnCollisionWrite) {
          CUtil::cout("stopping...\n");
          stop();
        }
      }
      else if(global.writeToRobot) {
        timediff = (int)(CPlatform::getTickCount() - writeTime);

        while (true) 
        {
            timediff = (int)(CPlatform::getTickCount() - writeTime);      
            if (timediff >= ipoPause)
               break;

        }
        
        
        // no collision detected -> write to robot
        ML(MUTEXCOMM); 
        writeTime = CPlatform::getTickCount();
 
        global.robotWrapper.writeAx12s();
        
        wroteData = true;
        MU(MUTEXCOMM);
        SLEEP;
      }

      // clean up interpolated trajectory point
      if(motionData != NULL) {
        delete motionData;
        motionData = NULL;
      }
    } 
    
    if (!wroteData)
    {

    // sleep for ipoPause - (currenttime - lasttime) ms
    // -> robot will be updated approx. every ipoPause ms
    timediff =  (int)(CPlatform::getTickCount() - time);
    //printf("pc: %d\n", timediff);
    if(timediff >= ipoPause) {
      timediff = 1;
    }
    else if(timediff <= 0) {
      timediff = ipoMinPause;
    }
    else {
      timediff = ipoPause - timediff;
    }

    if(timediff < ipoMinPause) {
      timediff = ipoMinPause;
    }

    CPlatform::sleep(timediff);
    }
  }

  delete[] tmpVec;
  tmpVec = NULL;
}

// stop all movement -> clear trajectory and interpolated list
// TODO: clear cmdlist?
void CMotionContainer::stop()
{
  ML(MUTEXTRAJECTORY);
  for(int i=0; i<global.robotInput.kinematicChains.length; i++) {
    trajectory[i].clear(true);
  }

  for(int i=0; i<AX12_COUNT; i++) {
    joints[i].clear(true);
  }
  MU(MUTEXTRAJECTORY);

  ML(MUTEXINTERPOLATION);
  interpolated.clear(true);
  MU(MUTEXINTERPOLATION);

  ML(MUTEXMOTION);
  motionList.clear(true);
  MU(MUTEXMOTION);
}

// returns sum of keyframes of all slots in ids-array
// -> total number of frames
int CMotionContainer::getSequenceLength(int *ids, int len) {
  int res = 0;
  for(int i=0; i<len; i++) {
    res += motions[ids[i]].count();
  }
  return res;
}

void CMotionContainer::moveLink(int id, float angle, int time, bool sync) {
  id--;
  if( (id < 0) || (id >= AX12_COUNT) ) {
    char text[255];
    sprintf(text, "Servo #%d doesn't exist.", id);
    CUtil::cout(text, TEXT_ERROR);
    return;
  }

  int count;
  float currentAngle;

  if(sync) {
    ML(MUTEXTRAJECTORY);
  }

  count = joints[id].count();

  if(count == 0) {
    CAnglePoint*currentPoint = new CAnglePoint();

    ML(MUTEXCOMM);
    currentAngle = global.robotWrapper.Ax12s[id].getCurrentAngle();
#ifndef MOTION_USECURRENTANGLE
    float targetAngle = global.robotWrapper.Ax12s[id].getTargetAngle();
    if(fabs(targetAngle - currentAngle) <= MOTION_CURRENTANGLE_DIFF) {
      currentAngle = targetAngle;
    }
#endif
    MU(MUTEXCOMM);
    currentPoint->angle = currentAngle;

    currentPoint->checkCollision = true;
    currentPoint->current = 0;
    currentPoint->duration = time;

    global.motion.joints[id].add(currentPoint);
  }
  else
  if(count == 1) {
    CAnglePoint*currentListItem = (CAnglePoint*)global.motion.joints[id].getFirst()->data;
    currentListItem->current = 0;
  }

  CAnglePoint*newPoint = new CAnglePoint();

  newPoint->angle = angle;
  newPoint->checkCollision = true;
  newPoint->current = 0;
  newPoint->duration = time;

  joints[id].add(newPoint);

  if(sync) {
    MU(MUTEXTRAJECTORY);
  }
}

// adds a movement point (arml, armr, legl, legr) to the trajectory list
void CMotionContainer::moveLimb(int id, float x, float y, float z, float rotx, float roty, float rotz, int time, bool sync)
{
  if(time < ipoPause) {
    time = ipoPause;
  }

  // we need a robot to calc forward kinematics
  if( (id < 0) || (id >= global.robotInput.kinematicChains.length) ) {
    char text[255];
    sprintf(text, "Limb #%d doesn't exist.", id);
    CUtil::cout(text, TEXT_ERROR);
    return;
  }

  // start and target interpolation point
  CPoint *points[2];
  points[0] = NULL;
  points[1] = NULL;

  CMatrix tmp1, currentPose, targetPose;

  // supplied orientation rotx, roty, rotz refers to base coordinate frame
  // transform into limb coordinate frame

  // tweak, ugly, make clean
  switch(global.robotInput.id)
  {
  case ROBOT_HUMANOID:
  case ROBOT_HUMANOID_INVERSE_HIP:
    rotx += 90.0;
    roty += 90.0;
    break;
  }
  CMathLib::getRotation(tmp1, M_PI/180.0*rotx,
                        M_PI/180.0*roty,
                        M_PI/180.0*rotz);

  // assign target position
  tmp1.a[12] = x;
  tmp1.a[13] = y;
  tmp1.a[14] = z;

  // inside of sync-block?
  if(sync) {
    ML(MUTEXTRAJECTORY);
  }

  // trajectory list empty? -> add current position as start position
  if(trajectory[id].count() <= 0) {
    // assign real angular values to local robot object
    ML(MUTEXINPUT);
#ifdef MOTION_USECURRENTANGLE
    global.robotCalc.updateDhParameters(true);
#else
    global.robotCalc.updateDhParameters(false);
#endif
    MU(MUTEXINPUT);

    global.robotCalc.calcForwardKinematics(false);

    currentPose = global.robotCalc.kinematicChains.chain[id].getRelativeToBase();

    // create trajectory point
    points[0] = new CPoint();
    if(points[0] == NULL) {
      CUtil::cout("CUtil::legarm: malloc failed().\n", TEXT_ERROR);
      return;
    }
    points[0]->current = 0;
    points[0]->duration = time;
    global.robotCalc.getAnglesFromDh(points[0]->angles);
    points[0]->pose = currentPose;
    points[0]->checkCollision = true;

    CUtil::cout("Trajectory lists: add current!\n", TEXT_DEBUG);

    // add to trajectory list of limb
    trajectory[id].add( (void*)points[0]);

  }
  else if(trajectory[id].count() == 1) {
    // only one point in trajectory list -> we supply a new target position so reset
    // interpolation
    CUtil::cout("Trajectory lists: reset current!\n", TEXT_DEBUG);
    CPoint*currentListItem = (CPoint*)trajectory[id].getFirst()->data;
    currentListItem->current = 0;
  }

  if(global.addMotionsWithCollision) {
    // todo: check for collisions
  }

  // create target trajectory point
  points[1] = new CPoint();
  if(points[1] == NULL) {
    CUtil::cout("CUtil::legarm: malloc failed().\n", TEXT_ERROR);
    if(points[0] != NULL) {
      delete points[0];
    }
    return;
  }

  points[1]->current = 0;
  points[1]->duration = time;

  global.robotCalc.calcInverseKinematics(id, tmp1);
  global.robotCalc.getAnglesFromDh(points[1]->angles);
  points[1]->pose = tmp1;
  points[1]->checkCollision = true;

  // add to trajectory list of limb
  trajectory[id].add( (void*)points[1]);

  // inside sync-block?
  if(sync) {
    MU(MUTEXTRAJECTORY);
  }
}

