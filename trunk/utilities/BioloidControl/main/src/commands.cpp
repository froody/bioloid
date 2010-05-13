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
#include "../include/vars.h"
#include "../include/commands.h"

float CCommandOmniWalk::parameters[CONSOLE_PARAMETER_COUNT] = {
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

// container commands -> stores several command-objects
CCommandContainer::CCommandContainer(unsigned int size, CMotionContainer *m)
{
  motion = m;

  listCount = 0;
  list = NULL;
  list = (CCommand**)CUtil::realloc(list, size * sizeof(CCommand*) );
  if(list == NULL) {
    CUtil::cout("CCommandContainer: malloc failed().\n", TEXT_ERROR);
    return;
  }
  listCount = size;
  listCurrent = 0;
  done = false;
}

// free container -> free children
CCommandContainer::~CCommandContainer()
{
  for(int i=0; i<listCount; i++) {
    delete list[i];
    list[i] = NULL;
  }
  CUtil::free( (void**)&list);
}

// execute command-objects stored in list from 0 to listCount-1
int CCommandContainer::execute(CCommand*control)
{
  int result = 0;

  if(listCurrent == 0) {
    done = false;
  }

  if( (listCount > 0) && !done) {
    if(listCurrent < listCount) {
      result = list[listCurrent]->execute(control);

      if(list[listCurrent]->hasFinished() ) {
        listCurrent++;
      }
    }

    if(listCurrent == listCount) {
      done = true;
      listCurrent = 0;
    }
  }

  return result;
}

// execution complete?
bool CCommandContainer::hasFinished()
{
  return done;
}

// add command-object to list at index
void CCommandContainer::add(unsigned int index, CCommand*cmd)
{
  if(index < listCount) {
    list[index] = cmd;
  }
}

// executes a sequence of motiondata objects (stored segIds[], pointing to global.motion.motions[])
CCommandSequence::CCommandSequence(unsigned int size, bool sleep, CMotionContainer *m, bool addImmediate)
{
  motion = m;
  access = 0;
  doSleep = sleep;
  addImmediateMotion = addImmediate;

  done = false;
  for(int i=0; i<COMMAND_PARAM_COUNT; i++) {
    params[i] = 0;
  }

  seqCount = 0;
  seqIds = NULL;
  seqIds = (int*)CUtil::realloc(seqIds, size * sizeof(int) );
  if(seqIds == NULL) {
    CUtil::cout("CCommandSequence: malloc failed().\n", TEXT_ERROR);
    return;
  }

  seqCount = size;
}

// free id-array
CCommandSequence::~CCommandSequence()
{
  CUtil::free( (void**)&seqIds);
  seqCount = 0;
}

void CCommandSequence::setSleep(bool sleep)
{
  doSleep = sleep;
}

// set id at seqIds[index]
void CCommandSequence::setId(unsigned int index, unsigned int id)
{
  if(index < seqCount) {
    seqIds[index] = id;
  }
}

// set param at params[index]
void CCommandSequence::setParam(unsigned int index, unsigned int value)
{
  if(index < COMMAND_PARAM_COUNT) {
    params[index] = value;
  }
}

// execute sequence of motions stored in seqIds[]
// params[0] = loop count
// params[1] = time delay between frames
// returns playtime left
int CCommandSequence::execute(CCommand*control)
{
  int result = 0;

  if( (access < params[0]) || (motion == NULL) ) {
    if(addImmediateMotion && (seqCount > 0) ) {
      motion->stop();

     
    }
    /*
 CListIterator iter = global.motion.motions[seqIds[0]].getIterator();
         CMotionData* target = (CMotionData*) iter.next();
         global.motion.getIntermediateMotion(target, INTERMEDIATE_MOTION, INTERMEDIATE_IPOTYPE, params[1]);
         int tmp[1];
         tmp[0] = INTERMEDIATE_MOTION;
         motion->addSequenceToTrajectory(tmp, 1, 10000, false);// params[1], false);
    */
    motion->addSequenceToTrajectory(seqIds, seqCount, params[1], false);
    result = motion->getSequenceLength(seqIds, seqCount)*params[1];

    if(doSleep) {
      CPlatform::sleep(result);
      result = 0;
    }
    access++;
    done = false;
  }
  else {
    done = true;
    access = 0;
  }
  // TODO: check if target pose reached

  return result;
}

// execution complete?
bool CCommandSequence::hasFinished()
{
  return done;
}

int CCommandStop::execute(CCommand*control)
{
  int len;
  ML(MUTEXCMDS);
  len = global.motion.cmds.count();
  global.motion.cmds.clear(true);
  MU(MUTEXCMDS);

  if(len > 0) {
    return CCommandSequence::execute(control);
  }
  else {
    done = true;
    return 0;
  }
}

// normalizes 'val' to fit between -PI<= newval <= PI (sven behnke paper)
float CCommandOmniWalk::piCut(float val)
{
  while(val < -M_PI) {
    val += 2.0*M_PI;
  }

  while(val > M_PI) {
    val -= 2.0*M_PI;
  }

  return val;
}

// calcs angles (sven behnke paper)
void CCommandOmniWalk::calcAngles(float*angles, float clockLeg, float ls, CVec speed, float step, bool isLeft)
{
  float leglen = 0.16;
  float aShift, tShift, tLegShift, tFootShift;
  CVec aRobot(asin(speed.x/(step*leglen) ),
              asin(speed.y/(step*leglen) ),
              speed.z/step);

  CVec tLegSwing, tFootSwing;

  float clockShort, vShort, oShort, gShort, aShort;
  float tFootShort;
  float tLoad, vLoad, gLoad, clockLoad;

  //shifting
  aShift = parameters[0] + parameters[1]*sqrt(aRobot.y*aRobot.y+aRobot.x*aRobot.x) + parameters[2]*fabs(aRobot.x);

  //meee
  aShift *= parameters[3];

  tShift = (aShift) * sin(clockLeg);
  if( (clockLeg <= parameters[100]*M_PI/180.0) && (clockLeg >= -0.5 * M_PI + parameters[101]*M_PI/180.0) ) {
    //if (clockLeg <= 0 && clockLeg >= -0.5 * M_PI)
    tFootShift = -parameters[102]*(aShift) * sin(parameters[104]*clockLeg + parameters[103]*M_PI/180.0);
  }
  else {
    tFootShift = -parameters[4]*tShift;
  }


  if(sin(clockLeg) < 0) {
    tShift *=  +(parameters[35]) * sin(clockLeg);
  }
  else {
    ; //  tShift +=  + (parameters[35]*M_PI/180.0) * sin(clockLeg);

  }
  tLegShift = tShift;

  vShort = parameters[5];
  oShort = -parameters[6];
  aShort = parameters[7] + parameters[8] * sqrt(aRobot.x*aRobot.x+aRobot.y*aRobot.y);

  aShort *= parameters[9];

  clockShort = vShort * (clockLeg + parameters[10]*M_PI + oShort);
  if( (clockShort >= -M_PI) && (clockShort < M_PI) ) {
    gShort = -aShort*parameters[11]*(cos(clockShort) + 1);
  }
  else {
    gShort = 0.0;
  }

  if( (clockShort >= -M_PI) && (clockShort < M_PI) ) {
    tFootShort = -aRobot.y*parameters[12]*(cos(clockShort) + 1);
  }
  else {
    tFootShort = 0.0;
  }

  //loading
  float aLoad;
  vLoad = parameters[13];
  clockLoad = vLoad * piCut(clockLeg + M_PI/2.0 - M_PI/vShort + oShort) - M_PI;
  aLoad = parameters[14] + parameters[15] * (1 - cos(fabs(aRobot.y) ) );
  aLoad *= parameters[16];

  if( (clockLoad >= -M_PI) && (clockLoad < M_PI) ) {
    gLoad = -parameters[17] * aLoad * (cos(clockLoad) + 1);
  }
  else {
    gLoad = 0.0;
  }

  //Swinging
  float gSwing, tSwing, vSwing, oSwing, clockSwing, b;
  vSwing = parameters[18];
  oSwing = -parameters[19];

  clockSwing = vSwing * (clockLeg + M_PI/2.0 + oSwing);
  b = -parameters[20]/(2.0 * M_PI * vSwing - M_PI);

  if( (clockSwing >= -M_PI/2.0) && (clockSwing < M_PI/2.0) ) {
    tSwing = sin(clockSwing);
  }
  else if(clockSwing >= M_PI/2.0) {
    tSwing = b * (clockSwing - M_PI/2.0) + 1;
  }
  else {
    tSwing = b * (clockSwing + M_PI/2.0) - 1;
  }

  tLegSwing.set(ls * aRobot.x * tSwing,
                aRobot.y * tSwing,
                ls * aRobot.z * tSwing);
  tFootSwing.set(parameters[21] * aRobot.x * tSwing,
                 ls * parameters[21] * aRobot.y * tSwing,
                 0.0);

  //balance
  float footBalx, footBaly, legBalx;
  footBalx = parameters[22] *ls *aRobot.x *cos(clockLeg + parameters[23]);
  footBaly = parameters[24] + parameters[25]*aRobot.y -parameters[26] *aRobot.y *cos(parameters[27] * clockLeg + parameters[28]);
  legBalx = parameters[29] + ls * aRobot.x + fabs(aRobot.x) + parameters[30] * fabsf(aRobot.z);

  //output
  float g, legx, legy, legz, footx, footy;
  g = gShort + gLoad;
  legx = tLegSwing.x + tLegShift + legBalx;
  legy = tLegSwing.y;
  legz = tLegSwing.z;
  footx = tFootSwing.x + tFootShift + footBalx;
  footy = tFootSwing.y + tFootShort + footBaly;

  float nMin = parameters[31];
  float n = 1 + (1-nMin)*g;
  n *= parameters[32];
  // knee
  angles[3] = -2.0*acos(n);
  //yaw
  angles[0] = legz;
  angles[1] = legx -0.5*angles[3]*sin(angles[0]);

  angles[1] -= parameters[36];

  angles[2] = legy -0.5*angles[3]*cos(angles[0]);

  angles[4] = -0.5 * angles[3] + sin(angles[0])*(footx - legx)  + cos(angles[0]) * (footy - legy);
  if(sin(clockLeg) < 0) {
    angles[4] +=  +(parameters[56]) * M_PI/180.0 * sin(clockLeg);
  }
  else {
    angles[4] +=  +(parameters[57]) * M_PI/180.0 * sin(clockLeg);
  }
  //angles[4] +=  + (parameters[57]) * M_PI/180.0;


  angles[5] = cos(angles[0])*(footx - legx)  - sin(angles[0]) * (footy - legy);
  angles[5] = -angles[5];

  if(angles[5] <= parameters[58]*M_PI/180.0) {
    angles[5] = parameters[58]*M_PI/180.0;
  }


  angles[4] = angles[4] + parameters[34]*M_PI/180.0;
  angles[2] = -angles[2] - parameters[33]*M_PI/180.0;


  if(isLeft) {
    angles[6] = -parameters[50]*M_PI/180.0 - parameters[51]*M_PI/180.0*sin(0.5 *clockSwing);
    angles[7] = -(parameters[52]*M_PI/180.0 + parameters[53]*M_PI/180.0*sin(0.5 *clockSwing) );
    angles[8] = -(parameters[54]*M_PI/180.0 + parameters[55]*M_PI/180.0*sin(0.5 *clockSwing) );

  }
  else {
    angles[6] = parameters[50]*M_PI/180.0 + parameters[51]*M_PI/180.0*sin(0.5 *clockSwing);
    angles[7] = parameters[52]*M_PI/180.0 + parameters[53]*M_PI/180.0*sin(0.5 *clockSwing);
    angles[8] = parameters[54]*M_PI/180.0 + parameters[55]*M_PI/180.0*sin(0.5 *clockSwing);

  }
}

int  CCommandOmniWalk::execute(CCommand* control)
{
     int j;
  int id = MOTION_MAXUSER + 2;

  float clock = -M_PI/2.0;
  float step = 1.573;
  CMatrix left, right;


  int max = steps;
  
  if(max < 0) {
    max = 5;
  }

  float t = (float)max;

  max = (int)t;

  float rAngles[AX12_COUNT];
  int counter = 0;

  CMotionData *newData;
  global.motion.motions[id].clear();

  int clockCounter = 0;
  int clockMax = (int)parameters[38];
  int clockInit = clockMax / 4 - 1;
  clockCounter = clockInit;

  while(counter < max)
  {
    if(clockCounter >= clockMax + clockInit) {
      clockCounter = 0 + clockInit;
      counter++;
    }
    clock = -M_PI + 2.0*M_PI*clockCounter / clockMax;
    clockCounter++;

    right = global.robotCalc.kinematicChains.chain[1].pose;
    left  = global.robotCalc.kinematicChains.chain[0].pose;

    newData = new CMotionData();

    float angles[9];
    calcAngles(angles, piCut(clock - 0.5*M_PI), 1, speed, step, true);
    global.robotCalc.kinematicChains.chain[0].dhParameters[0].setAngle(0);
    for(int i=0; i<6; i++) {
      global.robotCalc.kinematicChains.chain[0].dhParameters[i+1].setAngle(angles[i]);
    }

    for(int i=0; i<3; i++) {
      global.robotCalc.kinematicChains.chain[2].dhParameters[i+1].setAngle(angles[6+i]);
    }


    calcAngles(angles, piCut(clock + 0.5*M_PI), 1, speed, step, false);

    angles[0] = angles[0];
    angles[1] = -angles[1];
    angles[2] = -angles[2];
    angles[3] = -angles[3];
    angles[4] = -angles[4];
    angles[5] = -angles[5];


    global.robotCalc.kinematicChains.chain[1].dhParameters[0].setAngle(0);
    for(int i=0; i<6; i++) {
      global.robotCalc.kinematicChains.chain[1].dhParameters[i+1].setAngle(angles[i]);
    }

    for(int i=0; i<3; i++) {
      global.robotCalc.kinematicChains.chain[3].dhParameters[i+1].setAngle(angles[6+i]);
    }

    global.robotCalc.getAnglesFromDh(newData->angles);
    newData->pose = new CMatrix[4];

    ML(MUTEXMOTION);
    global.motion.motions[id].add(newData);
    MU(MUTEXMOTION);

  }
  
  
  CListIterator iter = global.motion.motions[id].getIterator();
  CMotionData* target = (CMotionData*) iter.next();
  global.motion.getIntermediateMotion(target, INTERMEDIATE_MOTION, INTERMEDIATE_IPOTYPE, 1000);
  int tmp[2];
  tmp[0] = INTERMEDIATE_MOTION;
  tmp[1] = id;
  global.motion.playSequence(tmp,
                                 (int)parameters[37],
                                 13,
                                 1,
                                 2);
  //global.motion.addSequenceToTrajectory(tmp, 2, 10000, false);

  /*int *intptr = (int*)malloc(1 * sizeof(int) );
  intptr[0] = id;
  global.motion.playSequence(intptr,
                                 (int)parameters[37],
                                 13,
                                 1,
                                 1);
  */
  // command has finished
  return 0;
}
