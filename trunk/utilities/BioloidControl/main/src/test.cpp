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

#include <math.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <stdio.h>

#include "../include/wrapper.h"
#include "../include/robot.h"
#include "../include/constants.h"
#include "../include/types.h"
#include "../include/util.h"
#include "../include/platform.h"
#include "../include/cmd.h"
#include "../include/crc.h"
#include "../include/vars.h"
#include "../include/global.h"
#include "../include/motion.h"
#include "../include/test.h"

// normalizes 'val' to fit between -PI<= newval <= PI (sven behnke paper)
float piCut(float val)
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
void calcAngles(float*angles, float clockLeg, float ls, CVec speed, float step, bool isLeft)
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
  aShift = global.parameters[0] + global.parameters[1]*sqrt(aRobot.y*aRobot.y+aRobot.x*aRobot.x) + global.parameters[2]*fabs(aRobot.x);

  //meee
  aShift *= global.parameters[3];

  tShift = (aShift) * sin(clockLeg);
  if( (clockLeg <= global.parameters[100]*M_PI/180.0) && (clockLeg >= -0.5 * M_PI + global.parameters[101]*M_PI/180.0) ) {
    //if (clockLeg <= 0 && clockLeg >= -0.5 * M_PI)
    tFootShift = -global.parameters[102]*(aShift) * sin(global.parameters[104]*clockLeg + global.parameters[103]*M_PI/180.0);
  }
  else {
    tFootShift = -global.parameters[4]*tShift;
  }


  if(sin(clockLeg) < 0) {
    tShift *=  +(global.parameters[35]) * sin(clockLeg);
  }
  else {
    ; //  tShift +=  + (global.parameters[35]*M_PI/180.0) * sin(clockLeg);

  }
  tLegShift = tShift;

  vShort = global.parameters[5];
  oShort = -global.parameters[6];
  aShort = global.parameters[7] + global.parameters[8] * sqrt(aRobot.x*aRobot.x+aRobot.y*aRobot.y);

  aShort *= global.parameters[9];

  clockShort = vShort * (clockLeg + global.parameters[10]*M_PI + oShort);
  if( (clockShort >= -M_PI) && (clockShort < M_PI) ) {
    gShort = -aShort*global.parameters[11]*(cos(clockShort) + 1);
  }
  else {
    gShort = 0.0;
  }

  if( (clockShort >= -M_PI) && (clockShort < M_PI) ) {
    tFootShort = -aRobot.y*global.parameters[12]*(cos(clockShort) + 1);
  }
  else {
    tFootShort = 0.0;
  }

  //loading
  float aLoad;
  vLoad = global.parameters[13];
  clockLoad = vLoad * piCut(clockLeg + M_PI/2.0 - M_PI/vShort + oShort) - M_PI;
  aLoad = global.parameters[14] + global.parameters[15] * (1 - cos(fabs(aRobot.y) ) );
  aLoad *= global.parameters[16];

  if( (clockLoad >= -M_PI) && (clockLoad < M_PI) ) {
    gLoad = -global.parameters[17] * aLoad * (cos(clockLoad) + 1);
  }
  else {
    gLoad = 0.0;
  }

  //Swinging
  float gSwing, tSwing, vSwing, oSwing, clockSwing, b;
  vSwing = global.parameters[18];
  oSwing = -global.parameters[19];

  clockSwing = vSwing * (clockLeg + M_PI/2.0 + oSwing);
  b = -global.parameters[20]/(2.0 * M_PI * vSwing - M_PI);

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
  tFootSwing.set(global.parameters[21] * aRobot.x * tSwing,
                 ls * global.parameters[21] * aRobot.y * tSwing,
                 0.0);

  //balance
  float footBalx, footBaly, legBalx;
  footBalx = global.parameters[22] *ls *aRobot.x *cos(clockLeg + global.parameters[23]);
  footBaly = global.parameters[24] + global.parameters[25]*aRobot.y -global.parameters[26] *aRobot.y *cos(global.parameters[27] * clockLeg + global.parameters[28]);
  legBalx = global.parameters[29] + ls * aRobot.x + fabs(aRobot.x) + global.parameters[30] * fabsf(aRobot.z);

  //output
  float g, legx, legy, legz, footx, footy;
  g = gShort + gLoad;
  legx = tLegSwing.x + tLegShift + legBalx;
  legy = tLegSwing.y;
  legz = tLegSwing.z;
  footx = tFootSwing.x + tFootShift + footBalx;
  footy = tFootSwing.y + tFootShort + footBaly;

  float nMin = global.parameters[31];
  float n = 1 + (1-nMin)*g;
  n *= global.parameters[32];
  // knee
  angles[3] = -2.0*acos(n);
  //yaw
  angles[0] = legz;
  angles[1] = legx -0.5*angles[3]*sin(angles[0]);

  angles[1] -= global.parameters[36];

  angles[2] = legy -0.5*angles[3]*cos(angles[0]);

  angles[4] = -0.5 * angles[3] + sin(angles[0])*(footx - legx)  + cos(angles[0]) * (footy - legy);
  if(sin(clockLeg) < 0) {
    angles[4] +=  +(global.parameters[56]) * M_PI/180.0 * sin(clockLeg);
  }
  else {
    angles[4] +=  +(global.parameters[57]) * M_PI/180.0 * sin(clockLeg);
  }
  //angles[4] +=  + (global.parameters[57]) * M_PI/180.0;


  angles[5] = cos(angles[0])*(footx - legx)  - sin(angles[0]) * (footy - legy);
  angles[5] = -angles[5];

  if(angles[5] <= global.parameters[58]*M_PI/180.0) {
    angles[5] = global.parameters[58]*M_PI/180.0;
  }
  angles[5] *= global.parameters[72];

  angles[4] = angles[4] + global.parameters[34]*M_PI/180.0;
  angles[2] = -angles[2] - global.parameters[33]*M_PI/180.0;


  if(isLeft) {
    angles[6] = -global.parameters[50]*M_PI/180.0 - global.parameters[51]*M_PI/180.0*sin(0.5 *clockSwing);
    angles[7] = -(global.parameters[52]*M_PI/180.0 + global.parameters[53]*M_PI/180.0*sin(0.5 *clockSwing) );
    angles[8] = -(global.parameters[54]*M_PI/180.0 + global.parameters[55]*M_PI/180.0*sin(0.5 *clockSwing) );

  }
  else {
    angles[6] = global.parameters[50]*M_PI/180.0 + global.parameters[51]*M_PI/180.0*sin(0.5 *clockSwing);
    angles[7] = global.parameters[52]*M_PI/180.0 + global.parameters[53]*M_PI/180.0*sin(0.5 *clockSwing);
    angles[8] = global.parameters[54]*M_PI/180.0 + global.parameters[55]*M_PI/180.0*sin(0.5 *clockSwing);

  }
}

// test function, calculates angles based on a paper of Sven Behnke:
// "Online Trajectory Generation for Omnidirectional Biped Walking"
void CTest::humanoidWalk(float*params, int paramcount)
{
  int j;
  byte data[2];
  int id = 0;

  unsigned int limit = 1023;
  data[0] = (byte)(limit & 0x00FF); //(word) angle_in_degree;
  data[1] = (byte)(limit >> 8);
  for(j=0; j<AX12_COUNT; j++) {
    global.robotWrapper.Ax12s[j].setSpeedLimit(1023);
    global.robotWrapper.Ax12s[j].setTorqueLimit(1023);
  }

  float clock = -M_PI/2.0;
  float step = 1.573;
  CVec speed(0.0, 0.0, 0.0);
  CMatrix left, right;

  if(paramcount >= 1) {
    speed.y = params[0];
  }

  if(paramcount >= 3) {
    speed.x = params[2];
  }
  if(paramcount >= 4) {
    speed.z = params[3];
  }
  if(paramcount >= 5) {
    id = (int)params[4];
  }

  int max = 5;
  if(paramcount >= 2) {
    max = (int)params[1];
  }

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
  int clockMax = (int)global.parameters[38];
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

  if(paramcount < 5) {
    char str[255];
    sprintf(str, "play %d %d 13 ", id, (int)global.parameters[37]);
    global.console.processLine(str, strlen(str) );
    global.console.executeCmds();
  }
}

double sinl(double value)
{
  value = piCut(value);

  if(value <= -M_PI/2.0) {
    return -(value + M_PI) / (0.5 * M_PI);
  }
  else if(value <= 0) {
    return (value) / (0.5 * M_PI);
  }
  else if(value <= M_PI/2.0) {
    return (value) / (0.5 * M_PI);
  }
  else {
    return (M_PI - value) / (0.5 * M_PI);
  }
}

/*
   void calc(float clock, CMatrix &tmpMatrix, bool isLeftLeg)
   {
    CVec leg;
    CVec lego;

    if (!isLeftLeg)
       clock = piCut(clock + M_PI);

    double xoff = global.parameters[56];
    double yoff =  global.parameters[57];
    double zoff = global.parameters[58];
    leg.x = xoff;
    leg.y = yoff;
    leg.z = zoff;

    double xooff = 90 + global.parameters[59];
    double yooff = -90;
    double zooff = 0;
    lego.x = xooff*M_PI/180.0;
    lego.y = yooff*M_PI/180.0;
    lego.z = zooff*M_PI/180.0;

    if (!isLeftLeg)
    leg.x = -leg.x;

    clock = piCut(clock);

    double xshift, zswing = 0.0;

    // shiftclock is same as main clock
    double shiftclock = clock+ global.parameters[71] * M_PI/180.0;
    if (!isLeftLeg)
       ;//shiftclock = piCut(clock - M_PI);

    xshift = global.parameters[60] * sinl(shiftclock);
    if (!isLeftLeg)
    xshift =-xshift;

    double swingclock = piCut(clock + M_PI + global.parameters[70] * M_PI/180.0);
    if (global.parameters[73]* M_PI/180.0 <= swingclock && swingclock <= global.parameters[74]* M_PI/180.0)
    {
        double tmp = (swingclock - global.parameters[73]* M_PI/180.0) /
               (global.parameters[74]* M_PI/180.0 - global.parameters[73]* M_PI/180.0);

        zswing = global.parameters[61] * sin(tmp*-M_PI);
    }

    double yrotswingclock  = piCut(clock + M_PI - global.parameters[63] * M_PI/180.0);
    double yrotswing = 0;

    if (yrotswingclock >= 0)
    {
        yrotswing = global.parameters[62] * M_PI/180.0 * sinl(yrotswingclock) ;//(0.5 + 0.5 * sin(yrotswingclock - M_PI/2.0));
        if (isLeftLeg)
            yrotswing =-yrotswing;
    }

    double xrotswingclock  = piCut(clock + M_PI - global.parameters[65] * M_PI/180.0);
    double xrotswing = 0;

    if (xrotswingclock <= 0)
    {
        xrotswing = global.parameters[64] * M_PI/180.0 * sinl(xrotswingclock) ;//(0.5 + 0.5 * sin(yrotswingclock - M_PI/2.0));
    }

    double xswingclock  = piCut(clock + M_PI - global.parameters[67] * M_PI/180.0);
    double xswing = 0;

    if (global.parameters[75]* M_PI/180.0 <= xswingclock && xswingclock <= global.parameters[76]* M_PI/180.0)
    {
        double tmp = (xswingclock - global.parameters[75]* M_PI/180.0) /
               (global.parameters[76]* M_PI/180.0 - global.parameters[75]* M_PI/180.0);

        xswing = global.parameters[66] * sinl(tmp*-M_PI);

        if (isLeftLeg)
           xswing =-xswing;
    }

    double yshiftclock = piCut((clock + global.parameters[69]*M_PI/180.0));
    double yshift = 0.0;

    //if (yshiftclock >= 0)
    yshift = global.parameters[68] * (sinl( yshiftclock));


    CVec tmpo;
    // x hinten vor
    tmpo.x = lego.x + xrotswing;
    tmpo.y = lego.y + yrotswing;
    tmpo.z = lego.z;
    CMathLib::getRotation(tmpMatrix, tmpo);
    tmpMatrix.a[12] = leg.x + xshift + xswing;
    tmpMatrix.a[13] = leg.y - yshift;
    tmpMatrix.a[14] = leg.z - zswing;
   }
 */

#define REL(X, MIN, MAX) ( (X >= MIN && X <= MAX) ? (X - MIN) / (MAX - MIN) : 0.0)
#define PARAM(X) global.parameters[80 + X]
#define ANGLE(X) (global.parameters[80 + X]*M_PI/180.0)

void calc2(float clock, CMatrix &tmpMatrix, bool isLeftLeg)
{
  CVec leg;
  CVec lego;

  if(!isLeftLeg) {
    clock = piCut(clock + M_PI);
  }

  double xoff = global.parameters[56];
  double yoff =  global.parameters[57];
  double zoff = global.parameters[58];
  leg.x = xoff;
  leg.y = yoff;
  leg.z = zoff;

  double xooff = 90 + global.parameters[59];
  double yooff = -90;
  double zooff = 0;
  lego.x = xooff*M_PI/180.0;
  lego.y = yooff*M_PI/180.0;
  lego.z = zooff*M_PI/180.0;

  if(!isLeftLeg) {
    leg.x = -leg.x;
  }

  clock = piCut(clock);

  double xrotswing, yrotswing;
  double xshift, yshift, xswing, zswing;
  xrotswing = yrotswing = xshift = yshift = xswing = zswing = 0.0;
  double tmpclock;

  // swing up and down
  tmpclock = piCut(clock + ANGLE(0) );
  zswing = PARAM(1) * sin(REL(tmpclock, ANGLE(2), ANGLE(3) ) * M_PI);

  // shift front back
  if(!isLeftLeg) {
    tmpclock = piCut(clock + ANGLE(4) + ANGLE(-1) );
  }
  else {
    tmpclock = piCut(clock + ANGLE(4) );
  }
  yshift = PARAM(5) * sinl(tmpclock);

  // shift left right
  tmpclock = piCut(clock + ANGLE(8) );
  xshift = PARAM(9) * sinl(tmpclock);
  if(!isLeftLeg) {
    xshift = -xshift;
  }

  // ankle rotation (body swings left right)
  tmpclock = piCut(clock + ANGLE(12) );
  xrotswing = PARAM(13) * REL(tmpclock, ANGLE(14), ANGLE(15) );


  // ankle rotation (body swing front back)

  CVec tmpo;
  // x hinten vor
  tmpo.x = lego.x + xrotswing;
  tmpo.y = lego.y + yrotswing;
  tmpo.z = lego.z;
  CMathLib::getRotation(tmpMatrix, tmpo);
  tmpMatrix.a[12] = leg.x + xshift + xswing;
  tmpMatrix.a[13] = leg.y - yshift;
  tmpMatrix.a[14] = leg.z - zswing;
}

void calc(float clock, CMatrix &tmpMatrix, bool isLeftLeg)
{
  CVec leg;
  CVec lego;

  if(!isLeftLeg) {
    clock += M_PI;
  }

  clock = piCut(clock + M_PI);


  double xoff = global.parameters[56];
  double yoff =  global.parameters[57];
  double zoff = global.parameters[58];
  leg.x = xoff;
  leg.y = yoff;
  leg.z = zoff;

  double xooff = 90 + global.parameters[59];
  double yooff = -90;
  double zooff = 0;
  lego.x = xooff*M_PI/180.0;
  lego.y = yooff*M_PI/180.0;
  lego.z = zooff*M_PI/180.0;

  if(!isLeftLeg) {
    leg.x = -leg.x;
  }

  clock = piCut(clock);

  double xrotswing, yrotswing;
  double xshift, yshift, xswing, zswing;
  xrotswing = yrotswing = xshift = yshift = xswing = zswing = 0.0;
  double tmpclock, tmpf;

  tmpclock = piCut(clock + ANGLE(0) );
  xshift = 37 + 38 * sin(tmpclock);
  xswing = 26 + 5 * sin(3*tmpclock);
  tmpf = 20*M_PI/180.0;
  if(fabs(tmpclock + 0.5 * M_PI) <= tmpf) {
    xshift += xswing;
  }

  if(!isLeftLeg) {
    xshift =-xshift;
  }

  tmpclock = piCut(clock + ANGLE(4) );

  /*
     if (clock <= -120.0 * M_PI/180.0)
     {
     tmpf = 20.0 / 60.0 * M_PI/180.0;
     zswing = tmpf * (clock - -M_PI);
     }
     else
     {
     tmpf = -20.0 / 300.0 * M_PI/180.0;
     zswing = 20.0 + tmpf * (clock - -120.0 * M_PI/180.0);
     }
     zswing = 20.0 - zswing;
   */
  double height = 25;
  if(tmpclock <= -120.0 * M_PI/180.0) {
    tmpf = height / (60.0 * M_PI/180.0);
    zswing = tmpf * (tmpclock +M_PI);
  }
  else {
    tmpf = -height / (300.0 * M_PI/180.0);
    zswing = height + tmpf * (tmpclock - -120.0 * M_PI/180.0);
  }

  tmpclock = M_PI + piCut(clock + ANGLE(8) );
  tmpf = 20.0 * M_PI/180.0;

  if(tmpclock <= 240*M_PI/180.0 - 2*tmpf) {
    yshift = -40 + 30 * sin(1.5 * (tmpclock + tmpf) );
  }
  else {
    yshift = -40 + 30 * sin(1.5 * (240*M_PI/180.0 - tmpf) );
    double tmpf2 = -40 + 30 * sin(1.5 * (tmpf) );
    yshift = yshift + (tmpf2 - yshift) * (tmpclock - (240*M_PI/180.0 - 2*tmpf) ) / (2.0 * M_PI -(240*M_PI/180.0 - 2*tmpf) );
  }

  // ankle rotation (body swings left right)
  tmpclock = piCut(clock + ANGLE(12) );
  yrotswing = PARAM(13) * REL(tmpclock, ANGLE(14), ANGLE(15) ) * M_PI/180.0;
  if(!isLeftLeg) {
    yrotswing =-yrotswing;
  }
  printf("%g %g %g \n", yrotswing, PARAM(13), REL(tmpclock, ANGLE(14), ANGLE(15) ) );
  //yshift = -yshift;

  // ankle rotation (body swings left right)
  tmpclock = piCut(clock + ANGLE(16) );
  xrotswing = PARAM(17) * REL(tmpclock, ANGLE(18), ANGLE(19) );



  CVec tmpo;
  // x hinten vor
  tmpo.x = lego.x + xrotswing;
  tmpo.y = lego.y + yrotswing;
  tmpo.z = lego.z;
  CMathLib::getRotation(tmpMatrix, tmpo);
  tmpMatrix.a[12] = xshift;
  tmpMatrix.a[13] = yshift;
  tmpMatrix.a[14] = leg.z + zswing;
}

void CTest::humanoidWalkOwn(float*params, int paramcount)
{
  CRobot robot;
  robot.init(global.dhFilename);
  robot.updateDhParameters();

  double clock = 0;
  double diff = M_PI/5.0;

  int counter = 0;
  int max = 40;

  if(paramcount > 0) {
    max = (int)params[0] * 10;
  }


  CMotionData *newData;
  global.motion.motions[0].clear();

  CMatrix tmpMatrix;


  while(counter < max)
  {
    clock = piCut(clock);

    double xshift, zswing = 0.0;

    double swingclock = piCut(clock);

    calc(swingclock, tmpMatrix, true);
    robot.calcInverseKinematics(0, tmpMatrix);

    calc(swingclock, tmpMatrix, false);
    robot.calcInverseKinematics(1, tmpMatrix);

    newData = new CMotionData();

    swingclock += 0.5 * M_PI;
    robot.kinematicChains.chain[2].dhParameters[1].setAngle(-global.parameters[50]*M_PI/180.0 - global.parameters[51]*M_PI/180.0*sin(swingclock + M_PI) );
    robot.kinematicChains.chain[2].dhParameters[2].setAngle(-(global.parameters[52]*M_PI/180.0 + global.parameters[53]*M_PI/180.0*sin(swingclock + M_PI) ) );
    robot.kinematicChains.chain[2].dhParameters[3].setAngle(-(global.parameters[54]*M_PI/180.0 + global.parameters[55]*M_PI/180.0*sin(swingclock + M_PI) ) );

    robot.kinematicChains.chain[3].dhParameters[1].setAngle(global.parameters[50]*M_PI/180.0 + global.parameters[51]*M_PI/180.0*sin(swingclock) );
    robot.kinematicChains.chain[3].dhParameters[2].setAngle( (global.parameters[52]*M_PI/180.0 + global.parameters[53]*M_PI/180.0*sin(swingclock) ) );
    robot.kinematicChains.chain[3].dhParameters[3].setAngle( (global.parameters[54]*M_PI/180.0 + global.parameters[55]*M_PI/180.0*sin(swingclock) ) );

    robot.getAnglesFromDh(newData->angles);

    ML(MUTEXMOTION);
    global.motion.motions[0].add(newData);
    MU(MUTEXMOTION);

    clock += diff;
    counter++;
  }

  char str[255];
  sprintf(str, "play 0 %d 1 ", (int)global.parameters[37]);
  global.console.processLine(str, strlen(str) );
  global.console.executeCmds();
}

// ptp movement (sinus wave acceleration)
void writePositionDataPTPPrepareSinus(float VMAX, float BMAX, unsigned long ipoTime, unsigned long totalTime, unsigned long time, unsigned long max, unsigned long param, float *prev, float*readBuffer, float*writeBuffer, float*tbs, float*tes, int*sgns, float*vmaxs, float*bmaxs)
{
  int i;
  float current, target;

  float MAXS = 680 / 10.0;
  float TIME = totalTime;
  float BTIME = (float)time / 100.0;
  // float VMAX = 0.75;//(MAXS / ((1.0 - BTIME) * TIME)) ;
  //float BMAX = 0.05;//(2.0 * VMAX / (BTIME * TIME));

  // printf("vmax: %g bmax: %g\n", VMAX, BMAX);
  float vmax, bmax, se;
  long int te, tb, tv, sgn;
  int ipo = ipoTime;

  // prepare write buffer -> unpack data from serial and store bytewise in buffer
  for(i=0; i<AX12_COUNT; i++) {
    target = readBuffer[i];
    current = prev[i];

    vmax = VMAX;
    bmax = BMAX;
    se = (target - current);
    if(se < 0) {
      sgn = -1;
      se = -se;
    }
    else { sgn = 1;}

    if(vmax*vmax > 0.5 * bmax * se) {
      //printf("too large\n");

      vmax =  sqrt(0.5 * bmax * se);

      tb = ( (long int)(2.0 * vmax / (bmax * ipo) ) ) * ipo;
      tv = (long int)(TIME - tb); // cast by mdda
    }
    else {
      tb = ( (long int)(2.0 * vmax / (bmax * ipo) + 0.5) ) * ipo;
      tv = (long int)(TIME - tb); // cast by mdda
      if(i==0) {
        ; // printf("tv: %d %g\n", tv, (( se / (vmax * (float)ipo) + 0.5)) * ipo);
      }
    }

    if(tb == 0) {
      tb = 1;
    }
    if(tv == 0) {
      tv = 1;
    }

    te = tv + tb;
    vmax = se / (float)tv;
    bmax = 2.0 * vmax / (float)tb;
    if(i==0) {
      ; // printf("tb: %d tv: %d vmax: %g bmax: %g\n", tb, tv, vmax, bmax);
    }
    tes[i] = te;
    sgns[i] = sgn;
    tbs[i] = tb;
    vmaxs[i] = vmax;
    bmaxs[i] = bmax;
  }
}

float grinding(float x, float y1, float dy1, float y2, float dy2)
{
  float a, b, c, d;
  d = y1;
  c = dy1;
  a = dy2 + dy1 -2*y2 + 2*y1;
  b = 3*y2 - 2*dy1 - 3*y1 -dy2;

  /*
     a*x^3 +b*x^2+c*x+d
     3*a*x^2 + 2*b*x + c

     d = y1
     a + b + c + d = y2
     c = dy1
     3*a+ 2*b + c = dy2
     -----------------
     a + b + dy1 + y1 = y2
     3*a+ 2*b + dy1 = dy2
     ---------------------
     b = y2 - dy1 - y1 - a
     3*a+ 2*b + dy1 = dy2
     ---------------------
     3*a + 2*y2 -2*dy1 - 2*y1 - 2* a +dy1 = dy2
     ---------------------
     a = dy2 - 2*y2 + 2*dy1 + 2*y1 - dy1
     a = dy2 - 2*y2 + dy1 + 2*y1
     b = y2 - dy1 - y1 - (dy2 - 2*y2 + 2*dy1 + 2*y1 - dy1)
     b = y2 - dy1 - y1 -  dy2 + 2*y2 - 2*dy1 - 2*y1 + dy1
     b = 3 *y2 - 3*y1-2*dy1-dy2

     -------------------------
     a = dy2 - 2*y2 + dy1 + 2*y1
     b = 3 *y2 - 3*y1-2*dy1-dy2
   */
  return d + x * (c + x * (b + x * a) );
}

// ptp movement (sinus wave acceleration)
void writePositionData(FILE*writeFile, unsigned long readPause, float *prev, float*readBuffer, float*writeBuffer, int index, int totalipo, float*tb, float*te, int*sgn, float*vmax, float*bmax)
{
  int i;
  long int current, target, next;
  float se, t, tv;
  float pos = (float)index / (float)totalipo;
  // prepare write buffer -> unpack data from serial and store bytewise in buffer
  for(i=0; i<AX12_COUNT; i++) {
    current = (long int)prev[i];
    target = (long int)readBuffer[i];
    next = (long int)writeBuffer[i];
    int maxdiff = 1;

    /*
       if (index > totalipo - maxdiff)
       {
       float y1, dy1, y2, dy2;

       dy1 = (target - current);
       dy2 = (next - target);

       y1 = current + ((float)(target - current) * (float)(index-1) / (float) totalipo);
       y2 = target + ((float)(next - target) * (float)(1) / (float) totalipo);
       unsigned int target2 = grinding(0.5, y1, dy1, y2, dy2);

       if (i == 0)
       ;// printf("%d %d %d: %d %g %g %g %g\n", current, target, next, (target - target2), y1, dy1, y2, dy2);

       current = target2;
       } else
     */
    current +=  (long int)( (float)(target - current) * pos);

    if(current > 1023) {
      current = 1023;
    }
    else if(current < 0) {
      current = 0;
    }

    if(i==0) {
      char buffer[255];
      sprintf(buffer, "%d\n", current);
      printf(buffer);
      fputs(buffer, writeFile);
    }
  }
}

// ptp movement (sinus wave acceleration)
void writePositionDataPTPSinus(FILE*writeFile, unsigned long readPause, float*prev, float*readBuffer, float*writeBuffer,  int index, int totalipo, float*tb, float*te, int*sgn, float*vmax, float*bmax)
{
  int i;
  long int current;
  float se, t, tv;

  float pos = (float)index / (float)totalipo;

  // prepare write buffer -> unpack data from serial and store bytewise in buffer
  for(i=0; i<AX12_COUNT; i++) {
    current = (long int)prev[i];

    t = pos * te[i];
    tv = te[i] - tb[i];
    if(t <= tb[i]) {
      se = bmax[i] * (0.25 * t * t + (tb[i] * tb[i] / (8.0 * M_PI * M_PI) ) * (cos(2.0*M_PI*t/tb[i])-1.0) );
    }
    else if(t <= te[i] - tb[i]) {
      se = vmax[i] * (t - 0.5 * tb[i]);
    }
    else {
      se = 0.5 * bmax[i] * (te[i] * (t + tb[i]) - 0.5*(t*t + te[i]*te[i] + 2.0 * tb[i]*tb[i]) + (tb[i]*tb[i]/(4.0 * M_PI * M_PI) )*(1.0 - cos( (2.0*M_PI/tb[i])*(t - tv) ) ) );
    }

    current +=  sgn[i] * (long int)(se);

    if(current > 1023) {
      current = 1023;
    }
    else if(current < 0) {
      current = 0;
    }

    if(i==0) {
      char buffer[255];
      sprintf(buffer, "%d\n", current);
      printf(buffer);
      fputs(buffer, writeFile);
    }
  }
}

void CTest::run(float*args, int argcount)
{
  humanoidWalk(args, argcount);
  return;

  float current[AX12_COUNT];
  float target[AX12_COUNT];
  float previous[AX12_COUNT];

  previous[0] = args[4];
  current[0] = args[0];
  target[0] = args[1];

  // important
  unsigned long readPause = 8; //pause between keyframes in milliseconds
  int totalipo  = 10; // total number of keyframes
  unsigned long time = 25; // acceleration end time 25% * totaltime



  unsigned long max = 100; //
  unsigned long param = 0;
  float tbs[AX12_COUNT];
  float tes[AX12_COUNT];
  int sgns[AX12_COUNT];
  float vmaxs[AX12_COUNT];
  float bmaxs[AX12_COUNT];

  FILE *file;
  char str[255];
  sprintf(str, "../docs/data_%g_%g.txt", global.parameters[0], global.parameters[1]);
  file = fopen(str, "a+");

  writePositionDataPTPPrepareSinus(args[2], args[3], readPause, totalipo * readPause, time, max, param, previous, current, target, tbs, tes, sgns, vmaxs, bmaxs);


  for(int i=1; i<=totalipo; i++) {
    writePositionData(file, readPause, previous, current, target, i, totalipo, tbs, tes, sgns, vmaxs, bmaxs);
  }

  previous[0] = current[0];
  current[0] = target[0];
  target[0] = 0;
  writePositionDataPTPPrepareSinus(args[2], args[3], readPause, totalipo * readPause, time, max, param, previous, current, target, tbs, tes, sgns, vmaxs, bmaxs);


  for(int i=1; i<=totalipo; i++) {
    writePositionData(file, readPause, previous, current, target, i, totalipo, tbs, tes, sgns, vmaxs, bmaxs);
  }

  fclose(file);
}

/*
   void CTest::run(float* args, int argcount)
   {
     spiderWalk(args, argcount);
   }
 */

/*
   void CLeg::process(CLeg *legs)
   {
    int doSwing = 0;

    if (isSwinging && isFront)
        doSwing = -1;
    else if (!isSwinging && isBack)
          doSwing = 1;


    if (next >= 0)
    {
        if (!isSwinging)
        {
           // stance, rule 1
           if (true)
           if (isBack)
           {
               // stance end
               if (legs[next].onGround == false &&  !legs[next].isFront)
               {
                   // prolong stance
                   //target.y = target.y + offset;
                   doSwing -= 2;
               };
           }

           // rule 2
           if (true)
           if (legs[next].onGround == true && !legs[next].isBack)
           {
                   // prolong stance
                   doSwing += 2;
           }
        }
    }

    if (prior >= 0)
    {
        // rule 3
        if (true)
        if (!isSwinging && isBack)
        {
           if (legs[prior].onGround == true && !legs[prior].isBack)
           {
                   // prolong stance
                   doSwing += 2;
           }
        }
    }

    // across

    // rule 3
    if (false)
        if (!isSwinging && isBack)
        {
           if (legs[across].onGround == true && legs[across].isBack)
           {
                   // prolong stance
                   doSwing += 2;
           }
        }

    // rule 2
    if (false)
    if (legs[across].onGround == true && !legs[across].isBack)
           {
                   // prolong stance
                   doSwing += 2;
           }

    if (id > 2)
    isSwinging = !legs[id-3].isSwinging;
    else
    {
    if (doSwing > 0)
    isSwinging = true;
    else if (doSwing < 0)
         isSwinging = false;
    }


    CVec tmp = current - oldCurrent;
    float tmpf = sqrt(tmp | tmp);

    if (tmpf < 15.0)
       oldCounter++;
    else oldCounter--;

    if (oldCounter < 0)
       oldCounter = 0;

    if (oldCounter > 2 && isSwinging)
    {
       printf("leg %d: %d\n", id, oldCounter);
       isSwinging = false;
       oldCounter  = 0;
    }

    current = target;

    if (isSwinging)
    {
       //if (!isFront && current.z >= legUp + 10)
       // if (isBack && current.z >= legUp + 10 && current.y <= 0)
       if (current.z >= legUp + offset)
       {
          target.z -= offset;
       } else
       {
          target.y += 1.5*offset;
       }
    } else if (!isSwinging)
    {
       // on ground
       if (!onGround)
       {
           target.z += offset;
       } else
       {
           target.y -= 1.5*offset;
       }
    }

    if (id >= 3)
       target.x = - (legInner + legOuter) / 2.0;
    else
        target.x = (legInner + legOuter) / 2.0;;

    if (target.z > legDown)
       target.z = legDown;
    else if (target.z < legUp)
         target.z = legUp;

    if (true)
    if (target.y > legFront + offset)
       target.y = legFront + offset;
    else if (target.y < legBack - offset)
         target.y = legBack - offset;

         if (false)
    if (target.x > legOuter)
       target.x = legOuter;
    else if (target.x < legInner)
         target.x = legInner;

   }
 */

CLeg::CLeg()
{
  onGround = true;
  isFront = isBack = isSwinging = false;
  doSwing = doStance = false;

  stanceCounter = swingCounter = 0.0;

  legDown = 130.0;
  legUp = 60.0;
  legFront = 25.0;
  legBack = -25.0;
  legInner = 50.0;
  legOuter = 50.0;

  /*legDown = 130.0;
     legUp = 60.0;
     legFront = 25.0;
     legBack = -25.0;
     legInner = 50.0;
     legOuter = 50.0; */

  offset = 5.0;
  offsetCounter = 1.0;
  maxCounter = 50.0;

  oldCounter = 0;
}

void CLeg::preprocess()
{
  if(stanceCounter < 0.0) {
    stanceCounter = 0.0;
  }
  if(swingCounter < 0.0) {
    swingCounter = 0.0;
  }

  onGround = (current.z > legDown - 5);

  // bewegungen muessen fuer alle beine gleich lang sein
  if( (id == 1) || (id == 4) ) {
    isFront = current.y > legFront  -5; //+ 7;
  }
  else {
    isFront = current.y > legFront - 5;
  }

  if( (id == 1) || (id == 4) ) {
    isBack = current.y < legBack + 5; //-7;
  }
  else {
    isBack = current.y < legBack + 5;
  }

  doSwing = doStance = false;

  if(id == 2) {
    //printf("Leg %d: S: %d G: %d F: %d B: %d\n", id, isSwinging, onGround, isFront, isBack);
  }
}

void CLeg::process(CLeg *legs)
{
  // rules
  //return;

  float muh = 1.0;

  if(true) {
    if(isSwinging && (current.y > 0) ) {
      stanceCounter += muh * offsetCounter;
      //swingCounter -= muh * offsetCounter;
    }
    else if(!isSwinging && current.y < 0) {
      //swingCounter += muh* offsetCounter;
      stanceCounter += muh * offsetCounter;
    }
  }

  if(prior >= 0) {
    // 1
    if(true) {
      if(isSwinging) {
        //legs[prior].doStance = true;
        if(!legs[prior].isSwinging) {
          legs[prior].stanceCounter -= offsetCounter;
        }
        //legs[prior].swingCounter -= offsetCounter;
      }
    }
    // 2
    if(false) {
      if(!isSwinging && isFront && !legs[prior].isSwinging) {
        //legs[prior].doSwing = true;
        legs[prior].stanceCounter += offsetCounter;
      }
    }
  }

  if(next >= 0) {
    // 3
    if(true) {
      if(!isSwinging && !legs[next].isSwinging) {
        //legs[next].doSwing = true;
        legs[next].stanceCounter += offsetCounter;
      }
    }
  }

  if(false) {
    if(across >= 0) {
      // 2
      if(!isSwinging && isFront && !legs[across].isSwinging) {
        //legs[prior].doSwing = true;
        legs[across].stanceCounter += offsetCounter;
      }

      // 3
      if( (id != 1) && (id != 4) ) {
        if(!isSwinging && (current.y < 0) && !legs[across].isSwinging) {
          //legs[next].doSwing = true;
          legs[across].stanceCounter += current.y * offsetCounter;
        }
      }

    }
  }

  if(stanceCounter > maxCounter) {
    isSwinging = !isSwinging;
    stanceCounter = 0;
  }
}

void CLeg::update(CLeg *legs)
{
  int count = 0;
  for(int i=0; i<6; i++) {
    if(legs[i].onGround) {
      count++;
    }
  }

  if(doSwing && doStance) {
    printf("Collision!\n");
  }

  if(id == 2) {
    printf("stance: %g swing %g\n", stanceCounter, swingCounter);
  }

  if(false) {
    if(isSwinging) {
      if(stanceCounter > maxCounter) {
        isSwinging = false;
        stanceCounter = 0.0;
        swingCounter = 0.0;
      }

      /*if (!isBack && !doSwing && doStance)
         isSwinging = false;
         else if (isFront)
           isSwinging = false;*/
    }
    else {
      if(stanceCounter> maxCounter) {
        isSwinging = true;
        stanceCounter = 0.0;
        swingCounter = 0.0;
      }

      /*
         {
         if (!isFront && doSwing && !doStance)
         isSwinging = true;
         else if (isBack)
           isSwinging = true;
         }*/
    }
  }

  /*
     if (isSwinging)
     {
      if (!isBack && !doSwing && doStance)
         isSwinging = false;
      else if (isFront)
           isSwinging = false;
     } else
     {

      {
      if (!isFront && doSwing && !doStance)
         isSwinging = true;
      else if (isBack)
           isSwinging = true;
      }
     }
   */

  CVec tmp = current - oldCurrent;
  float tmpf = sqrt(tmp | tmp);

  if(tmpf < offset / 2.0) {
    oldCounter++;
  }
  else { oldCounter--;}

  if(oldCounter < 0) {
    oldCounter = 0;
  }

  if(false) {
    if( (oldCounter > 5) && isSwinging) {
      printf("Leg %d: %d\n", id, oldCounter);
      isSwinging = false;
      oldCounter  = 0;
    }
  }

  float tmpup = 2.0;
  float tmpdown = 4.0;

  // motion
  if(isSwinging) {
    if(current.z >= legUp + offset) {
      target.z -= tmpup*offset;
    }
    else {
      target.y += offset;
    }
  }
  else {
    if(!onGround) {
      target.z += tmpdown*offset;
    }
    else {
      target.y -= offset;
    }
  }

  if(id >= 3) {
    target.x = -(legInner + legOuter) / 2.0;
  }
  else {
    target.x = (legInner + legOuter) / 2.0;
  };

  if(true) {
    if(target.z > legDown + offset) {
      target.z = legDown + offset;
    }
    else if(target.z < legUp - offset) {
      target.z = legUp - offset;
    }
  }

  if(true) {
    if(target.y > legFront + offset) {
      target.y = legFront + offset;
    }
    else if(target.y < legBack - offset) {
      target.y = legBack - offset;
    }
  }

}

void CTest::spiderWalk(float*params, int paramcount)
{
  bool onRobot = global.writeToRobot;

  global.motion.enableIpo(false);
  printf("Start!\n");
  int i, j, k, counter;
  int ids[6];
  float angles[AX12_COUNT];

  for(i=0; i<6; i++) {
    legs[i].base = global.robotInput.kinematicChains.chain[i].frames[0]->getRelativeToBase()[3];
  }

  legs[0].isSwinging = false;
  legs[1].isSwinging = false;
  legs[2].isSwinging = true;
  legs[3].isSwinging = false;
  legs[4].isSwinging = false;
  legs[5].isSwinging = false;

  float offset =  40.0;
  float offset2 = 0.0;

  legs[0].id = 0;
  legs[0].prior = -1;
  legs[0].next = 1;
  legs[0].across = 3;
  legs[0].base.y += offset;

  legs[1].id = 1;
  legs[1].prior = 0;
  legs[1].next = 2;
  legs[1].across = 4;
  legs[1].base.y += offset2;

  legs[2].id = 2;
  legs[2].prior = 1;
  legs[2].next = -1;
  legs[2].across = 5;
  legs[2].base.y -= offset;

  legs[3].id = 3;
  legs[3].prior = -1;
  legs[3].next = 4;
  legs[3].across = 0;
  legs[3].base.y += offset;

  legs[4].id = 4;
  legs[4].prior = 3;
  legs[4].next = 5;
  legs[4].across = 1;
  legs[4].base.y += offset2;

  legs[5].id = 5;
  legs[5].prior = 4;
  legs[5].next = -1;
  legs[5].across = 2;
  legs[5].base.y -= offset;

  unsigned long time;
  int timediff;
  counter = 0;


  if(!onRobot) {
    global.robotWrapper.readAx12s();
    global.robotInput.updateDhParameters();
  }



  while(counter < 200)
  {
    time = CPlatform::getTickCount();

    if(onRobot) {
      global.robotWrapper.readAx12s();
    }

    global.robotInput.updateDhParameters(!onRobot);
    global.robotInput.calcForwardKinematics(true);

    for(i=0; i<6; i++) {
      legs[i].oldCurrent = legs[i].current;
      CVec tmp = global.robotInput.kinematicChains.chain[i].getRelativeToBase()[3];
      legs[i].current = tmp - legs[i].base;

      if(false) {
        if(!onRobot) {
          legs[i].current.x += (rand() % 10) - 5;
          legs[i].current.y += (rand() % 10) - 5;
          legs[i].current.z += (rand() % 10) - 5;
        }
      }

    }
    for(i=0; i<6; i++) {
      legs[i].preprocess();
    }

    for(i=0; i<6; i++) {
      ids[i] = -1;
    }

    for(i=0; i<6; i++) {
      int k = rand() % (6-i);
      for(j=0; j<6; j++) {
        if(ids[j] == -1) {
          if(k == 0) {
            ids[j] = i;
            break;
          }
          k--;
        }
      }
    }

    for(i=0; i<6; i++) {
      legs[ids[i]].process(legs);
    }

    for(i=0; i<6; i++) {
      legs[i].update(legs);
    }

    for(i=0; i<6; i++) {
      CMatrix tmp;
      tmp.a[12] = legs[i].base.x + legs[i].target.x;
      tmp.a[13] = legs[i].base.y + legs[i].target.y;
      tmp.a[14] = legs[i].base.z + legs[i].target.z;

      legs[i].oldTarget = legs[i].target;
      global.robotInput.calcInverseKinematics(i, tmp);
    }

    //global.robotInput.calcForwardKinematics(false);
    global.robotInput.getAnglesFromDh(angles);

    for(i=0; i<AX12_COUNT; i++) {
      global.robotWrapper.Ax12s[i].controlTorque(angles[i], global.motion.ipoPause);

      //global.robotWrapper.Ax12s[i].controlSpeed(angles[i], global.motion.ipoPause);

      global.robotWrapper.Ax12s[i].setTargetAngle(angles[i]);

      if(!onRobot) {
        global.robotWrapper.Ax12s[i].setCurrentAngle(angles[i]);
      }
    }

    if(onRobot) {
      global.robotWrapper.writeAx12s();
    }

    timediff =  (int)(CPlatform::getTickCount() - time);

    if(timediff >= global.motion.ipoPause) {
      timediff = 1;
    }
    else if(timediff <= 0) {
      timediff = global.motion.ipoMinPause;
    }
    else {
      timediff = global.motion.ipoPause - timediff;
    }

    if(timediff < global.motion.ipoMinPause) {
      timediff = global.motion.ipoMinPause;
    }

    timediff = 20;

    CPlatform::sleep(timediff);

    counter++;
  }

  printf("Stop!\n");
  global.motion.enableIpo(true);
}

