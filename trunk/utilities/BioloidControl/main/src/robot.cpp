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


// TODO: split into several files

#include <math.h>
#include <iostream>
#include <cstdlib>
#include <cstring>

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

#define ROUND(x) (float)( (int)(x*100) )/100.0
#define DEG(x) x*180.0/M_PI

#define _ASK_TO_WRITE
#define _SHOW_WRITE
#define _SHOW_READ

// use old style robot (servos in legs point outwards)

#define SHOWREADERROR
#define _SHOWCURRENTANGLEERROR
#define _SHOWTARGETANGLEERROR



#define TESTANGLE(i) ;
#define TESTANGLE2(i) if(angles[5-i]*180.0/M_PI < anglesMinMaxStart[8+2*i][0]) {\
    angles[5-i] += M_PI;} \
  else                    \
  if(angles[5-i]*180.0/M_PI > anglesMinMaxStart[8+2*i][1]) { \
    angles[5-i] -= M_PI;}


const char*robotNames[ROBOT_COUNT]  = {
  "HUMANOID", "HUMANOID_INVERSE_HIP", "CARAUSIUSMOROSUS"
};

using namespace std;



// Axis Aligned Boundary Box - Class
CBox::CBox()
{
  // atm not used
  mass = 0.1;

  frame = NULL;

  // variable rotation matrix
  rot = NULL;
}

CBox::~CBox()
{
  if(frame != NULL) {
    delete (frame);
  }
}

// init
CBox::CBox(float pos_x, float pos_y, float pos_z, float width, float depth, float height,
           CFrame *transformation, float mass)
{
  set(pos_x, pos_y, pos_z, width, height, depth, transformation, mass);
}

// calculate extents-vector and sets position vector & fixed rotation matrix
void CBox::set(float pos_x, float pos_y, float pos_z, float width, float depth, float height,
               CFrame *transformation, float mass)
{
  center.set(pos_x, pos_y, pos_z);
  extents.set(fabsf(width) / 2.0, fabsf(depth) / 2.0, fabsf(height) / 2.0);
  extents.set( (width) / 2.0, (depth) / 2.0, (height) / 2.0);

  // inflate (collision test)
  //extents = extents * 1.05;

  if( (frame != NULL) && (frame != transformation) ) {
    delete frame;
  }

  frame = transformation;
  rot = NULL;
  this->mass = mass;
}

// Denavit Hartenberg Storage Class
CDh::CDh()
{
  rot_z = trans_z = rot_x = trans_x = angle = 0.0;
  sgn = 1.0;
}

void CDh::setMinMax(float min, float max)
{ 
     if (max < min)
        max = min;
         
     this->min = min;
     this->max = max;
}

// get variable rotation angle (variable portion of theta)
float CDh::getAngle()
{
  return this->angle;
}

// set variable rotation angle (variable portion of theta)
void CDh::setAngle(float angle)
{
  this->angle = angle;
}

// set fixed dh parameters
void CDh::set(float rot_z, float trans_z, float rot_x, float trans_x)
{
  this->rot_z = rot_z;
  this->trans_z = trans_z;
  this->rot_x = rot_x;
  this->trans_x = trans_x;
}

/*
   Robot Class, contains kinematic chains, geometry modell, inverse kinematics, collision checking
 */
CRobot::CRobot()
{
/*	kinematicChains.chain[KC_LEGL].setLength(KC_LEGL_COUNT);
    kinematicChains.chain[KC_LEGR].setLength(KC_LEGR_COUNT);
    kinematicChains.chain[KC_ARML].setLength(KC_ARML_COUNT);
    kinematicChains.chain[KC_ARMR].setLength(KC_ARMR_COUNT);*/

  for(int i=0; i<FRAMES_COUNT; i++) {
    frames[i] = NULL;
  }
  framesCount = 0;
}

CKinematicChainContainer::CKinematicChainContainer()
{
  chain = NULL;
  length = 0;
}

CKinematicChainContainer::~CKinematicChainContainer()
{
  if(chain != NULL) {
    delete[] chain;
    chain = NULL;
  }
  length = 0;
}

void CKinematicChainContainer::update()
{
  for(int i=0; i<length; i++) {
    chain[i].update();
  }
}

void CKinematicChainContainer::loadFromXml(CRobot *rb, TiXmlElement*kinChainsNode)
{
  char text[255];
  int len = CConfiguration::getAttributeInteger(kinChainsNode, "len", 0);
  length = len;
  if(chain != NULL) {
    delete[] chain;
    chain = NULL;
  }
  chain = new CKinematicChain[len];

  TiXmlElement*node = kinChainsNode->FirstChildElement();
  for(int i=0; i<len; i++) {
    chain[i].loadFromXml(rb, node);
    node = node->NextSiblingElement();

    /*
       for (int j=0; j<chain[i].length; j++)
       {
         chain[i].frames[j].base = chain[i].base;
         sprintf(chain[i].frames[j].name, "%s:%d", chain[i].name, j);

         rb->frames[rb->framesCount++] = &chain[i].frames[j];
       } */
  }
}

int CRobot::getFrameByName(char*name, bool create)
{
  for(int i=0; i<framesCount; i++) {
    if(strcasecmp(frames[i]->name, name) == 0) {
      return i;
    }
  }

  if(create && (framesCount < FRAMES_COUNT) && (strlen(name) > 0) ) {
    int id = framesCount;
    framesCount++;
    frames[id] = new CFrame(name);

    return id;
  }
  return -1;
}

bool CRobot::xmlToFrame(CFrame *frame, TiXmlElement*frameNode)
{
  float a, b, g, x, y, z;
  a = CConfiguration::getAttributeFloat(frameNode, "a", 0.0);
  b = CConfiguration::getAttributeFloat(frameNode, "b", 0.0);
  g = CConfiguration::getAttributeFloat(frameNode, "g", 0.0);
  x = CConfiguration::getAttributeFloat(frameNode, "x", 0.0);
  y = CConfiguration::getAttributeFloat(frameNode, "y", 0.0);
  z = CConfiguration::getAttributeFloat(frameNode, "z", 0.0);

  frame->pose.set(1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0);
  CMathLib::getRotation(frame->pose, a*M_PI/180.0, b*M_PI/180.0, g*M_PI/180.0);

  frame->pose.a[12] = x;
  frame->pose.a[13] = y;
  frame->pose.a[14] = z;

  sprintf(frame->name, "%s", CConfiguration::getAttributeString(frameNode, (char *)"name", (char *)"") );

  char buffer[255];
  sprintf(buffer, "%s", CConfiguration::getAttributeString(frameNode, (char *)"base", (char *)"") );

  frame->base = NULL;

  // create referenced base frame if it doesnt exist yet
  int i = getFrameByName(buffer, true);
  if(i >= 0) {
    frame->base = frames[i];
  }

  return true;
}

CFrame*CFrame::getByName(char*str)
{
  if(hasName(str) ) {
    return this;
  }
  else {
    return NULL;
  }
}

CFrame*CKinematicChain::getByName(char*str)
{
  if(hasName(str) ) {
    return this;
  }
  else {
    return NULL;
  }


  char buffer[255];
  strcpy(buffer, str);
  char*delimiter = strchr(buffer, ':');

  if(delimiter == NULL) {
    if(hasName(str) ) {
      return this;
    }
    else {
      return NULL;
    }
  }
  else {
    *delimiter = 0;
    std::cout << "name: " << buffer << " : " << name << "\n";
    delimiter++;

    if(!hasName(buffer) ) {
      return NULL;
    }

    int number = atoi(delimiter);
    std::cout << "chain: " << str << " number: " << number << "\n";

    //return this;
    if( (number >= 0) && (number < length) ) {
      return frames[number];
    }
  }

  return NULL;
}

void CKinematicChain::loadFromXml(CRobot *rb, TiXmlElement*kinChainsNode)
{
  if(kinChainsNode == NULL) {
    return;
  }

  char text[255];
  int len = CConfiguration::getAttributeInteger(kinChainsNode, "len", 0);

  sprintf(name, "%s", CConfiguration::getAttributeString(kinChainsNode, "name", (char *)"") );

  char buffer[255];
  sprintf(buffer, "%s", CConfiguration::getAttributeString(kinChainsNode, "base", (char *)"") );

  base = NULL;
  // create referenced base frame if it doesnt exist yet
  int id = rb->getFrameByName(buffer);
  if(id >= 0) {
    base = rb->frames[id];
  }

  setLength(len);

  TiXmlElement*node = kinChainsNode->FirstChildElement();
  for(int i=0; i<len; i++) {
    sprintf(text, "%s:%d", name, i);
    id = rb->getFrameByName(text, true);
    if(id >= 0) {
      frames[i] = rb->frames[id];
    }
    else {
      CUtil::cout("CKinematicChain::loadFromXml() failed.\n", TEXT_ERROR);
      return;
    }

    dhParameters[i].angle = 0.0;
    dhParameters[i].rot_z = M_PI / 180.0 * CConfiguration::getAttributeFloat(node, "rotz", 0.0);
    dhParameters[i].rot_x = M_PI / 180.0 * CConfiguration::getAttributeFloat(node, "rotx", 0.0);
    dhParameters[i].trans_z = CConfiguration::getAttributeFloat(node, "transz", 0.0);
    dhParameters[i].trans_x = CConfiguration::getAttributeFloat(node, "transx", 0.0);
    dhParameters[i].id = CConfiguration::getAttributeInteger(node, "id", -1);
    dhParameters[i].sgn = CConfiguration::getAttributeInteger(node, "sgn", 1);

    dhParameters[i].min = CConfiguration::getAttributeFloat(node, "min", -180.0) * M_PI/180.0;
    dhParameters[i].max = CConfiguration::getAttributeFloat(node, "max",  180.0) * M_PI/180.0;

    frames[i]->pose.setDh(dhParameters[i]);
    frames[i]->base = base;

    node = node->NextSiblingElement();
  }
}

// inits kinematic chains, geometry modell
void CRobot::init(char*filename)
{
  initDhParameters(filename);

  geometry.init(this, filename);
  updateDhParameters();
}

// calculates inverse kinematics with the CCD method
void CRobot::estimateInverseKinematics(int kinematicChain, CMatrix &target)
{
  // ccd
  float angles[AX12_COUNT];
  int ids[AX12_COUNT];
  getAnglesFromDh(angles);

  int len = 0;    
  for(int i=0; i<kinematicChains.chain[kinematicChain].length; i++) {
       if(kinematicChains.chain[kinematicChain].dhParameters[i].id >= 1) {
         ids[len] = i;
         len++;
         
         angles[kinematicChains.chain[kinematicChain].dhParameters[i].id-1] = 0.5 * 180.0/M_PI * (kinematicChains.chain[kinematicChain].dhParameters[i].max + kinematicChains.chain[kinematicChain].dhParameters[i].min);
       }
     }

      
  double distance = 10000.0;
  int iter = 100000;
  
  
  
  double k1,k2,k3,solution;
  double weights[AX12_COUNT];
  for (unsigned int i=0; i<AX12_COUNT; i++)
      weights[i] = 1.0 * 180.0/M_PI;
      
  CVec Pic, Pid, Pd, joint, Pc, U1c, U2c, U3c, U1d, U2d, U3d;
  
  Pd = target[3];
  U1d = target[0];
  U2d = target[1];
  U3d = target[2];
  
  CVec z(0.0, 0.0, 1.0);
  z.w = 0.0;
  CVec axis, tmpVec;
  
  double wp, wo, s;
  
  double PidLen, PicLen;
  
  setDhFromAngles(angles);
  calcForwardKinematics(false);
  
  bool changed = true;
  const double epsilon = 0.001;
  while (changed && iter > 0)
  {
       changed = false;
       for (int i=0; i<len; i++)   
       {
           int id = ids[len-1-i];
           
           Pc = kinematicChains.chain[kinematicChain].pose[3];
           U1c = kinematicChains.chain[kinematicChain].pose[0];
           U2c = kinematicChains.chain[kinematicChain].pose[1];
           U3c = kinematicChains.chain[kinematicChain].pose[2];
           
           joint = kinematicChains.chain[kinematicChain].frames[id-1]->pose[3];
           
           Pid = Pd - joint;
           Pic = Pc - joint;
           
           PidLen = Pid.length();
           PicLen = Pic.length();
           
           if (PidLen > PicLen)
              s = PicLen / PidLen;
           else
              s = PidLen / PicLen;
           
           wo = len > 3 ? 1.0 : 0.0;
           wp = (1.0/10.0) * (1.0 + s);
           axis = kinematicChains.chain[kinematicChain].frames[id-1]->pose * z;
           
           k1 = wp * (Pid | axis)*(Pic |axis) + wo * ((U1d | axis)*(U1c | axis) + (U2d | axis)*(U2c | axis) + (U3d | axis)*(U3c | axis));
           k2 = wp * (Pid | Pic) + wo * ((U1d | U1c) + (U2d | U2c) + (U3d | U3c)); 
           
           tmpVec = ((Pic ^ Pid) * wp) + ((((U1c ^ U1d) + (U2c ^ U2d)) + (U3c ^ U3d)) * wo);
           k3 = axis | tmpVec;
           
           solution = atan2(k3, (k2-k1));

           if (angles[kinematicChains.chain[kinematicChain].dhParameters[id].id-1] > 180.0/M_PI*kinematicChains.chain[kinematicChain].dhParameters[id].max)
           {
               if (solution > 0.0)
                  solution = 0.0;
           } else if (angles[kinematicChains.chain[kinematicChain].dhParameters[id].id-1] < 180.0/M_PI*kinematicChains.chain[kinematicChain].dhParameters[id].min)
           {
               if (solution < 0.0)
                  solution = 0.0;
           }
          
           solution *= weights[kinematicChains.chain[kinematicChain].dhParameters[id].id-1];
           angles[kinematicChains.chain[kinematicChain].dhParameters[id].id-1] += solution;
             
           if (fabsf(solution) > epsilon)
              changed = true;

           kinematicChains.chain[kinematicChain].dhParameters[id].setAngle(M_PI/180.0*angles[kinematicChains.chain[kinematicChain].dhParameters[id].id-1]);
           kinematicChains.chain[kinematicChain].frames[id]->pose.setDh(kinematicChains.chain[kinematicChain].dhParameters[id]);
           kinematicChains.chain[kinematicChain].update();
       }  

       iter--;   
  }


  // get final distance to target
  char text[255];
  
  if (len > 3)
  {
      CMatrix m;
      m = kinematicChains.chain[kinematicChain].pose;
      m.invert();
      m.mul(m, target);
      
      sprintf(text, "Distance 6d: %g\n", m.length());
  } else
  {
      CVec tmp = kinematicChains.chain[kinematicChain].pose[3] - target[3];
      sprintf(text, "Distance 3d: %g\n", tmp.length());  
  }
  CUtil::cout(text, TEXT_DEBUG);
}

/* calculate inverse kinematics
   kinematicChains.chain[]->pose -> angles */
   #define INVKIN_ARM_DISTANCE 1.0
void CRobot::calcInverseKinematics(int kinematicChain, CMatrix &pose)
{
  switch(global.robotInput.id)
  {
  case ROBOT_HUMANOID:
       {


    CMatrix rot = pose, rotOld = rot;
    CMatrix tmp, tmp2;
    CVec pos = pose[3];

    CVec or1, or2, tmp1;
    float angles[6];
    float tmpf, tmpsin, tmpcos;
    float s[6], c[6];
    int i, j;

    switch(kinematicChain)
    {
    case 2:
      // analytic method

      // first koordinate transformation is fix
      // Pose = M1 * M2 * M3 * M4
      // M1^-1 * Pose = M2 * M3 * M4
      // -> newPose = M1^-1 * Pose
      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
      tmp.setDh(kinematicChains.chain[kinematicChain].dhParameters[0]);
      tmp.invert();
      rot.mul(tmp, pose);

//legl 64 -39.39 -179.74 178.291 75.5684 77.23
//legl 64.1279 -39.5194 -179.688 88.0485 -14.4861 76.9516
//set 8 -15 100, set 10 -15 100, set 12 -15 100, set 14 -15 100, set 16 -15 100, set 18 -15 100

      // see studienarbeit.pdf for an explanation
      angles[0] = atan2(-rot.a[8], rot.a[9]);
      s[0] = sin(angles[0]);
      c[0] = cos(angles[0]);

      tmpsin = (1.0 / -68.0) * (-99.0 * rot.a[2] - (-75.0) + rot.a[14]);

      if(fabs(s[0]) < 0.01) {
        tmpcos = (1.0 / (68.0 * c[0]) ) * (-99.0 * rot.a[0] + rot.a[12] - (17.5)* c[0]);
      }
      else {
        tmpcos = (1.0 / (68.0 * s[0]) ) * (-99.0 * rot.a[1] + rot.a[13] - (17.5)* s[0]);
      }

      angles[1] = atan2(tmpsin, tmpcos) - 0.5 * M_PI;

      angles[2] = atan2(-tmpcos * rot.a[2] - tmpsin * s[0]*rot.a[1] - tmpsin*c[0] * rot.a[0],
                        -tmpsin * rot.a[2] + tmpcos * s[0] * rot.a[1] + tmpcos* c[0] * rot.a[0]);


      // writes angles to dh structs
      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
      for(i=0; i<3; i++) {
        while(angles[i] > M_PI) {
          angles[i] -= 2*M_PI;
        }
        while(angles[i] < -M_PI) {
          angles[i] += 2*M_PI;
        }

        kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[i]);
      }
      // compare result to input pose, calc forward kinematics
      kinematicChains.chain[kinematicChain].update();

      // calculate absolute matrix distance
      tmpf = 0.0;
      for(i=0; i<15; i++) {
        tmpf += fabs(kinematicChains.chain[kinematicChain].pose.a[i] - pose.a[i]);
      }

      // if distance is too large, calculate angles using a geometric approach
      if(tmpf > INVKIN_ARM_DISTANCE) {
        // shoulder position
        or1.set(fabs(kinematicChains.chain[kinematicChain].dhParameters[1].trans_z),
                -fabs(kinematicChains.chain[kinematicChain].dhParameters[0].trans_x),
                fabs(kinematicChains.chain[kinematicChain].dhParameters[0].trans_z) );

        // hand relative to shoulder
        tmp1 = pos - or1;

        // shoulder rotation angle
        angles[0] = 0.5*M_PI + atan2(tmp1.y, tmp1.z);

        // calc other angles
        tmpf = tmp1.y*tmp1.y+tmp1.z*tmp1.z;
        CMathLib::calcAngles(fabs(kinematicChains.chain[kinematicChain].dhParameters[3].trans_x),
                             fabs(kinematicChains.chain[kinematicChain].dhParameters[2].trans_x),
                             sqrt(tmpf)-fabs(kinematicChains.chain[kinematicChain].dhParameters[1].trans_x),
                             -(tmp1.x),
                             angles[2], angles[1]);

        // normalize angles
        for(i=0; i<3; i++) {
          if(angles[i] > M_PI) {
            angles[i] -= 2*M_PI;
          }
          else if(angles[i] < -M_PI) {
            angles[i] += 2*M_PI;
          }
        }

        kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
        for(i=0; i<3; i++) {
          kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[i]);
        }
      }
      break;

    case 3:
      // check KC_ARML for an explanation
      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
      tmp.setDh(kinematicChains.chain[kinematicChain].dhParameters[0]);
      tmp.invert();
      rot.mul(tmp, pose);


      angles[0] = atan2(-rot.a[8], rot.a[9]);
      s[0] = sin(angles[0]);
      c[0] = cos(angles[0]);

      tmpsin = (1.0 / -68.0) * (-99.0 * rot.a[2] - (-75.0) + rot.a[14]);

      if(fabs(s[0]) < 0.001) {
        tmpcos = (1.0 / (68.0 * c[0]) ) * (-99.0 * rot.a[0] + rot.a[12] - (17.5)* c[0]);
      }
      else {
        tmpcos = (1.0 / (68.0 * s[0]) ) * (-99.0 * rot.a[1] + rot.a[13] - (17.5)* s[0]);
      }

      angles[1] = atan2(tmpsin, tmpcos) + 0.5 * M_PI;
      angles[2] = atan2(-tmpcos * rot.a[2] - tmpsin * s[0]*rot.a[1] - tmpsin*c[0] * rot.a[0],
                        -tmpsin * rot.a[2] + tmpcos * s[0] * rot.a[1] + tmpcos* c[0] * rot.a[0]);

      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
      for(i=0; i<3; i++) {
        while(angles[i] > M_PI) {
          angles[i] -= 2*M_PI;
        }
        while(angles[i] < -M_PI) {
          angles[i] += 2*M_PI;
        }

        kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[i]);
      }
      kinematicChains.chain[kinematicChain].update();
      tmpf = 0.0;
      for(i=0; i<15; i++) {
        tmpf += fabs(kinematicChains.chain[kinematicChain].pose.a[i] - pose.a[i]);
      }

      if(tmpf > INVKIN_ARM_DISTANCE) {
        or1.set(-fabs(kinematicChains.chain[kinematicChain].dhParameters[1].trans_z),
                -fabs(kinematicChains.chain[kinematicChain].dhParameters[0].trans_x),
                fabs(kinematicChains.chain[kinematicChain].dhParameters[0].trans_z) );

        tmp1 = pos - or1;

        angles[0] = -0.5*M_PI - atan2(tmp1.y, tmp1.z);

        tmpf = tmp1.y*tmp1.y+tmp1.z*tmp1.z;
        CMathLib::calcAngles(fabs(kinematicChains.chain[kinematicChain].dhParameters[3].trans_x),
                             fabs(kinematicChains.chain[kinematicChain].dhParameters[2].trans_x),
                             sqrt(tmpf)-fabs(kinematicChains.chain[kinematicChain].dhParameters[1].trans_x),
                             (tmp1.x),
                             angles[2], angles[1]);
        angles[1] = -angles[1];
        angles[2] = -angles[2];


        for(i=0; i<3; i++) {
          if(DEG(angles[i]) > 180.0) {
            angles[i] -= 2*M_PI;
          }
          else
          if(DEG(angles[i]) < -180.0) {
            angles[i] += 2*M_PI;
          }
        }

        kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
        for(i=0; i<3; i++) {
          kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[i]);
        }
      }

      break;

    case 0:
    {
       // todo: check singularities, format & clean up code
       // legl 32.5 0 -165 90 -90 90
       
       // first coordinate transformation is fix -> exclude it from calculation
       rot.a[12] = rot.a[12]  - kinematicChains.chain[kinematicChain].dhParameters[0].trans_x;
       rotOld = rot;

       // remove last transformation (ankle->foot plate)
       CVec xtrans(-kinematicChains.chain[kinematicChain].dhParameters[6].trans_x, 0.0, 0.0);
       CVec newpos = rot * xtrans;
       rot.a[12] = newpos.x;
       rot.a[13] = newpos.y;
       rot.a[14] = newpos.z;      
            
       // calculate plane: origin-ankle-heel
       CVec dir1, dir2;
       dir1 = rot[3];
       CVec z(0.0, 0.0, 1.0); z.w = 0.0;
       dir2 = rot * z;
       dir1.normalize();
       dir2.normalize();
       
       //parallel? printf("acos: %g\n", 180.0/M_PI*acos(dir1 | dir2));      
       CVec n = dir1 ^ dir2;
       
       // intersect plane and x-y-plane and calculate slope of the result (straight line) -> angle of first servo in hip
       if (fabsf(n.y) < 0.0001)
          angles[5] = 0.0;
       else
           angles[5] = - 0.5* M_PI + atan2(-n.x , n.y);

      // calculate position of tcp relative to hip (tmp1) (intersection of first and second servo in hip)     
      rot = rotOld;
      rot.a[12] -= -sin(angles[5])*(kinematicChains.chain[kinematicChain].dhParameters[2].trans_z);
      rot.a[13] -= cos(angles[5])*(kinematicChains.chain[kinematicChain].dhParameters[2].trans_z);
      rot.a[14] -= 0.0;
      rot.invert();

      // remove last transformation (ankle->foot plate)
      tmp1.set(rot.a[12] + kinematicChains.chain[kinematicChain].dhParameters[6].trans_x, rot.a[13], rot.a[14]);

      // geometric part
      angles[0] = atan2( -tmp1.x, tmp1.y);
      tmpf = tmp1.x*tmp1.x + tmp1.y*tmp1.y;

      CMathLib::calcAngles(kinematicChains.chain[kinematicChain].dhParameters[4].trans_x,
                           kinematicChains.chain[kinematicChain].dhParameters[3].trans_x,
                           tmp1.z, sqrt(tmpf),
                           angles[2], angles[1]);

      angles[0] = -angles[0] + 0.5*M_PI;
      angles[1] = -angles[1]+M_PI;
      angles[2] = -angles[2];

      angles[1] = -angles[1];
      angles[2] = -angles[2];

      angles[1] -= kinematicChains.chain[kinematicChain].dhParameters[5].rot_z;
      angles[2] -= kinematicChains.chain[kinematicChain].dhParameters[4].rot_z;

      // analytic part:
      // calculate last part of kinematic chain based on known angles
      for(i=4; i<7; i++) {
        kinematicChains.chain[kinematicChain].dhParameters[i].setAngle(angles[6-i]);
        kinematicChains.chain[kinematicChain].frames[i]->pose.setDh(kinematicChains.chain[kinematicChain].dhParameters[i]);

        kinematicChains.chain[kinematicChain].frames[i]->pose.invert();
      }
      tmp2.mul(rotOld, kinematicChains.chain[kinematicChain].frames[6]->pose);
      tmp.mul(tmp2, kinematicChains.chain[kinematicChain].frames[5]->pose);
      rot.mul(tmp, kinematicChains.chain[kinematicChain].frames[4]->pose);
      
      angles[3] = atan2(-rot.a[6], -rot.a[2]);
      angles[4] = atan2(-rot.a[2]*cos(angles[3])-rot.a[6]*sin(angles[3]), rot.a[10]);

      angles[3] += -fabsf(kinematicChains.chain[kinematicChain].dhParameters[3].rot_z);
      angles[4] += -0.5*M_PI;
      angles[4] = -angles[4];
      
      // normalize angles
      for(i=0; i<6; i++) {
        if(angles[i] > M_PI) {
          angles[i] -= 2*M_PI;
        }
        else
        if(angles[i] < -M_PI) {
          angles[i] += 2*M_PI;
        }
      }

      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0);

      for(i=0; i<6; i++) {
        kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[5-i]);
      }
      break;
    }
    case 1:
    {
      // same as above
      rot.a[12] = rot.a[12] - kinematicChains.chain[kinematicChain].dhParameters[0].trans_x;

      rotOld = rot;
      
      CVec xtrans(-kinematicChains.chain[kinematicChain].dhParameters[6].trans_x, 0.0, 0.0);
      CVec newpos = rot * xtrans;
      rot.a[12] = newpos.x;
      rot.a[13] = newpos.y;
      rot.a[14] = newpos.z;      
            
      CVec dir1, dir2;
      dir1 = rot[3];
      CVec z(0.0, 0.0, 1.0);
      z.w = 0.0;
      dir2 = rot * z;
      dir1.normalize();
      dir2.normalize();
      CVec n = dir1 ^ dir2;
       
      if (fabsf(n.y) < 0.0001)
         angles[5] = 0.0;
      else
          angles[5] = - 0.5* M_PI + atan2(-n.x , n.y);

      rot = rotOld;
      rot.a[12] -= -sin(angles[5])*(kinematicChains.chain[kinematicChain].dhParameters[2].trans_z);
      rot.a[13] -= cos(angles[5])*(kinematicChains.chain[kinematicChain].dhParameters[2].trans_z);
      rot.a[14] -= 0.0;
      rot.invert();

      tmp1.set(rot.a[12] + kinematicChains.chain[kinematicChain].dhParameters[6].trans_x, rot.a[13], rot.a[14]);

      // geometric part
      angles[0] = atan2( -tmp1.x, tmp1.y);
      tmpf = tmp1.x*tmp1.x + tmp1.y*tmp1.y;

      CMathLib::calcAngles(kinematicChains.chain[kinematicChain].dhParameters[4].trans_x,
                           kinematicChains.chain[kinematicChain].dhParameters[3].trans_x,
                           tmp1.z,
                           sqrt(tmpf),
                           angles[2], angles[1]);

      angles[0] = -angles[0] + 0.5*M_PI;
      angles[1] = angles[1]-M_PI;
      angles[2] = angles[2];

      angles[1] += fabsf(kinematicChains.chain[kinematicChain].dhParameters[5].rot_z);
      angles[2] -= fabsf(kinematicChains.chain[kinematicChain].dhParameters[4].rot_z);

      angles[1] = -angles[1];
      angles[2] = -angles[2];

      for(i=4; i<7; i++) {
        kinematicChains.chain[kinematicChain].dhParameters[i].setAngle(angles[6-i]);
        kinematicChains.chain[kinematicChain].frames[i]->pose.setDh(kinematicChains.chain[kinematicChain].dhParameters[i]);

        kinematicChains.chain[kinematicChain].frames[i]->pose.invert();
      }
      tmp2.mul(rotOld, kinematicChains.chain[kinematicChain].frames[6]->pose);
      tmp.mul(tmp2, kinematicChains.chain[kinematicChain].frames[5]->pose);
      rot.mul(tmp, kinematicChains.chain[kinematicChain].frames[4]->pose);

      angles[3] = atan2(rot.a[0]*sin(angles[5])-rot.a[1]*cos(angles[5]), -sin(angles[5])*rot.a[4]+cos(angles[5])*rot.a[5]);

      angles[4] = atan2(rot.a[8]*cos(angles[5])+rot.a[9]*sin(angles[5]), rot.a[10]);
      angles[4] -= kinematicChains.chain[kinematicChain].dhParameters[2].rot_z;

      angles[3] += fabsf(kinematicChains.chain[kinematicChain].dhParameters[3].rot_z);
      //angles[3] -= M_PI;

      for(i=0; i<6; i++) {
        if(angles[i] > M_PI) {
          angles[i] -= 2*M_PI;
        }
        else
        if(angles[i] < -M_PI) {
          angles[i] += 2*M_PI;
        }
      }

      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0);
      for(i=0; i<6; i++) {
        kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[5-i]);
      }
      break;
    }
  }
  break;
}
  case ROBOT_HUMANOID_INVERSE_HIP:
  {


    CMatrix rot = pose, rotOld = rot;
    CMatrix tmp, tmp2;
    CVec pos = pose[3];

    CVec or1, or2, tmp1;
    float angles[6];
    float tmpf, tmpsin, tmpcos;
    float s[6], c[6];
    int i, j;

    switch(kinematicChain)
    {
    case 2:
      // analytic method

      // first koordinate transformation is fix
      // Pose = M1 * M2 * M3 * M4
      // M1^-1 * Pose = M2 * M3 * M4
      // -> newPose = M1^-1 * Pose
      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
      tmp.setDh(kinematicChains.chain[kinematicChain].dhParameters[0]);
      tmp.invert();
      rot.mul(tmp, pose);

      // see studienarbeit.pdf for an explanation
      angles[0] = atan2(-rot.a[8], rot.a[9]);
      s[0] = sin(angles[0]);
      c[0] = cos(angles[0]);

      tmpsin = (1.0 / -68.0) * (-99.0 * rot.a[2] - (-75.0) + rot.a[14]);

      if(fabs(s[0]) < 0.01) {
        tmpcos = (1.0 / (68.0 * c[0]) ) * (-99.0 * rot.a[0] + rot.a[12] - (17.5)* c[0]);
      }
      else {
        tmpcos = (1.0 / (68.0 * s[0]) ) * (-99.0 * rot.a[1] + rot.a[13] - (17.5)* s[0]);
      }

      angles[1] = atan2(tmpsin, tmpcos) - 0.5 * M_PI;

      angles[2] = atan2(-tmpcos * rot.a[2] - tmpsin * s[0]*rot.a[1] - tmpsin*c[0] * rot.a[0],
                        -tmpsin * rot.a[2] + tmpcos * s[0] * rot.a[1] + tmpcos* c[0] * rot.a[0]);


      // writes angles to dh structs
      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
      for(i=0; i<3; i++) {
        while(angles[i] > M_PI) {
          angles[i] -= 2*M_PI;
        }
        while(angles[i] < -M_PI) {
          angles[i] += 2*M_PI;
        }

        kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[i]);
      }
      // compare result to input pose, calc forward kinematics
      kinematicChains.chain[kinematicChain].update();

      // calculate absolute matrix distance
      tmpf = 0.0;
      for(i=0; i<15; i++) {
        tmpf += fabs(kinematicChains.chain[kinematicChain].pose.a[i] - pose.a[i]);
      }

      // if distance is too large, calculate angles using a geometric approach
      if(tmpf > INVKIN_ARM_DISTANCE) {
        // shoulder position
        or1.set(fabs(kinematicChains.chain[kinematicChain].dhParameters[1].trans_z),
                -fabs(kinematicChains.chain[kinematicChain].dhParameters[0].trans_x),
                fabs(kinematicChains.chain[kinematicChain].dhParameters[0].trans_z) );

        // hand relative to shoulder
        tmp1 = pos - or1;

        // shoulder rotation angle
        angles[0] = 0.5*M_PI + atan2(tmp1.y, tmp1.z);

        // calc other angles
        tmpf = tmp1.y*tmp1.y+tmp1.z*tmp1.z;
        CMathLib::calcAngles(fabs(kinematicChains.chain[kinematicChain].dhParameters[3].trans_x),
                             fabs(kinematicChains.chain[kinematicChain].dhParameters[2].trans_x),
                             sqrt(tmpf)-fabs(kinematicChains.chain[kinematicChain].dhParameters[1].trans_x),
                             -(tmp1.x),
                             angles[2], angles[1]);

        // normalize angles
        for(i=0; i<3; i++) {
          if(angles[i] > M_PI) {
            angles[i] -= 2*M_PI;
          }
          else if(angles[i] < -M_PI) {
            angles[i] += 2*M_PI;
          }
        }

        kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
        for(i=0; i<3; i++) {
          kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[i]);
        }
      }
      break;

    case 3:
      // check KC_ARML for an explanation
      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
      tmp.setDh(kinematicChains.chain[kinematicChain].dhParameters[0]);
      tmp.invert();
      rot.mul(tmp, pose);


      angles[0] = atan2(-rot.a[8], rot.a[9]);
      s[0] = sin(angles[0]);
      c[0] = cos(angles[0]);

      tmpsin = (1.0 / -68.0) * (-99.0 * rot.a[2] - (-75.0) + rot.a[14]);

      if(fabs(s[0]) < 0.001) {
        tmpcos = (1.0 / (68.0 * c[0]) ) * (-99.0 * rot.a[0] + rot.a[12] - (17.5)* c[0]);
      }
      else {
        tmpcos = (1.0 / (68.0 * s[0]) ) * (-99.0 * rot.a[1] + rot.a[13] - (17.5)* s[0]);
      }

      angles[1] = atan2(tmpsin, tmpcos) + 0.5 * M_PI;
      angles[2] = atan2(-tmpcos * rot.a[2] - tmpsin * s[0]*rot.a[1] - tmpsin*c[0] * rot.a[0],
                        -tmpsin * rot.a[2] + tmpcos * s[0] * rot.a[1] + tmpcos* c[0] * rot.a[0]);

      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
      for(i=0; i<3; i++) {
        while(angles[i] > M_PI) {
          angles[i] -= 2*M_PI;
        }
        while(angles[i] < -M_PI) {
          angles[i] += 2*M_PI;
        }

        kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[i]);
      }
      kinematicChains.chain[kinematicChain].update();
      tmpf = 0.0;
      for(i=0; i<15; i++) {
        tmpf += fabs(kinematicChains.chain[kinematicChain].pose.a[i] - pose.a[i]);
      }

      if(tmpf > INVKIN_ARM_DISTANCE) {
        or1.set(-fabs(kinematicChains.chain[kinematicChain].dhParameters[1].trans_z),
                -fabs(kinematicChains.chain[kinematicChain].dhParameters[0].trans_x),
                fabs(kinematicChains.chain[kinematicChain].dhParameters[0].trans_z) );

        tmp1 = pos - or1;

        angles[0] = -0.5*M_PI - atan2(tmp1.y, tmp1.z);

        tmpf = tmp1.y*tmp1.y+tmp1.z*tmp1.z;
        CMathLib::calcAngles(fabs(kinematicChains.chain[kinematicChain].dhParameters[3].trans_x),
                             fabs(kinematicChains.chain[kinematicChain].dhParameters[2].trans_x),
                             sqrt(tmpf)-fabs(kinematicChains.chain[kinematicChain].dhParameters[1].trans_x),
                             (tmp1.x),
                             angles[2], angles[1]);
        angles[1] = -angles[1];
        angles[2] = -angles[2];


        for(i=0; i<3; i++) {
          if(DEG(angles[i]) > 180.0) {
            angles[i] -= 2*M_PI;
          }
          else
          if(DEG(angles[i]) < -180.0) {
            angles[i] += 2*M_PI;
          }
        }

        kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0.0);
        for(i=0; i<3; i++) {
          kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[i]);
        }
      }

      break;

    case 0:
      // first koordinate transformation is fix -> exclude it from calculation
      rot.a[12] = rot.a[12]  - kinematicChains.chain[kinematicChain].dhParameters[0].trans_x;

      // invert problem -> see studienarbeit.pdf for an explanation
      rotOld = rot;
      rot.invert();

      tmp1.set(rot.a[12] + kinematicChains.chain[kinematicChain].dhParameters[6].trans_x, rot.a[13], rot.a[14]);

      // geometric part
      angles[0] = atan2( -tmp1.x, tmp1.y);
      tmpf = tmp1.x * tmp1.x + tmp1.y*tmp1.y;

      CMathLib::calcAngles(kinematicChains.chain[kinematicChain].dhParameters[4].trans_x,
                           kinematicChains.chain[kinematicChain].dhParameters[3].trans_x,
                           tmp1.z, sqrt(tmpf),
                           angles[2], angles[1]);

      angles[0] = -angles[0] + 0.5*M_PI;
      angles[1] = -angles[1]+M_PI;
      angles[2] = -angles[2];

      angles[1] = -angles[1];
      angles[2] = -angles[2];

      angles[1] -= kinematicChains.chain[kinematicChain].dhParameters[5].rot_z;
      angles[2] -= kinematicChains.chain[kinematicChain].dhParameters[4].rot_z;

      // analytic part:
      // calculate last part of kinematic chain based on known angles
      for(i=4; i<7; i++) {
        kinematicChains.chain[kinematicChain].dhParameters[i].setAngle(angles[6-i]);
        kinematicChains.chain[kinematicChain].frames[i]->pose.setDh(kinematicChains.chain[kinematicChain].dhParameters[i]);

        kinematicChains.chain[kinematicChain].frames[i]->pose.invert();
      }
      tmp2.mul(rotOld, kinematicChains.chain[kinematicChain].frames[6]->pose);
      tmp.mul(tmp2, kinematicChains.chain[kinematicChain].frames[5]->pose);
      rot.mul(tmp, kinematicChains.chain[kinematicChain].frames[4]->pose);


      angles[5] = atan2(-rot.a[9], -rot.a[8]);
      angles[3] = atan2(-rot.a[6], -rot.a[2]);
      angles[4] = atan2(-rot.a[2]*cos(angles[3])-rot.a[6]*sin(angles[3]), rot.a[10]);

      angles[3] += -fabsf(kinematicChains.chain[kinematicChain].dhParameters[3].rot_z);
      angles[4] += -0.5*M_PI;
      angles[4] = -angles[4];

      // normalize angles
      for(i=0; i<6; i++) {
        if(angles[i] > M_PI) {
          angles[i] -= 2*M_PI;
        }
        else
        if(angles[i] < -M_PI) {
          angles[i] += 2*M_PI;
        }
      }

      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0);

      for(i=0; i<6; i++) {
        kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[5-i]);
      }
      break;

    case 1:
      // same as above
      rot.a[12] = rot.a[12] - kinematicChains.chain[kinematicChain].dhParameters[0].trans_x;

      rotOld = rot;
      rot.invert();

      tmp1.set(rot.a[12] + kinematicChains.chain[kinematicChain].dhParameters[6].trans_x, rot.a[13], rot.a[14]);

      angles[0] = atan2( -tmp1.x, tmp1.y);
      tmpf = tmp1.x * tmp1.x + tmp1.y*tmp1.y;

      CMathLib::calcAngles(kinematicChains.chain[kinematicChain].dhParameters[4].trans_x,
                           kinematicChains.chain[kinematicChain].dhParameters[3].trans_x,
                           tmp1.z,
                           sqrt(tmpf),
                           angles[2], angles[1]);

      angles[0] = -angles[0] + 0.5*M_PI;
      angles[1] = angles[1]-M_PI;
      angles[2] = angles[2];

      angles[1] += fabsf(kinematicChains.chain[kinematicChain].dhParameters[5].rot_z);
      angles[2] -= fabsf(kinematicChains.chain[kinematicChain].dhParameters[4].rot_z);

      angles[1] = -angles[1];
      angles[2] = -angles[2];

      for(i=4; i<7; i++) {
        kinematicChains.chain[kinematicChain].dhParameters[i].setAngle(angles[6-i]);
        kinematicChains.chain[kinematicChain].frames[i]->pose.setDh(kinematicChains.chain[kinematicChain].dhParameters[i]);

        kinematicChains.chain[kinematicChain].frames[i]->pose.invert();
      }
      tmp2.mul(rotOld, kinematicChains.chain[kinematicChain].frames[6]->pose);
      tmp.mul(tmp2, kinematicChains.chain[kinematicChain].frames[5]->pose);
      rot.mul(tmp, kinematicChains.chain[kinematicChain].frames[4]->pose);

      angles[5] = atan2(rot.a[9], rot.a[8]);

      angles[3] = atan2(rot.a[0]*sin(angles[5])-rot.a[1]*cos(angles[5]), -sin(angles[5])*rot.a[4]+cos(angles[5])*rot.a[5]);

      angles[4] = atan2(rot.a[8]*cos(angles[5])+rot.a[9]*sin(angles[5]), rot.a[10]);
      angles[4] -= kinematicChains.chain[kinematicChain].dhParameters[2].rot_z;

      angles[3] += fabsf(kinematicChains.chain[kinematicChain].dhParameters[3].rot_z);
      //angles[3] -= M_PI;

      for(i=0; i<6; i++) {
        if(angles[i] > M_PI) {
          angles[i] -= 2*M_PI;
        }
        else
        if(angles[i] < -M_PI) {
          angles[i] += 2*M_PI;
        }
      }

      kinematicChains.chain[kinematicChain].dhParameters[0].setAngle(0);
      for(i=0; i<6; i++) {
        kinematicChains.chain[kinematicChain].dhParameters[i+1].setAngle(angles[5-i]);
      }
      break;
    }
  }
  break;

  case ROBOT_CARAUSIUSMOROSUS:
  {
    // carausius morosus - 'stick insect'
    CMatrix root, tmp;
    float angles[3], armlen;
    CVec shoulder, hand;
    int i, j, k;

    if(kinematicChains.chain[kinematicChain].base != NULL) {
      tmp = kinematicChains.chain[kinematicChain].base->getRelativeToBase();
    }

    //tmp.invert();
    //root.mul(tmp, pose);

    shoulder = tmp[3];
    //shoulder.print();printf("\n");
    hand = pose[3] - shoulder;

    if(kinematicChain >= 3) {
      hand.x = -hand.x;
      hand.y = -hand.y;
      hand.z = hand.z;
    }
    angles[0] = atan2(hand.y, hand.x);


    armlen = (hand.y*hand.y + hand.x*hand.x);
    CMathLib::calcAngles(fabs(kinematicChains.chain[kinematicChain].dhParameters[1].trans_x),
                         fabs(kinematicChains.chain[kinematicChain].dhParameters[2].trans_x),
                         sqrt(armlen)-fabs(kinematicChains.chain[kinematicChain].dhParameters[0].trans_x),
                         hand.z,
                         angles[2], angles[1]);


    angles[1] = angles[1] + 0.5*M_PI;

    if(kinematicChain <= 2) {
      angles[1] = -angles[1];
      angles[2] = -angles[2];
    }

    for(i=0; i<3; i++) {
      if(angles[i] < -M_PI) {
        angles[i] += 2*M_PI;
      }
      else if(angles[i] > M_PI) {
        angles[i] -= 2*M_PI;
      }

      //angles[i] *= (float)kinematicChains.chain[kinematicChain].dhParameters[i].sgn;

      kinematicChains.chain[kinematicChain].dhParameters[i].setAngle(angles[i]);
    }
    break;
  }

  default:
    CUtil::cout("Inverse Kinematics: Using numerical techniques\n", TEXT_DEBUG);
    estimateInverseKinematics(kinematicChain, pose);
    break;
  }
}

// calculate tcp position and orientation
void CRobot::getTCPs(CVec *positionAndOrientation)
{
  CVec tmpVec;
  CMatrix tmpMatrix, tmpMatrix2;
  for(int i=0; i<kinematicChains.length; i++) {
    positionAndOrientation[i * 2 + 0] = kinematicChains.chain[i].getRelativeToBase()[3];

    tmpMatrix2 = kinematicChains.chain[i].getRelativeToBase();
    CMathLib::getOrientation(tmpMatrix2, positionAndOrientation[i * 2 + 1], tmpVec);
  }
}

// load dh parameters from xml
void CRobot::initDhParameters(char*filename)
{
  {
    framesCount = 0;

    CConfiguration config;
    bool okay = config.load(filename);
    sprintf(config.rootNode, "Robot");

    char*name = config.getString("Name", "");

    id = -1;
    for(int i=0; i<ROBOT_COUNT; i++) {
      if(strcasecmp(robotNames[i], name) == 0) {
        id = i;
        break;
      }
    }

    TiXmlElement*framesNode = config.findNode("Frames");


    if(framesNode != NULL) {
      TiXmlElement*node = framesNode->FirstChildElement();
      while(node != NULL)
      {
        int i = framesCount;
        framesCount++;
        frames[i] = new CFrame();
        xmlToFrame(frames[i], node);

        node = node->NextSiblingElement();
      }
    }

    TiXmlElement*kinNode = config.findNode("KinematicChains");

    if(kinNode == NULL) {goto LOAD_ERROR;}

    kinematicChains.loadFromXml(this, kinNode);

    return;
  }
LOAD_ERROR:
  CUtil::cout("Couldn't load Robot Structure.\n", TEXT_ERROR);
}

void CRobot::calcForwardKinematics(bool draw)
{
  int i;
  kinematicChains.update();

  if(draw) {
    CPlatform::writeToMemoryMappedFile(this);
  }
}

void CRobot::getAnglesFromDh(float *angles)
{
  int i, j;

  for(i=0; i<AX12_COUNT; i++) {
    angles[i] = 0.0;
  }

  for(i=0; i<kinematicChains.length; i++) {
    for(j=0; j<kinematicChains.chain[i].length; j++) {
      if(kinematicChains.chain[i].dhParameters[j].id > 0) {
        angles[kinematicChains.chain[i].dhParameters[j].id-1] = 180.0/M_PI * kinematicChains.chain[i].dhParameters[j].getAngle();
      }
    }
  }
}

CRobot::~CRobot()
{
  cleanUp();
}

void CRobot::setDhFromAngles(float*angles)
{
  int i, j;
  for(i=0; i<kinematicChains.length; i++) {
    for(j=0; j<kinematicChains.chain[i].length; j++) {
      if(kinematicChains.chain[i].dhParameters[j].id > 0) {
        kinematicChains.chain[i].dhParameters[j].setAngle(angles[kinematicChains.chain[i].dhParameters[j].id-1] * M_PI / 180.0);
      }
      else {
        kinematicChains.chain[i].dhParameters[j].setAngle(0.0);
      }
    }
  }
}

void CRobot::updateDhParameters(bool current)
{
  int i;
  float angles[AX12_COUNT];
  for(i=0; i<AX12_COUNT; i++) {
    if(current) {
      angles[i] = global.robotWrapper.Ax12s[i].getCurrentAngle();
    }
    else {
      if(fabs(global.robotWrapper.Ax12s[i].getCurrentAngle() - global.robotWrapper.Ax12s[i].getTargetAngle() ) <= MOTION_CURRENTANGLE_DIFF) {
        angles[i] = global.robotWrapper.Ax12s[i].getTargetAngle();
      }
      else {
        angles[i] = global.robotWrapper.Ax12s[i].getCurrentAngle();
      }
    }
  }

  setDhFromAngles(angles);
}

void CRobot::cleanUp()
{
  geometry.cleanUp();
}

int CRobot::checkCollisions(bool msg, bool useOld)
{
  int first, second;
  bool res = geometry.checkCollisions(first, second);

  if(res && msg) {
    if(useOld) {
      float tmp[AX12_COUNT];
      float tmpf = 0.0;

      getAnglesFromDh(tmp);

      for(int i=0; i<AX12_COUNT; i++) {
        tmpf += fabsf(global.collisionAnglesBackup[i] - tmp[i]);
      }

      if(tmpf > 2.0) {
        char str[255];
        sprintf(str, "Collision detected: %s and %s.\n", geometry.geometry[first].name, geometry.geometry[second].name, (rand() % 100) );
        CUtil::cout(str, TEXT_COLLISION);
      }

      for(int i=0; i<AX12_COUNT; i++) {
        global.collisionAnglesBackup[i] = tmp[i];
      }

    }
    else {
      char str[255];
      sprintf(str, "Collision detected: %s and %s.\n", geometry.geometry[first].name, geometry.geometry[second].name, (rand() % 100) );
      CUtil::cout(str, TEXT_COLLISION);
    }
  }

  return res;
}

int CRobot::checkCollisions(int &first, int &second)
{
  return geometry.checkCollisions(first, second);
}

/*
   TEST FUNCTION: newton euler algorithm
 */
void CRobot::calcDynamics(float*angles, float*angles_v, float*angles_a, int view, float*result)
{
  // obsolete
  // see physics.cpp
}

// Kinematic Chain Class
// - chain of homogenous transformation matrices based
CKinematicChain::CKinematicChain()
{
  dhParameters = 0;
  frames = 0;
  length = 0;
}

void CKinematicChain::update()
{
  CMatrix tmp;

  if(length == 0) {
    return;
  }

  frames[0]->pose.setDh(dhParameters[0]);

  int i;
  for(i=1; i<length; i++) {
    tmp.setDh(dhParameters[i]);

    frames[i]->pose.mul(frames[i-1]->pose, tmp);
  }

  pose = frames[length-1]->pose;
}

CFrame::CFrame()
{
  base = NULL;
  sprintf(name, "");
}

CFrame::CFrame(char*str)
{
  base = NULL;
  sprintf(name, str);
}

CFrame::~CFrame()
{
}

CMatrix CFrame::getRelativeToBase()
{
  CFrame *frame = this;
  CMatrix tmp = pose;
  while(frame->base != NULL)
  {
    frame = frame->base;
    tmp.mul(frame->pose, tmp);
  }

  return tmp;
}

bool CFrame::hasName(char*str)
{
  return(strcasecmp(str, name) == 0);
}

void CFrame::setName(char*str)
{
  sprintf(name, str);
}

void CFrame::update()
{

}

void CKinematicChain::getPose(CMatrix &result)
{
  CMatrix b;

  if(base != NULL) {
    b = base->getRelativeToBase();
  }

  result.mul(b, pose);
}

CKinematicChain::~CKinematicChain()
{
  setLength(0);
}

void CKinematicChain::setLength(int len)
{
  if(length > 0) {
    if(dhParameters != NULL) {
      delete[] dhParameters;
    }

    if(frames != NULL) {
      delete[] frames;
    }

    frames = NULL;
    dhParameters = NULL;
    length = 0;
  }

  if(len > 0) {
    dhParameters = new CDh[len];
    if(dhParameters == NULL) {
      CUtil::cout("setLength: malloc failed().\n", TEXT_ERROR);
      return;
    }

    frames     = new CFrame*[len];
    if(frames == NULL) {
      CUtil::cout("setLength: malloc failed().\n", TEXT_ERROR);
      return;
    }

    length       = len;
  }
}

CBox CGeometry::getBox(int id)
{
  CBox box;
  if( (id>=0) && (id<length) ) {
    box.extents = geometry[id].extents;
    box.center = geometry[id].center;
    box.mass = geometry[id].mass;
    box.rot = geometry[id].rot;
  }
  return box;
}

void CGeometry::setBox(int id, CBox &box)
{
  if( (id>=0) && (id<length) ) {
    geometry[id].extents = box.extents;
    geometry[id].center = box.center;
    geometry[id].mass = box.mass;
    geometry[id].rot = box.rot;
  }
}

CBox CGeometry::getBoxRobotSpace(int id)
{
  CBox box;
  if( (id>=0) && (id<length) ) {
    CMatrix tmp;
    if(geometry[id].frame != NULL) {
      tmp = geometry[id].frame->getRelativeToBase();
    }
    box.center = tmp * geometry[id].center;
    tmp.a[12] = tmp.a[13] = tmp.a[14] = 0.0;
    box.extents = tmp * geometry[id].extents;
    box.mass = geometry[id].mass;
    box.rot = geometry[id].rot;
  }
  return box;
}

CGeometry::CGeometry()
{
  for(int i=0; i<GEOMETRY_BOXCOUNT; i++) {
    for(int j=0; j<GEOMETRY_BOXCOUNT; j++) {
      check[i][j] = false;
    }
  }
}

void CGeometry::init(CRobot *robot, char*filename)
{
  length = 0;

  CConfiguration config;
  bool okay = config.load(filename);
  sprintf(config.rootNode, "Robot");

  // load 3d geometry
  TiXmlElement*kinNode = config.findNode("Geometry");

  if(kinNode != NULL) {
    TiXmlElement*node = kinNode->FirstChildElement();
    while(node != NULL)
    {
      CBox::xmlToBox(&geometry[length], node, robot);
      length++;
      node = node->NextSiblingElement();
    }
  }

  // load collision config
  kinNode = config.findNode("Collisions");

  if(kinNode != NULL) {
    TiXmlElement*node = kinNode->FirstChildElement();
    while(node != NULL)
    {
      xmlToCollision(node, robot);
      node = node->NextSiblingElement();
    }
  }
}

bool CBox::xmlToColor(CVec &color, TiXmlElement*colorNode)
{
  float r, g, b;
  r = CConfiguration::getAttributeFloat(colorNode, "r", 0.0);
  g = CConfiguration::getAttributeFloat(colorNode, "g", 0.0);
  b = CConfiguration::getAttributeFloat(colorNode, "b", 0.0);

  color.set(r, g, b);
}

bool CBox::xmlToBox(CBox *box, TiXmlElement*boxNode, CRobot *robot)
{
  float width, depth, height, mass, x, y, z;
  width = CConfiguration::getAttributeFloat(boxNode, "width", 0.0);
  depth = CConfiguration::getAttributeFloat(boxNode, "depth", 0.0);
  height = CConfiguration::getAttributeFloat(boxNode, "height", 0.0);
  x = CConfiguration::getAttributeFloat(boxNode, "x", 0.0);
  y = CConfiguration::getAttributeFloat(boxNode, "y", 0.0);
  z = CConfiguration::getAttributeFloat(boxNode, "z", 0.0);
  mass = CConfiguration::getAttributeFloat(boxNode, "mass", 0.0);
  sprintf(box->name, CConfiguration::getAttributeString(boxNode, "name", (char *)"") );
  CFrame *relative = new CFrame();

  TiXmlElement*frameNode = boxNode->FirstChildElement("Frame");
  if(frameNode != NULL) {
    robot->xmlToFrame(relative, frameNode);
  }

  CVec cl;
  TiXmlElement*colorNode = boxNode->FirstChildElement("Color");
  if(colorNode != NULL) {
    xmlToColor(cl, colorNode);
    box->color = cl;
  }
  box->set(x, y, z, width, depth, height, relative, mass);
  return true;
}

bool CGeometry::xmlToCollision(TiXmlElement*boxNode, CRobot *robot)
{
  char*first = CConfiguration::getAttributeString(boxNode, "first", (char *)"");
  char*second = CConfiguration::getAttributeString(boxNode, "second", (char *)"");

  bool result = false;

  char *pch = strtok(second, ",");

  //printf("collision %s with ", first);

  while(pch != NULL)
  {

    for(int i=0; i<length; i++) {
      if(strcasecmp(geometry[i].name, first) == 0) {
        for(int j=0; j<length; j++) {
          if(strcasecmp(geometry[j].name, pch) == 0) {
            check[i][j] = check[j][i] = true;
            result = true;
            //printf("%s, ", geometry[j].name);
          }
        }
      }
    }



    pch = strtok(NULL, ",");
  }

  //printf("\n");

  return result;
}

void CGeometry::cleanUp()
{

}

int CGeometry::checkCollisions(int &first, int &second)
{
  bool bResult = false;
  int i, j;
  CBox tmpBoxes[GEOMETRY_BOXCOUNT];
  CMatrix tmpMatrices[GEOMETRY_BOXCOUNT];
  CMatrix id;

  for(i=0; i<length; i++) {
    tmpBoxes[i].extents = geometry[i].extents;
    tmpBoxes[i].center = geometry[i].center;

    if(geometry[i].frame != NULL) {
      tmpMatrices[i] = geometry[i].frame->getRelativeToBase();
    }
    else {
      tmpMatrices[i] = id;
    }

    tmpBoxes[i].rot = &tmpMatrices[i];
  }

  for(i=0; i<length; i++) {
    for(j=i; j<length; j++) {
      if(check[i][j] && getCollision(tmpBoxes[i], tmpBoxes[j]) ) {
        first = i;
        second = j;
        return true;
      }
    }
  }

  return false;
}

void CGeometry::projection(const CBox&box, const CVec&axis, float &from, float&to)
{
  float len = fabs( (axis | (*box.rot)[0]) * box.extents.x) +
  fabs( (axis | (*box.rot)[1]) * box.extents.y) +
  fabs( (axis | (*box.rot)[2]) * box.extents.z);

  CVec tmp = box.center;
  float center = ( (*box.rot) * tmp) | axis;

  from = center-len;
  to = center+len;
}

bool CGeometry::checkOverlapping(const CVec&axis, const CBox&first, const CBox&second)
{

  float l1, r1, l2, r2;

  projection(first, axis, l1, r1);
  projection(second, axis, l2, r2);

  return(l1 > r2 || l2 > r1);
}

bool CGeometry::getCollision(CBox &obj1, CBox &obj2)
{
  if(checkOverlapping( (*obj1.rot)[0], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj1.rot)[1], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj1.rot)[2], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj2.rot)[0], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj2.rot)[1], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj2.rot)[2], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj1.rot)[0] ^ (*obj2.rot)[0], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj1.rot)[0] ^ (*obj2.rot)[1], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj1.rot)[0] ^ (*obj2.rot)[2], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj1.rot)[1] ^ (*obj2.rot)[0], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj1.rot)[1] ^ (*obj2.rot)[1], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj1.rot)[1] ^ (*obj2.rot)[2], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj1.rot)[2] ^ (*obj2.rot)[0], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj1.rot)[2] ^ (*obj2.rot)[1], obj1, obj2) ) { return false;}
  if(checkOverlapping( (*obj1.rot)[2] ^ (*obj2.rot)[2], obj1, obj2) ) { return false;}
  return true;
}

