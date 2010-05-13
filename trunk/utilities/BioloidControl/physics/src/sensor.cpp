/**************************************************************************

    Copyright 2008 Martin Andrews <martin.andrew@PLATFORMedia.com>

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
#include <cstdlib>
#include <iostream>
#include <cstring>
#include <math.h>

using namespace std;
#include "world.h"
#include "util.h"

// #include "configuration.h"

#include "sensor.h"

Sensor::Sensor(World *w) {
  world=w;
}

Sensor::~Sensor() {
}

bool Sensor::loadFromXML(TiXmlElement *node) {
  id=CConfiguration::getAttributeInteger(node, (char *)"id", -1);
  name=CConfiguration::getAttributeString(node, (char *)"name", (char *)"");

  WORLD_DEBUG(80, cout << "In sensorNode '" << name << "'" << endl);

  type=CConfiguration::getAttributeString(node, (char *)"type", (char *)"");
  version=CConfiguration::getAttributeString(node, (char *)"version", (char *)"");

  bool attached=0;
  TiXmlElement *subnode = node->FirstChildElement();
  while(subnode != NULL) {
    if(strequali(subnode->Value(), "attached") ) {
      WORLD_DEBUG(80, cout << " found the attached definition" << endl);
      Body *body=world->getBodyByName(CConfiguration::getAttributeString(subnode, (char *)"body", (char *)"") );
      if(body) {
        WORLD_DEBUG(90, cout << " attached to body '" << body->name << "'" << endl);
        body_id=body->id;
        attached=true;
      }
    }
    subnode = subnode->NextSiblingElement();
  }
  if(!attached) {
    cerr << "No attachment joint found for sensor '" << name << "'" << endl;
  }
  return(attached);
}

void Sensor::getCurrentAccel(float *ret) {
//  cerr << "getCurrentAccel not implemented for sensor '" << name << "'" << endl;

/*
    const dReal *accel=dBodyGetForce(body_id); // This is in m/s/s
    for(int i=0; i<3; i++) {
        ret[i]=(float)accel[i];
    }
 */
  float g[3];
  dVector3 accel;

  world->getGravity(g);
  dBodyVectorFromWorld(body_id, g[0], g[1], g[2], accel);

  for(int i=0; i<3; i++) {
    ret[i]=(float)accel[i];
  }
  return;
}

void Sensor::getCurrentAngularRate(float *ret) {
//  cerr << "getCurrentAngularRate not implemented for sensor '" << name << "'" << endl;
  const dReal *rate=dBodyGetAngularVel(body_id); // This is in rad/s
  for(int i=0; i<3; i++) {
    ret[i]=(float)rate[i];
  }
  return;
}

