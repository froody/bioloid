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

#include "configuration.h"

Joint::Joint(World *w) {
  world=w;

  // the 'temp_servo_bus_id' is really there for the Servo creation (and linking to the Bus structures)
  // -2 is not set at all (joint not from XML?)
  // -1 is not present in XML for this joint
  // <0 implies no servo created on bus (if there's a bus)
//  temp_servo_bus_id=-2;
}

Joint::~Joint() {
// noop
}

void Joint::display() {
  cout << "Joint::display\n";
}

bool Joint::loadFromXML(TiXmlElement*jointNode) {
  name=CConfiguration::getAttributeString(jointNode, (char *)"name", (char *)"");

  WORLD_DEBUG(80, cout << "In jointNode '" << name << "'" << endl);

  bool gotCenter=0, gotAxis=0;

  Body *body1=world->getBodyByName(CConfiguration::getAttributeString(jointNode, (char *)"body1", (char *)"") );
  Body *body2=world->getBodyByName(CConfiguration::getAttributeString(jointNode, (char *)"body2", (char *)"") );
  if(body1 && body2) { // If we found these bodies
//   void dJointAttach (dJointID, dBodyID body1, dBodyID body2);
    id = dJointCreateHinge(world->id, 0);
    dJointAttach(id, body1->id, body2->id);
    WORLD_DEBUG(80, cout << " creating joint between '" << body1->name << "' and '" << body2->name << "'" << endl);
  }
  else {
    return false;  // Couldn't attach => fail
  }

  TiXmlElement*node = jointNode->FirstChildElement();
  while(node != NULL) {
    if(strequali(node->Value(), "center") || strequali(node->Value(), "centre") ) {
      dReal scale=(dReal)world->file_lengths_to_meters;
      WORLD_DEBUG(80, cout << " found the center" << endl);

      dReal x=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"x", 0.0);
      dReal y=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"y", 0.0);
      dReal z=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"z", 0.0);
// void dJointSetHingeAnchor (dJointID, dReal x, dReal y, dReal z);  // xyz=Centre of Joint in world frame
      dJointSetHingeAnchor(id, x, y, z);
      gotCenter=true;
    }
    if(strequali(node->Value(), "axis") ) {
      dReal scale=(dReal)world->file_lengths_to_meters;
      WORLD_DEBUG(80, cout << " found the axis" << endl);

      dReal x=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"x", 0.0);
      dReal y=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"y", 0.0);
      dReal z=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"z", 0.0);
// void dJointSetHingeAxis (dJointID, dReal x, dReal y, dReal z); // xyz=axis of Joint in world frame (relative to Joint?)
      dJointSetHingeAxis(id, x, y, z);
      gotAxis=true;
    }
    node = node->NextSiblingElement();
  }
  if(!gotCenter) {
    cerr << "No center found for joint id=" << id << endl;
  }
  if(!gotAxis) {
    cerr << "No axis found for joint id=" << id << endl;
  }
  return(gotCenter && gotAxis);
}

//int Joint::getServoBusId() {
//  return temp_servo_bus_id;
//}

// See : http://opende.sourceforge.net/mediawiki-1.6.10/index.php/Manual_%28Joint_Types_and_Functions%29

