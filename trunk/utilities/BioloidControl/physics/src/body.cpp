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

Body::Body(World *w) {
  world=w;
// noop - need more time prior to creation for proper ODE setup
}

Body::~Body() {
// noop - handle more sensitively because of ODE
}

void Body::display() {
  const dReal *pos = dBodyGetPosition(id);

  if(1) {
    dReal scale=world->file_lengths_to_meters;
    cout << " Body '" << name << "' CoG at ("
         << pos[0]/scale << ","
         << pos[1]/scale << ","
         << pos[2]/scale << ")" << endl;
  }

  if(1) {
    dMass m;
    dBodyGetMass(id, &m);
    cout << "  mass : " << m.mass/world->file_masses_to_kilograms;

    dReal scale=world->file_inertia_to_si;
    cout << ", inertia : ("
         << "XX:" << m.I[0*4+0]/scale << ","
         << "YY:" << m.I[1*4+1]/scale << ","
         << "ZZ:" << m.I[2*4+2]/scale << ","
         << "XY:" << m.I[0*4+1]/scale << ","
         << "XZ:" << m.I[0*4+2]/scale << ","
         << "YZ:" << m.I[1*4+2]/scale << ")" << endl;
  }
}

bool Body::loadFromXML(TiXmlElement*bodyNode) {
  id = dBodyCreate(world->id);
  name=CConfiguration::getAttributeString(bodyNode, (char *)"body", (char *)"");
  WORLD_DEBUG(80, cout << "In BodyNode '" << name << "'" << endl);

  bool gotCoG=0, gotInertia=0;
  dReal x, y, z, mass;        // around CoG
  dReal xx, yy, zz, xy, yz, zx; // Inertial matrix

  TiXmlElement*node = bodyNode->FirstChildElement();
  while(node != NULL) {
    if(strequali(node->Value(), "cog") ) {
      WORLD_DEBUG(80, cout << " found : " << node->Value() << endl);

      //	dReal cgx, dReal cgy, dReal cgz,

      dReal scale=(dReal)world->file_lengths_to_meters;
      x=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"x", 0.0);
      y=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"y", 0.0);
      z=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"z", 0.0);

      mass =(dReal)world->file_masses_to_kilograms*(dReal) CConfiguration::getAttributeFloat(node, (char *)"mass", 1.0);

      gotCoG=1;
    }
    if(strequali(node->Value(), "inertia") ) {
      WORLD_DEBUG(80, cout << " found : " << node->Value() << endl);

      //	dReal I11, dReal I22, dReal I33, dReal I12, dReal I13, dReal I23);

      dReal scale=(dReal)world->file_inertia_to_si;
      xx=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"xx", 1.0);
      yy=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"yy", 1.0);
      zz=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"zz", 1.0);
      xy=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"xy", 0.0);
      yz=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"yz", 0.0);
      zx=scale*(dReal) CConfiguration::getAttributeFloat(node, (char *)"zx", 0.0);

      gotInertia=1;
    }
    node = node->NextSiblingElement();
  }
  if(gotCoG && gotInertia) {
    dMass m;
//		dMassSetZero(&m);

//void dMassSetParameters (dMass *, dReal themass, dReal cgx, dReal cgy, dReal cgz, dReal I11, dReal I22, dReal I33, dReal I12, dReal I13, dReal I23);
    dMassSetParameters(&m, mass, 0, 0, 0, xx, yy, zz, xy, zx, yz);
    dBodySetMass(id, &m);

//void dMassTranslate (dMass *, dReal x, dReal y, dReal z);
    dBodySetPosition(id, x, y, z);

//    dBodySetLinearDamping(id, 0.1);
  }
  else {
    cerr << " missing CoG or Inertia definition" << endl;
  }
  WORLD_DEBUG(50, display() );

  return(gotCoG && gotInertia);
}

/* From : http://opende.sourceforge.net/mediawiki-1.6.10/index.php/HOWTO_save_and_restore
 # Reset each body's position - dBody[Get,Set]Position()
 # Reset each body's quaternion - dBody[Get,Set]Quaternion() ODE stores rotation in quaternions, so don't save/restore in euler angles because it will have to convert to/from quaternions and you won't get a perfect restoration.
 # Reset each body's linear velocity - dBody[Get,Set]LinearVel()
 # Reset each body's angular velocity - dBody[Get,Set]AngularVel()
 */

void BodyState::save(Body *b) {
  int i;
  const dReal *temp_pos, *temp_lin, *temp_ang, *temp_q;
  temp_pos=dBodyGetPosition(b->id);
  temp_lin=dBodyGetLinearVel(b->id);
  temp_ang=dBodyGetAngularVel(b->id);
  temp_q=dBodyGetQuaternion(b->id);

  for(i=0; i<3; i++) {
    position[i]=temp_pos[i];
    linear_vel[i]=temp_lin[i];
    angular_vel[i]=temp_ang[i];
  }

  for(i=0; i<4; i++) {
    rotation[i]=temp_q[i];
  }
}

void BodyState::restore(Body *b) {
  dBodySetPosition(b->id, position[0], position[1], position[2]);
  dBodySetLinearVel(b->id, linear_vel[0], linear_vel[1], linear_vel[2]);
  dBodySetAngularVel(b->id, angular_vel[0], angular_vel[1], angular_vel[2]);
  dBodySetQuaternion(b->id, rotation);
}

