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

Geom::Geom(World *w) {
  world=w;
// noop - need more time prior to creation for proper ODE setup
}

Geom::~Geom() {
// The problem with destroying here is that <vector>push_back()s do copy-and-destroy implicitly
// noop - handle more sensitively because of ODE
}

void Geom::display() {
  dReal scale=world->file_lengths_to_meters;
  const dReal *pos = dGeomGetPosition(id);
  cout << "  Geom '" << name << "' at ("
       << pos[0]/scale << ","
       << pos[1]/scale << ","
       << pos[2]/scale << "),";

  dVector3 sides;
  dGeomBoxGetLengths(id, sides);
  cout << " sized :("
       << sides[0]/scale << "x"
       << sides[1]/scale << "x"
       << sides[2]/scale << ")\n";

//	const dReal *pos = dGeomGetPosition(id);
}

bool Geom::loadFromXML(TiXmlElement*geomNode) {
  bool haveXYZ=0;
  name=CConfiguration::getAttributeString(geomNode, (char *)"name", (char *)"");
  WORLD_DEBUG(80, cout << "In GeomNode '" << name << "'" << endl);

  dReal x, y, z; // Need to keep these coordinates fresh
  if(strequali(geomNode->Value(), "box") ) { // Create a box
    dReal scale=world->file_lengths_to_meters;
    WORLD_DEBUG(80, cout << " found a box" << endl);

    x=scale*(dReal) CConfiguration::getAttributeFloat(geomNode, (char *)"x", 0.0);
    y=scale*(dReal) CConfiguration::getAttributeFloat(geomNode, (char *)"y", 0.0);
    z=scale*(dReal) CConfiguration::getAttributeFloat(geomNode, (char *)"z", 0.0);
    WORLD_DEBUG(90, cout << "  at :(" << x << "," << y << "," << z << ")" << endl);

// # OpenGL frame (?) : X back, Y right, Z up (x~depth, y~width, z~height)
    dReal depth, width, height;
    depth =scale*(dReal)fabs(CConfiguration::getAttributeFloat(geomNode, (char *)"depth", 1.0) );
    width =scale*(dReal)fabs(CConfiguration::getAttributeFloat(geomNode, (char *)"width", 1.0) );
    height=scale*(dReal)fabs(CConfiguration::getAttributeFloat(geomNode, (char *)"height", 1.0) );
    WORLD_DEBUG(90, cout << "  sized :(" << depth << "x" << width << "x" << height << ")" << endl);

    // A random number in (0, 1)
#define rand01() ( (float)rand()/(float)RAND_MAX)
    if(1) {
      float v=0.01f;
      depth  *= 1.0 - rand01() * v; // shrink with a % variability
      width  *= 1.0 - rand01() * v; // shrink with a % variability
      height *= 1.0 - rand01() * v; // shrink with a % variability
    }

    id=dCreateBox(world->space, depth, width, height);
//  dGeomSetPosition(id, x, y, z);

//		dGeomSetBody(id, 0);
//		dGeomSetOffsetWorldPosition(id, x, y, z);
//		dGeomSetWorldPosition(id, x, y, z);
//		dBodySetPosition(id, x,y,z);

//		display();

    haveXYZ=1;
  }
  TiXmlElement*node = geomNode->FirstChildElement();
  while(node != NULL) {
    if(strequali(node->Value(), "color") ) {
      WORLD_DEBUG(80, cout << " found color attributes " << endl);

      r=CConfiguration::getAttributeFloat(node, (char *)"r", 0.5);
      g=CConfiguration::getAttributeFloat(node, (char *)"g", 0.5);
      b=CConfiguration::getAttributeFloat(node, (char *)"b", 0.5);
      WORLD_DEBUG(90, cout << "  rgb :(" << r << "," << g << "," << b << ")" << endl);
    }
    node = node->NextSiblingElement();
  }

  //	if(okay) {
  if(true) {
    // Now it's created, we need to assign it to a body
    string body_name=CConfiguration::getAttributeString(geomNode, (char *)"body", (char *)"");

    // Search through world's bodies, looking for name
    for(vector<Body>::iterator body=world->body_list.begin(); body!=world->body_list.end(); ++body) {
      if(strequali(body_name.c_str(), body->name.c_str() ) ) {
//				cout << " found body '" << body_name << "' for geom '" << name << "'\n";
//				body->display();

        dGeomSetBody(id, body->id); // attach it to the body

        if(haveXYZ) { // Now need to set its coordinates - relative to the body...
//       const dReal *pos = dBodyGetPosition(body->id);
//			  dGeomSetPosition(id, x-pos[0], y-pos[1], z-pos[2]);
          dGeomSetOffsetWorldPosition(id, x, y, z);
        }
      }
    }
    // Just check we're still at same place...
    WORLD_DEBUG(50, display() );
  }
  return haveXYZ;
}

