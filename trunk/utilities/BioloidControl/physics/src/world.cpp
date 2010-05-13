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

#include "servo.h"
#include "sensor.h"

#include "util.h"
#include "configuration.h"

// See also : http://wiki.nccaforum.com/index.php?title=Ode_tutorial
World::World() {
  // Guard against anyone looking at incomplete data
  boost::mutex::scoped_lock scoped_lock(guard_ode_data);
  id = dWorldCreate();

  dWorldSetAutoDisableFlag(id, 0); // don't auto-disable

//	dWorldSetContactMaxCorrectingVel(id, 0.1);  // Reduce 'popping' of deeply embedded contacts
  dWorldSetContactMaxCorrectingVel(id, dInfinity);

  dWorldSetContactSurfaceLayer(id, 0.0); // This is ZERO
//  dWorldSetContactSurfaceLayer(id, 0.0005); // This is .5mm 'mushyness on surfaces'

//	dWorldSetLinearDamping(id, .8);
//  dWorldSetLinearDampingThreshold(id, 0.0);

  space = dSimpleSpaceCreate(0);

  contactgroup = dJointGroupCreate(0);
  dJointGroupEmpty(contactgroup);

  setGravity(9.81); // Units are Metres, Seconds, Kilograms,

  dWorldSetCFM(id, 0.0001); // Hard
//  dWorldSetCFM(id, 0.001); // Firmer
//  dWorldSetCFM(id, 0.01); // Bouncy / hard ground
//  dWorldSetCFM(id, 0.1); // Very soft ground

//  dWorldSetERP(id,  3); // Exploder
  dWorldSetERP(id,  0.3); //
//  dWorldSetERP(id,  0.03); // Very loose joints

  ground=dCreatePlane(space, 0, 0, 1, 0);  // Plane with normal=z, height=0 : Ground

  resetTime();
  geometry_variability_factor=0.01f; // Jitter all the geometry a little

  load_debug=100; // default
}

World::~World() {
  // Guard against anyone looking at incomplete data
  boost::mutex::scoped_lock scoped_lock(guard_ode_data);

//	dJointGroupDestroy (contactgroup);
//	dSpaceDestroy (space);

  for(vector<Joint>::iterator joint=joint_list.begin(); joint!=joint_list.end(); ++joint) {
    dJointDestroy(joint->id);
  }
  for(vector<Body>::iterator body=body_list.begin(); body!=body_list.end(); ++body) {
    dBodyDestroy(body->id);
  }
  for(vector<Geom>::iterator geom=geom_list.begin(); geom!=geom_list.end(); ++geom) {
    dGeomDestroy(geom->id);
  }
  dWorldDestroy(id);

  // No need to destroy servos and sensors - they have no 'ode' reality
}

void World::display() {
  cout << "Scaling Factors: \n"
       << "	file->meters       :"<< file_lengths_to_meters << "\n"
       << "	file->kg           :"<< file_masses_to_kilograms << "\n"
       << "	file->si (inertia) :"<< file_inertia_to_si << endl;

  cout << "#Geoms =" << geom_list.size() << endl;
  cout << "#Bodies=" << body_list.size() << endl;
}

void World::setGravity(float g) {
  dWorldSetGravity(id, 0, 0, -g);  // Units are Metres, Seconds, Kilograms, z is up
}

void World::getGravity(float *g) { // pass a float[3] in
  dVector3 gravity;
  dWorldGetGravity(id, gravity);  // Units are Metres, Seconds, Kilograms, z is up
  for(int i=0; i<3; i++) {
    g[i]=(float)gravity[i];
  }
}

bool World::loadXML(char *xmlFilename) {
  // Guard against anyone looking at incomplete data
  boost::mutex::scoped_lock scoped_lock(guard_ode_data);

  CConfiguration config;
  bool okay = config.load(xmlFilename);
  sprintf(config.rootNode, "Robot");
  if(okay) {
    cout << xmlFilename << " : loaded ok" << endl;

    TiXmlElement*physicsNode = config.findNode( (char *)"physics");
    if(physicsNode == NULL) {
      cerr << xmlFilename << " : could find physics node" << endl;
      okay=0;
    }
    else {
      file_lengths_to_meters  =(dReal) CConfiguration::getAttributeFloat(physicsNode, (char *)"length_to_m", 1.0);
      file_masses_to_kilograms=(dReal) CConfiguration::getAttributeFloat(physicsNode, (char *)"mass_to_kg", 1.0);
      file_inertia_to_si      =(dReal) CConfiguration::getAttributeFloat(physicsNode, (char *)"inertia_to_si", 1.0);

      TiXmlElement*node = physicsNode->FirstChildElement();
      while(node != NULL) {
        if(strequali(node->Value(), "body") ) { // create a body
          Body *body=new Body(this);
          if(!body->loadFromXML(node) ) {
            cerr << "  *** Could not initialize body from XML ***" << endl;
            okay=0;
          }
          body_list.push_back(*body);
        }
// *
        if(strequali(node->Value(), "joint") ) { // create a joint
          Joint *joint=new Joint(this);
          if(!joint->loadFromXML(node) ) {
            cerr << "   *** Could not initialize joint from XML ***" << endl;
            okay=0;
          }
          joint_list.push_back(*joint);
        }
// * /
        node = node->NextSiblingElement();
      }
      if(load_debug>20) {
        cout << xmlFilename << " : total of " << body_list.size() << " bodies found" << endl;
      }
    }

    TiXmlElement*electronicsNode = config.findNode( (char *)"electronics");
    if(electronicsNode == NULL) {
      cerr << xmlFilename << " : could find electronics node" << endl;
      okay=0;
    }
    else {
      TiXmlElement *node = electronicsNode->FirstChildElement();
      while(node != NULL) {
        if(strequali(node->Value(), "servo") ) { // create a servo
          Servo *servo=new Servo(this);
          if(!servo->loadFromXML(node) ) {
            cerr << "Could not initialize servo from XML" << endl;
            okay=0;
          }
          else {
            servo_list.push_back(*servo);
          }
        }
        if(strequali(node->Value(), "sensor") ) { // create a sensor
//          cerr << xmlFilename << " : Can't deal with sensors yet ! " << endl;
          Sensor *sensor=new Sensor(this);
          sensor->loadFromXML(node);
          sensor_list.push_back(*sensor);
        }
        node = node->NextSiblingElement();
      }
      if(load_debug>20) {
        cout << xmlFilename << " : total of " << servo_list.size() << " servos found" << endl;
        cout << xmlFilename << " : total of " << sensor_list.size() << " sensors found" << endl;
      }
    }

    TiXmlElement*geometryNode = config.findNode( (char *)"geometry");
    if(geometryNode == NULL) {
      cerr << xmlFilename << " : could find geometrynode" << endl;
      okay=0;
    }
    else {
      TiXmlElement*node = geometryNode->FirstChildElement();
      int i=0;
      while(node != NULL) {
        Geom *geom=new Geom(this);
        geom->loadFromXML(node);
        geom_list.push_back(*geom);
        i++;
        node = node->NextSiblingElement();
      }
      if(load_debug>20) {
//        cout << xmlFilename << " : total of " << i << " geoms found\n";
        cout << xmlFilename << " : total of " << geom_list.size() << " geoms found" << endl;
      }
    }
  }
  return okay;
}

Body *World::getBodyByName(const char *name) {
  Body *b=NULL;
  bool found=false;
  for(int i=0; i<body_list.size() && !found; i++) {
    if(strequali(body_list[i].name.c_str(), name) ) {
      b=&body_list[i];
      found=true;
    }
  }
  if(!found) {
    cerr << "Could not find body '" << name << "'" << endl;
  }
  return b;
}

Joint *World::getJointByName(const char *name) {
  Joint *j=NULL;
  bool found=false;
  for(int i=0; i<joint_list.size() && !found; i++) {
    if(strequali(joint_list[i].name.c_str(), name) ) {
      j=&joint_list[i];
      found=true;
    }
  }
  if(!found) {
    cerr << "Could not find joint '" << name << "'" << endl;
  }
  return j;
}

void World::setGeometryVariabilityFactor(float f) {
  geometry_variability_factor=f;
}

void World::resetTime(void) {
  wall_time_t0=bpt::microsec_clock::local_time();
  simulation_time=0.0f;
}

float World::elapsedTime(void) {
  bpt::time_duration t_elapsed=bpt::microsec_clock::local_time() - wall_time_t0;
  return (float)(t_elapsed.total_microseconds()/1000000.0f);
}

void World::setTimestep(float _simulation_time_step) {
  simulation_time_step=_simulation_time_step;
}

void World::simulateToMatchCurrentWallClock(void(callback) (void *data, dGeomID o1, dGeomID o2) ) {
  float simulation_time_step=0.01f; // in seconds
  float wall_time_elapsed=elapsedTime();

  // slight approximation, since time is elapsing while simulating too...
  int simulations=0;
  while(simulation_time<wall_time_elapsed) {
    simulateSingleStep(callback);
    simulations++;
  }
// cout << "Simulations per refresh : " << simulations << endl;
// cout << "Actual time : " << wall_time_elapsed << endl;
// cout << "Simulation time : " << simulation_time << endl;
}

void World::simulateSingleStep(void(callback) (void *data, dGeomID o1, dGeomID o2) ) {
  // Protect this from being copied out by another thread
  boost::mutex::scoped_lock scoped_lock(guard_ode_data);

  // Move all the servos
  for(vector<Servo>::iterator servo=servo_list.begin(); servo!=servo_list.end(); servo++) {
    servo->update_joint();
  }

  dSpaceCollide(space, (void *)this, callback);

//  dWorldQuickStep(id, simulation_time_step);
  dWorldStep(id, simulation_time_step);
  dJointGroupEmpty(contactgroup);

  simulation_time += simulation_time_step;
}

WorldState::WorldState(World *w) {
  world=w;
}

WorldState::~WorldState() {
// Don't try and delete the World*
}

void WorldState::save() {
  // Protect the World from modified by ODE
  boost::mutex::scoped_lock scoped_lock(world->guard_ode_data);

  body_state_list.clear();
  for(int i=0; i<world->body_list.size(); i++) {
    BodyState bs;
    bs.save(&world->body_list[i]);
    body_state_list.push_back(bs);
  }
  cout << "Saved Body List has " << body_state_list.size() << " entries" << endl;
}

void WorldState::restore() {
  // Protect the World from modified by ODE

  boost::mutex::scoped_lock scoped_lock(world->guard_ode_data);
  if(world->body_list.size() != body_state_list.size() ) {
    cerr << "Saved body state size mismatch : "
         << world->body_list.size() << " in world, and "
         << body_state_list.size() << " in saved WorldState" << endl;
    return;
  }
  for(int i=0; i<body_state_list.size(); i++) {
    body_state_list[i].restore(&world->body_list[i]);
  }
}

// SEE : http://www.ode.org/ode-latest-userguide.html

// this is called by dSpaceCollide when two objects in space are potentially colliding.
void nearCallback(void *data, dGeomID o1, dGeomID o2) {
  World *w=(World *)data;

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);

  if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact) ) {
    return;
  }

  dContact contact_list[ODE_MAX_CONTACTS]; // Used for ODE to dump the parameters to us
  dContact contact; // There is only one set of parameters required for all of these contacts...

  // friction parameter
//  contact.surface.mu = dInfinity;
//  contact.surface.mu = 0.5;   // Pavement
  contact.surface.mu = 0.7;   // Rough
// contact.surface.mu = 0.01;  // Icy

//  contact.surface.mu2 = 0.5;

  // bounce is the amount of "bouncyness".  0..1
//  contact.surface.bounce = 0.5;
  contact.surface.bounce = 0.1;

  // bounce_vel is the minimum incoming velocity to cause a bounce
//  contact.surface.bounce_vel = 0.00001;
  contact.surface.bounce_vel = 0.01;
//  contact.surface.bounce_vel = 0.01;

  //  What is this?  Seems to reduce some Error messages (if mu<dInfinity)
  contact.surface.slip1 = 0.0;
  contact.surface.slip2 = 0.0;

  // constraint force mixing parameter
// contact.surface.soft_cfm = 0.0001; // Gave rise to foot tapping problem
  contact.surface.soft_cfm = 0.02;
  contact.surface.soft_erp = 0.3;
//  contact.surface.soft_cfm = 10;
//  contact.surface.soft_erp = 200;

  contact.surface.mode  = 0;
//  contact.surface.mode |= dContactSoftCFM;
//  contact.surface.mode |= dContactSoftERP;
//  contact.surface.mode |= dContactApprox1;
//  contact.surface.mode |= dContactSlip1 | dContactSlip2;

  if( (o1 == w->ground) || (o2 == w->ground) ) { // This is a contact with the ground...
    // The ground is a little different..  maybe
    contact.surface.mode |= dContactSoftCFM;
    contact.surface.mode |= dContactSoftERP;
//    contact.surface.mode |= dContactBounce;
  }
  else { // Contact of robot with itself...
    contact.surface.mu = 0.01;  // Icy
    contact.surface.mode |= dContactBounce;
  }

  int number_of_contacts = dCollide(o1, o2, ODE_MAX_CONTACTS, &contact_list[0].geom, sizeof(dContact) ); // last parameter is a struct-size skip thing (not needed here)
  if(number_of_contacts>0) {
    if(1 && (number_of_contacts>3) ) {
      cout << "Number of contacts (t=" << w->simulation_time << ") : " << number_of_contacts << endl;
    }
    for(int i=0; i<number_of_contacts; i++) {
      contact_list[i].surface=contact.surface; // Copy over the standard stuff

/*
      contact_list[i].surface.mode      =contact.surface.mode;
      contact_list[i].surface.mu        =contact.surface.mu;
      contact_list[i].surface.bounce    =contact.surface.bounce;
      contact_list[i].surface.bounce_vel=contact.surface.bounce_vel;
      contact_list[i].surface.slip1     =contact.surface.slip1;
      contact_list[i].surface.slip2     =contact.surface.slip2;
      contact_list[i].surface.soft_cfm  =contact.surface.soft_cfm;
      contact_list[i].surface.soft_erp  =contact.surface.soft_erp;
 */
      dJointID c_temp = dJointCreateContact(w->id, w->contactgroup, &contact_list[i]);
      dJointAttach(c_temp, b1, b2);
//			if (globalConf.show_contacts) dsDrawBoxReal(contact[i].geom.pos,RI,ss);
    }
  }
}

