#ifndef __WORLD_H__
#define __WORLD_H__

#include <vector>
#include <string>

#include <ode/ode.h>
// #include "config.h" // Created by ./configure
#include "configuration.h" // This is for XMLparser helper functions

// This is for the locking of the World object so that the GUI thread can read
// positions safely
#include <boost/thread.hpp>

#include <boost/date_time/posix_time/posix_time_types.hpp>
namespace bpt=boost::posix_time;

#ifdef __cplusplus
extern "C" {
#endif

#define ODE_MAX_CONTACTS 20

#define WORLD_DEBUG(L, X)    if(world->load_debug > L) { X;  }

class Body;
class Joint;
class Geom;

class Servo;
class Sensor;

class World {
public:
  dWorldID id;                  //World (Physics)
  dSpaceID space;
  dJointGroupID contactgroup;

  vector<Body> body_list;
  vector<Geom> geom_list;
  vector<Joint> joint_list;

  vector<Servo> servo_list;
  vector<Sensor> sensor_list;

  dGeomID ground;

  bpt::ptime wall_time_t0;  // This is wall-clock time base (boost::posix_time)
  float simulation_time;    // This is the time up to which we've currently simulated in seconds (ODE units)
  float simulation_time_end;  //  Simulation time at which we exit the program (<=0 for never)
  float simulation_time_dilation;  //  Ratio of wall-clock time to simulation time (0 to simulate as-fast-as-possible)
  float simulation_time_step;  //  Timesteps to be used for simulations (don't mix and match, according to ODE docs) - in secs

// Convert to SI units immediately on loading (ODE works in meters-kilograms-seconds)
  dReal file_lengths_to_meters;
  dReal file_masses_to_kilograms;
  dReal file_inertia_to_si;
  float geometry_variability_factor;

  int load_debug;

  World();
  ~World();

  bool loadXML(char *xmlFilename);
  void display();

  void setGravity(float g);  // (0,0,-g) setting
  void getGravity(float *g); // returns the vector
  void setGeometryVariabilityFactor(float f);

  void setTimestep(float _simulation_time_step);

  void simulateToMatchCurrentWallClock(void(callback) (void *data, dGeomID o1, dGeomID o2) );
  void simulateSingleStep(void(callback) (void *data, dGeomID o1, dGeomID o2) );

  void resetTime();
  float elapsedTime(void);

  Body *getBodyByName(const char *name);
  Joint *getJointByName(const char *name);

  boost::mutex guard_ode_data;

private:
};

class Geom {
public:
  dGeomID id;
  float r, g, b;
  string name;

  Geom(World *w);
  ~Geom();

  bool loadFromXML(TiXmlElement*geomNode);
  void display();

private:
  World *world;
};

class Joint {
public:
  dJointID id;
  string name;

  Joint(World *w);
  ~Joint();

  bool loadFromXML(TiXmlElement*jointNode);
  void display();

// int  getServoBusId();

private:
  World *world;
// int temp_servo_bus_id;  // This is read from the XML - really relates to electronics, not world

//	 dBodyID body1, body2;
};

class Body  {
public:
  dBodyID id;
  string name;

  Body(World *w);
  ~Body();

  bool loadFromXML(TiXmlElement*node);
  void display();

private:
  World *world;
};

class BodyState {
public:
  void save(Body *b);
  void restore(Body *b);

private:
  dReal position[3];
  dQuaternion rotation;
  dReal linear_vel[3];
  dReal angular_vel[3];
};

class WorldState {
public:
  WorldState(World *);
  ~WorldState();

  void save();
  void restore();

private:
  vector<BodyState> body_state_list;
  World *world;
};

void nearCallback(void *data, dGeomID o1, dGeomID o2);

/* closing bracket for extern "C" */
#ifdef __cplusplus
}
#endif

#endif

