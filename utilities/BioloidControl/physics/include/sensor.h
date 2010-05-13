#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <ode/ode.h>

#ifdef __cplusplus
extern "C" {
#endif

class World;

class Sensor {
public:
  int id;
  string name;

  string type;
  string version;

  Sensor(World *w);
  ~Sensor();

  bool loadFromXML(TiXmlElement *sensorNode);
  void display();

  void getCurrentAccel(float *ret);
  void getCurrentAngularRate(float *ret);

private:
  World *world;
  dBodyID body_id;
};

/* closing bracket for extern "C" */
#ifdef __cplusplus
}
#endif

#endif

