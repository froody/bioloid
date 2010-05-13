#ifndef __SERVO_H__
#define __SERVO_H__

#include <ode/ode.h>
#include "configuration.h" // This is for XMLparser helper functions
#include <boost/thread.hpp>

#ifdef __cplusplus
extern "C" {
#endif

class World;

/*
   class Body;
   class Joint;
   class Geom;
 */

#define BIOLOID_VALUE_TO_RADIANS (1.0/1024.0*300.0/360.0 * 2.0*M_PI)
#define RADIANS_TO_BIOLOID_VALUE (1024.0*360.0/300.0 / 2.0 / M_PI)

class Servo {
public:
  int id;
  string name;

  string type;
  string version;

//	 boost::mutex guard_servo_data;

//Servo(World *w, int _id, string _name, dJointID _joint_id);
  Servo(World *w);
  ~Servo();

  bool loadFromXML(TiXmlElement *servoNode);
  void display();
  void apply_limits();
  void update_joint();

  void  setTargetPosition(float pos_radians);
  float getCurrentPosition(); // In radians
  float getCurrentVelocity(); // In radians/second
  float getCurrentTorque();   // In % of capability
  void  setMotorCurrentMax(float _motor_current_max); // 0..100%
  void  setMovingSpeed(float fraction_of_capability);
  bool  isMovingUnderPower();
  void  setCompliance(float slope_ccw, float margin_ccw, float _punch, float margin_cw, float slope_cw);

  float current_to_torque(float _current);

private:
  World *world;
  dJointID joint_id;

  float angle_limit_max, angle_limit_min, motor_current_max;

  float compliance_margin_cw, compliance_margin_ccw;
  float compliance_slope_cw,  compliance_slope_ccw;

  float target_position, moving_speed;

// Used for scaling of the outputs
  float torque_capability;
  float moving_speed_capability;

  float latest_position, latest_velocity, latest_load;
  float punch;

  bool moving_under_power;

  float initial_offset_to_zero_position;

  dJointFeedback feedback; // This gets filled in dynamically by the dJointSetFeedback function
};


/* closing bracket for extern "C" */
#ifdef __cplusplus
}
#endif

#endif

