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

#include "servo.h"

#define KgM_TO_NewtonM 9.808385
#define JOINT_CONST_FRICTION (0.0203 *KgM_TO_NewtonM)
#define JOINT_RATE_FRICTION  (0.045  *KgM_TO_NewtonM)

// This was from Simloid, which seems to have a newtons->Kg.m problem
// #define JOINT_CONST_FRICTION (0.0203 * 1.0)
// #define JOINT_RATE_FRICTION  (0.045  * 1.0)

Servo::Servo(World *w) {
  world=w;

// NB : Don't use joint_id until we determine it from the XML file !

  // These get told to us over the bus (maybe - print out AX12 manual)
  angle_limit_max =  M_PI *150.0/180.0;
  angle_limit_min = -M_PI *150.0/180.0;
// apply_limits();

  setMotorCurrentMax(1.0);

  // These are to do with the servo control logic
  // See page 17/38 of AX-12 manual to see what's going on generically
  // and page 28/38 to get units correct
  // in %/1024 = 300 degrees of full circle

  setCompliance(
    64.0 * BIOLOID_VALUE_TO_RADIANS, // A - In radians
    1.0 * BIOLOID_VALUE_TO_RADIANS,  // B - In radians
    32.0/1024.0,                     // Punch : The minimum current/torque supplied to motor to get it to move (as a % of maximum torque)
    1.0 * BIOLOID_VALUE_TO_RADIANS,  // C - In radians
    64.0 * BIOLOID_VALUE_TO_RADIANS  // D - In radians
    );

  moving_speed_capability=114.0*2.0*M_PI/60.0; // 114 RPM in Radian per second
  setMovingSpeed(1.0);

  // The newton is the unit of force derived in the SI system; it is equal to the amount of force required to give a mass of one kilogram an acceleration of one metre per second squared. Algebraically:
  // {\rm 1~N = 1~\frac{kg\cdot m}{s^2}}.
  // See : http://www.linengineering.com/flash/Calculator.swf

  torque_capability=13.0/100.0*KgM_TO_NewtonM;  // 13 kg.cm in kg.m - converted to Newton.meters

  // These get updated by the physics
  latest_position=0;
  latest_velocity=0;

  // Update if requested..
  latest_load=0;

  initial_offset_to_zero_position=0;  // Default - will get set below
  moving_under_power=false;
}

Servo::~Servo() {
  dJointSetFeedback(joint_id, (dJointFeedback *)0);
}

bool Servo::loadFromXML(TiXmlElement *node) {
  id=CConfiguration::getAttributeInteger(node, (char *)"id", -1);
  name=CConfiguration::getAttributeString(node, (char *)"name", (char *)"");

  WORLD_DEBUG(80, cout << "In servoNode '" << name << "'" << endl);

  type=CConfiguration::getAttributeString(node, (char *)"type", (char *)"");
  version=CConfiguration::getAttributeString(node, (char *)"version", (char *)"");

  bool attached=0;
  TiXmlElement *subnode = node->FirstChildElement();
  while(subnode != NULL) {
    if(strequali(subnode->Value(), "attached") ) {
      WORLD_DEBUG(80, cout << " found the attached definition" << endl);
      Joint *joint=world->getJointByName(CConfiguration::getAttributeString(subnode, (char *)"joint", (char *)"") );
      if(joint) {
        WORLD_DEBUG(90, cout << " attached to joint '" << joint->name << "'" << endl);
        joint_id=joint->id;
        dJointSetFeedback(joint_id, &feedback);
        attached=true;

        apply_limits(); // Must be first - meaningless limits, but sets frictional drag to zero velocity

        // These get updated by the electronics
        setTargetPosition(0.0);
      }
      initial_offset_to_zero_position=CConfiguration::getAttributeFloat(subnode, (char *)"initial_degrees", 0.0f) * 2.0 * M_PI / 360.0;
      initial_offset_to_zero_position=CConfiguration::getAttributeFloat(subnode, (char *)"initial_radians", initial_offset_to_zero_position);
    }
//
    subnode = subnode->NextSiblingElement();
  }
  if(!attached) {
    cerr << "No attachment joint found for servo=" << id << endl;
  }
  display();
  return(attached);
}

float Servo::current_to_torque(float _current) { // In range 0 .. 1.0
  return _current*torque_capability; // This is definitely simplistic
}

void Servo::apply_limits() {
  cout << "Servo : " << name << " - apply_limits()" << endl;

// dReal dJointGetHingeParam (dJointID, int parameter);
//	void  dJointSetHingeParam (dJointID, int parameter, dReal value);
// parameter : {dParamLoStop,dParamHiStop, dParamVel, dParamFMax}

//	dJointSetHingeParam(joint_id, dParamLoStop, (dReal) -.2);
//	dJointSetHingeParam(joint_id, dParamHiStop, (dReal) .2);
  dJointSetHingeParam(joint_id, dParamLoStop, (dReal)angle_limit_min);
  dJointSetHingeParam(joint_id, dParamHiStop, (dReal)angle_limit_max);

//	dJointSetHingeParam(joint_id, dParamFMax, (dReal)0); // Apply this much torque
  dJointSetHingeParam(joint_id, dParamFMax, (dReal)JOINT_CONST_FRICTION); // Apply this much torque
  dJointSetHingeParam(joint_id, dParamVel, (dReal)0);  // to get to this speed
  return;
}

void Servo::display(void) {
// boost::mutex::scoped_lock scoped_lock(guard_servo_data);
  cout << "Servo : " << name << endl;
  cout << " Position = " << latest_position+initial_offset_to_zero_position << ", velocity = " << latest_velocity << endl;
}

void Servo::update_joint(void) {
  latest_position=dJointGetHingeAngle(joint_id);
  latest_velocity=dJointGetHingeAngleRate(joint_id);

  if(0) { // This is random jitter testing
    target_position += ( ( (float)rand()/(float)RAND_MAX)-.5)*.1; //  Jitter
//	target_position = .5;
  }

//      return; //  No foot-tapping if stopped here...  //TEST

  if(0) {
    // Impose bounds on target_position
    if(target_position>angle_limit_max) {
      cerr << "Target " << target_position << "> max=" << angle_limit_max << endl;
      target_position=angle_limit_max;
    }
    if(target_position<angle_limit_min) {
      cerr << "Target " << target_position << "< min=" << angle_limit_min << endl;
      target_position=angle_limit_min;
    }
  }

/*
    if(0) { // This is version 1 - Pretty much discredited
        // Compliance curves too difficult to calculate for SERVO itself
        // Also : Friction too fiddly to add; Oscillations appear at all timesteps
        float torque_mult=0.0, position_error=latest_position-target_position, offset_temp;
        if( (position_error>-compliance_margin_ccw) &&
                        (position_error<compliance_margin_cw)
        ) {
            // Nothing to do - we're already within close enough tolerance
            // Don't add any torque
                if(id==1) {
                    cerr << name << " - nothing to do " << endl;
                }
        }
        else {
            if(position_error<-compliance_margin_ccw)  { // On LHS of diagram - ultra negative position error
                offset_temp=(compliance_margin_ccw+compliance_slope_ccw)+position_error;
                torque_mult=1.0-offset_temp*(1.0-punch)/compliance_slope_ccw;
                if(id==1) {
                    cerr << name << " on LHS - error = " << (position_error*(180.0/M_PI)) << ", torque_mult=" << torque_mult << endl;
                }
                if(torque_mult>1.0) { torque_mult=1.0;	}
                else if(torque_mult<punch) {	torque_mult=0;	}
            }
            else { // On RHS of diagram : ultra positive position error
                offset_temp=(compliance_margin_cw+compliance_slope_cw)-position_error;
                torque_mult=-1.0+offset_temp*(1.0-punch)/compliance_slope_cw;
                if(id==1) {
                    cerr << name << " on RHS - error = " << (position_error*(180.0/M_PI)) << ", torque_mult=" << torque_mult << endl;
                }
                if(torque_mult<-1.0) { torque_mult=-1.0;	}
                else if(torque_mult>-punch) {	torque_mult=0;	}
            }
        }

        moving_under_power=false;
        if(torque_mult != 0.0) {
            moving_under_power=true;
            if(torque_mult<0) {
                dJointSetHingeParam(joint_id, dParamFMax,  (dReal)(-torque_mult*torque_max));  // Set max (positive) torque
                dJointSetHingeParam(joint_id, dParamVel,   (dReal)-moving_speed); // Move in -ve direction
            }
            else {
                dJointSetHingeParam(joint_id, dParamFMax,  (dReal)(torque_mult*torque_max));   // Set max torque
                dJointSetHingeParam(joint_id, dParamVel,   (dReal)moving_speed);  // Move in +ve direction
            }
    //  dJointAddHingeTorque(joint_id, (dReal)(torque_mult*torque_max));  // NO!
        }
    }
 */

  if(1) { // This is version 2 - seems to be nice and stable, and realistic
    // Compliance curves can be calculated by SERVO
    float current=0.0, motor_torque=0.0, friction_torque=0.0;
    int direction=0;
    bool outside_compliance=false;

    float error_lhs = (target_position-compliance_margin_ccw) - latest_position;
    if(error_lhs>0) {  // Something to do
      direction=+1;
      current = punch + error_lhs/compliance_slope_ccw; // This goes from punch + (0 .. 1) in compliance_slope steps
      if(id==-1) {
        cerr << name << " on LHS error = " << (error_lhs*(180.0/M_PI) ) << ", current=" << current << endl;
      }
    }

    float error_rhs = latest_position - (target_position+compliance_margin_cw);
    if(error_rhs>0) {  // Something to do
      direction=-1;
      current = punch + error_rhs/compliance_slope_cw; // This goes from punch + (0 .. 1) in compliance_slope steps
      if(id==-1) {
        cerr << name << " on RHS error = " << (error_rhs*(180.0/M_PI) ) << ", current=" << current << endl;
      }
    }

    moving_under_power=(direction != 0);

    if(moving_under_power) {
      if(current>motor_current_max) {
        current=motor_current_max;
        outside_compliance=true;
      }
      motor_torque =  current_to_torque(current) * (float)direction;
    }
    else {
      if(id==-1) {
        cerr << name << " No additional torque" << endl;
      }
    }

    if(1) {
      float friction_velocity=latest_velocity;
      float friction_direction=(friction_velocity>0.0) ? -1.0 : 1.0;

      friction_torque = friction_direction*JOINT_CONST_FRICTION -friction_velocity*JOINT_RATE_FRICTION;
    }

    if(id==-1) {
      cerr << name << " velocity = " << latest_velocity << endl;
    }

    // Apply up to this much force 'net' (slight approximation)
    float net_torque=motor_torque+friction_torque;

    if(1) {
      net_torque=fabs(net_torque);  // This needs to be a non-negative value

      // The issue : The motor may get 'caught' on the Slope where
      // (torque output == static friction) => no motion (possible whining sound...)

      //if(net_torque<JOINT_CONST_FRICTION) {
      //  net_torque==JOINT_CONST_FRICTION; // This ensures that there is at least static friction at play
      //}
      if(net_torque<fabs(friction_torque) ) {
        net_torque=fabs(friction_torque); // This ensures that there is at least friction at play
      }
    }

//    dJointSetHingeParam(joint_id, dParamFMax, (dReal)(net_torque) );
    dJointSetHingeParam(joint_id, dParamFMax, (dReal)JOINT_CONST_FRICTION);

    float target_velocity=0.0;
    if(outside_compliance) { // If we're outside the compliance zone, don't allow ourselves to go too quickly
//			target_velocity=moving_speed*(float)direction;
    }
    dJointSetHingeParam(joint_id, dParamVel,  (dReal)target_velocity);  // to get to this speed
  }

  if(0) { // This velocity method isn't the way AX12s are controlled
    float gain=0.5;
    float direction=(target_position > latest_position) ? 1.0 : -1.0;
//	target_velocity = (target_position - latest_position)*gain;
//	dJointSetHingeAngleRate(joint_id, target_velocity);
    dJointSetHingeParam(joint_id, dParamVel,   (dReal)moving_speed*gain*direction);
  }

  return;
}

void Servo::setTargetPosition(float pos_radians) {
  target_position=pos_radians+initial_offset_to_zero_position;
//  cout << name << " setTargetPosition=" << target_position*(180.0/M_PI) << endl;

  dJointSetHingeParam(joint_id, dParamLoStop, (dReal)(target_position-compliance_margin_ccw) );
  dJointSetHingeParam(joint_id, dParamHiStop, (dReal)(target_position+compliance_margin_cw) );
}

float Servo::getCurrentPosition() { // in radians from initial position
  return latest_position-initial_offset_to_zero_position;
}

float Servo::getCurrentVelocity() { // in % of capability
  return latest_velocity/moving_speed_capability;
}

float Servo::getCurrentTorque() { // as a fraction of capability...
  // Because of the 'dJointSetFeedback', the feedback structure already holds this data...
//	dVector3 *v=&feedback.t2;
//	float torque=sqrt(v->x*v->x + v->y*v->y  + v->z*v->z);
  dReal *v=&feedback.t2[0];
  float torque=sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  return torque/torque_capability;
}

void Servo::setMotorCurrentMax(float _motor_current_max) {
  motor_current_max=_motor_current_max;
}

void Servo::setMovingSpeed(float fraction_of_capability) {
  moving_speed=fraction_of_capability*moving_speed_capability;
}

bool Servo::isMovingUnderPower() {
  return moving_under_power;
}

void Servo::setCompliance(float slope_ccw, float margin_ccw, float _punch, float margin_cw, float slope_cw) {
  compliance_slope_ccw  = slope_ccw; // A - In radians
  compliance_margin_ccw = margin_ccw; // B - In radians
  punch                 = _punch; // The minimum current/torque supplied to motor to get it to move (as a % of maximum torque)
  compliance_margin_cw  = margin_cw; // C - In radians
  compliance_slope_cw   = slope_cw; // D - In radians
}

// See : http://opende.sourceforge.net/mediawiki-1.6.10/index.php/Manual_%28Joint_Types_and_Functions%29

// dReal dJointGetHingeAngle(dJointID);
// dReal dJointGetHingeAngleRate(dJointID);

//void dJointSetHingeParam (dJointID, int parameter, dReal value);
//dReal dJointGetHingeParam (dJointID, int parameter);
// dParamLoStop,dParamHiStop, dParamVel, dParamFMax

// dJointAddHingeTorque(dJointID joint, dReal torque)

//void dJointSetFeedback (dJointID, dJointFeedback *);
//dJointFeedback *dJointGetFeedback (dJointID);

/*
   typedef struct dJointFeedback {
   dVector3 f1; // force that joint applies to body 1
   dVector3 t1; // torque that joint applies to body 1
   dVector3 f2; // force that joint applies to body 2
   dVector3 t2; // torque that joint applies to body 2
   } dJointFeedback;
 */

// void dJointDestroy (dJointID);

// groups seem to be optional : use in JointCreate to ignore JointGroup behaviour
// dJointGroupID dJointGroupCreate (int max_size);
// dJointGroupID dJointGroupCreate (0); // max_size unused
// void dJointGroupDestroy (dJointGroupID);


/*

   dReal Gain = 0.1;
   dReal MaxForce = 100;

   dReal TruePosition = dJointGetHingeAngle(joint);
   dReal DesiredPosition = // whatever;
   dReal Error = TruePosition - DesiredPosition;

   dReal DesiredVelocity = -Error * Gain;

   dJointSetHingeParam(joint[a], dParamFMax, MaxForce);
   dJointSetHingeParam(joint[a], dParamVel, DesiredVelocity);

 */

/* SIMLOID AX-12 code : (units may be different!)

   //AX-12 - Lengths are in Metres, Weights in Kg : Are these SI units? - they appear to be...
 #define LEN_AX12_X     0.050
 #define LEN_AX12_Y     0.035
 #define LEN_AX12_Z     0.030
 #define WEIGHT_DYNAMIXEL 0.060	//Gewicht eines AX-12

   // The units of TORQUE for this one #define don't seem to agree ...
   // - should be * by Kg->Newton factor... to get 1.11 (which agrees approximately with Common::axtotorque
 #define AX12_FMAX      0.1132			//Maximale Kraft des AX-12 - Maximum Power (really TORQUE) of an AX-12

   // This seems to be the resting friction
 #define JOINT_CONST_FRICTION 0.0203 //0.0139 //0.0207          //innere Jointreibung (BIOLOID laufen: 0.7)

   // This friction (times the current angular velocity) is a force applying against the torque of the motor
   // dJointSetAMotorParam(joint[1], dParamFMax, fabs(dJointGetHingeAngleRate(joint[0]))*JOINT_RATE_FRICTION)
 #define JOINT_RATE_FRICTION 0.045 //0.07	//Faktor der Zunahme innerer Reibung bei Bewegung (Daempfung)

   // This is the torque applied by the motor itself
   //	activeTorque = Common::axtotorque(100);
   // Add the torque to the motor's joint
   // dJointAddAMotorTorques(joint[1], activeTorque, 0.0, 0.0);

   // Initial guess ?
 #define AXTOTORQUE(x) (AX12_FMAX * (double)x / 1023.0)

   // Experimental controller
   inline double Common::axtotorque(const double i)
   {
   //	 Berechung des Drehmoments (Kraft) aus dem 0...1023 Wert (Input)
   //	   Idee: Drehmoment haengt sinusfoermig von Input ab => M = a * sin(b*x)
   //	   Problem: bei kleinen Inputs ist Drehmoment doch groesser als Sinusfunktion hergibt,
   //	   daher fuer den Input Bereich 0...~550 ein linear abfallender Bonus drauf (experimentell ermittelt
   //	   mit dem Meissel-Heben Test (verschiedene Torques, Endwinkel gemessen))

   //	Calculation of torque (power) from the value 0 .. 1023 (input)
   // Solution: torque depends on input roughly like the sinusoid approximation : M * = a sin (b * x)
   //	Problem: For small inputs, torque is larger than sine function being made,
   //          therefore for the input range 0 .. ~ 550 a linear descending bonus is applied
   //	         (experimental determined with the Meissel-lifting test (various Torques, end flange measured))

   const double a = 1.2; //0.1085;
    const double b = HPI/1080.0;  // pi/2 /1080
    if (i < 0.0) {
        return -(a * sin( b*(double)(-i) ) - 0.0015 + ((-i) < 550 ? -0.0000257143 * (double)(-i) + 0.014: 0.0));
    }
    else {
        return a * sin( b*(double)i ) - 0.0015 + (i < 550 ? -0.0000257143 * (double)i + 0.014: 0.0);
    }
   }


 */



