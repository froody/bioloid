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

#include <sstream>
#include <iomanip>

#include <exception>
using namespace std;

#include "bioloid.h"

BDevice::BDevice(byte _id) {
  id=_id;
  reset_base_device();
}

BDevice::~BDevice() {
}

void BDevice::reset_base_device() {
  memory[P_ID] = id;
  memory[P_BAUD_RATE] = 1;

  memory[P_RETURN_DELAY_TIME] = 250;  // Measured in units of 2 microsecs. i.e. default = .5 ms
  memory[P_RETURN_LEVEL] = 2; // 0=Never respond, 1=Respond to READ only, 2=always respond

  // This is a bit field :
  //   64=on Instruction error
  //   32=on Overload error
  //   16=on Checksum error
  //    8=on Range error
  //    4=on Overheating error
  //    2=on Angle limit error
  //    1=on Input Voltage error
  memory[P_ALARM_LED]      = 4;
  memory[P_ALARM_SHUTDOWN] = 4;
// memory[P_OPERATING_MODE] = 0;

//	queued_messages.clear();  // Created in a clear()ed state by definition
  memory[P_REGISTERED_INSTRUCTION] = 0;

}

bool BDevice::handle_message(BMessage &m, BBus *bus) {
  byte status=m.status;
  BMessage reply(id, status);

  // Decide whether to send a status packet back...
  bool send_status=(memory[P_RETURN_LEVEL]==2); // true if 'always respond'
  if( (memory[P_RETURN_LEVEL]==1) && (m.instruction==INST_READ) ) {
    send_status=true;
  }
  if( (m.id == BROADCASTING_ID) || m.inhibit_status_return) {
    send_status=false;
  }

  if(status == 0) { // Only do this if the current status is ZERO
    switch(m.instruction) {
    case INST_PING: // Don't 'do' anything - status gets sent back below
      break;

    case INST_READ:
      if(m.param.size()!=2) {
        status |= STATUS_ERROR_INSTRUCTION;
      }
      else {
        int location=m.param[0];
        int length=m.param[1];
        about_to_read_memory(location, location+length-1);
        for(int i=0; i<length; i++) {
          reply.param.push_back(memory[location & 0xFF]);  // Sanitized location
          location++;
        }
      }
      break;

    case INST_WRITE:
      if(m.param.size()<2) {
        status |= STATUS_ERROR_INSTRUCTION;
        cerr << "Wrong # of parameters for INST_WRITE" << endl;
      }
      else {
        int location=m.param[0];
        for(int i=1; i<m.param.size(); i++) {
          memory[location & 0xFF]=m.param[i]; // Sanitized location
          location++;
        }
        just_written_memory(m.param[0], location-1);
      }
      break;

    case INST_REG_WRITE: // Store this as a simple write instruction in the Instruction Queue...
      m.instruction=INST_WRITE;
      cout << "Queuing REG_WRITE for servo " << output_hex(id) << endl;
      if(queued_messages.empty() ) { // For now, only one message allowed in queue
        queued_messages.push(m);
      }
      // Alternative : Queue this in  memory[P_REGISTERED_INSTRUCTION]
      break;

    case INST_ACTION: // Fire off instructions in the Queue...
      while(!queued_messages.empty() ) {
        cout << "Got an ACTION for servo " << output_hex(id) << endl;
        BMessage queued_message=queued_messages.front();
        handle_message(queued_message, bus);
        queued_messages.pop();
      }
      break;

    case INST_RESET:
      reset();  // Delegate action to the sub-class
      break;

    default:
      cerr << "Unknown instruction : " << output_hex(m.instruction) << endl;
      status |= STATUS_ERROR_INSTRUCTION;
    }
  }
  if(send_status) {
    reply.instruction=status; // In case it has changed

    // This allows for having a spread of delays from different devices - for multiple reply READs, for instance
    BBusTime t=bus->time_plus(bus->current_bus_time(), ( (float)memory[P_RETURN_DELAY_TIME]) * 0.002);
    bus->send_queue.add(reply, t);
  }
  return true; // sucess
}

#define GET_MEMORY_LH(i) (memory[i]+memory[i+1]*256)
#define SET_MEMORY_LH(i, v) memory[i]=v & 0xFF; memory[i+1]=(v>>8) & 0xFF

BDeviceAX12::BDeviceAX12(byte _id, Servo *_servo) : BDevice(_id) {
  servo=_servo;
  reset();
}

#include "servo.h"
void BDeviceAX12::reset() {
  //
  //  This is the ROM memory area - unfortunately, it's volatile in this simulation
  //

  memory[P_MODEL_NUMBER_L] = 12;
  memory[P_MODEL_NUMBER_H] = 0;
//	memory[] = ;
  memory[P_CW_ANGLE_LIMIT_L] = 0;
  memory[P_CW_ANGLE_LIMIT_H] = 0;
  memory[P_CCW_ANGLE_LIMIT_L] = 0xFF;
  memory[P_CCW_ANGLE_LIMIT_H] = 0x03;

  memory[P_LIMIT_TEMPERATURE] = 85; // This is 85 degrees C
  memory[P_DOWN_LIMIT_VOLTAGE]= 60; // This is  6.0V
  memory[P_UP_LIMIT_VOLTAGE] = 190; // This is 19.0 V

  memory[P_MAX_TORQUE_L] = 0xFF;
  memory[P_MAX_TORQUE_H] = 0x03;

  //
  //  This is the RAM memory area
  //

  memory[P_TORQUE_ENABLE] = 0; // zero for 'free-wheel'

  memory[P_CW_COMPLIANCE_SLOPE] = 32;
  memory[P_CW_COMPLIANCE_MARGIN] = 0;
  memory[P_CCW_COMPLIANCE_MARGIN] = 0;
  memory[P_CCW_COMPLIANCE_SLOPE] = 32;

  // Goal position set to current position when initialized
  if(1) {
    int v=servo->getCurrentPosition()/BIOLOID_VALUE_TO_RADIANS + 511;
    SET_MEMORY_LH(P_PRESENT_POSITION_L, v);  // write in current
    SET_MEMORY_LH(P_GOAL_POSITION_L, v);     // Set goal==current
  }

  memory[P_GOAL_SPEED_L] = 0;
  memory[P_GOAL_SPEED_H] = 0;

  memory[P_TORQUE_LIMIT_L] = memory[P_MAX_TORQUE_L];
  memory[P_TORQUE_LIMIT_H] = memory[P_MAX_TORQUE_H];

  memory[P_MOVING] = 0;
  memory[P_LOCK] = 0; // Not used here

  memory[P_PUNCH_L]=32;
  memory[P_PUNCH_H]=0;
}

void BDeviceAX12::about_to_read_memory(int addr_lo, int addr_hi) {
  if( (addr_lo<=P_PRESENT_POSITION_L) && (addr_hi>=P_PRESENT_POSITION_H) ) {
    int v=servo->getCurrentPosition()/RADIANS_TO_BIOLOID_VALUE + 512;
    SET_MEMORY_LH(P_PRESENT_POSITION_L, v);
  }
  if( (addr_lo<=P_PRESENT_SPEED_L) && (addr_hi>=P_PRESENT_SPEED_H) ) {
    int v=servo->getCurrentVelocity()*1023.0; // This is a signed number
    int sign=0;
    if(v<-1023) { v=-1023; }
    if(v>+1023) { v=+1023; }
    if(v<0)     { v=-v; sign=1; }
    SET_MEMORY_LH(P_PRESENT_SPEED_L, (sign<<10)+(v & 0x3FF) );
  }
  if( (addr_lo<=P_MOVING) && (addr_hi>=P_MOVING) ) {
    memory[P_MOVING]=(servo->isMovingUnderPower() ) ? 1 : 0;
  }
  if( (addr_lo<=P_PRESENT_LOAD_L) && (addr_hi>=P_PRESENT_LOAD_H) ) {
    int v=servo->getCurrentTorque()*1023.0; // This is a signed number
    int sign=0;
    if(v<-1023) { v=-1023; }
    if(v>+1023) { v=+1023; }
    if(v<0)     { v=-v; sign=1; }
    SET_MEMORY_LH(P_PRESENT_LOAD_L, (sign<<10)+(v & 0x3FF) );
  }

  // No need to getCompliance, since the servo only gets the information from here anyway
  // More 'getParameter'

  // TODO :

}

void BDeviceAX12::just_written_memory(int addr_lo, int addr_hi) {
  if( (addr_lo<=P_GOAL_POSITION_L) && (addr_hi>=P_GOAL_POSITION_H) ) {
    servo->setTargetPosition( (float)(GET_MEMORY_LH(P_GOAL_POSITION_L)-511) * BIOLOID_VALUE_TO_RADIANS);
  }
  if( (addr_lo<=P_GOAL_SPEED_L) && (addr_hi>=P_GOAL_SPEED_H) ) {
    float speed=(float)(GET_MEMORY_LH(P_GOAL_SPEED_L) )/1024.0;
    if(speed==0.0) { speed=1.0; }
    servo->setMovingSpeed(speed);
  }
  if( (addr_lo<=P_TORQUE_LIMIT_L) && (addr_hi>=P_TORQUE_LIMIT_H) ) {
    servo->setMotorCurrentMax( (float)(GET_MEMORY_LH(P_TORQUE_LIMIT_L) )/1024.0);
  }
  if( ( (addr_lo<=P_PUNCH_L) && (addr_hi>=P_PUNCH_H) )
      || ( (addr_lo<=P_CW_COMPLIANCE_MARGIN) && (addr_hi>=P_CCW_COMPLIANCE_SLOPE) ) // Funky logic - quicker than individual checks
      ) {
    servo->setCompliance(
      (float)(memory[P_CCW_COMPLIANCE_SLOPE]) * BIOLOID_VALUE_TO_RADIANS,
      (float)(memory[P_CCW_COMPLIANCE_MARGIN]) * BIOLOID_VALUE_TO_RADIANS,
      (float)(GET_MEMORY_LH(P_PUNCH_L) )/1024.0,
      (float)(memory[P_CW_COMPLIANCE_MARGIN]) * BIOLOID_VALUE_TO_RADIANS,
      (float)(memory[P_CW_COMPLIANCE_SLOPE]) * BIOLOID_VALUE_TO_RADIANS
      );
  }

  // More 'setParameter'

  // TODO : LEDs ?

}

BDeviceIMUideal::BDeviceIMUideal(byte _id, Sensor *_sensor) : BDevice(_id) {
  sensor=_sensor;
  reset();
}

#include "sensor.h"
void BDeviceIMUideal::reset() {
}

// The standard HVU IMU has a range of approximately +/- 3 g
#define GRAVITY_TO_BIOLOID_VALUE (3.0*9.81/512.0)

void BDeviceIMUideal::about_to_read_memory(int addr_lo, int addr_hi) {
  if( (addr_lo<=P_IMUHUV_VERTICAL_ACCEL_H) && (addr_hi>=P_IMUHUV_SIDEWAYS_ACCEL_L) ) { // If anything in this block is requested...
    float accel[3];
    sensor->getCurrentAccel(accel); // Copy data into vector in some units...
    for(int i=0; i<3; i++) {
      int v=accel[i]/GRAVITY_TO_BIOLOID_VALUE + 512;
      if(v>1023) { v=1023; }
      if(v<0)    { v=0; }
      SET_MEMORY_LH(P_IMUHUV_SIDEWAYS_ACCEL_L+(i*2), v); // Nasty address maths...
    }
  }
  if( (addr_lo<=P_IMUHUV_YAW_RATE_H) && (addr_hi>=P_IMUHUV_PITCH_RATE_L) ) { // If anything in this block is requested...
    float angular_rate[3];
    sensor->getCurrentAngularRate(angular_rate); // Copy data into vector in radians/sec
    for(int i=0; i<3; i++) {
      int v=angular_rate[i]/RADIANS_TO_BIOLOID_VALUE + 512;
      if(v>1023) { v=1023; }
      if(v<0)    { v=0; }
      SET_MEMORY_LH(P_IMUHUV_PITCH_RATE_L+(i*2), v); // Nasty address maths...
    }
  }
}

void BDeviceIMUideal::just_written_memory(int addr_lo, int addr_hi) {
  // Control of the LED ?
}