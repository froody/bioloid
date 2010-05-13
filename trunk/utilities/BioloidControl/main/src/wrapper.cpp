/**************************************************************************

    Copyright 2007, 2008 Rainer J�kel <rainer.jaekel@googlemail.com>

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


#include <iostream>
#include <cstdlib>
#include <cstring>

#include "../include/wrapper.h"
#include "../include/robot.h"
#include "../include/constants.h"
#include "../include/platform.h"
#include "../include/vars.h"
#include "../include/crc.h"

#define WRITEBUFFERRETRYDELAY (2*global.motion.ipoPause)

#ifdef WIN32
 #include <windows.h>
 #define _SHOWREADTIME

#endif

using namespace std;

CController::CController() {
  esum = eold = Ki = Kp = Kd = 0.0;
  Ta = 1.0;
}

double CController::update(double error) {
  error = error;

  esum = (esum + error) * global.parameters[48];

  double y = Kp * error + Ki * Ta * esum + Kd * (error - eold) / Ta;
  eold = error;

  return y;
}

void CWrapperCM5::rechargeBatteries(unsigned int maxCycles) {
  char buffer[255];
  byte msg[6];
  unsigned long voltage, rate;
  bool isDone;

  CPlatform::TxD8(COMM);
  CPlatform::TxD8(COMM_RECHARGE);
  CPlatform::TxD8(maxCycles & 0xFF);
  CPlatform::TxD8( (maxCycles & 0xFF00) >> 8);

  isDone = false;
  rate = 0;
  while(!isDone) {
    if(CPlatform::RxD8Buffer(msg, 8) ) {
      voltage = msg[2]+ 256*msg[3];
      rate = msg[4]+ 256*msg[5];

      if(voltage > 1250) {
        sprintf(buffer, "Charging complete\n");
        isDone = true;
      }
      else {
        sprintf(buffer, "Current Battery ADC0: %dmV, Average: %d, Unchanged: %d (max: %d).\n", msg[0]+ 256*msg[1], voltage, rate, msg[6]+ 256*msg[7]);
      }

      CUtil::cout(buffer);
    }
    else {
      CUtil::cout("Couldn't read battery status.\n");
    }

    CPlatform::sleep(1000);
  }
}

// reads timer from robot (atmega128 tickcount)
unsigned long CWrapperCM5::readTimer() {
  byte buffer[4];
  CPlatform::TxD8(COMM);
  CPlatform::TxD8(COMM_TIME);
  CPlatform::RxD8Buffer(buffer, 4);

  return buffer[0] + (buffer[1] << 8) +
         (buffer[2] << 16) + (buffer[3] << 24);
}

// reads button state from robot
byte CWrapperCM5::readButtonState(byte id) {
  CPlatform::TxD8(COMM);
  CPlatform::TxD8(COMM_RBS);
  CPlatform::TxD8(id);
  return CPlatform::RxD8();
}

// reads led state from robot
byte CWrapperCM5::readLedState(byte id) {
  CPlatform::TxD8(COMM);
  CPlatform::TxD8(COMM_RLS);
  CPlatform::TxD8(id);
  return CPlatform::RxD8();
}

// writes led state to robot
void CWrapperCM5::writeLedState(byte id, byte state) {
  CPlatform::TxD8(COMM);
  CPlatform::TxD8(COMM_WLS);
  CPlatform::TxD8(id);
  CPlatform::TxD8(state);
}

void CWrapperCM5::sendControlData(byte id, byte*data, byte len) {
  CPlatform::TxD8(COMM);
  CPlatform::TxD8(COMM_PARAM);
  CPlatform::TxD8(id);
  CPlatform::TxD8Buffer(data, len);
}

// reads data of all ax12 from robot
bool CWrapperCM5::readFromAllAx(byte addr, void *data, int len) {
  byte buffer[READOFFSET + AX12_DATA_READ_TOTAL];

  byte send[2];
  send[0] = COMM;
  send[1] = COMM_RFAAX;

  CPlatform::TxD8Buffer(send, 2);

  bool read = CPlatform::RxD8Buffer( (byte*)buffer, READOFFSET + AX12_DATA_READ_TOTAL);
  if(!read) {
    CUtil::cout("Error: RxD8Buffer() failed.\n", TEXT_ERROR);
    return false;
  }

  unsigned int crcLow  = CPlatform::RxD8();
  unsigned int crcHigh = CPlatform::RxD8();

  // check last 2 bytes = crc
  // calculate crc and compare with received one
  word crc = CCrc::calc( (byte*)buffer, READOFFSET + AX12_DATA_READ_TOTAL);
  if( (crc & 0xFFFF) != (crcLow + 256*crcHigh) ) {
    CUtil::cout("Error: Wrong Checksum.\n", TEXT_ERROR);
    return false;
  }

  // uncompress byte stream
  bool compress = uncompressByteStream(buffer, READOFFSET + AX12_DATA_READ_TOTAL, (byte*)data, len);
  if(!compress) {
    CUtil::cout("Error: uncompressByteStream() failed.\n", TEXT_ERROR);
  }

  return compress;
}

// writes data of all ax12 to robot
bool CWrapperCM5::writeToAllAx(byte addr, void *buffer, int len) {
  byte data[AX12_DATA_WRITE_TOTAL + WRITEOFFSET];
  if(!compressByteStream( (byte*)buffer, len, data, AX12_DATA_WRITE_TOTAL) ) {
    CUtil::cout("Error: compressByteStream() failed.\n", TEXT_ERROR);
  }

  len = AX12_DATA_WRITE_TOTAL;
  CPlatform::TxD8(COMM);
  CPlatform::TxD8(COMM_WTAAX);

  if(!CPlatform::TxD8Buffer( (byte*)data, len) ) {
    CUtil::cout("Error: TxD8Buffer failed.\n", TEXT_ERROR);
    return CRobotWrapper::WRAPPER_ERROR_AX12_WRITE;
  }

  // add 2 byte crc
  word crc = CCrc::calc( (byte*)data, len);
  byte low = (crc & 0x00FF);
  byte high = (crc & 0xFF00) >> 8;
  CPlatform::TxD8(low);
  CPlatform::TxD8(high);

  // read return value
  byte result;
  byte status[50];

  if(!CPlatform::RxD8Buffer(&result, 1) ) {
    CUtil::cout("Error: RxD8Buffer() failed.\n", TEXT_ERROR);
    return CRobotWrapper::WRAPPER_ERROR_AX12_WRITE;
  }

  if( (result == COMM_WTAAX) && CPlatform::RxD8Buffer(status, 7) ) {
    char str[255];
    sprintf(str, "Average: %d Read: %d Pause: %d Writebuffer: %d\n", status[0] + (status[1] << 8), status[2] + (status[3] << 8), status[4] + (status[5] << 8), (unsigned int)status[6]);
    CUtil::cout(str, TEXT_DEBUG);

    if( (unsigned int)status[6] >= 11) {
      CUtil::cout("Error: Write Buffer is full.\n", TEXT_ERROR);
      return CRobotWrapper::WRAPPER_ERROR_AX12_WRITEBUFFER;
    }

    return CRobotWrapper::WRAPPER_ERROR_NONE;
  }

  CUtil::cout("Error: RxD8Buffer() failed.\n", TEXT_ERROR);
  return CRobotWrapper::WRAPPER_ERROR_AX12_WRITE;
}

// reads 'len' bytes at address 'addr' from servo 'id'
bool CWrapperCM5::readFromAx(byte id, byte addr, void *buffer, byte len) {
  CPlatform::TxD8(COMM);
  CPlatform::TxD8(COMM_RFAX);
  CPlatform::TxD8(id);
  CPlatform::TxD8(addr);
  CPlatform::TxD8(len);
  if(!CPlatform::RxD8Buffer( (byte*)buffer, len) ) {
    return false;
  }

  // check last 2 bytes = crc
  // calculate crc and compare with received one
  unsigned int low  = CPlatform::RxD8();
  unsigned int high = CPlatform::RxD8();
  word crc = CCrc::calc( (byte*)buffer, len);
  return( (crc & 0xFFFF) == (low + 256*high) );
}

// writes 'len' bytes to address 'addr' of servo 'id'
bool CWrapperCM5::writeToAx(byte id, byte addr, void *data, byte len, byte schedule) {
  byte buffer[256];
  buffer[0] = id;
  buffer[1] = addr;
  buffer[2] = len;
  buffer[3] = schedule;
  CUtil::copymem(&buffer[4], data, len);

  // append crc to buffer
  word crc = CCrc::calc(&buffer[0], len + 4);
  buffer[4 + len + 0] = (crc & 0x00FF);
  buffer[4 + len + 1] = (crc & 0xFF00) >> 8;

  // send to robot
  CPlatform::TxD8(COMM);
  CPlatform::TxD8(COMM_WTAX);
  if(!CPlatform::TxD8Buffer( (byte*)buffer, 4 + len + 2) ) {
    return false;
  }

  // get return value
  byte b;
  if(!CPlatform::RxD8Buffer(&b, 1) ) {
    return false;
  }
  else {
    return(b == COMM_WTAX);
  }
}

// runs scheduled write operations
void CWrapperCM5::runScheduledAx(void) {
  CPlatform::TxD8(COMM); CPlatform::TxD8(COMM_RSAX);
}

// reads charactes from console
int CWrapperCM5::readFromConsole(byte*buffer) {
  char tmp;
  cin.getline( (char*)buffer, 255);
  buffer[254] = 0;
  return strlen( (const char*)buffer);
}

// writes charactes to console
void CWrapperCM5::writeToConsole(byte*buffer, int len) {
  byte c = buffer[len-1];
  buffer[len-1] = 0;
  cout << buffer;
  buffer[len-1] = c;
}

/* Button Class */
::CButton::CButton() {
  this->pressed = BUTTON_NOT_PRESSED;
}

byte CButton::isPressed() {
  return this->pressed;
}

bool CButton::readFromRobot() {
  this->pressed = global.wrapper->readButtonState(this->getId() );
  // fix this
  return true;
}

/* LED Class */
::CLed::CLed() {
  this->on = LED_OFF;
}

byte CLed::isOn() {
  return this->on;
}

void CLed::set(byte on) {
  this->on = on;
}

bool CLed::readFromRobot() {
  this->on = global.wrapper->readLedState(this->getId() );
  // fix this
  return true;
}

bool CLed::writeToRobot() {
  global.wrapper->writeLedState(this->getId(), this->on);
  // fix this
  return true;
}

/* Base Class, shared by all Classes */
CIdentifiable::CIdentifiable() {
  this->id = 0;
}

byte CIdentifiable::getId() {
  return this->id;
}

void CIdentifiable::setId(byte id) {
  this->id = id;
}

/* Ax Base Class, shared by AX-12 and AX-S1 */
CAx::CAx() {
  setId(0);
}

// writes parameter value to module
bool CAx::setParameter(byte addr, byte*data, byte len, byte schedule) {
  return global.wrapper->writeToAx(this->id, addr, data, len, schedule);
}

// reads parameter value from module
bool CAx::getParameter(byte addr, byte*buffer, byte len) {
  return global.wrapper->readFromAx(this->id, addr, buffer, len);
}

/* Servo Class (AX-12) */
CAx12::CAx12() {
  for(int i=0; i<AX12_DATA_READ; i++) {
    data[i] = 0;
  }

  angleMax = ANGLE_MAX;
  angleMin = ANGLE_MIN;
  angleSign = 1.0;

  setTargetAngle(0);
  setSpeedLimit(0);
  setTorqueLimit(0);

  checkRange  = true;
  useCurrentAngle = true;
  angleError  = false;

  // angle history, cyclic buffer
  for(int i=0; i<OLDANGLESCOUNT; i++) {
    oldAngles[i] = 0.0;
  }

  oldAnglesIndex = 0;

  // store current time
  timestamp = CPlatform::getTickCount();
}

bool CAx12::rangeError() {
  return this->angleError;
}

// reads goal position, target speed, target torque,
// current position, current speed and current torque from robot
bool CAx12::readFromRobot() {
  return getParameter( (byte)P_GOAL_POSITION_L, &data[0], AX12_DATA_READ);
}

// writes goal position, target speed, target torque,
// current position, current speed and current torque to robot
bool CAx12::writeToRobot(byte schedule) {
  return setParameter( (byte)P_GOAL_POSITION_L, (byte*)&data[0], AX12_DATA_WRITE, schedule);
}

// sets upper and lower bound of robot angle
void CAx12::setTargetAngleMinMax(float min, float max) {
  angleMin = (min < ANGLE_MIN) ? ANGLE_MIN : min;
  angleMax = (max > ANGLE_MAX) ? ANGLE_MAX : max;
}

// sets target angle
//void  ::CAx12::setTargetAngle(float angle_in_degree) {         // puzzled - mdda
void CAx12::setTargetAngle(float angle_in_degree) {
  // check if target angle falls within angleMin and angleMax
  angle_in_degree *= angleSign;

  bool bColl = false;
  if(checkRange) {
    if( (angle_in_degree > angleMax) || (angle_in_degree < angleMin) ) {
      if(fabs(angleErrorTarget - angle_in_degree) > AX12_ANGLE_DISTANCE) {
#ifdef SHOWTARGETANGLEERROR
        char str[255];
        sprintf(str, "%d: Angle out of range (%g)\n", (int)id, angle_in_degree);
        CUtil::cout(str);
#endif
        angleErrorTarget = angle_in_degree;
      }
      bColl = true;
      angle_in_degree = (fabs(angle_in_degree - angleMax) < fabs(angle_in_degree - angleMin) ) ?
                        angleMax : angleMin;
    }
  }

  // !useCurrentAngle -> angleError = target angle out of range
  // (angleError -> mark servos red in 3d visualization)
  if(!useCurrentAngle) {
    angleError = bColl;
  }

  // update storage buffer
  CUtil::angleToWord(angle_in_degree, data[AX12_DATA_OFFSET+0], data[AX12_DATA_OFFSET+1]);
}

// sets current angle
void ::CAx12::setCurrentAngle(float angle_in_degree) {
  bool bColl = false;

  angle_in_degree *= angleSign;

  // check if target angle falls within angleMin and angleMax
  if(checkRange) {
    if( (angle_in_degree > angleMax - AX12_ANGLE_DISTANCE) ||
        (angle_in_degree < angleMin + AX12_ANGLE_DISTANCE) ) {
      if(fabs(angleErrorCurrent - angle_in_degree) > AX12_ANGLE_DISTANCE) {
#ifdef SHOWCURRENTANGLEERROR
        char str[255];
        sprintf(str, "%d: Angle out of range (%g)\n", (int)id, angle_in_degree);
        CUtil::cout(str);
#endif
        angleErrorCurrent = angle_in_degree;
      }
      bColl = true;
      angle_in_degree = (fabs(angle_in_degree - angleMax) < fabs(angle_in_degree - angleMin) ) ?
                        angleMax : angleMin;
    }
  }

  // useCurrentAngle -> angleError = current angle out of range
  // (angleError -> mark servos red in 3d visualization)
  if(useCurrentAngle) {
    angleError = bColl;
  }

  // store current angle in history
  if(oldAnglesIndex >= OLDANGLESCOUNT-1) {
    oldAnglesIndex = 0;
  }
  else {
    oldAnglesIndex++;
  }

  oldAngles[oldAnglesIndex] = angle_in_degree;

  // last angle update occured at timestamp
  timestamp = CPlatform::getTickCount();

  // update storage buffer
  CUtil::angleToWord(angle_in_degree, data[AX12_DATA_OFFSET+6], data[AX12_DATA_OFFSET+7]);
}

// returns target angle
float CAx12::getTargetAngle() {
  float tmp = CUtil::wordToAngle(data[AX12_DATA_OFFSET+0], data[AX12_DATA_OFFSET+1]);
  tmp *= angleSign;
  return tmp;
}

// returns current angle
float CAx12::getCurrentAngle() {
  float tmp = CUtil::wordToAngle(data[AX12_DATA_OFFSET+6], data[AX12_DATA_OFFSET+7]);
  tmp *= angleSign;
  return tmp;
}

// holds position (target = current position)
void CAx12::setTargetToCurrentAngle() {
  setTargetAngle(getCurrentAngle() );
}

// sets speed limit
void CAx12::setSpeedLimit(unsigned int limit) {
  if(limit > SPEED_MAX) {
    limit = SPEED_MAX;
  }

  data[AX12_DATA_OFFSET+2] = (byte)(limit & 0x00FF);
  data[AX12_DATA_OFFSET+3] = (byte)(limit >> 8);
}

// returns speed limit
unsigned int CAx12::getSpeedLimit() {
  return(data[AX12_DATA_OFFSET+2]+256*data[AX12_DATA_OFFSET+3]);
}

// returns current speed
//unsigned int  ::CAx12::getCurrentSpeed() {   // mdda - puzzled
unsigned int CAx12::getCurrentSpeed() {
  return(data[AX12_DATA_OFFSET+8]+256*data[AX12_DATA_OFFSET+9]);
}

void CAx12::controlTorque(float targetPosition, float time, unsigned int torqueMax) {
  float diff = (targetPosition - getCurrentAngle() );
  unsigned int torque =  (unsigned int)(torquePID.offset + fabs(torquePID.update(diff) ) );
  setTorqueLimit(torque);
}

// check if going to position targetPosition within time milliseconds will result
// in a velocity > speedMax -> limit it
// DOESNT WORK PROPERLY
// TODO: include angle history of servo to estimate speed
void CAx12::controlSpeed(float targetPosition, float time, int speedMax) {
  // position difference
  float diff = (targetPosition - getCurrentAngle() );

  unsigned int speed = (unsigned int)(speedPID.offset + fabs(speedPID.update(diff) ) ); // cast by mdda
  setSpeedLimit(speed);
}

// sets torque limit
void CAx12::setTorqueLimit(unsigned int limit)
{
  if(limit > TORQUE_MAX) {
    limit = TORQUE_MAX;
  }

  data[AX12_DATA_OFFSET+4] = (byte)(limit & 0x00FF);
  data[AX12_DATA_OFFSET+5] = (byte)(limit >> 8);
}

// returns torque limit
unsigned int CAx12::getTorqueLimit() {
  return(data[AX12_DATA_OFFSET+4]+256*data[AX12_DATA_OFFSET+5]);
}

// returns current torque
int CAx12::getCurrentTorque() {
  word tor = (data[AX12_DATA_OFFSET+10]+256*data[AX12_DATA_OFFSET+11]);

  // absolute value = lower 9 bits
  int tmp = tor & 0x1FF;

  // sign = 10th bit
  if( (tor & 0x200) == 0x200) {
    tmp = -tmp;
  }

  return tmp;
}

/* Sensor Class (AX-S1) */
::CAxs1::CAxs1()
{
  int n = AXS1_DATA_READ;
  while(--n >= 0) {
    data[n] = 0;
  }
}

void CAxs1::playSound(byte*sounds, int len, byte duration) {
  for(int i=0; i<len; i++) {
    setBuzzerScale(sounds[i]);
    setBuzzerDuration(duration);
    writeToRobot();
    CPlatform::sleep(duration * 100);
  }
}

// reads sensor data from robot
bool CAxs1::readFromRobot() {
  return getParameter(P_AXS1_DIST_STD_VAL, data, AXS1_DATA_READ);
}

// writes sensor data to robot
bool CAxs1::writeToRobot(byte schedule)
{
  bool result1 = setParameter(P_AXS1_DIST_STD_VAL, data, AXS1_DATA_WRITE_1, schedule);
  bool result2 = setParameter(P_AXS1_MAX_SOUND_VOL, &data[P_AXS1_MAX_SOUND_VOL-P_AXS1_DIST_STD_VAL], AXS1_DATA_WRITE_2, schedule);

  return result1 && result2;
}

/* write/read functions */
void CAxs1::setDistCmpVal(byte value)
{
  data[AXS1_DATA_OFFSET+0] = value;
}

byte CAxs1::getDistCmpVal()
{
  return data[AXS1_DATA_OFFSET+0];
}

void CAxs1::setLightCmpVal(byte value)
{
  data[AXS1_DATA_OFFSET+1] = value;
}

byte CAxs1::getLightCmpVal()
{
  return data[AXS1_DATA_OFFSET+1];
}

void CAxs1::setSoundCmpVal(byte value)
{
  data[AXS1_DATA_OFFSET+2] = value;
}

byte CAxs1::getSoundCmpVal()
{
  return data[AXS1_DATA_OFFSET+2];
}

void CAxs1::setLed(byte on)
{
  data[AXS1_DATA_OFFSET+5] = (on > 0) ? 1 : 0;
}

byte CAxs1::getLed()
{
  return data[AXS1_DATA_OFFSET+5];
}

byte CAxs1::getDistLeft()
{
  return data[AXS1_DATA_OFFSET+6];
}

byte CAxs1::getDistCenter()
{
  return data[AXS1_DATA_OFFSET+7];
}

byte CAxs1::getDistRight()
{
  return data[AXS1_DATA_OFFSET+8];
}

byte CAxs1::getLightLeft()
{
  return data[AXS1_DATA_OFFSET+9];
}

byte CAxs1::getLightCenter()
{
  return data[AXS1_DATA_OFFSET+10];
}

byte CAxs1::getLightRight()
{
  return data[AXS1_DATA_OFFSET+11];
}

byte CAxs1::isDistLeft()
{
  return (data[AXS1_DATA_OFFSET+12] && 0x01) == 0x01;
}

byte CAxs1::isDistCenter()
{
  return (data[AXS1_DATA_OFFSET+12] && 0x02) == 0x02;
}

byte CAxs1::isDistRight()
{
  return (data[AXS1_DATA_OFFSET+12] && 0x04) == 0x04;
}

byte CAxs1::isLightLeft()
{
  return (data[AXS1_DATA_OFFSET+13] && 0x01) == 0x01;
}

byte CAxs1::isLightCenter()
{
  return (data[AXS1_DATA_OFFSET+13] && 0x02) == 0x02;
}

byte CAxs1::isLightRight()
{
  return (data[AXS1_DATA_OFFSET+13] && 0x04) == 0x04;
}

byte CAxs1::getSoundVolume()
{
  return data[AXS1_DATA_OFFSET+15];
}

byte CAxs1::getMaxSoundVolume()
{
  return data[AXS1_DATA_OFFSET+16];
}

void CAxs1::setMaxSoundVolume(byte max)
{
  data[AXS1_DATA_OFFSET+16] = max;
}

byte CAxs1::getSoundsCounter()
{
  return data[AXS1_DATA_OFFSET+17];
}

void CAxs1::setSoundsCounter(byte count)
{
  data[AXS1_DATA_OFFSET+17] = count;
}

word CAxs1::getSoundTimestamp()
{
  return data[AXS1_DATA_OFFSET+18] + (data[AXS1_DATA_OFFSET+19] * 256); // low + high * 256
}

void CAxs1::setSoundTimestamp(word time)
{
  data[AXS1_DATA_OFFSET+18] = (byte)(time && 0x00FF);
  data[AXS1_DATA_OFFSET+19] = (byte)( (time >> 8) && 0x00FF);
}

byte CAxs1::getBuzzerScale()
{
  return data[AXS1_DATA_OFFSET+20];
}

void CAxs1::setBuzzerScale(byte scale)
{
  data[AXS1_DATA_OFFSET+20] = scale;
}

byte CAxs1::getBuzzerDuration()
{
  return data[AXS1_DATA_OFFSET+21];
}

void CAxs1::setBuzzerDuration(byte time)
{
  data[AXS1_DATA_OFFSET+21] = time;
}

/* Hardware global.wrapper Class, contains one object / module */
CRobotWrapper::CRobotWrapper()
{
  int i;

  for(i=0; i<BUTTON_COUNT; i++) {
    Buttons[i].setId(i+1);
  }

  for(i=0; i<LED_COUNT; i++) {
    Leds[i].setId(i+1);
  }

  for(i=0; i<AX12_COUNT; i++) {
    Ax12s[i].setId(i+1);
  }

  for(i=0; i<AXS1_COUNT; i++) {
    Axs1s[i].setId(100 + i);
  }
}

/* reads data from all servos at once:
   data is bitwise stored in a buffer (since most data is only 10 bits wide)
   and secured by a 2 byte crc */
bool CRobotWrapper::readAx12s()
{
  byte buffer[AX12_COUNT * AX12_DATA_WRITE];
  int i, j;

  bool result, error = false;

  // read data from robot
  error = !global.wrapper->readFromAllAx( (byte)P_PRESENT_POSITION_L, buffer, AX12_COUNT * AX12_DATA_WRITE);

  if(error) {
    CUtil::cout("Error: readFromAllAx() failed.\n");
  }

  if(error) {

    //CPlatform::clearLine();
    CPlatform::sleep(10);

    // read data from robot
    error = !global.wrapper->readFromAllAx( (byte)P_PRESENT_POSITION_L, buffer, AX12_COUNT * AX12_DATA_WRITE);

    if(error) {
      CUtil::cout("Error: readFromAllAx() failed #2.\n");
    }
  }

  if(error) {
    ; // do nothing
  }
  else {
    unsigned int byteCounter = 0;
    for(i=0; i<AX12_COUNT; i++) {
      float angle = CUtil::wordToAngle(buffer[byteCounter + 0], buffer[byteCounter + 1]);
      Ax12s[i].setCurrentAngle(Ax12s[i].angleSign*angle);

      Ax12s[i].data[AX12_DATA_READ/2 + 2] = buffer[byteCounter + 2];
      Ax12s[i].data[AX12_DATA_READ/2 + 3] = buffer[byteCounter + 3];

      Ax12s[i].data[AX12_DATA_READ/2 + 4] = buffer[byteCounter + 4];
      Ax12s[i].data[AX12_DATA_READ/2 + 5] = buffer[byteCounter + 5];
      byteCounter += 6;
    }
  }


  return !error;
}

/* writes data to all servos at once:
   data is bitwise stored in a buffer (since most data is only 10 bits wide)
   and secured by a 2 byte crc */
bool CRobotWrapper::writeAx12s() {
//    CUtil::cout("Writing to AX12s !", TEXT_DEBUG);
  if(!global.writeToRobot) {
    return true;
  }
  /*
  for(unsigned int i=0; i<AX12_COUNT; i++) 
    // if (fabsf(Ax12s[i].getCurrentAngle() - Ax12s[i].getTargetAngle()) > 0.3)
    if ( !Ax12s[i].writeToRobot() )
      return false;

  return true;
  */
  byte bufferUncompressed[AX12_COUNT*AX12_DATA_WRITE];
  for(unsigned int i=0; i<AX12_COUNT; i++) {
    for(unsigned int j=0; j<AX12_DATA_WRITE; j++) {
      bufferUncompressed[i*AX12_DATA_WRITE + j] = Ax12s[i].data[j];
    }
  }
  // send data to robot
  int result = global.wrapper->writeToAllAx( (byte)AX12_DATA_ADDRESS,
                                             bufferUncompressed,
                                             AX12_COUNT*AX12_DATA_WRITE);

  switch(result) {
  case WRAPPER_ERROR_AX12_WRITE:
    CUtil::cout("Error: writeToAllAx() failed.\n", TEXT_ERROR);
    break;

  case WRAPPER_ERROR_AX12_WRITEBUFFER:
    char str[255];
    sprintf(str, "Error: writeToAllAx() failed. Retrying in %d milliseconds.\n", WRITEBUFFERRETRYDELAY);
    CUtil::cout(str, TEXT_DEBUG);

    //CPlatform::clearLine();
    CPlatform::sleep(WRITEBUFFERRETRYDELAY);
    result = global.wrapper->writeToAllAx( (byte)AX12_DATA_ADDRESS, bufferUncompressed, AX12_COUNT*AX12_DATA_WRITE);
    break;
  }
  return(result == WRAPPER_ERROR_NONE);
}

// reads data from sensor modules
void CRobotWrapper::readAxs1s() {
  for(int i=0; i<AXS1_COUNT; i++) {
    Axs1s[i].readFromRobot();
  }
}

// writes data to sensor modules
void CRobotWrapper::writeAxs1s() {
  for(int i=0; i<AXS1_COUNT; i++) {
    Axs1s[i].writeToRobot();
  }

  //global.wrapper->runScheduledAx();
}

// reads button data
void CRobotWrapper::readButtons() {
  for(int i=0; i<BUTTON_COUNT; i++) {
    Buttons[i].readFromRobot();
  }
}

// reads led data
void CRobotWrapper::readLeds() {
  for(int i=0; i<LED_COUNT; i++) {
    Leds[i].readFromRobot();
  }
}

// writes led data
void CRobotWrapper::writeLeds() {
  for(int i=0; i<LED_COUNT; i++) {
    Leds[i].writeToRobot();
  }
}

// reads time from robot (atmega128 tickcount)
unsigned long CRobotWrapper::getTime() {
  return global.wrapper->readTimer();
}

void CRobotWrapper::holdPosition() {
  ML(MUTEXCOMM);
  for(int i=0; i<AX12_COUNT; i++) {
    Ax12s[i].setTargetToCurrentAngle();
  }

  writeAx12s();
  MU(MUTEXCOMM);
}

void CRobotWrapper::enableTorque(bool enable) {
  byte on = enable ? 1 : 0;
  byte buffer[2];
  unsigned int t;

  int i;
  ML(MUTEXCOMM);
  for(i=0; i<AX12_COUNT; i++) {
    Ax12s[i].setParameter(P_TORQUE_ENABLE, &on, 1);

    // hmm, safety first ;)
    CPlatform::sleep(10);
  }

  MU(MUTEXCOMM);

  if(enable) {
    CUtil::cout("Torque enabled.\n");
  }
  else {
    CUtil::cout("Torque disabled.\n");
  }

  CPlatform::sleep(100);
}

void CRobotWrapper::loadAx12FromXml(TiXmlElement*ax12Node) {
  // set servo controller parameters
#define AX12LOADDATA 11
  byte buffer[AX12LOADDATA * AX12_COUNT];

  int i, j;

  // init
  for(i=0; i<AX12_COUNT; i++) {
    buffer[i*AX12LOADDATA + 0] = 1;
    buffer[i*AX12LOADDATA + 1] = 1;
    buffer[i*AX12LOADDATA + 2] = 32;
    buffer[i*AX12LOADDATA + 3] = 32;
    // punch = minimum current supplied to servo
    int punch = 32;
    buffer[i*AX12LOADDATA + 4] = punch & 0xFF;
    buffer[i*AX12LOADDATA + 5] = (punch & 0xFF00) >> 8;
    // only turn servo off when overheating
    buffer[i*AX12LOADDATA + 6] = 4;

    buffer[i*AX12LOADDATA + 7] = INITSPEED & 0xFF;
    buffer[i*AX12LOADDATA + 8] = (INITSPEED & 0xFF00) >> 8;
    buffer[i*AX12LOADDATA + 9] = INITTORQUE & 0xFF;
    buffer[i*AX12LOADDATA +10] = (INITTORQUE & 0xFF00) >> 8;
  }

  TiXmlNode*valueNode;
  TiXmlText*valueElement;

  TiXmlElement*node = ax12Node->FirstChildElement();
  while(node != NULL) {
    char *ids = CConfiguration::getAttributeString(node, "id", (char *)"0");
    char *pch = strtok(ids, " ");

    valueNode = node->FirstChild();
    valueElement = valueNode->ToText();
    char *value = (char*)valueElement->Value();

    while(pch != NULL) {
      int id = (int)atof(pch);
      id--;

      if( (id >=0) && (id < AX12_COUNT) ) {
        if(strcasecmp(node->Value(), "cwmargin") == 0) {
          buffer[id*AX12LOADDATA + 0] = ( (unsigned int)atof(value) & 0xFF);
        }
        else if(strcasecmp(node->Value(), "ccwmargin") == 0) {
          buffer[id*AX12LOADDATA + 1] = ( (unsigned int)atof(value) & 0xFF);
        }
        else if(strcasecmp(node->Value(), "cwslope") == 0) {
          buffer[id*AX12LOADDATA + 2] = ( (unsigned int)atof(value) & 0xFF);
        }
        else if(strcasecmp(node->Value(), "ccwslope") == 0) {
          buffer[id*AX12LOADDATA + 3] = ( (unsigned int)atof(value) & 0xFF);
        }
        else if(strcasecmp(node->Value(), "punch") == 0) {
          unsigned int punch = (unsigned int)atof(value);
          buffer[id*AX12LOADDATA + 4] = punch & 0xFF;
          buffer[id*AX12LOADDATA + 5] = (punch & 0xFF00) >> 8;
        }
        else if(strcasecmp(node->Value(), "status") == 0) {
          buffer[id*AX12LOADDATA + 6] = ( (unsigned int)atof(value) & 0xFF);
        }
        else if(strcasecmp(node->Value(), "speed") == 0) {
          unsigned int speed = (unsigned int)atof(value);
          buffer[id*AX12LOADDATA + 7] = speed & 0xFF;
          buffer[id*AX12LOADDATA + 8] = (speed & 0xFF00) >> 8;
        }
        else if(strcasecmp(node->Value(), "torque") == 0) {
          unsigned int torque = (unsigned int)atof(value);
          buffer[id*AX12LOADDATA + 9] = torque & 0xFF;
          buffer[id*AX12LOADDATA +10] = (torque & 0xFF00) >> 8;
        }
      }

      pch = strtok(NULL, " ");
    }

    node = node->NextSiblingElement();
  }

  for(i=0; i<AX12_COUNT; i++) {
    bool found;
    Ax12s[i].setParameter(P_CW_COMPLIANCE_MARGIN, &buffer[i*AX12LOADDATA + 0], 4);
    CPlatform::sleep(10);
    Ax12s[i].setParameter(P_PUNCH_L, &buffer[i*AX12LOADDATA + 4], 2);
    CPlatform::sleep(10);
    Ax12s[i].setParameter(P_ALARM_SHUTDOWN, &buffer[i*AX12LOADDATA + 6], 1);
    CPlatform::sleep(10);
    Ax12s[i].setParameter(P_GOAL_SPEED_L, &buffer[i*AX12LOADDATA + 7], 2);
    Ax12s[i].setSpeedLimit(buffer[i*AX12LOADDATA + 7] + 256 * buffer[i*AX12LOADDATA + 8]);
    CPlatform::sleep(10);
    found = Ax12s[i].setParameter(P_TORQUE_LIMIT_L, &buffer[i*AX12LOADDATA + 9], 2);
    Ax12s[i].setTorqueLimit(buffer[i*AX12LOADDATA + 9] + 256 * buffer[i*AX12LOADDATA + 10]);
    CPlatform::sleep(10);
  }

  for(i=0; i<AXS1_COUNT; i++) {
    bool found = Axs1s[i].readFromRobot();
  }
}

void CRobotWrapper::loadPidFromXml(TiXmlElement*controllerNode) {
  TiXmlElement*node = controllerNode->FirstChildElement();
  while(node != NULL) {
    char *ids = CConfiguration::getAttributeString(node, "id", (char *)"0");
    char *pch = strtok(ids, " ");

    while(pch != NULL) {
      int id = (int)atof(pch);
      id--;

      if( (id >=0) && (id < AX12_COUNT) ) {
        float time = CConfiguration::getAttributeFloat(node, "time", 1.0);
        float p = CConfiguration::getAttributeFloat(node, "p", 10.0);
        float i = CConfiguration::getAttributeFloat(node, "i", 0.0);
        float d = CConfiguration::getAttributeFloat(node, "d", 0.0);
        float offset = CConfiguration::getAttributeFloat(node, "offset", 10.0);

        printf("Set PID %d to P: %g I: %g D: %g T: %g O: %g\n", id+1, p, i, d, time, offset);

        if(strcasecmp(node->Value(), "speed") == 0) {
          Ax12s[id].speedPID.Ta = time;
          Ax12s[id].speedPID.Kd = d;
          Ax12s[id].speedPID.Ki = i;
          Ax12s[id].speedPID.Kp = p;
          Ax12s[id].speedPID.offset = offset;
        }
        else {
          Ax12s[id].torquePID.Ta = time;
          Ax12s[id].torquePID.Kd = d;
          Ax12s[id].torquePID.Ki = i;
          Ax12s[id].torquePID.Kp = p;
          Ax12s[id].torquePID.offset = offset;
        }
      }

      pch = strtok(NULL, " ");
    }

    node = node->NextSiblingElement();
  }
}

bool CWrapperCM5::uncompressByteStream(byte *bufferIn, unsigned int sizeIn, byte *bufferOut, unsigned int sizeOut)
{
  int bit, start, end, value;
  float angle;
  byte low, high;

  unsigned int currentOut = 0;
  for(unsigned int i=0; i<sizeOut/2; i++) {
    // calculate bit number
    bit = i * 10;

    // calc start and end byte
    start = bit / 8;
    end = (bit + 9) / 8;

    // byte [i][i+1]
    // bits  3   7    offset 3
    int tmp;
    byte mask;
    byte offset;
    offset = (bit % 8);
    mask = 0x00FF >> offset;
    offset = 8 - offset;

    tmp = (bufferIn[READOFFSET + start]) & mask;
    value = (tmp & 0xFF) << (10 - offset);

    offset = (8 - (10 - offset) );
    mask = 0x00FF << offset;
    tmp = (bufferIn[READOFFSET + start + 1]) & mask;
    value += (tmp & 0xFF) >> (offset);


    // 10 bits are in wrong order 11111111 11 = low high
    tmp = (value & 0x3);
    high = (byte)tmp;
    tmp = (value - tmp) >> 2;
    low = (byte)tmp;

    bufferOut[currentOut] = low;
    currentOut++;
    bufferOut[currentOut] = high;
    currentOut++;
  }

  return true;
}

bool CWrapperCM5::compressByteStream(byte *bufferIn, unsigned int sizeIn, byte *bufferOut, unsigned int sizeOut)
{
  if(sizeOut < AX12_DATA_WRITE_TOTAL + WRITEOFFSET) {
    return false;
  }

  unsigned int i, j;

  // initialize with 0 -> |= works properly
  for(i=0; i<AX12_DATA_WRITE_TOTAL + WRITEOFFSET; i++) {
    bufferOut[i] = 0;
  }

  int k = 0;
  i = 0;
  j = 0;
  int bc = 0;

  // store data bitwise in buffer
  int bit, start, end, value, tmp;
  float angle;
  byte low, high;

  for(i=0; i<sizeIn/2; i++) {
    // calculate bit number
    bit = i * 10;

    // calc start and end byte
    start = bit / 8;
    end = (bit + 9) / 8;

    low = bufferIn[2*i];
    high = bufferIn[2*i + 1];

    // 10 bits are in wrong order value = 11111111 11 = low high
    tmp = (int)low;
    tmp = tmp << 2;
    value = tmp + (high & 0x3);


    byte mask;
    byte offset;
    offset = (bit % 8);
    mask = 0x00FF >> offset;
    offset = 8 - offset;

    tmp = (value >> (10 - offset) ) & mask;
    bufferOut[start] |= tmp;

    offset = (8 - (10 - offset) );
    mask = 0x00FF << offset;
    tmp = (value << offset) & mask;

    bufferOut[start + 1] |= tmp;
  }
  return true;
}

bool CWrapperDynamixelBus::readFromAx(byte id, byte addr, void *buffer, byte len)
{
  byte gbpTxBuffer[128];
  byte gbpRxBuffer[128];
  byte gbpParameter[32];

  gbpParameter[0] = addr; //Address
  gbpParameter[1] = len;  //Read Length

  CPlatform::TxPacket(id, INST_READ, 2, gbpParameter, gbpTxBuffer);       // send packet
  // read answer
  if(CPlatform::RxPacket(gbpRxBuffer, DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1], gbpTxBuffer) == DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1]) {
    for(unsigned int i=0; i<len; i++) {
      ( (byte*)buffer)[i] = gbpRxBuffer[5 + i];
    }
    return true;
  }
  else {
    return false;
  }
}

bool CWrapperDynamixelBus::writeToAx(byte id, byte addr, void *data, byte len, byte schedule)
{
  byte gbpTxBuffer[128];
  byte gbpRxBuffer[128];
  byte gbpParameter[32];

  byte op;
  if(schedule) {
    op = INST_REG_WRITE;
  }
  else {
    op = INST_WRITE;
  }

  gbpParameter[0] = addr; //Address
  for(unsigned i=0; i<len; i++) {
    gbpParameter[1 + i] = ( (byte*)data)[i];
  }

  CPlatform::TxPacket(id, op, len+1, gbpParameter, gbpTxBuffer);      // send packet
  // read answer
  if(CPlatform::RxPacket(gbpRxBuffer, DEFAULT_RETURN_PACKET_SIZE, gbpTxBuffer) == DEFAULT_RETURN_PACKET_SIZE) {
    return true;
  }
  else {
    return false;
  }
}

bool CWrapperDynamixelBus::readFromAllAx(byte addr, void *buffer, int len)
{
  byte gbpTxBuffer[128];
  byte gbpRxBuffer[128];
  byte gbpParameter[32];
  unsigned int id, i, j;
  unsigned int size = len / AX12_COUNT;
  // command: read AX12_DATA_READ bytes from address P_PRESENT_POSITION_L /ax12
  gbpParameter[0] = addr; //Address
  gbpParameter[1] = size;  //Read Length

  // send command to every ax12 and store result in readBuffer[]
  // failure = 1 signals a read failure
  bool failure = false;
  for(id=0; id<AX12_COUNT; id++) {
    if (global.ax12ToKinematicChain[id] >= 0)
    failure = true;
    i = 0;
    // on failure: retry AX12_RETRY times
    while( (failure) && (i < 5) )
    {
      int tmp = CPlatform::TxPacket(id+1, INST_READ, 2, gbpParameter, gbpTxBuffer);

      tmp = CPlatform::RxPacket(gbpRxBuffer, DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1], gbpTxBuffer);

      if (gbpRxBuffer[4] != 0)
      {
        char text[256];
        sprintf(text, "Status: %d %d\n", id+1, gbpRxBuffer[4]);
        CUtil::cout(text, TEXT_DEBUG);
      }
      if(tmp == DEFAULT_RETURN_PACKET_SIZE + gbpParameter[1]) {
        for(j=0; j<size; j++) {
          ( (byte*)buffer)[id*size + j] = gbpRxBuffer[5 + j];
        }
        
        
        failure = false;
      }
      else { i++;}
    }

    if(failure) {
      break;
    }
  }

  return !failure;
}

bool CWrapperDynamixelBus::writeToAllAx(byte addr, void *data, int len)
{
//    byte gbpTxBuffer[128]; // Nearly too small
    byte gbpRxBuffer[128]; // No need for this one here
//    byte gbpParameter[32]; // This is MUCH TOO SMALL - mdda : 18*(6+1) > 32

  byte gbpTxBuffer[256];   // generous space - mdda
  byte gbpParameter[256];  // generous space - mdda

  // conversion variables
  unsigned int size = len / AX12_COUNT;

  /* gbpParameter[0] =  P_GOAL_POSITION_L; //Address

  for(unsigned int id=0; id<AX12_COUNT; id++) {
  for(unsigned i=0; i<size; i++) {
    gbpParameter[1 + i] = ( (byte*)data)[id*size + i];
  }

  CPlatform::TxPacket(id+1, INST_WRITE, size+1, gbpParameter, gbpTxBuffer);      // send packet
  // read answer
  // if(CPlatform::RxPacket(gbpRxBuffer, DEFAULT_RETURN_PACKET_SIZE, gbpTxBuffer) != DEFAULT_RETURN_PACKET_SIZE) {
    ;// return false;
    //}
  }
  return true;
  */
  // sync write AX12_DATA_WRITE bytes starting at address P_GOAL_POSITION_L
  byte op = INST_SYNC_WRITE;
  gbpParameter[0] = P_GOAL_POSITION_L; //Address
  gbpParameter[1] = 2;

//    CUtil::cout("Preparing SYNC_WRITE buffer\n");  // mdda

  // prepare write buffer -> unpack data from serial and store bytewise in buffer
  for(unsigned int i=0; i<AX12_COUNT; i++) {
    gbpParameter[2 + i * (2+1)] = i + 1; // Id of this AX12

    for(unsigned int j=0; j<2; j++) {
      gbpParameter[2+1 + i * (2+1) + j] = ( (byte*)data)[i*size + j];
    }
  }

//				CUtil::cout("About to write to Comms\n");  // mdda

  // send all data to ax12s AT ONCE
  CPlatform::TxPacket(BROADCASTING_ID, op, 2+AX12_COUNT*(2 + 1), gbpParameter, gbpTxBuffer);

//				CUtil::cout("Done writing to Comms\n");  // mdda

  // broadcast, no return packet
}

void CWrapperDynamixelBus::runScheduledAx(void)
{
  byte gbpTxBuffer[128];
  byte gbpParameter[32];
  CPlatform::TxPacket(BROADCASTING_ID, INST_ACTION, 0, gbpParameter, gbpTxBuffer);
}

