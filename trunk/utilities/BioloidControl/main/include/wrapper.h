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


#ifndef __WRAPPER

#define __WRAPPER

#define INITSPEED 1023 // default max speed value
#define INITTORQUE 1023 // default max torque value

// IMPORTANT!
#define AX12_COUNT 18 // number of ax12s connected to cm-5
#define AXS1_COUNT 0 // number of axs1s connected to cm-5
// #define AXS1_COUNT 0 // number of axs1s connected to cm-5 - no need to force this (mdda)

#define WRITEOFFSET 0 // start byte in buffer of CAx12 when writing
#define READOFFSET 0 // start byte in buffer of CAx12 when reading
#define AX12_DATA_READ 12 // number of bytes to read from ax12 in a single read message
// number of bytes to read from ax12 in a combined read message
#define AX12_DATA_READ_TOTAL (AX12_COUNT % 4 == 0 ? (3 * 10 * AX12_COUNT) / 8 : (3 * 10 * AX12_COUNT) / 8 + 1) // 3 = torque speed position, 10 = 10bit each, 8 = 8 Bit, 1 Byte
#define AX12_DATA_WRITE 6 // see AX12_DATA_READ
#define AX12_DATA_WRITE_TOTAL AX12_DATA_READ_TOTAL
#define AX12_DATA_OFFSET 0 // obsolete: global offset in data buffer of CAx12
#define AX12_DATA_ADDRESS P_GOAL_POSITION_L // address to read data from in a single read message
#define AX12_ANGLE_DISTANCE 1.0 // only display out-of-range error message if difference between last error angle and current error angle is larger than AX12_ANGLE_DISTANCE

#define AX12_INVALID 1e30; // invalid angle value
#define OLDANGLESCOUNT 10 // number of angles stored in angle history of each CAx12

#define TORQUE_MAX 1023 // max torque value
#define SPEED_MAX 1023 // max speed value
#define ANGLE_MAX 150.0 // max angle in degree
#define ANGLE_MIN -150.0 // min angle in degree

#define LED_COUNT 0 // number of leds

#define AXS1_DATA_OFFSET 0
#define AXS1_DATA_READ 19 // number of bytes to read from axs1
#define AXS1_DATA_WRITE_1 4
#define AXS1_DATA_WRITE_2 6

#define RECHARGEMAX 30 // 15 minutes = 30 * 30seconds = 30 cycles

#include "types.h"
#include "robot.h"

class CController
{
public:
  double esum, eold, Ki, Kp, Ta, Kd, offset;
  CController();

  double update(double error);

};

/*! \brief Communication Wrapper Base Class

   Provides functions to read data from or write data to the robot.
 */
class CWrapper
{
public:
  /*! \brief Returns the timer value of the uC
      TODO: implement on robot
   */
  virtual unsigned long readTimer() {};

  /*! \brief Recharges batteries of bioloid and displays current voltage
   */
  virtual void rechargeBatteries(unsigned int maxCycles = RECHARGEMAX) {};

  /*! \brief Sends Control/Configuration Data to the robot
   */
  virtual void sendControlData(byte id, byte*data, byte len) {};

  /*! \brief Returns the state of button \a id on the CM-5
      TODO: implement on robot
   */
  virtual byte readButtonState(byte id) { return 0; };

  /*! \brief Returns the state of led \a id on the CM-5
      TODO: implement on robot
   */
  virtual byte readLedState(byte id) { return 0; };

  /*! \brief Sets the state of led \a id on the CM-5 to \a state
      TODO: implement on robot
   */
  virtual void writeLedState(byte id, byte state) {};

  /*! \brief Reads \a len bytes from servo/sensor \a id starting at address \a addr to \a buffer
   */
  virtual bool readFromAx(byte id, byte addr, void *buffer, byte len) { return false; };

  /*! \brief Writes \a len bytes from \a data to servo/sensor \a id starting at address \a addr to \a buffer

      \param schedule If set to 1, the \a data won't be processed by the servos/sensors until \a runScheduledAx is called (use this to send data synchronously to multiple servos/sensors).
   */
  virtual bool writeToAx(byte id, byte addr, void *data, byte len, byte schedule) { return false; };

  /*! \brief Reads data from all servos at once (with CRC)

      The current angle, torque and speed values from all servos will be read.
      The data is sent as a \a bit string, use \a CRobotWrapper::readAx12s to automatically
      decode it.

      \param buffer Has to be of size READOFFSET + AX12_DATA_READ_TOTAL
      \param len Has to be READOFFSET + AX12_DATA_READ_TOTAL
   */
  virtual bool readFromAllAx(byte addr, void *buffer, int len) { return false; };

  /*! \brief Writes data from all servos at once (with CRC)

      The target angle, torque and speed values of all servos will be overwritten.
      The data is sent as a \a bit string, use \a CRobotWrapper::writeAx12s to automatically
      encode it.

      \param data Has to be of size WRITEOFFSET + AX12_DATA_WRITE_TOTAL
      \param len Has to be WRITEOFFSET + AX12_DATA_WRITE_TOTAL
   */
  virtual bool writeToAllAx(byte addr, void *data, int len) { return false; };

  /*! \brief Sends the schedule command to all servos

      If data has been sent to multiple servos/sensors via \a writeToAx with
      the schedule parameter set to 1, the servos/sensor will take on the new values
      nearly at the same time if this command is sent.
   */
  virtual void runScheduledAx(void) {};

  /*! \brief Returns user input (on the console) on the pc
      TODO: implement on the pc
   */
  virtual int  readFromConsole(byte*buffer) { return 0; };

  /*! \brief Prints data to the console on the pc
      TODO: implement on the pc
   */
  virtual void writeToConsole(byte*buffer, int len) {};
};

/*! \brief Robot Communication Wrapper (Cm-5)

   Implements the communication protocol.
   Provides functions to read data from or write data to the cm-5.
 */
class CWrapperCM5 : public CWrapper
{
public:
  /*! \brief Robot Communication Protocol
   */
  enum COMMUNICATION_COMMANDS
  {
    COMM_NONE,
    COMM_RFC,   //read from console
    COMM_WTC,   //write to console
    COMM_RFAX,  //read from ax
    COMM_WTAX,  //write to ax
    COMM_TIME,  //get time
    COMM_RBS,   //read button state
    COMM_RLS,   //read led state
    COMM_WLS,   //write led state
    COMM_RSAX,  //run scheduled ax
    COMM_RFAAX, //read from ALL ax
    COMM_WTAAX, //write to  ALL ax
    COMM_PARAM, //set controller parameters
    COMM_RECHARGE, // recharge batteries
    COMM = 255
  };

  bool compressByteStream(byte *bufferIn, unsigned int sizeIn, byte *bufferOut, unsigned int sizeOut);
  bool uncompressByteStream(byte *bufferIn, unsigned int sizeIn, byte *bufferOut, unsigned int sizeOut);

  /*! \brief Returns the timer value of the uC
     TODO: implement on robot
   */
  unsigned long readTimer();

  /*! \brief Recharges batteries of bioloid and displays current voltage
   */
  void rechargeBatteries(unsigned int maxCycles = RECHARGEMAX);

  /*! \brief Sends Control/Configuration Data to the robot
   */
  void sendControlData(byte id, byte*data, byte len);

  /*! \brief Returns the state of button \a id on the CM-5
     TODO: implement on robot
   */
  byte readButtonState(byte id);

  /*! \brief Returns the state of led \a id on the CM-5
     TODO: implement on robot
   */
  byte readLedState(byte id);

  /*! \brief Sets the state of led \a id on the CM-5 to \a state
     TODO: implement on robot
   */
  void writeLedState(byte id, byte state);

  /*! \brief Reads \a len bytes from servo/sensor \a id starting at address \a addr to \a buffer
   */
  bool readFromAx(byte id, byte addr, void *buffer, byte len);

  /*! \brief Writes \a len bytes from \a data to servo/sensor \a id starting at address \a addr to \a buffer

     \param schedule If set to 1, the \a data won't be processed by the servos/sensors until \a runScheduledAx is called (use this to send data synchronously to multiple servos/sensors).
   */
  bool writeToAx(byte id, byte addr, void *data, byte len, byte schedule);

  /*! \brief Reads data from all servos at once (with CRC)

     The current angle, torque and speed values from all servos will be read.
     The data is sent as a \a bit string, use \a CRobotWrapper::readAx12s to automatically
     decode it.

     \param buffer Has to be of size READOFFSET + AX12_DATA_READ_TOTAL
     \param len Has to be READOFFSET + AX12_DATA_READ_TOTAL
   */
  bool readFromAllAx(byte addr, void *buffer, int len);

  /*! \brief Writes data from all servos at once (with CRC)

     The target angle, torque and speed values of all servos will be overwritten.
     The data is sent as a \a bit string, use \a CRobotWrapper::writeAx12s to automatically
     encode it.

     \param data Has to be of size WRITEOFFSET + AX12_DATA_WRITE_TOTAL
     \param len Has to be WRITEOFFSET + AX12_DATA_WRITE_TOTAL
   */
  bool writeToAllAx(byte addr, void *data, int len);

  /*! \brief Sends the schedule command to all servos

     If data has been sent to multiple servos/sensors via \a writeToAx with
     the schedule parameter set to 1, the servos/sensor will take on the new values
     nearly at the same time if this command is sent.
   */
  void runScheduledAx(void);

  /*! \brief Returns user input (on the console) on the pc
     TODO: implement on the pc
   */
  int  readFromConsole(byte*buffer);

  /*! \brief Prints data to the console on the pc
     TODO: implement on the pc
   */
  void writeToConsole(byte*buffer, int len);

};

/*! \brief Robot Communication Wrapper (USB dongle)

   Implements the communication protocol.
   Provides functions to read data from or write data to the cm-5.
 */
class CWrapperDynamixelBus : public CWrapper
{
public:
  /*! \brief Reads \a len bytes from servo/sensor \a id starting at address \a addr to \a buffer
   */
  bool readFromAx(byte id, byte addr, void *buffer, byte len);

  /*! \brief Writes \a len bytes from \a data to servo/sensor \a id starting at address \a addr to \a buffer

      \param schedule If set to 1, the \a data won't be processed by the servos/sensors until \a runScheduledAx is called (use this to send data synchronously to multiple servos/sensors).
   */
  bool writeToAx(byte id, byte addr, void *data, byte len, byte schedule);

  /*! \brief Reads data from all servos at once (with CRC)

      The current angle, torque and speed values from all servos will be read.
      The data is sent as a \a bit string, use \a CRobotWrapper::readAx12s to automatically
      decode it.

      \param buffer Has to be of size READOFFSET + AX12_DATA_READ_TOTAL
      \param len Has to be READOFFSET + AX12_DATA_READ_TOTAL
   */
  bool readFromAllAx(byte addr, void *buffer, int len);

  /*! \brief Writes data from all servos at once (with CRC)

      The target angle, torque and speed values of all servos will be overwritten.
      The data is sent as a \a bit string, use \a CRobotWrapper::writeAx12s to automatically
      encode it.

      \param data Has to be of size WRITEOFFSET + AX12_DATA_WRITE_TOTAL
      \param len Has to be WRITEOFFSET + AX12_DATA_WRITE_TOTAL
   */
  bool writeToAllAx(byte addr, void *data, int len);

  /*! \brief Sends the schedule command to all servos

      If data has been sent to multiple servos/sensors via \a writeToAx with
      the schedule parameter set to 1, the servos/sensor will take on the new values
      nearly at the same time if this command is sent.
   */
  void runScheduledAx(void);

  /*! \brief Returns user input (on the console) on the pc
      TODO: implement on the pc
   */
};



/*! \brief Base class for every identifiable robot part (buttons, servos, sensors, leds)
 */
class CIdentifiable
{
protected:
  byte id;

public:
  CIdentifiable();

  byte getId();
  void setId(byte id);

};

/*! \brief Button (Hardware abstraction)
 */
class CButton : public CIdentifiable
{
private:
  byte pressed;

public:
  /*! \brief Button states
   */
  enum BUTTON_STATES
  {
    BUTTON_PRESSED,
    BUTTON_NOT_PRESSED
  };
  CButton();

  /*! \brief Returns button state

     \see BUTTON_STATES
   */
  byte isPressed();

  /*! \brief Reads current state from robot
   */
  bool readFromRobot();

};

/*! \brief Ax module (Sensor or Servo, Hardware Abstraction)
 */
class CAx : public CIdentifiable
{
public:
  CAx();

  /*! \brief Sends data to the module

     Sends \a len bytes from \a data to the module starting at address \a addr.
     \a schedule indicates that the data should be sent to the robot but
     won't be activated until the \a CWrapper::runScheduledAx function is called.
   */
  bool setParameter(byte addr, byte*data, byte len, byte schedule = 0);

  /*! \brief Retrieves data from the module

     Reads \a len bytes from the module into \a buffer starting at address \a addr.
   */
  bool getParameter(byte addr, byte*buffer, byte len);

};

/*! \brief Ax12 module (Servo motor, Hardware Abstraction)

   After calling \a readFromRobot, the current state of the servo module is accessible through
   the get-functions. The current values can be changed via the set-functions and can be written to
   the robot by calling \a writeToRobot.
 */
class CAx12 : public CAx
{
private:
  float angleMin; ///< Lower bound for valid angles (in degree)
  float angleMax; ///< Upper bound for valid angles (in degree)
  float angleErrorCurrent; ///<
  float angleErrorTarget; ///< Target angle (in degree) that produces the last out-of-bounds error
  bool angleError; ///< Indicates if an out-of-bounds error occured

  unsigned long timestamp; ///< Timestamp of last angle update
  float oldAngles[OLDANGLESCOUNT]; ///< Cyclic buffer of past target angles
  int oldAnglesIndex; ///< Current index in cyclic buffer

public:
  CController torquePID, speedPID;

  float angleSign; ///< Should be +1.0 (clockwise, standard value) or -1.0 (counterclockwise)

  byte data[AX12_DATA_READ]; ///< Stores servo motor information (Current and target angle, torque, speed)
  bool checkRange; ///< Indicates if out-of-bound error should be produced
  bool useCurrentAngle; ///< Indicates if current or target angle should be used to test out-of-bound errors

  CAx12();

  /*! \brief Reads servo motor information from robot into \a data
   */
  bool    readFromRobot();

  /*! \brief Writes servo motor information from \a data to robot
   */
  bool    writeToRobot(byte schedule = 0);

  /*! \brief Indicates if an out-of-bounds error occurred.
   */
  bool    rangeError();

  void    setTargetAngle(float angle_in_degree);
  void    setTargetAngleMinMax(float min, float max);
  float   getTargetAngle();

  float   getCurrentAngle();
  void    setCurrentAngle(float angle_in_degree);
  void    setTargetToCurrentAngle();

  void            setSpeedLimit(unsigned int limit);
  unsigned int    getSpeedLimit();
  unsigned int    getCurrentSpeed();
  void            controlSpeed(float targetPosition, float time, int speedMax = 1023);

  void            setTorqueLimit(unsigned int limit);
  unsigned int    getTorqueLimit();
  int             getCurrentTorque();
  void            controlTorque(float targetPosition, float time, unsigned int torqueMax = 1023);

};

/*! \brief Led (Hardware abstraction)
 */
class CLed : public CIdentifiable
{
private:
  byte on;

public:
  /*! \brief Led states
   */
  enum LED_STATES
  {
    LED_OFF,
    LED_ON
  };

  CLed();

  /*! \brief Indicates if Led is active

     \see LED_STATES
   */
  byte isOn();

  /*! \brief Sets Led state

     \see LED_STATES
   */
  void set(byte on);

  /*! \brief Reads Led state from robot
   */
  bool readFromRobot();

  /*! \brief Writes Led state to robot
   */
  bool writeToRobot();

};

/*! \brief AxS1 module (Sensor, Hardware Abstraction)

   After calling \a readFromRobot, the current state of the sensor module is accessible through
   the get-functions. The current values can be changed via the set-functions and can be written to
   the robot by calling \a writeToRobot.
 */
class CAxs1 : public CAx
{
private:
  byte data[19]; ///< sensor module information

public:
  CAxs1();

  /*! \brief Reads sensor module from robot into \a data
   */
  bool readFromRobot();

  /*! \brief Writes sensor module information from \a data to robot
   */
  bool writeToRobot(byte schedule = 0);

  void setDistCmpVal(byte value);
  byte getDistCmpVal();

  void setLightCmpVal(byte value);
  byte getLightCmpVal();

  void setSoundCmpVal(byte value);
  byte getSoundCmpVal();

  void setLed(byte on);
  byte getLed();

  byte getDistLeft();
  byte getDistCenter();
  byte getDistRight();

  byte getLightLeft();
  byte getLightCenter();
  byte getLightRight();

  byte isDistLeft();
  byte isDistCenter();
  byte isDistRight();

  byte isLightLeft();
  byte isLightCenter();
  byte isLightRight();

  byte getSoundVolume();

  byte getMaxSoundVolume();
  void setMaxSoundVolume(byte max);

  byte getSoundsCounter();
  void setSoundsCounter(byte count);

  word getSoundTimestamp();
  void setSoundTimestamp(word time);

  byte getBuzzerScale();
  void setBuzzerScale(byte scale);

  byte getBuzzerDuration();
  void setBuzzerDuration(byte time);

  void playSound(byte*sounds, int len = 1, byte duration = 255);

};

/*! \brief Robot (Servos, Sensors, Leds, Buttons, Hardware Abstraction)

   The robot wrapper holds information about all servo motors, sensors, leds and buttons. The
   information can be updated by calling the read/write functions.
 */
class CRobotWrapper
{
public:
  /*! \brief Sensor modules (AxS1s)
   */
  enum AXS1S
  {
    AXS1_1 = 0
  };

  /*! \brief Buttons
   */
  enum BUTTON_IDS
  {
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_LEFT,
    BUTTON_RIGHT,
    BUTTON_START,
    BUTTON_MODE,
    BUTTON_COUNT
  };

  /*! \brief Servo motors (Ax12s)
   */
  enum AX12S
  {
    AX12_1 = 0,
    AX12_2,
    AX12_3,
    AX12_4,
    AX12_5,
    AX12_6,
    AX12_7,
    AX12_8,
    AX12_9,
    AX12_10,
    AX12_11,
    AX12_12,
    AX12_13,
    AX12_14,
    AX12_15,
    AX12_16,
    AX12_17,
    AX12_18
  };

  enum ERROR_CODES
  {
    WRAPPER_ERROR_NONE,
    WRAPPER_ERROR_AX12_WRITEBUFFER,
    WRAPPER_ERROR_AX12_WRITE
  };

  CButton Buttons[BUTTON_COUNT]; ///< Buttons
  CLed Leds[LED_COUNT]; ///< Leds
  CAx12 Ax12s[AX12_COUNT]; ///< Servo motors (Ax12s)
  CAxs1 Axs1s[AXS1_COUNT]; ///< Sensor modules (AxS1s)

  CRobotWrapper();

  void loadPidFromXml(TiXmlElement*controllerNode);
  void loadAx12FromXml(TiXmlElement*ax12Node);

  bool readAx12s(); ///< Reads all servo motor information from robot
  bool writeAx12s(); ///< Writes all servo motor information to robot
  void readAxs1s(); ///<  Reads all sensor module information from robot
  void writeAxs1s(); ///< Writes all sensor module information to robot
  void readButtons(); ///<  Reads all button information from robot
  void readLeds(); ///<  Reads all led information from robot
  void writeLeds(); ///< Writes all led information to robot
  unsigned long getTime(); ///< Reads current tickcount from uC


  /*! \brief Sets target angle of every servo motor to its current angle

     Reduces beeping of the servo motors
   */
  void holdPosition();

  /*! \brief Enables or disables torque of all servo motors

     Reduces the torque to a minimum so the user can move the robot links by hand.
   */
  void enableTorque(bool enable);

};

#endif
