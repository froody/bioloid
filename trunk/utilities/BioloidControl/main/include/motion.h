/**************************************************************************

    Copyright 2007, 2008 Rainer Jäkel <rainer.jaekel@googlemail.com>

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


#ifndef __MOTION

#define __MOTION

#include "wrapper.h"
#include "robot.h"
#include "util.h"
#include "list.h"

#define MOTION_COUNT 256 // max. motions in global storage buffer
#define MOTION_MAXUSER 99 // user can access motions 0 .. MOTION_MAXUSER
/*
MOTION_MAXUSER + 1 = Intermediate Motion
MOTION_MAXUSER + 2 = OmniWalk
*/
#define MOTION_MAXSYSTEM 128 
// use target position instead of current position as start position for robot movements
// if max distance between a current angle-target angle pair is less than MOTION_CURRENTANGLE_DIFF
#define MOTION_CURRENTANGLE_DIFF 4.0

#define MOTIONBUFFERMAX 1080000 // max length of keyframe buffer = 15min

#define INTERMEDIATE_IPOTYPE 0 // the transition-motion between two motions uses this interpolationtype
#define INTERMEDIATE_PAUSE 1000 // and this pause in milliseconds
#define INTERMEDIATE_MOTION  (MOTION_MAXUSER + 1) // and is stored at this index in the global motion storage
#define INTERMEDIATE_DIFFERENCE 2.0 // only calc transition-motion if difference between one angle-pair is larger than this

#define MOTION_SEQUENCE_MAX 32 // max. number of motions in a sequence

// Interpolation
#define LOOKAHEAD 5 // command lookahead
#define TRAJ_LOOKAHEAD 10 // trajectory lookahead
#define IPO_LOOKAHEAD 5 // interpolation (points) lookahead
#define COLLISION_LOOKAHEAD 5 // collision lookahead

#define MOTION_CAPTURE_PAUSE 80 // pause in milliseconds between two frames in the CMotionContainer::capture/CMotionContainer::play process
#define MOTION_IPO_PAUSE 80
#define MOTION_IPO_MINPAUSE 0 // min pause in milliseconds between two interpolation keyframes, see CMotionContainer::updateRobot
#define MOTION_TRAJ_PAUSE 100 // pause in milliseconds between the calcilation of two trajectory points, see CMotionContainer::updateRobot
#define MOTION_STATE_PAUSE 1000

// motion xml file config
#define XML_MOTION_ROOT "MotionSequence"
#define XML_MOTION_CHILD "Frame"
#define XML_MOTION_ATTR "pause"
#define XML_MOTION_ATTR_VALUE 100
#define XML_MOTION_ANGLE "AX"

// command defaults
#define MOTION_DEFAULT_NUM 1
#define MOTION_DEFAULT_PAUSE 200

/*! \brief Motion Key Frame

   Stores all joint angles and the cartesian pose of all limbs (redundant).
 */
class CMotionData
{
public:
  CMotionData();
  ~CMotionData();

  float angles[AX12_COUNT]; ///< Joint angles

  /*! \brief Array of TCP position & orientation of all limbs

     Will be allocated in constructor.
   */
  CMatrix  *pose;
  int pause; ///<
};

/*! \brief Interpolated joint angles

   Stores interpolated joint angles and the result of the (optional) collision check.
   <BR>
   TODO: Could be replaced by CMotionData.
 */
class CIpoData
{
public:
  /*! \brief Result of Collision Check
   */
  enum COLLISION_TYPES
  {
    COLLISION_UNKNOWN,
    COLLISION_YES,
    COLLISION_NO
  };

  CIpoData() : checkCollision(true), collision(0) {};

  float angles[AX12_COUNT]; ///< Joint angles
  bool checkCollision; ///< Perform collision check before sending data to robot
  byte collision; ///< Result of collision check
};

/*! \brief Sequence of Key Frames

   Stores motion data as a sequence of key frames (\a CMotionData). The time delay between
   two key frames is equal to the \a CMotionData::pause associated with the first key frame.
   The sequence can be stored in a xml file and loaded from it.
   Additionally motion data can be loaded from .mtn files created by the Motion Editor.
 */
class CMotionSequence
{
private:
  CList motionData; ///< Keyframe-Storage

public:
  CMotionSequence();

  int  count();
  void add(CMotionData *item);
  void clear();
  void remove();
  void saveToFile(char*filename);
  void loadFromFile(char*filename);
  void loadFromMotionEditorFile(char*filename, unsigned int firstId = 1, unsigned int lastId = 128);
  void loadFromMotionEditorFile(char*filename, unsigned int*ids, unsigned int length);
  CListIterator getIterator();
  CMotionData*operator [](int i);

};

/*! \brief Motion Sequence Container

   Combines several motion sequences into one contiguous motion.
   <BR>
   TODO: replace \a ids by array of CMotionSequence-pointers.
 */
class CMotion
{
public:
  int loop; ///< The whole motion will be played \a loop times.
  int count; ///< Number of motion sequences.
  int pause; ///< An interpolated point will be calculated every /a pause milliseconds (independet of time delay between motion keyframes)
  int ipo; ///< Interpolation type, \see IPO_TYPE
  int ids[MOTION_SEQUENCE_MAX]; ///< indexes of motion sequences in CMotionContainer::motions.
};

/*! \brief Cartesian Position and Orientation of one limb (Interpolation)

   The pose will be used as a stopover (for one limb) in the interpolation process, see \a CMotionContainer.
   <BR>
   TODO: rethink.
 */
class CPoint
{
public:
  int duration; ///< Pause (in milliseconds) between this and the next key frame
  int current; ///< Elapsed time (in milliseconds)
  bool checkCollision; ///< Run Collision Check
  CMatrix pose; ///< Pose of limb
  float angles[AX12_COUNT]; ///< Values of joint angles belonging to pose (only the angles of the limb are set)
};

/*! \brief Value of one joint angle (Interpolation)

   \see CPoint.
   <BR>
   TODO: rethink.
 */
class CAnglePoint
{
public:
  int duration;
  int current;
  bool checkCollision;
  double angle;
};

/*! \brief Motion Command, Interpolation and Storage Class (threaded)

   The class is used as a singleton and runs in its own thread (synchronization by state variables, see \a MOTION_STATES).<BR>
   Furthermore, the class splits up into three modules which each run in an own thread. <BR>
   The modules are:
    \li Command processing and trajectory generation, see \a calcTrajectory
    \li Interpolation of trajectory points, see \a interpolate
    \li Reading from the robot and writing interpolation data to the robot, see \a updateRobot

   Additionally the class provides a global storage for motion sequences (\a motions) and mechanisms to:
    \li capture robot movement (see \a capture, \a stopCapture, \a captureMotion)
    \li play motion sequences (see \a playSequence, \a stopSequence, \a joinSequence, \a addSequenceToTrajectory)
    \li process motion commands (see \a addCommand)
    \li process control commands, f.e. emergency stop (see \a addControlComman)
    \li control the robot read/write process (see \a enableIpo, \a getIpoState)
    \li move limbs (see \a moveLimb)
 */
class CMotionContainer
{
private:
  /*! \brief States of the motion thread
   */
  enum MOTION_STATES
  {
    MOTION_NONE,
    MOTION_PLAY,
    MOTION_STOP,
    MOTION_CAPTURE,
    MOTION_STOPCAPTURE
  };

  /*! \brief Handshake constants to control ipo thread
   */
  enum IPO_STATES
  {
    IPO_ON,
    IPO_OFF
  };

  int state; ///< Current state of the motion thread  \see MOTION_STATES
  int stateParam[5]; ///< Parameter values associated with current state
  int ipoState; ///< Current state of the read/write thread \see IPO_STATES
  int ipoReturn; ///< Handshake variable to suspend/resume read/write thread

  void setState(int newstate); ///< Set state of read/write thread \see IPO_STATES
  int  getState(); ///< Returns current state of read/write thread \see IPO_STATES

  /*! \brief Captures Robot movement and stores the result in the motion buffer (threaded)

     Will be called automatically after a call to \a capture.<BR>
     Reduces servo motor torque to a minimum so the user can move the limbs of the robot by hand.
     Depending on the \a type of the capture process the motion data will be captured
     every \a MOTION_IPO_PAUSE milliseconds or when the user presses the spacebar.

     \param id Index in the motion buffer where the captured movement is stored
     \param type Type of the capture process (contigous or discrete) \see CAPTURE_STATES
   */
  void captureMotion(int id, int type);

  /*! \brief Plays motion sequences (threaded)

     Will be called automatically after a call to \a playSequence.<BR>
     Stops the read/write thread and processes all motions in the \a motionList.
     Additional motions can be added during runtime (via playSequence).
     Interpolates between the key frames in the motion sequences and
     provides a smooth transition between the different motions.

     \see playSequence
   */
  void play();

public:
  enum CAPTURE_TYPES
  {
    CAPTURE_TIME,
    CAPTURE_KEY
  };

  /*! \brief Motion Command Types
   */
  enum MOTION_COMMAND_TYPES
  {
    COMMAND_NONE,
    COMMAND_WALK,
    COMMAND_STANDWALK,
    COMMAND_TURNRIGHT,
    COMMAND_TURNLEFT,
    COMMAND_STOP,
    COMMAND_OMNIWALK,
    COMMAND_COUNT
  };

  /*! \brief Motion Command Names (console input)
   */
  static const char*command_names[COMMAND_COUNT];

  /*! \brief Motion Command Types
   */
  enum MOTION_CONTROL_COMMAND_TYPES
  {
    CONTROL_COMMAND_STOP,
    CONTROL_COMMAND_COUNT
  };

  /*! \brief Motion Command Names (console input)
   */
  static const char*control_command_names[CONTROL_COMMAND_COUNT];

  static int ipoPause;
  static int capturePause;
  static int statePause;
  static int trajectoryPause;
  static int trajectoryLookAhead;
  static int interpolationLookAhead;
  static int collisionLookAhead;
  static int ipoMinPause;

  /*! \brief Motion buffer

     Can be filled by calls to \a playSequence, cleared by \a stop and run by \a play,
     which will be called automatically.
   */
  CList motionList;

  /*! \brief Motion buffer

     Stores motion sequences.
   */
  CMotionSequence motions[MOTION_COUNT];

  CList cmds; ///< Motion Command Buffer, will be executed one after another
  CList cmdControl; ///< Motion Control Command Buffer, will be executed one after another

  CList joints[AX12_COUNT]; ///< Target Angle Buffer for every joint angle
  CList *trajectory; ///< Target pose Buffer for every limb
  CList interpolated; ///< List of interpolated robot poses (CIpoData)

  /*! \brief Processes motion and control commands

     Depending on the processed command one or several limb trajectories (lists of CPoints)
     will be generated and stored in the corresponding \a trajectory list.
   */
  void calcTrajectory();

  /*! \brief Interpolates between the points in a trajectory or joint angles

     The function takes data from the following sources:
      \li joints, one list for every joint angle
      \li trajectory, one list for every limb

     First, data from the trajectory lists is taken and IPO_COUNT interpolated points are calculated
     (by interpolation in cartesian (plus inverse kinematics) or joint space).
     If not enough data is present, f.e. no motion for limb number 2 was specified, the actual
     (read from the robot) limb pose will be used.
     Afterwards, the \a non-empty joint angles lists are processed and the interpolated angles will
     be used instead of the ones retrieved through the trajectory lists.<BR>

     The result of the interpolation process are a number of objects of type CIpoData which will be added
     to the \a interpolated list.
   */
  void interpolate();

  /*! \brief Reads current servo and sensor data from the robot and sends new joint angles to the robot (threaded).

     If active (see \a enableIpo) every \a MOTION_IPO_PAUSE milliseconds the current robot pose and/or the sensor data
     will be read from the robot. Then the next object in the \a interpolated list will be processed (collision check, ...)
     and the interpolated angles will be send to the robot.<BR>
     If a collision occurs all robot movement will be stopped (optional, see \a) <BR>
     Additionally the current robot pose is visualized in the opengl-viewer and limb information is shown in the console.
   */
  void updateRobot();
  void stop(); ///< Stops the execution of a motion sequence \see play, playSequence

  /*! \brief Adds motion sequence data to limb trajectory lists

     Splits every key frame into target poses for every limb and adds the data to the \a trajectory lists.
   */
  void addSequenceToTrajectory(int ids[], int len, int pause, bool checkCollision = true, int kid = 15);

  /*! \brief Calculates a smooth transition between the current pose and the target motion

     The resulting motion sequence will be stored at index \a id in the motion buffer.

     \param target Motion Sequence Container (the motion to play next)
   */
  CMotion*getIntermediateMotion(CMotion*target, int id = INTERMEDIATE_MOTION, int ipoType = INTERMEDIATE_IPOTYPE, int pause = INTERMEDIATE_PAUSE);
  CMotion*getIntermediateMotion(CMotionData *target, int id = INTERMEDIATE_MOTION, int ipoType = INTERMEDIATE_IPOTYPE, int pause = INTERMEDIATE_PAUSE);


  /*! \brief Returns the total playtime of an array of motion sequences.
   */
  int  getSequenceLength(int *ids, int len);

  /*! \brief Adds a motion command to the \a cmds list.
   */
  void addCommand(char*cmd, float params[], int paramCount);
  void addCommand(char*cmd, int param1, int param2);

  /*! \brief Adds a control command to the \a cmdControls list
   */
  void addControlCommand(char*cmd, int params[], int paramCount);

  /*! \brief Returns the state of the read/write thread */
  int  getIpoState();

  /*! \brief Suspends or Resumes the read/write thread*/
  int  enableIpo(bool enable);

  CMotionContainer();
  void init(); ///< Initialization
  void process(); ///< Thread function

  /*! \brief Plays multiple motion sequences

     Supports looping, different interpolation types.

     \param id Array of indexes of motion sequences in /a motions
     \param pause Time in milliseconds between interpolation points
     \param ipo Interpolation type, see \a IPO_TYPES
     \param loop How often to repeat the motion
     \param num Length of \a id
   */
  void playSequence(int id[], int pause, int ipo, int loop, int num);

  /*! \brief Stops the playing of motion sequences

     Calls stop in its own thread.

     \param wait Not used at the moment
   */
  void stopSequence(bool wait);

  /*! \brief Joins motion sequences

     The key frames of all motion sequences will be joined and stored under the index \a store.
   */
  void joinSequence(int store, int ids[], int len);

  /*! \brief Captures movement data
     \see captureMotion
   */
  void capture(int id, byte input);

  /*! \brief Stops the capture process
   */
  void stopCapture();

  /*! \brief Moves a limb to the specified position with the specified orientation

     The pose will be added to the corresponding trajectory list (does use inverse kinematics to
     retrieve the corresponding joint angles).
   */
  void moveLimb(int id, float x, float y, float z, float rotx, float roty, float rotz, int time = 1000, bool sync = true);

  /*! \brief Moves a servo motor to the specified position

     The target angle will be added to the corresponding angle list.
   */
  void moveLink(int id, float angle, int time = 1000, bool sync = true);


  /*! \brief Loads motion data from a .mtn file into the motion buffer

     The .mtn-file \a filename will be parsed and the keyframes from \a start to \a end will
     be stored in the motion buffer, starting with index \a startid.
   */
  void loadMotionFromFile(int startid, char *filename, int start, int end);

};





/*! \brief Key Frame (Bioloid Motion File Data)

   Thanks to Bullit from Tribotix-forums
 */
struct _RECORD
{
  byte posData[2*31];
  byte delay;
  byte speed;
};

/*! \brief Page Header (Bioloid Motion File Data)

   Thanks to Bullit from Tribotix-forums
 */
struct _PHEADER
{
  char name[14];
  byte res1[6];
  byte pageStep;
  byte playCode;
  byte pageSpeed;
  byte dxlSetup;
  byte accelTime;
  byte nextPage;
  byte exitPage;
  byte linkedPage1;
  byte linkedPage1PlayCode;
  byte linkedPage2;
  byte linkedPage2PlayCode;
  byte checkSum;
  byte res2[32];
};

/*! \brief Page (Bioloid Motion File Data)

   Thanks to Bullit from Tribotix-forums
 */
struct _PAGE
{
  struct _PHEADER header;
  struct _RECORD rec[7];
};


#endif
