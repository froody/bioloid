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


#ifndef __ROBOT

#define __ROBOT

#include "tinyxml.h"
#include "types.h"
#include "vecmath.h"
#include "motion.h"

#define GEOMETRY_BOXCOUNT 50 // max boxes in a geometric model
#define FRAMES_COUNT 100 // max frames

/*! \brief Robot types with implemented inverse kinematics
 */
enum ROBOT_TYPES
{
  ROBOT_HUMANOID,
  ROBOT_HUMANOID_INVERSE_HIP,
  ROBOT_CARAUSIUSMOROSUS,
  ROBOT_COUNT
};

extern const char*robotNames[ROBOT_COUNT];

class CFrame;
class CRobot;

/*! \brief Cubic base component of geometry model
 */
class CBox
{
public:
  CVec center; ///< Center of cube in local frame
  CVec extents; ///< Extents of cube in x-, y- and z-direction
  CVec color; ///< Color associated with cube (xyz = rgb)
  CFrame *frame; ///< Local frame of cube (might be attached to a link joint)
  CMatrix *rot; ///< Homogenous transformation of box in local frame
  float mass; ///< Mass associated with the cube
  char name[256]; ///< Name of body part the box belongs to

  CBox();
  ~CBox();
  CBox(float pos_x, float pos_y, float pos_z, float width, float depth, float height,
       CFrame*transformation, float mass = 0.1);
  void set(float pos_x, float pos_y, float pos_z, float width, float depth, float height,
           CFrame*transformation, float mass = 0.1);

  /*! \brief Transforms a <BOX> (xml object) into a CBox
   */
  static bool xmlToBox(CBox *box, TiXmlElement*boxNode, CRobot *robot);

  /*! \brief Transforms a <COLOR> (xml object) into a vector of rgb values
   */
  static bool xmlToColor(CVec &color, TiXmlElement*colorNode);

};


/*! \brief Geometry model

   The robot is modelled as a set of cubic base elements (CBox), which are stored in the \a geometry
   array. The position and orientation of every box is determined by the frames associated with each box.
   The frames might be attached to a certain link of the robot and therefore will be moved if the
   robot is moved.<BR>
   Provides functions to:
   \li load the geometry model from a xml-file on disk
   \li do a (fast) collision check
 */
class CGeometry
{
private:
  /*! \brief Checks if the two boxes overlap
   */
  bool  getCollision(CBox &obj1, CBox &obj2);

  /*! \brief Checks if the projections of the \a first and \a second box on the \a axis intersect
   */
  bool checkOverlapping(const CVec&axis, const CBox&first, const CBox&second);

  /*! \brief Calculates the \a start- and \a endpoint of the projection of the \a box on the \a axis
   */
  void projection(const CBox&box, const CVec&axis, float &startpoint, float&endpoint);

public:
  CBox geometry[GEOMETRY_BOXCOUNT]; ///< Set of cubic base elements making up the geometry model
  int length; ///< Number of elements in \a geometry

  CGeometry();
  bool check[GEOMETRY_BOXCOUNT][GEOMETRY_BOXCOUNT]; ///< Signals for every pair of cubic base elements if a collision check should be performed

  /*! \brief Initializes the geometry model

     \param robot Robot structure, needed to initialize the local frame
     \param filename XML-file to load the model from
   */
  void init(CRobot *robot, char*filename);
  void cleanUp();

  CBox getBox(int id);
  void setBox(int id, CBox &box);
  CBox getBoxRobotSpace(int id); ///< Returns a representation of the box in robot space

  /*! \brief Tests if two or more cubic base elements overlap

     \param first Index of first overlapping box
     \param second Index of second overlapping box

     \return True if a collision occurred, False otherwise
   */
  int  checkCollisions(int &first, int &second);

  /*! \brief Parses the <COLLISION>s (xml objects)

     The <COLLISION> objects hold information about which box should be tested against
     which box in the collision check.
   */
  bool xmlToCollision(TiXmlElement*boxNode, CRobot *robot);

};

/*! \brief Denavit Hartenberg Link information
 */
class CDh
{
public:
  float rot_z; ///< Rotation offset around z-axis
  float trans_z; ///< Translation offset along z-axis
  float rot_x; ///< Rotation offset around rotated x-axis
  float trans_x; ///< Translation offset along rotated x-axis
  float angle; ///< Current angle of rotation aroung z-axis (changes with the real servo position)
  float sgn; ///< Direction of rotation (+1.0 clockwise, -1.0 counterclockwise)
  int id; ///< Index of servo motor associated with the frame
  CDh();
  
  float min, max;

  float getAngle(); ///< Returns current angle
  void  setAngle(float angle); ///< Sets current angle
  void  setMinMax(float min, float max);
  void  set(float rot_z, float trans_z, float rot_x, float trans_x);

};

/*! \brief Frame in cartesian space
 */
class CFrame
{
public:
  CFrame *base; ///< (Relative) base frame (f.e. the static robot frame or the frame of the previous link in a kinematic chain)
  CMatrix pose; ///< Translation and rotation relative to base frame (f.e. denavit hartenberg matrix)
  char name[255]; ///< Name associated with frame

  CFrame();
  CFrame(char*name);
  ~CFrame();

  CMatrix getRelativeToBase(); ///< Returns pose in the base (frame with no predecessor) frame

  void setName(char*str);
  bool hasName(char*str);

  virtual CFrame*getByName(char*str); ///< Returns frame associated with /a str
  virtual void update(); ///< Updates the frames

};

/*! \brief Kinematic chain
 */
class CKinematicChain : public CFrame
{
public:
  CDh     *dhParameters; ///< Array of denavit hartenberg parameters

  CFrame  **frames; ///< Array of frames associated with every link in the chain
  int length; ///< Number of links in the chain

  CKinematicChain();
  ~CKinematicChain();

  /*! \brief Updates denavit hartenberg matrices
   */
  void    update();

  /*! \brief Returns frame object if frame with name \a str exists in the chain
   */
  CFrame*getByName(char*str);

  /*! \brief Returns the pose of the last link in the chain (in the base frame)
   */
  void    getPose(CMatrix &result);

  /*! \brief Sets chain length and allocates memory
   */
  void    setLength(int len);

  /*! \brief Transforms <CHAIN> (xml object) into kinematic chain object
   */
  void    loadFromXml(CRobot *robot, TiXmlElement*kinChainsNode);

};

/*! \brief Kinematic Robot Model

   The kinematic model of the robot is given by a number of (connected) kinematic chains.
 */
class CKinematicChainContainer
{
public:
  int length; ///< Number of kinematic chains

  CKinematicChain *chain; ///< Array of kinematic chains

  CKinematicChainContainer();
  ~CKinematicChainContainer();

  void update(); ///< Updates denavit hartenberg matrices of all chains
  void loadFromXml(CRobot *robot, TiXmlElement*kinChainsNode); ///< Loads kinematic structure of robot from <KINEMATICCHAINS> (xml object)

};

/*! \brief Robot (kinematic, geometric, dynamic model)
 */
class CRobot
{
public:
  CKinematicChainContainer kinematicChains; ///< Kinematic model
  CFrame*frames[FRAMES_COUNT]; ///< All frames associated with the kinematic and geometric model
  int framesCount; ///< Number of frames

  int id; ///< Robot type \see ROBOT_TYPES

  CGeometry geometry; ///< Geometric model

  CRobot();
  ~CRobot();

  void init(char*filename); ///< Initializes the robot
  void cleanUp();

  /*! \brief Reads kinematic model data from xml file

     Loads frames and kinematic chains.
   */
  void initDhParameters(char*filename);

  /*! \brief Reads current angles from the robot wrapper and updates the denavit hartenberg parameters

     \param current True, if current angles of the servo motors should be used. False, if target angles should be used
   */
  void updateDhParameters(bool current = true);

  void getAnglesFromDh(float*angles); ///< Extracts angles from denavit hartenberg parameters
  void setDhFromAngles(float*angles); ///< Updates denavit hartenberg parameters with array of \a angles

  /*! \brief Calculates forward kinematics based on denavit hartenberg parameters

     \param draw True, if the new robot pose should be visualized in the opengl-viewer
   */
  void calcForwardKinematics(bool draw = true);

  /*! \brief Calculates inverse kinematics

     The result of the inverse kinematics calculation is stored in the denavit hartenberg structures
     of the links in the kinematic chain. Use \a getAnglesFromDh to extract the angle values.

     \param kinematicChain Index of kinematic chain
     \param pose Pose the last link in the kinematic chain should take on
   */
  void calcInverseKinematics(int kinematicChain, CMatrix &pose);

  /*! \brief Calculates dynamic behaviour of robot

     TODO: implement

     \param angles Current angles
     \param angles_v Current angle speed
     \param angles_a Current angle acceleration
   */
  void calcDynamics(float*angles, float*angles_v, float*angles_a, int view, float*result);

  /*! \brief Numerical estimation of the inverse kinematics

     DOES NOT WORK PROPERLY!
   */
  void estimateInverseKinematics(int kinematicChain, CMatrix &pose);

  /*! \brief Generates an array of Position and Orientation vectors

     The position (xzy) and orientation (euler angles) of every limb will be stored in the array.
     Used to display the limb information on the screen. \see CConsole::printInfo

     \param positionAndOrientation Has to be allocated of size kinematicChains::length * 2
   */
  void getTCPs(CVec *positionAndOrientation);

  /*! \brief
   */
  void calcKinematicChain(int len, CBox* geometry[], CDh* dhValues[]);

  /*! \brief Starts collision test

     \param msg Show collision messages
     \param useOld Only show collision message if the robot pose changed (a bit)
   */
  int  checkCollisions(bool msg = true, bool useOld = false);

  /*! \brief Starts collision test

     \see CGeometry::checkCollisions
   */
  int  checkCollisions(int &first, int &second);

  /*! \brief Transforms a <FRAME> (xml object) into a frame
   */
  bool xmlToFrame(CFrame *frame, TiXmlElement*frameNode);

  /*! \brief Returns index of frame with the stated \a name
   */
  int getFrameByName(char*name, bool create = false);

};


#endif
