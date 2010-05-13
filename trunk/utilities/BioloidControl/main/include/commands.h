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


#ifndef __COMMANDS

#define __COMMANDS

#include "cmd.h"




/*! \brief Humanoid Motion Command States
 */
enum MOTION_COMMAND_STATES
{
  STATE_NONE,
  STATE_STARTWALKING,
  STATE_LOOPWALKING,
  STATE_STOPWALKING,
  STATE_LOOPSTANDWALKING,
  STATE_TURNRIGHT,
  STATE_TURNLEFT,
  STATE_STOP
};

/*! \brief Motion Command

    High-level robot control command: f.e. \a Walk, \a TurnRight and \a Stop.
 */
class CCommand
{
protected:
  CMotionContainer *motion;

public:
  CCommand() : motion(NULL) {};
  CCommand(CMotionContainer *m) : motion(m) {};
  virtual ~CCommand() {};

  virtual int  execute(CCommand*control = NULL) { return 0; };
  virtual bool  hasFinished() { return true; };
};

/*! \brief Motion Command Aggregation Class

    Aggregate class, i.e. behaves like a normal command but stores several Motion Commands,
    executes them and signals to caller if all commands have finished.
 */
class CCommandContainer : public CCommand
{
protected:
  CCommand**list;
  int listCount;
  int listCurrent;
  bool done;

public:
  CCommandContainer(unsigned int size, CMotionContainer *motion);
  ~CCommandContainer();

  int  execute(CCommand*control = NULL);
  bool  hasFinished();

  virtual void add(unsigned int id, CCommand*cmd);

};

#define COMMAND_PARAM_COUNT 2

/*! \brief Motions Command: Container

    Executes several motions stored in global motion buffer. \see CMotionContainer.motions
 */
class CCommandSequence : public CCommand
{
protected:
  int params[COMMAND_PARAM_COUNT];
  int*seqIds;
  int seqCount;
  bool done;
  int access;
  bool doSleep;
  unsigned long startTime;

public:
  bool addImmediateMotion;

  CCommandSequence(unsigned int size, bool sleep, CMotionContainer *motion, bool addImmediateMotion = false);
  ~CCommandSequence();

  void setSleep(bool sleep);
  void  setId(unsigned int index, unsigned int id);
  void  setParam(unsigned int index, unsigned int value);

  int  execute(CCommand*control = NULL);
  bool  hasFinished();

};

/*! \brief Motion Command: Stop

    Removes all commands from command buffer and executes a motion sequence
 */
class CCommandStop : public CCommandSequence
{
public:
  CCommandStop(unsigned int size, bool sleep, CMotionContainer *motion, bool addImmediateMotion = false) : CCommandSequence(size, sleep, motion, addImmediateMotion) {};

  int  execute(CCommand*control = NULL);

};

/*! \brief Motion Command: Omnidirectional Walking

    Implementation of the omnidirectional walking algorithm described in "Online Trajectory Generation for Omnidirectional Biped Walking" by Sven Behnke (http://citeseer.ist.psu.edu/behnke06online.html).
 */
class CCommandOmniWalk : public CCommand
{
private:
  bool addImmediateMotion;
  CVec speed;
  int steps;
  float piCut(float val);
  void calcAngles(float*angles, float clockLeg, float ls, CVec speed, float step, bool isLeft);
public:
  static float parameters[200];
  CCommandOmniWalk(int steps, float speedx, float speedy, float speedz, bool addImmediateMotion = false) : CCommand() { this->steps = steps; speed.set(speedx, speedy, speedz); this->addImmediateMotion = addImmediateMotion; };
       
  int  execute(CCommand*control = NULL);
};

#endif


