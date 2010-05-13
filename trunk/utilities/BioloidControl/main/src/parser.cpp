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


/* Parser library, contains functions used by the scripting langugae */

#include <cstring>
#include <iostream>

#include "../include/robot.h"
#include "../include/platform.h"
#include "../include/wrapper.h"
#include "../include/util.h"
#include "../include/parser.h"
#include "../include/motion.h"
#include "../include/vars.h"
#include "../include/y.tab.h"

#define SENSOR_TAG_COUNT 7
const char*sensor_tags[SENSOR_TAG_COUNT] = {
  "DISTLEFT", "DISTCENTER", "DISTRIGHT",
  "LIGHTLEFT", "LIGHTCENTER", "LIGHTRIGHT",
  "SOUNDS"
};

const char*vec[6] = {
  "x", "y", "z", "rotx", "roty", "rotz"
};

int CParserBuffer::addVariable(char *name, double value) {
  int index = 0;
  for(index = 0; index < variables_count; index++) {
    if(strcasecmp(name, variables_buffer[index].name) == 0) {
      return index;
    }
  }

  if(variables_count >= VARIABLES_MAX) {
    return -1;
  }

  index = variables_count; // shouldnt be necessary
  strcpy(variables_buffer[index].name, name);
  variables_buffer[index].value = value;
  variables_count++;
  return index;
}

int CParserBuffer::addString(char *value)
{
  if(strings_count >= STRINGS_MAX) {
    return -1;
  }

  strcpy(strings_buffer[strings_count++], value);
  return strings_count-1;
}

int CParserBuffer::addItem(char id, int first, int second)
{
  if(items_count >= ITEMS_MAX) {
    CUtil::cout("CParserBuffer::addItem() failed.", TEXT_ERROR);
    return -1;
  }

  items[items_count].id =id;
  items[items_count].first = first;
  items[items_count].second = second;

  items_count++;
  return items_count-1;
}

void CParserBuffer::save(CParserBufferTimestamp &timestamp)
{
  timestamp.strings_count = strings_count;
  timestamp.variables_count = variables_count;
  timestamp.items_count = items_count;
}

void CParserBuffer::restore(CParserBufferTimestamp &timestamp)
{
  strings_count = timestamp.strings_count;
  variables_count = timestamp.variables_count;
  items_count = timestamp.items_count;
}

void CParserBuffer::execute()
{
  if(items_count > 0) {
    execute(items_count-1);
  }
}

#define PARAMMAX 20
double CParserBuffer::execute(int id)
{
  double first, second;
  int tmp, j;
  int params[PARAMMAX];
  char str[255];

  switch(items[id].id)
  {
  case '+':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first + second;

  case '>':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first > second;

  case '<':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first < second;

  case 'u':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first != second;

  case 'e':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first == second;

  case 'g':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first >= second;

  case 'G':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first <= second;

  case '*':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first * second;

  case '_':
    return -execute(items[id].first);

  case '-':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first - second;

  case '/':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first / second;

  case '=':
    variables_buffer[items[id].first].value = execute(items[id].second);
    return 1;

  case '|':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first || second;

  case 'O':
  {
    strcpy(str, strings_buffer[items[id].first]);

    char*param = NULL;
    int id = 1;
    for(j=0; j<strlen(str); j++) {
      if(str[j] == '.') {
        str[j] = '\0';
        param = &str[j+1];
      }
    }
    if(param == NULL) {
      return 0;
    }
    else {
      if(strcasecmp(str, "legl") == 0) {
        id = 0;
      }
      else if(strcasecmp(str, "legr") == 0) {
        id = 1;
      }
      else if(strcasecmp(str, "arml") == 0) {
        id = 2;
      }
      else if(strcasecmp(str, "armr") == 0) {
        id = 3;
      }

      return global.parser.pos(id, param);
    }
  }

  case 'v':
    global.parser.set( (int)execute(items[id-1].first), execute(items[id-1].second), (int)execute(items[id].first) );
    return 0.0;

  case 'V':
    return global.parser.get( (int)execute(items[id].first) );

  case '&':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return first && second;

  case '9':
    return global.parser.pos( (int)execute(items[id].first), strings_buffer[items[id].second]);

  case 'L':
    return variables_buffer[items[id].first].value;

  case ';':
    first = execute(items[id].first);
    second = execute(items[id].second);
    return 1;

  case 'P':
    first = execute(items[id].first);
    sprintf(str, "%g", first);
    CUtil::cout(str);
    return 1;

  case 'q':
    sprintf(str, "\n");
    CUtil::cout(str);
    return 1;

  case 'p':
    sprintf(str, "%s", strings_buffer[items[id].first]);
    for(j=0; j<strlen(str)-2; j++) {
      ;
    }
    {
      if( (str[j] == '\\') && (str[j+1] == 'n') ) {
        str[j] = ' ';
        str[j+1] = '\n';
        j++;
      }
    }
    CUtil::cout(str);
    return 1;

  case 'y':
    global.parser.sync_start();
    tmp = (int)execute(items[id].first);
    global.parser.sync_end();
    return tmp;

  case 'i':
    tmp = (int)execute(items[id-1].first);
    if(tmp) {
      execute(items[id].first);
      return 1;
    }
    else {
      execute(items[id].second);
      return 0;
    }

  case 'I':
    tmp = (int)execute(items[id].first);
    if(tmp) {
      execute(items[id].second);
      return 1;
    }
    else { return 0;}
  case 'd':
    tmp = (int)execute(items[id].first);
    while(tmp)
    {
      execute(items[id].second);
      tmp = (int)execute(items[id].first);
    }
    return 1;

  case 'D':
    global.parser.stop();
    return 1;

  case 'h':
    global.parser.hold();
    return 1;

  case 'w':
    global.parser.wait();
    return 1;

  case 'c':
    global.parser.script(strings_buffer[items[id].first], (char *)"");
    return 1;

  case 'C':
    return global.parser.collision();

  case '1':
    global.parser.moveto(3, execute(items[id-1].first), execute(items[id-1].second), execute(items[id].first), (int)execute(items[id].second) );
    return 1;

  case '2':
    global.parser.moveto(4, execute(items[id-1].first), execute(items[id-1].second), execute(items[id].first), (int)execute(items[id].second) );
    return 1;

  case '3':
    global.parser.moveto(1, execute(items[id-3].first), execute(items[id-3].second),
                         execute(items[id-2].first), execute(items[id-2].second), execute(items[id-1].first),
                         execute(items[id-1].second), (int)execute(items[id].first) );
    return 1;

  case '4':
    global.parser.moveto(2, execute(items[id-3].first), execute(items[id-3].second),
                         execute(items[id-2].first), execute(items[id-2].second), execute(items[id-1].first),
                         execute(items[id-1].second), (int)execute(items[id].first) );
    return 1;

  case '5':
    global.parser.moveto( (int)execute(items[id-2].first), execute(items[id-2].second), execute(items[id-1].first), execute(items[id-1].second), (int)execute(items[id].first) );
    return 1;

  case '6':
    global.parser.moveto( (int)execute(items[id-3].first), execute(items[id-3].second), execute(items[id-2].first),
                          execute(items[id-2].second), execute(items[id-1].first), execute(items[id-1].second), execute(items[id].first), (int)execute(items[id].second) );
    return 1;

  case 'T':
    return global.parser.torque(execute(items[id].first) );

  case 't':
    return global.parser.time();

  case 'A':
    return global.parser.fabs(execute(items[id].first) );

  case 'B':
    return sin(M_PI/180.0 * execute(items[id].first) );

  case 'b':
    return asin(execute(items[id].first) );

  case 'X':
    return cos(M_PI/180.0 * execute(items[id].first) );

  case 'x':
    return acos(execute(items[id].first) );

  case 'Z':
    return tan(M_PI/180.0 * execute(items[id].first) );

  case 'z':
    return atan2(execute(items[id].first), execute(items[id].second) );

  case 's':
    global.parser.sleep(execute(items[id].first) );
    return 1;

  case 'S':
    return global.parser.sensor(strings_buffer[items[id].first]);

  case 'M':
    global.parser.motion(strings_buffer[items[id].first], execute(items[id].second) );
    return 1;

  case 'm':
    global.parser.play(execute(items[id-1].first), execute(items[id-1].second), execute(items[id].first) );
    return 1;

  case 'n':
    tmp = items[id-1].first;
    j = 0;
    while(j < PARAMMAX)
    {
      params[j]=(int)execute(items[tmp].first);
      j++;

      if(items[tmp].second < 0) {
        break;
      }
      tmp = items[tmp].second;
    }
    global.parser.run(params, j, (int)execute(items[id-1].second), (int)execute(items[id].first) ); //execute(items[id-1].first)
    return 1;

  case 'R':
    return global.parser.rotation(strings_buffer[items[id].first], strings_buffer[items[id].second]);

  case 'l':
    return global.parser.load(execute(items[id].first), strings_buffer[items[id].second]);

  default:
    CUtil::cout("CParserBuffer::execute() failed. Unreachable statement.", TEXT_ERROR);
  }
}

int CParser::parseFile(char*filename, float *argv, int argc)
{
  this->filename = filename;
  this->params = params;

  CParserBufferTimestamp backup;
  buffer.save(backup);

  buffer.addVariable( (char *)"paramcount", argc);

  char str[255];
  for(int i=0; i<argc; i++) {
    sprintf(str, "param%d", i);
    buffer.addVariable(str, argv[i]);
  }

  yyrestart(0);
  if(yyparse() == 0) {
    buffer.execute();

    CParser::finish();
  }

  buffer.restore(backup);
  return 1;
}

// returns 1.0 if a collision occured, 0.0 otherwise.
double CParser::collision()
{
  int b;
  ML(MUTEXCOMM);
  b = global.robotInput.checkCollisions();
  MU(MUTEXCOMM);
  return (double)b;
}

// runs another scripting file
double CParser::script(char*filename, char*params)
{
  float dparams[10];
  char *pch = strtok(params, ",");
  int count = 0;
  bool error;

  while(pch != NULL && count < 10)
  {
    error = false;
    dparams[count] = CUtil::strtodouble(pch, strlen(pch), &error);
    if(error) {
      break;
    }

    count++;
    pch = strtok(NULL, ",");
  }

  parseFile(filename, dparams, count);
  return 0;
}

// prints text
// obsolete
double CParser::print(char*text)
{
  return 0;
}

// returns current torque applied to motor with id number
double CParser::torque(double number)
{
  //std:cout << "torque \n";
  int id = (int)number;
  double retval = 0;

  if( (id >= 1) && (id <= AX12_COUNT) ) {
    ML(MUTEXCOMM);
    retval = (double)global.robotWrapper.Ax12s[id-1].getCurrentTorque();
    MU(MUTEXCOMM);
  }
  return retval;
}

// pauses the script for number milliseconds
double CParser::sleep(double number)
{
  CPlatform::sleep( (unsigned long)number);
}

// sync semaphore
int syncCounter = 0;

// enter semaphore
double CParser::sync_start()
{
  syncCounter++;
  if(syncCounter == 1) {
    ML(MUTEXTRAJECTORY);
  }

}

// leave semaphore
double CParser::sync_end()
{
  //std::cout << "sync end\n";
  syncCounter--;
  if(syncCounter == 0) {
    MU(MUTEXTRAJECTORY);
  }
}

// clean up semaphore
void CParser::finish()
{
  if(syncCounter > 0) {
    MU(MUTEXTRAJECTORY);
    syncCounter = 0;
  }
}

// plays a motion stored in slot id with delay milliseconds between the frames
// kid determines which arms and legs participate in the movement.
// kid is a bitwise combination of
// 1: left leg, 2: right leg, 4: left arm, 8: right arm
double CParser::play(double id, double delay, double kid)
{
  int ids[1];
  ids[0] = (int)id;

  //  if (delay < MOTION_IPO_PAUSE)
  //    delay = MOTION_IPO_PAUSE;

  // play motion with sequence control
  //motion.playSequence(ids, delay, 0, 1, 1);

  // play motion with trajectory control
  global.motion.addSequenceToTrajectory(ids, 1, (int)delay, true, (int)kid);
}

// load motion file into slot id
double CParser::load(double id, char*file)
{
  int i = (int)id;

  if( (i >= 0) && (i < MOTION_COUNT) ) {
    global.motion.motions[i].loadFromFile(file);
  }
}

// plays motion-sequence with specified delay and interpolation type
double CParser::run(int *ids, int len, int delay, int ipo)
{
  // play motion with sequence control
  global.motion.playSequence(ids, delay, ipo, 1, len);
}

// returns current time in milliseconds
double CParser::time()
{
  return (double)CPlatform::getTickCount();
}

// executes the motion command 'name' 'count' times
double CParser::motion(char*name, double count)
{
  global.motion.addCommand(name, (int)count, 35);
}

// returns value of selected sensor, see sensor_tags for valid parameters
double CParser::sensor(char*name)
{
  //std:cout << "sensor \n";
  int i=0;
  int id = -1;
  for(; i<SENSOR_TAG_COUNT; i++) {
    if(strcasecmp(sensor_tags[i], name) == 0) {
      id = i;
      break;
    }
  }
  int tmp;
  ML(MUTEXCOMM);
  switch(id)
  {
  case 0:
    tmp = (int)global.robotWrapper.Axs1s[0].getDistLeft(); break;

  case 1:
    tmp = (int)global.robotWrapper.Axs1s[0].getDistCenter(); break;

  case 2:
    tmp = (int)global.robotWrapper.Axs1s[0].getDistRight(); break;

  case 3:
    tmp = (int)global.robotWrapper.Axs1s[0].getLightLeft(); break;

  case 4:
    tmp = (int)global.robotWrapper.Axs1s[0].getLightCenter(); break;

  case 5:
    tmp = (int)global.robotWrapper.Axs1s[0].getLightRight(); break;

  case 6:
    tmp = (int)global.robotWrapper.Axs1s[0].getSoundsCounter(); break;

  default:
    tmp = 0;
  }
  MU(MUTEXCOMM);

  return (double)tmp;
}

// returns absolute value of 'number'
double CParser::fabs(double number)
{
  return (double)fabs(number);
}

// move left leg to position x y z with orientation rotx roty rotz
//(euler angles, 0 0 0 = foot parallel to floor) in time milliseconds
double CParser::moveto(int id, double x, double y, double z, int time)
{
  global.motion.moveLimb(id-1, x, y, z, 0, 0, 0, time, syncCounter == 0);
}

// move left leg to position x y z with orientation rotx roty rotz
//(euler angles, 0 0 0 = foot parallel to floor) in time milliseconds
double CParser::moveto(int id, double x, double y, double z, double rotx, double roty, double rotz, int time)
{
  global.motion.moveLimb(id-1, x, y, z, rotx, roty, rotz, time, syncCounter == 0);
}

// obsolete
double CParser::rotation(char*kin, char*xyz)
{
  return 0.0;
}

// hold position -> sets target-angle of all servos to current-angle
double CParser::hold()
{
  global.robotWrapper.holdPosition();
  return 0.0;
}

// stop script-execution until user presses the return-key
double CParser::wait()
{
  CUtil::waitForReturn();
  return 0.0;
}

// stop all movement
double CParser::stop()
{
  global.motion.stop();
  return 0.0;
}

double CParser::get(int id)
{
  if( (id <= 0) || (id > AX12_COUNT) ) {
    return 0.0;
  }

  double angle;
  ML(MUTEXCOMM);
  angle = global.robotWrapper.Ax12s[id-1].getCurrentAngle();
  MU(MUTEXCOMM);

  return angle;
}

double CParser::set(int id, double angle, int time)
{
  if( (id <= 0) || (id > AX12_COUNT) ) {
    return 0.0;
  }

  global.motion.moveLink(id, (float)angle, time, syncCounter == 0);
}

// returns postion or orientation of selected object
// f.e. x-position of left foot: legl.x
double CParser::pos(int id, char*vecname)
{
  int i, kinid;

  kinid = id;
  if( (id < 0) || (id >= global.robotInput.kinematicChains.length) ) {
    char text[255];
    sprintf(text, "Limb #%d doesn't exist.", id);
    CUtil::cout(text, TEXT_ERROR);
    return 0.0;
  }

  id = 0;
  // get id of referenced coordinate: x = 0, y = 1, z = 2, rotx = 3, ...
  for(i=0; i<6; i++) {
    if(strcasecmp(vec[i], vecname) == 0) {
      id = i;
      break;
    }
  }

  float tmp = 0.0;
  CVec tmpVec1, tmpVec2;
  CMatrix tmp1, tmp2;

  // read referenced value, f.e. arml.x = current x-position of limb arml
  ML(MUTEXINPUT);
  global.robotCalc.updateDhParameters(true);

  global.robotCalc.calcForwardKinematics(false);

  switch(id)
  {
  case 0:
    tmp = global.robotCalc.kinematicChains.chain[kinid].getRelativeToBase().a[12];
    break;

  case 1:
    tmp = global.robotCalc.kinematicChains.chain[kinid].getRelativeToBase().a[13];
    break;

  case 2:
    tmp = global.robotCalc.kinematicChains.chain[kinid].getRelativeToBase().a[14];
    break;

  case 3:
  case 4:
  case 5:
  {
    tmp1 = global.robotCalc.kinematicChains.chain[kinid].frames[global.robotCalc.kinematicChains.chain[kinid].length-1]->pose;
    tmp1.a[12] = 0.0;
    tmp1.a[13] = 0.0;
    tmp1.a[14] = 0.0;
    tmp1.invert();

    tmp2 = global.robotCalc.kinematicChains.chain[kinid].getRelativeToBase();
    CMathLib::getOrientation(tmp2, tmpVec1, tmpVec2);

    // tweak, ugly
    float rotx, roty, rotz;
    rotx = roty = rotz = 0.0;
    switch(global.robotInput.id)
    {
    case ROBOT_HUMANOID:
    case ROBOT_HUMANOID_INVERSE_HIP:
      rotx = 90.0;
      roty = 90.0;
      break;
    }
    switch(id)
    {
    case 3:
      tmp = -rotx + (180.0 / M_PI) * tmpVec1.x; break;

    case 4:
      tmp = -roty + (180.0 / M_PI) * tmpVec1.y; break;

    case 5:
      tmp = -rotz + (180.0 / M_PI) * tmpVec1.z; break;
    }
  }
  break;
  }
  MU(MUTEXINPUT);

  return tmp;
}

