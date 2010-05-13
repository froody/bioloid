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


#ifndef __TEST
#define __TEST

class CLeg
{
public:
  CVec start, end, current, target, base, oldTarget, oldCurrent;
  int oldCounter;

  float legDown;
  float legUp;
  float legFront;
  float legBack;
  float legInner;
  float legOuter;

  float offset;
  float stanceCounter, swingCounter, offsetCounter, maxCounter;
  bool onGround, isFront, isBack, isSwinging, doSwing, doStance;

  int id, prior, next, across;
  CLeg();

  void preprocess();
  void process(CLeg *legs);
  void update(CLeg *legs);

};

/*! \brief Test Class
 */
class CTest
{
public:
  CLeg legs[6];

  void humanoidWalkOwn(float*args, int argcount);

  void humanoidWalk(float*args, int argcount);
  void spiderWalk(float*args, int argcount);

  void run(float*args, int argcount);

};

#endif
