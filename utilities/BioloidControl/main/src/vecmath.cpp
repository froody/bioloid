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


#include <math.h>
#include <iostream>

#include "../include/robot.h"
#include "../include/wrapper.h"
#include "../include/vecmath.h"
#include "../include/util.h"

#define SWAP(x, y) tmp = a[x]; a[x]=a[y]; a[y] = tmp;
#define ROUND(x) rnd(x*10)/10.0)
using namespace std;

// inits vector
CVec::CVec()
{
  x = y = z = 0.0; w = 1.0;
}

// inits vector
CVec::CVec(PRECISION x, PRECISION y, PRECISION z)
{
  set(x, y, z);
}

// sets vector to (x y z 1.0)
void CVec::set(PRECISION x, PRECISION y, PRECISION z)
{
  this->x = x;
  this->y = y;
  this->z = z;
  this->w = 1.0;
}

void CVec::normalize()
{
  PRECISION len = length();
  if (len > 0.00001)
  {
    x /= len;
    y /= len;
    z /= len;
  }
}
  

PRECISION CVec::length()
{
  return sqrt(x*x + y*y + z*z);
}

// prints vector value
void CVec::print()
{
  char str[255];
  sprintf(str, "%f %f %f", x, y, z);
  CUtil::cout(str);
}

// minus operator
CVec CVec::operator -() const
{
  return CVec(-x, -y, -z);
}

// scalar multiplication
CVec CVec::operator * (PRECISION s) const
{
  return CVec(x*s, y*s, z*s);
}

// assignment operator
CVec&CVec::operator =(const CVec&v)
{
  x = v.x; y = v.y; z = v.z; w = 1.0f;
  return *this;
}

// array access [], read online
PRECISION CVec::operator [](unsigned int i) const
{
  return (&x)[i];
}

// array access [], read & write
PRECISION&CVec::operator [](unsigned int i)
{
  return (&x)[i];
}

// vector add operator
CVec CVec::operator +(const CVec&v) const
{
  return CVec(x + v.x, y + v.y, z + v.z);
}

// vector sub operator
CVec CVec::operator -(const CVec&v) const
{
  return CVec(x - v.x, y - v.y, z - v.z);
}

// scalar product
PRECISION CVec::operator |(const CVec&v) const
{
  return x*v.x + y*v.y + z*v.z;
}

// cross product
CVec CVec::operator ^(const CVec&v) const
{
  return CVec(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
}

// inits matrix with 1-Matrix
CMatrix::CMatrix()
{
  set(1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0);
}

// transposes matrix
void CMatrix::transpose()
{
  PRECISION tmp;
  SWAP(1, 4);
  SWAP(2, 8);
  SWAP(6, 9);
  SWAP(3, 12);
  SWAP(7, 13);
  SWAP(11, 14);
}

// invert homogenous transformation matrix
void CMatrix::invert()
{
  PRECISION tmp;
  CVec vec, res;
  vec.set(-a[12], -a[13], -a[14]);
  SWAP(1, 4);
  SWAP(2, 8);
  SWAP(6, 9);
  a[12] = a[13] = a[14] = 0.0;
  res = (*this) * vec;
  a[12] = res.x;
  a[13] = res.y;
  a[14] = res.z;
}

// sets matrix to supplied value
CMatrix::CMatrix(PRECISION a0, PRECISION a4, PRECISION a8,  PRECISION a12,
                 PRECISION a1, PRECISION a5, PRECISION a9,  PRECISION a13,
                 PRECISION a2, PRECISION a6, PRECISION a10, PRECISION a14,
                 PRECISION a3, PRECISION a7, PRECISION a11, PRECISION a15)
{
  a[0] = a0;  a[4] = a4;  a[8]  = a8;   a[12] = a12;
  a[1] = a1;  a[5] = a5;  a[9]  = a9;   a[13] = a13;
  a[2] = a2;  a[6] = a6;  a[10] = a10;  a[14] = a14;
  a[3] = a3;  a[7] = a7;  a[11] = a11;  a[15] = a15;
}

// sets matrix to supplied value
void CMatrix::set(PRECISION a0, PRECISION a4, PRECISION a8,  PRECISION a12,
                  PRECISION a1, PRECISION a5, PRECISION a9,  PRECISION a13,
                  PRECISION a2, PRECISION a6, PRECISION a10, PRECISION a14,
                  PRECISION a3, PRECISION a7, PRECISION a11, PRECISION a15)
{
  a[0] = a0;  a[4] = a4;  a[8]  = a8;   a[12] = a12;
  a[1] = a1;  a[5] = a5;  a[9]  = a9;   a[13] = a13;
  a[2] = a2;  a[6] = a6;  a[10] = a10;  a[14] = a14;
  a[3] = a3;  a[7] = a7;  a[11] = a11;  a[15] = a15;
}

// creates denavit hartenberg transformation matrix	based on CDh struct
void CMatrix::setDh(const CDh &dh)
{
  a[0] = cos(dh.rot_z +dh.angle);  a[4] = -sin(dh.rot_z + dh.angle)*cos(dh.rot_x);  a[8]  = sin(dh.rot_x)*sin(dh.rot_z + dh.angle);   a[12] = dh.trans_x*cos(dh.rot_z + dh.angle);
  a[1] = sin(dh.rot_z + dh.angle); a[5] = cos(dh.rot_x)*cos(dh.rot_z + dh.angle);   a[9]  =  -cos(dh.rot_z + dh.angle)*sin(dh.rot_x); a[13] = dh.trans_x*sin(dh.rot_z + dh.angle);
  a[2] = 0.0;                      a[6] = sin(dh.rot_x);                            a[10] = cos(dh.rot_x);                            a[14] = dh.trans_z;
  a[3] = 0.0;                      a[7] = 0.0;                                      a[11] = 0.0;                                      a[15] = 1.0;
}

// trace of rotational 3x3 part of matrix
PRECISION CMatrix::trace()
{
  return a[0] + a[5] + a[10];
}

// print matrix contents
void CMatrix::print(bool round)
{
  char str[1024];

#define RND(x) round ? rnd(x*10)/10.0 : x
  sprintf(str,
          "%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n",
          RND(a[0]), RND(a[4]), RND(a[8]), RND(a[12]),
          RND(a[1]), RND(a[5]), RND(a[9]), RND(a[13]),
          RND(a[2]), RND(a[6]), RND(a[10]), RND(a[14]),
          RND(a[3]), RND(a[7]), RND(a[11]), RND(a[15]) );
  CUtil::cout(str);
}

// update dh matrix with a new theta
void CMatrix::updateDh(const CDh &dh)
{
  a[0] = cos(dh.rot_z + dh.angle); a[4] = -sin(dh.rot_z + dh.angle)*cos(dh.rot_x); a[8]  = sin(dh.rot_x)*sin(dh.rot_z + dh.angle);       a[12] = dh.trans_x*cos(dh.rot_z + dh.angle);
  a[1] = sin(dh.rot_z + dh.angle); a[5] = cos(dh.rot_x)*cos(dh.rot_z + dh.angle);  a[9]  =  -cos(dh.rot_z + dh.angle)*sin(dh.rot_x);     a[13] = dh.trans_x*sin(dh.rot_z + dh.angle);
}

// calculates matrix product: this = first * second
void CMatrix::mul(const CMatrix &first, const CMatrix &second)
{
  // optimize it!!
  PRECISION tmp[16];

  int i, j, k;

  // first or second could be *this -> local buffer
  for(i=0; i<16; i++) {
    tmp[i] = 0.0;
  }

  PRECISION tmpn;
  for(i = 0; i<4; i++) {
    for(k = 0; k<4; k++) {
      tmpn = first.a[i+4*k];
      for(j = 0; j<4; j++) {
        tmp[i+4*j] += tmpn * second.a[k+4*j];
      }
    }
  }

  for(i=0; i<16; i++) {
    a[i] = tmp[i];
  }
}

// scalar multiplication
CMatrix CMatrix::operator * (PRECISION f) const
{
  return CMatrix(a[0]*f, a[4]*f, a[8]*f, a[12]*f,
                 a[1]*f, a[5]*f, a[9]*f, a[13]*f,
                 a[2]*f, a[6]*f, a[10]*f, a[14]*f,
                 0.0, 0.0, 0.0, 1.0);
}

// matrix add operator
CMatrix CMatrix::operator +(const CMatrix &b) const
{
  return CMatrix(a[0]+b.a[0], a[4]+b.a[4], a[8]+b.a[8], a[12]+b.a[12],
                 a[1]+b.a[1], a[5]+b.a[5], a[9]+b.a[9], a[13]+b.a[13],
                 a[2]+b.a[2], a[6]+b.a[6], a[10]+b.a[10], a[14]+b.a[14],
                 0.0, 0.0, 0.0, 1.0);
}

// matrix sub operator
CMatrix CMatrix::operator -(const CMatrix &b) const
{
  return CMatrix(a[0]-b.a[0], a[4]-b.a[4], a[8]-b.a[8], a[12]-b.a[12],
                 a[1]-b.a[1], a[5]-b.a[5], a[9]-b.a[9], a[13]-b.a[13],
                 a[2]-b.a[2], a[6]-b.a[6], a[10]-b.a[10], a[14]-b.a[14],
                 0.0, 0.0, 0.0, 1.0);
}

// matrix product operator
CMatrix CMatrix::operator * (const CMatrix &m) const
{
  CMatrix tmp;
  tmp.mul( (*this), m);
  return tmp;
}

// add a translation to homogenous matrix
CMatrix&CMatrix::operator +=(const CVec &v)
{
  a[12] += v.x;
  a[13] += v.y;
  a[14] += v.z;
  return *this;
}

// calculate matrix * vector
CVec CMatrix::operator * (const CVec &v) const
{
  PRECISION x, y, z;
  x = a[0]*v.x + a[4]*v.y + a[8]*v.z + a[12]*v.w;
  y = a[1]*v.x + a[5]*v.y + a[9]*v.z + a[13]*v.w;
  z = a[2]*v.x + a[6]*v.y + a[10]*v.z + a[14]*v.w;
  return CVec(x, y, z);
}

// array operator, read only
const CVec&CMatrix::operator [](int i) const
{
  return *(const CVec*)&a[4*i];
}

// assignment operator
CMatrix&CMatrix::operator =(const CMatrix&v)
{
  CUtil::copymem(a, v.a, 16*sizeof(PRECISION) );
  return *this;
}

// rounds a value
int rnd(PRECISION value)
{
  return (int)(value < 0.0 ? ceil(value - 0.5) : floor(value +0.5) );
}

PRECISION CMatrix::length()
{
  double dist = sqrt(a[12]*a[12]+a[13]*a[13]+a[14]*a[14]);
  CVec axis;
  float angle;
  CMathLib::getRotationFromMatrix(*this, axis, angle);
  
  return dist + fabsf(angle) * 180.0/M_PI;
}
