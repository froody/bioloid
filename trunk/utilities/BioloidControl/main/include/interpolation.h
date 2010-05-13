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


#ifndef INTERPOLATION_INC
#define INTERPOLATION_INC

#define IPOPRECISION double // interpolation-precision (todo: use template instead :))
#define MAXINTERPOLATIONDIM 50 // max. dimension in interpolation space (at least of size AX12_COUNT)

#include "vecmath.h"
#include "robot.h"

enum
{
  IPO_CART_X,
  IPO_CART_Y,
  IPO_CART_Z,
  IPO_CART_ROT,
  IPO_CART_COUNT
};

/*! \brief Interpolation Types

   Interpolation Types (robot or cartesian space)
 */
enum IPO_TYPE
{
  IPO_JOINTSPACE,
  IPO_LINEAR = IPO_JOINTSPACE,
  IPO_SPLINE,
  IPO_BEZIER,
  IPO_BEZIEROWN,
  IPO_PTP,
  IPO_PTPSINE,

  IPO_CARTSPACE,
  IPO_LINEARKART = IPO_CARTSPACE,
  IPO_SPLINEKART,
  IPO_BEZIERKART,
  IPO_BEZIEROWNKART,
  IPO_PTPKART,
  IPO_PTPSINEKART,

  IPO_NOCOLLISION,
  IPO_TOTAL_COUNT = IPO_NOCOLLISION
};

extern const char*ipoTypeNames[IPO_TOTAL_COUNT];

/*! \brief Interpolation Base Class
 */
class CInterpolation
{
public:
  char name[255];

  CInterpolation() { sprintf(name, ""); };
  virtual ~CInterpolation() {};

  virtual void setLength(int size) {};
  virtual int  getLength() {};

  virtual void  setPoint(int index, void*data) {};
  virtual void  getPoint(double relative_position, void*result) {};

  virtual void start() {};
  virtual void reset() {};
  virtual void finish() {};
};

/*! \brief Interpolation Class for an array of double values
 */
class CInterpolationBase : public CInterpolation
{
protected:
  IPOPRECISION *data;
  int len;
  int size;
  int currentPosition;
  int time;
  
public:
  CInterpolationBase();
  ~CInterpolationBase();

  void setLength(int size);
  int  getLength();
  
  void setTime(int time);
  int  getTime();
  
  int  getCurrentPosition() { return currentPosition; }
  void reset() { currentPosition = 0; };

  void  setPoint(int index, void *data);

};

/*! \brief Linear Interpolation
 */
class CLinear : public CInterpolationBase
{
public:
  CLinear() { sprintf(name, "Linear"); };

  void getPoint(double relative_position, void *result);

};

class SCurveParameters
{
public:
  double vmax, bmax;

  SCurveParameters() : vmax(2.0), bmax(0.001) {};
};

/*! \brief S-Curve with constant acceleration Interpolation
 */
class CSCurve : public CInterpolationBase
{
public:
  SCurveParameters parameters;

  CSCurve();

  void start();
  void getPoint(double relative_position, void *result);

};

/*! \brief S-Curve with sinoidal acceleration Interpolation
 */
class CSCurveSine : public CInterpolationBase
{
public:
  SCurveParameters parameters;

  CSCurveSine();

  void start();
  void getPoint(double relative_position, void *result);

};

/*! \brief Bezier Splines
 */
class CBezier : public CInterpolationBase
{
public:
  CBezier();
  void getPoint(double relative_position, void *result);

};

/*! \brief Bezier Splines
 */
class CBezierCustom : public CBezier
{
public:
  CBezierCustom();

  void getPoint(double relative_position, void *result);

};

/*! \brief Cubic Splines
 */
class CSpline : public CInterpolationBase
{
public:
  CSpline();

  void  getPoint(double relative_position, void*result);

  void start();

};

/*! \brief Interpolation data used by the CLinearQuaternion (timestamp, cartesian position and rotation matrix)
 */
class CLinearQuaternionData
{
public:
  double position;

  // only pointer!!
  CMatrix *pose;
};

/*! \brief Interpolation in Cartesian Space (Orientation only: Quaternions)
 */
class CLinearQuaternion : public CInterpolationBase
{
public:
  CLinearQuaternion();

  void  setPoint(int index, void *data);
  void  getPoint(double relative_position, void*result);

  void start();

};

/*! \brief Interpolation in Cartesian Space (Position: linear, Orientation: Quaternions)
 */
class CLinearQuaternionLinear : public CLinearQuaternion
{
public:
  CLinearQuaternionLinear();

  void  setPoint(int index, void *data);
  void  getPoint(double relative_position, void*result);

};

/*! \brief Interpolation Data

   Stores data belonging to a position in robot and cartesian space.
 */
class CInterpolationData
{
public:
  double angles_position;
  float angles[AX12_COUNT];
  double  *pose_position;
  CMatrix *pose;

  void malloc();
  void free();

};

/*! \brief Interpolation Wrapper Base Class

   Stores key frames and generates interpolated points.

   Procedure:
   \li Use \a setLength to set the number of interpolation points
   \li Use \a setPoint to add an interpolation point (CInterpolationData)
   \li Use \a getPoint to get the \a interpolated point (CInterpolationData) at a specified position
 */
class CInterpolationWrapper : public CInterpolation
{
protected:
  int size, time;

public:
  CInterpolationBase*interpolation[MAXINTERPOLATIONDIM];
  CInterpolationWrapper();
  ~CInterpolationWrapper();

  void setLength(int size);
  void setTime(int size);
  void start();
  void reset();

};

/*! \brief Wrapper class for interpolation in joint space

   \see CInterpolationWrapper
 */
class CJointSpaceWrapper : public CInterpolationWrapper
{
public:
  CJointSpaceWrapper();

  // data and result of type CInterpolationData
  void  setPoint(int index, void *data);
  void  getPoint(double relative_position, void*result);

};

/*! \brief Wrapper class for interpolation in cartesian space

   \see CInterpolationWrapper
 */
class CCartesianSpaceWrapper : public CInterpolationWrapper
{
protected:
  CRobot *robot;

public:
  CCartesianSpaceWrapper(CRobot *robot);

  // data and result is of type CInterpolationData
  void  setPoint(int index, void *data);
  void  getPoint(double relative_position, void*result);

};

/*! \brief Interpolation Factory

   Creates an Interpolation Wrapper Object based on selected interpolation \a type.
 */
class CInterpolationFactory
{
public:
  static CInterpolationWrapper*getWrapper(unsigned int type, CRobot *robot);

};

IPOPRECISION calcBezier(IPOPRECISION *values, int last, IPOPRECISION factor);
IPOPRECISION calcBezierCustom(IPOPRECISION *values, int last, IPOPRECISION factor);

//void linear_subsequent(IPOPRECISION *x, IPOPRECISION *y, IPOPRECISION *z, IPOPRECISION *h, int n, IPOPRECISION x, IPOPRECISION *y, int &oldindex);void linear_subsequent(IPOPRECISION *xin, IPOPRECISION *yin, int n, IPOPRECISION x, IPOPRECISION *y, int &oldindex);
void linear_subsequent(IPOPRECISION *xin, IPOPRECISION *yin, int n, IPOPRECISION x, IPOPRECISION *y, int &old);
void spline_prepare(IPOPRECISION *x, IPOPRECISION *y, IPOPRECISION *u, IPOPRECISION *y2, int n);
void spline_subsequent(IPOPRECISION *xo, IPOPRECISION *yo, IPOPRECISION *y2a, IPOPRECISION *yin, int n, IPOPRECISION x, IPOPRECISION *y, int &old);

#endif

