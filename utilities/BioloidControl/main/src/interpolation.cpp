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

#include "../include/interpolation.h"
#include "../include/util.h"
#include "../include/robot.h"
#include "../include/vars.h"

const char*ipoTypeNames[IPO_TOTAL_COUNT] = {
  "Linear (joint space)",
  "Cubic Spline (joint space)",
  "Bezier Curve (joint space)",
  "Custom Bezier Curve (joint space)",
  "Point-To-Point (joint space)",
  "Sinoidal Point-To-Point (joint space)",
  "Linear (cartesian space)",
  "Cubic Spline (cartesian space)",
  "Bezier Curve (cartesian space)",
  "Custom Bezier Curve (cartesian space)",
  "Point-To-Point (cartesian space)",
  "Sinoidal Point-To-Point (cartesian space)",
};

IPOPRECISION getRelativePosition(IPOPRECISION *data, int n, IPOPRECISION x, int &oldIndex)
{
  int lower, upper;
  IPOPRECISION xdiff;

  if(oldIndex >= n-2) {
    lower = n-2;
    upper = n-1;
  }
  else {
    upper = oldIndex+1;
    while(data[upper] <= x) {
      upper++;
    }
    oldIndex = lower = upper-1;
  }

  xdiff = data[upper]-data[lower];

  return (x - data[lower]) / xdiff;
}

/*
   Abstract Double Interpolation Class

   data = double[len][size]

 */
CInterpolationBase::CInterpolationBase()
{
  data = NULL;
  len = 0;
  size = 2;
  currentPosition = 0;
}

CInterpolationBase::~CInterpolationBase()
{
  setLength(0);
  data = NULL;
}

void CInterpolationBase::setTime(int size)
{
  time = size;
}

int CInterpolationBase::getTime()
{
  return time;
}

void CInterpolationBase::setLength(int size)
{
  // malloc array of (x y) pairs
  data = (IPOPRECISION*)CUtil::realloc(data, size * this->size * sizeof(IPOPRECISION) );

  len = (data == NULL) ? 0 : size;
}

int CInterpolationBase::getLength()
{
  return len;
}

// Set Interpolation Point
void CInterpolationBase::setPoint(int index, void*point)
{
  for(int i=0; i<size; i++) {
    data[index + i * len] = ( (IPOPRECISION*)point)[i];
  }
}

/*
   Linear Interpolation Class
   linear search through array -> optimized for subsequent calls
 */

// linear interpolation
void CLinear::getPoint(double relative_position, void*result)
{
  int lower, upper;
  IPOPRECISION xdiff, ydiff;

  // get x-value at relative_position in array
  // x[0] = 0.2, x[1] = 0.4 -> x a relative_position = 0.5 -> 0.3
  IPOPRECISION x = data[0] + relative_position * (data[len-1] - data[0]);

  // get index lower: data[lower] <= x <= data[upper]
  if(currentPosition >= len-2) {
    lower = len-2;
    upper = len-1;
  }
  else {
    upper = currentPosition+1;
    while(data[upper] < x) {
      upper++;
    }

    lower = upper-1;
  }

  // x difference
  xdiff = data[upper]-data[lower];

  // y difference
  ydiff = data[upper + len]-data[lower + len];

  // result = y[lower] + (dy/dx) * (x - x[lower])
  *(IPOPRECISION*)result = data[lower + len] + (ydiff / xdiff) * (x - data[lower]);

  currentPosition = lower;
}

/*
   Bezier Interpolation Class
   only use with <= ~150 ipo points (higher num will throw precision errors )
 */
CBezier::CBezier()
{
  size = 2; // only y - value needed but store both for compatability
  sprintf(name, "Bezier");
};

void CBezier::getPoint(double relative_position, void *result)
{
  *(IPOPRECISION*)result = calcBezier(&data[len], len-1, relative_position);
}

CBezierCustom::CBezierCustom()
{
  size = 2; // only y - value needed but store both for compatability
  sprintf(name, "Custom Bezier");
};

void CBezierCustom::getPoint(double relative_position, void *result)
{
  *(IPOPRECISION*)result = calcBezierCustom(&data[len], len, relative_position);
}

/*
   Spline Interpolation Class
   linear search through array -> optimized for subsequent calls
   reset() - reset position for linear search
   start() - call before getPoint(), calculate accelerations
 */
CSpline::CSpline()
{
  size = 4; // store x, y, y'' and a temporary buffer
  sprintf(name, "Cubic Splines");
}



void CSpline::getPoint(double relative_position, void*result)
{
  spline_subsequent(&data[0 + 0*len], &data[0 + 1*len], &data[0 + 2*len], &data[0 + 3*len], len, data[0] + relative_position * (data[len-1] - data[0]), (double*)result, currentPosition);
}

// interpolation preparation
void CSpline::start()
{
  spline_prepare(&data[0 + 0*len], &data[0 + 1*len], &data[0 + 2*len], &data[0 + 3*len], len);
}

CLinearQuaternion::CLinearQuaternion()
{
  size = 1 + 9 + 3 + 1; // position, 3x3-Matrix, Rotationaxis, Rotationangle
  currentPosition = 0;
}

void CLinearQuaternion::setPoint(int index, void *m)
{
  CLinearQuaternionData*quaterData = (CLinearQuaternionData*)m;
  CMatrix *matrix = quaterData->pose;

  // store position and 3x3- Matrix, axis and angle will be calculated in start()
  int i=0;
  data[index * size + i++] = quaterData->position;
  data[index * size + i++] = matrix->a[0];
  data[index * size + i++] = matrix->a[4];
  data[index * size + i++] = matrix->a[8];
  data[index * size + i++] = matrix->a[1];
  data[index * size + i++] = matrix->a[5];
  data[index * size + i++] = matrix->a[9];
  data[index * size + i++] = matrix->a[2];
  data[index * size + i++] = matrix->a[6];
  data[index * size + i++] = matrix->a[10];
}

void CLinearQuaternion::getPoint(double relative_position, void*result)
{
  int i = (int)(relative_position * (len - 1) );
  double pos  = data[0] + relative_position * (data[(len-1)*size] - data[0]);

  // get index lower: data[lower] <= x <= data[upper]
  int lower, higher;

  if(currentPosition >= len-2) {
    lower = len-2;
    higher = len-1;
  }
  else {
    higher = currentPosition+1;
    while(data[higher * size] < pos) {
      higher++;
    }

    lower = higher-1;
  }
  currentPosition = lower;
  i = lower;

  CMatrix*resultMatrix = (CMatrix*)result;

  CMatrix base(data[i * size + 1], data[i * size + 2], data[i * size + 3], 0,
               data[i * size + 4], data[i * size + 5], data[i * size + 6], 0,
               data[i * size + 7], data[i * size + 8], data[i * size + 9], 0,
               0,                  0,                  0,                  1);
  CMatrix transformation;

  if(i >= len-1) {
    *resultMatrix = base;
  }
  else {

    // get rotation axis
    CVec axis(data[i * size +  10], data[i * size +  11], data[i * size +  12]);

    // get linear interpolated rotation angle
    // position 1.32 -> 1 <= 1.32 < 2 -> percent = 0.32 = 1.32 - 1
    double percent = (pos - (double)data[i*size]) / ( (double)data[(i+1)*size] - (double)data[i*size]);
    // percent * angle = linear interpolated angle
    float angle = data[i * size +  13] * percent;

    // transform into rotation matrix
    CMathLib::getMatrixFromRotation(transformation, axis, angle);

    // result = base * T(alpha)
    resultMatrix->mul(base, transformation);
  }
}

void CLinearQuaternion::start()
{
  CMatrix old, current, tmp;
  CVec axis;
  float angle;

  int i, j;
  for(i=0; i<len-1; i++) {
    // old^-1 -> transpose 3x3 part of matrix
    old.set(data[i * size + 1], data[i * size + 4], data[i * size + 7], 0,
            data[i * size + 2], data[i * size + 5], data[i * size + 8], 0,
            data[i * size + 3], data[i * size + 6], data[i * size + 9], 0,
            0,                  0,                  0,                  1);

    // current
    j = i + 1;
    current.set(data[j * size + 1], data[j * size + 2], data[j * size + 3], 0,
                data[j * size + 4], data[j * size + 5], data[j * size + 6], 0,
                data[j * size + 7], data[j * size + 8], data[j * size + 9], 0,
                0,                  0,                  0,                  1);

    // current = old * T
    // old^-1 * current = T
    tmp.mul(old, current);

    // transform matrix into rotationaxis and angle
    CMathLib::getRotationFromMatrix(tmp, axis, angle);

    // store axis
    data[i * size + 10] = axis.x;
    data[i * size + 11] = axis.y;
    data[i * size + 12] = axis.z;

    // store angle
    data[i * size + 13] = (double)angle;
  }
}

CLinearQuaternionLinear::CLinearQuaternionLinear()
{
  size = 1 + 9 + 3 + 1 + 3; // position, 3x3-Matrix, Rotationaxis, Rotationangle, xyz
  currentPosition = 0;
}

void CLinearQuaternionLinear::setPoint(int index, void *m)
{
  CLinearQuaternion::setPoint(index, m);

  CLinearQuaternionData*quaterData = (CLinearQuaternionData*)m;
  CMatrix *matrix = quaterData->pose;

  // store position and 3x3- Matrix, axis and angle will be calculated in start()
  data[index * size + 14] = matrix->a[12];
  data[index * size + 15] = matrix->a[13];
  data[index * size + 16] = matrix->a[14];
}

void CLinearQuaternionLinear::getPoint(double relative_position, void*result)
{
  CLinearQuaternion::getPoint(relative_position, result);

  int i;
  double pos  = data[0] + relative_position * (data[(len-1)*size] - data[0]);

  i = currentPosition;

  CMatrix*resultMatrix = (CMatrix*)result;

  if(i >= len-1) {
    resultMatrix->a[12] = data[(len-1)*size + 14];
    resultMatrix->a[13] = data[(len-1)*size + 15];
    resultMatrix->a[14] = data[(len-1)*size + 16];
  }
  else {
    // x difference
    double xdiff = data[(i+1)*size]-data[i*size];
    double ydiff;
    // y difference
    ydiff = data[(i+1)*size + 14] - data[i*size + 14];
    resultMatrix->a[12] = data[i*size + 14] + (ydiff / xdiff) * (pos - data[i*size]);
    ydiff = data[(i+1)*size + 15] - data[i*size + 15];
    resultMatrix->a[13] = data[i*size + 15] + (ydiff / xdiff) * (pos - data[i*size]);
    ydiff = data[(i+1)*size + 16] - data[i*size + 16];
    resultMatrix->a[14] = data[i*size + 16] + (ydiff / xdiff) * (pos - data[i*size]);
  }
}

/* Interpolation Wrapper
   provides identical access to interpolation in joint and cartesian space */

CInterpolationWrapper::CInterpolationWrapper()
{
  for(int i=0; i<MAXINTERPOLATIONDIM; i++) {
    interpolation[i] = NULL;
  }
  size = 0;
}

CInterpolationWrapper::~CInterpolationWrapper()
{
  for(int i=0; i<MAXINTERPOLATIONDIM; i++) {
    if(interpolation[i] != NULL) {
      delete interpolation[i];
      interpolation[i] = NULL;
    }
  }
  size = 0;
}

void CInterpolationWrapper::setLength(int size)
{
  for(int i=0; i<MAXINTERPOLATIONDIM; i++) {
    if(interpolation[i] != NULL) {
      interpolation[i]->setLength(size);
    }
  }
  this->size = size;
}

void CInterpolationWrapper::setTime(int size)
{
  for(int i=0; i<MAXINTERPOLATIONDIM; i++) {
    if(interpolation[i] != NULL) {
      interpolation[i]->setTime(size);
    }
  }
  this->time = size;
}

void CInterpolationWrapper::start()
{
  for(int i=0; i<MAXINTERPOLATIONDIM; i++) {
    if(interpolation[i] != NULL) {
      interpolation[i]->start();
    }
  }
}

void CInterpolationWrapper::reset()
{
  for(int i=0; i<MAXINTERPOLATIONDIM; i++) {
    if(interpolation[i] != NULL) {
      interpolation[i]->reset();
    }
  }
}

/* Joint Space Wrapper */
CJointSpaceWrapper::CJointSpaceWrapper()
{
}

void CJointSpaceWrapper::setPoint(int index, void *data)
{
  CInterpolationData *ipoData = (CInterpolationData*)data;
  double angle[2];

  angle[0] = ipoData->angles_position;

  for(int i=0; i<AX12_COUNT; i++) {
    angle[1] = (double)ipoData->angles[i];
    interpolation[i]->setPoint(index, angle);
  }
}

void CJointSpaceWrapper::getPoint(double relative_position, void*result)
{
  CInterpolationData *ipoData = (CInterpolationData*)result;
  double angle;

  for(int i=0; i<AX12_COUNT; i++) {
    interpolation[i]->getPoint(relative_position, &angle);
    ipoData->angles[i] = (float)angle;
  }
}

/* Cartesian Space Wrapper */
CCartesianSpaceWrapper::CCartesianSpaceWrapper(CRobot *robot)
{
  this->robot = robot;
}

void CCartesianSpaceWrapper::setPoint(int index, void *data)
{
  CInterpolationData *ipoData = (CInterpolationData*)data;
  CLinearQuaternionData quaterData;
  double tmp[2];

  for(int i=0; i<global.robotInput.kinematicChains.length; i++) {
    // absolute position
    tmp[0] = ipoData->pose_position[i];

    // position
    tmp[1] = ipoData->pose[i][3].x;
    interpolation[i * IPO_CART_COUNT + IPO_CART_X]->setPoint(index, tmp);
    tmp[1] = ipoData->pose[i][3].y;
    interpolation[i * IPO_CART_COUNT + IPO_CART_Y]->setPoint(index, tmp);
    tmp[1] = ipoData->pose[i][3].z;
    interpolation[i * IPO_CART_COUNT + IPO_CART_Z]->setPoint(index, tmp);

    // orientation
    quaterData.pose = &ipoData->pose[i];
    quaterData.position = ipoData->pose_position[i];
    interpolation[i * IPO_CART_COUNT + IPO_CART_ROT]->setPoint(index, &quaterData);
  }
}

void CCartesianSpaceWrapper::getPoint(double relative_position, void*result)
{
  CInterpolationData *ipoData = (CInterpolationData*)result;
  double x, y, z;

  for(int k=0; k<global.robotInput.kinematicChains.length; k++) {
    // get relative position
    relative_position = ipoData->pose_position[k];

    // position
    interpolation[k * IPO_CART_COUNT + IPO_CART_X]->getPoint(relative_position, &x);
    interpolation[k * IPO_CART_COUNT + IPO_CART_Y]->getPoint(relative_position, &y);
    interpolation[k * IPO_CART_COUNT + IPO_CART_Z]->getPoint(relative_position, &z);

    // orientation
    interpolation[k * IPO_CART_COUNT + IPO_CART_ROT]->getPoint(relative_position, &ipoData->pose[k]);

    // add position to matrix
    ipoData->pose[k].a[12] = x;
    ipoData->pose[k].a[13] = y;
    ipoData->pose[k].a[14] = z;

    // calculate inverse kinematics
    robot->calcInverseKinematics(k, ipoData->pose[k]);
  }

  // retrieve angles
  robot->getAnglesFromDh(ipoData->angles);
}

/*
   Interpolation Factory - creates CInterpolationWrapper based on
   type = IPO_*, see enum IPO_TYPE

   add new Interpolation Classes here!!
 */
CInterpolationWrapper*CInterpolationFactory::getWrapper(unsigned int type, CRobot *robot)
{
  CInterpolationWrapper*wrapper;

  if(type < IPO_CARTSPACE) {
    wrapper = new CJointSpaceWrapper();
  }
  else {
    wrapper = new CCartesianSpaceWrapper(robot);
  }

  int i;
  switch(type)
  {
  case IPO_LINEAR:
    for(i=0; i<AX12_COUNT; i++) {
      wrapper->interpolation[i] = new CLinear();
    }
    break;

  case IPO_SPLINE:
    for(i=0; i<AX12_COUNT; i++) {
      wrapper->interpolation[i] = new CSpline();
    }
    break;

  case IPO_BEZIER:
    for(i=0; i<AX12_COUNT; i++) {
      wrapper->interpolation[i] = new CBezier();
    }
    break;

  case IPO_BEZIEROWN:
    for(i=0; i<AX12_COUNT; i++) {
      wrapper->interpolation[i] = new CBezierCustom();
    }
    break;

  case IPO_PTP:
    for(i=0; i<AX12_COUNT; i++) {
      wrapper->interpolation[i] = new CSCurve();
    }
    break;

  case IPO_PTPSINE:
    for(i=0; i<AX12_COUNT; i++) {
      wrapper->interpolation[i] = new CSCurveSine();
    }
    break;

  case IPO_LINEARKART:
    for(i=0; i<global.robotInput.kinematicChains.length; i++) {
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_X] = new CLinear();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Y] = new CLinear();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Z] = new CLinear();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_ROT] = new CLinearQuaternion();
    }
    break;

  case IPO_SPLINEKART:
    for(i=0; i<global.robotInput.kinematicChains.length; i++) {
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_X] = new CSpline();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Y] = new CSpline();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Z] = new CSpline();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_ROT] = new CLinearQuaternion();
    }
    break;

  case IPO_BEZIERKART:
    for(i=0; i<global.robotInput.kinematicChains.length; i++) {
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_X] = new CBezier();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Y] = new CBezier();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Z] = new CBezier();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_ROT] = new CLinearQuaternion();
    }
    break;

  case IPO_BEZIEROWNKART:
    for(i=0; i<global.robotInput.kinematicChains.length; i++) {
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_X] = new CBezierCustom();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Y] = new CBezierCustom();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Z] = new CBezierCustom();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_ROT] = new CLinearQuaternion();
    }
    break;

  case IPO_PTPKART:
    for(i=0; i<global.robotInput.kinematicChains.length; i++) {
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_X] = new CSCurve();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Y] = new CSCurve();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Z] = new CSCurve();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_ROT] = new CLinearQuaternion();
    }
    break;

  case IPO_PTPSINEKART:
    for(i=0; i<global.robotInput.kinematicChains.length; i++) {
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_X] = new CSCurveSine();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Y] = new CSCurveSine();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_Z] = new CSCurveSine();
      wrapper->interpolation[i * IPO_CART_COUNT + IPO_CART_ROT] = new CLinearQuaternion();
    }
    break;
  }
  return wrapper;
}

/*
   Number of control points is last+1
   0 <= factor <= 1
 */

#define MAXBEZIERPOINTS 30

// splits points into segements of size MAXBEZIERPOINTS and
// calcs bezier spline for segment the point factor*last falls into
IPOPRECISION calcBezier(IPOPRECISION *values, int last, IPOPRECISION factor)
{
  //only
  IPOPRECISION buffer[MAXBEZIERPOINTS];
  int i, j, start, len;

  if(last <= MAXBEZIERPOINTS) {
    start = 0;
    len = last + 1;
  }
  else {
    j = (int)(factor * (IPOPRECISION)last);
    start = j - (j % MAXBEZIERPOINTS);

    if(start + MAXBEZIERPOINTS > last) {
      len = last - start + 1;
    }
    else {
      len = MAXBEZIERPOINTS;
    }

    factor = factor - ( (IPOPRECISION)start)/( (IPOPRECISION)last);

    factor *= ( (IPOPRECISION)last+1.0)/( (IPOPRECISION)MAXBEZIERPOINTS);
  }

  memcpy(buffer, &values[start], len*sizeof(IPOPRECISION) );
  for(i=1; i<=len; i++) {
    for(j=0; j<len-i; j++) {
      buffer[j] = (1.0-factor)*buffer[j] + factor*buffer[j+1];
    }
  }

  return(buffer[0]);
}

/*
   Linear Interpolation
 */
void linear_subsequent(IPOPRECISION *xin, IPOPRECISION *yin, int n, IPOPRECISION x, IPOPRECISION *y, int &old)
{
  int lower, upper;
  IPOPRECISION ydiff, xdiff, a;

  if(old >= n-2) {
    lower = n-2;
    upper = n-1;
  }
  else {
    upper = old+1;
    while(xin[upper] <= x) {
      upper++;
    }
    old = lower = upper-1;
  }

  xdiff = xin[upper]-xin[lower];
  ydiff = yin[upper]-yin[lower];
  *y = yin[lower] + (ydiff / xdiff) * (x - xin[lower]);
}

/*
   Spline Interpolation
 */
 
 void spline_prepare(IPOPRECISION *x, IPOPRECISION *y, IPOPRECISION *u, IPOPRECISION *y2, int n)
{
	int i,k;
	IPOPRECISION p,qn,sig,un;
	
	y2[0]=u[0]=0.0; 
	
	for (i=1;i<=n-2;i++)  // This is the decomposition loop of the tridiagonal algorithm.
	{						// y2 and u are used for temporary storage of the decomposed factors.
		sig=(x[i]-x[i-1])/(x[i+1]-x[i-1]);
		p=sig*y2[i-1]+2.0;
		y2[i]=(sig-1.0)/p;
		u[i]=(y[i+1]-y[i])/(x[i+1]-x[i]) - (y[i]-y[i-1])/(x[i]-x[i-1]);
		u[i]=(6.0*u[i]/(x[i+1]-x[i-1])-sig*u[i-1])/p;
	}
	
    qn=un=0.0; 

	y2[n-1]=(un-qn*u[n-2])/(qn*y2[n-2]+1.0);
	
	for (k=n-2;k>=0;k--)			// This is the backsubstitution loop of the tridiagonal
	y2[k]=y2[k]*y2[k+1]+u[k]; 		// algorithm.
}


/* slightly modified version of splint, faster for subsequent calls */
void spline_subsequent(IPOPRECISION *xa, IPOPRECISION *ya, IPOPRECISION *y2a, IPOPRECISION *ho, int n, IPOPRECISION x, IPOPRECISION *y, int &old)
{
	int klo,khi,k;
	IPOPRECISION h,b,a;
	
	if (old >= n-2)
	{
       klo = n-2;
       khi = n-1;
    } else
	{
	   khi = old+1;
	   while (xa[khi] < x)
	         khi++;
       klo = khi-1; 
    }
	h=xa[khi]-xa[klo];
	
	a=(xa[khi]-x)/h; 
	b=(x-xa[klo])/h; //Cubic spline polynomial is now evaluated.
	*y=a*ya[klo]+b*ya[khi]+((a*a*a-a)*y2a[klo]+(b*b*b-b)*y2a[khi])*(h*h)/6.0;
	
	old = klo;
}




void CInterpolationData::malloc()
{
  pose = new CMatrix[global.robotInput.kinematicChains.length];
  if(pose == NULL) {
    CUtil::cout("play: malloc failed().\n", TEXT_ERROR);
    return;
  }
  pose_position = (double*)std::malloc(global.robotInput.kinematicChains.length* sizeof(double) );
  if(pose_position == NULL) {
    CUtil::cout("CInterpolationData: malloc failed().\n", TEXT_ERROR);
    return;
  }
}

void CInterpolationData::free()
{
  if(pose != NULL) {;}
  delete[] pose;

  if(pose_position != NULL) {
    std::free(pose_position);
  }

  pose = NULL;
  pose_position = NULL;
}

/*
 #include <math.h>
 #include "../include/interpolation.h"
 */

/*! \brief Bézier-Spline Berechnung (Praktikum)

   \param values Array der Zwischenpunkte (z.B. Winkel oder die x-Koordinate)
   \param len Anzahl der Zwischenpunkte
   \param factor Wert zwischen 0 und 1, der die aktuelle Position auf dem Spline angibt (vergangene Zeit/Gesamtzeit)
 */
IPOPRECISION calcBezierCustom(IPOPRECISION values[], int len, IPOPRECISION factor)
{
  // Simple Lineare Interpolation als Beispiel
  int lower, upper, last;
  IPOPRECISION ydiff, xdiff;

  last = len-1;
  lower = (int)(factor * last);  // cast by mdda

  if(lower >= last) {
    return values[last];
  }

  upper = lower+1;

  ydiff = values[upper] - values[lower];
  xdiff = factor*last - lower;
  return values[lower] + ydiff * xdiff;
}

/*
   S-Curve Interpolation Class

   start() - call before getPoint(), calculates acceleration start and end times, max velocity, max speed
 */
CSCurve::CSCurve()
{
  size = 7; // store x, y, te, sgn, tb, vmax, bmax
  sprintf(name, "PTP");
}

void CSCurve::getPoint(double relative_position, void*result)
{
  IPOPRECISION se, t, pos;
  float current, target;
  IPOPRECISION *te = &data[2*len];
  IPOPRECISION *sgn = &data[3*len];
  IPOPRECISION *tb = &data[4*len];
  IPOPRECISION *vmax = &data[5*len];
  IPOPRECISION *bmax = &data[6*len];

  if(currentPosition == len -1) {
    *(double*)result = data[len-1 + 1*len];
    return;
  }
  pos = getRelativePosition(&data[0 + 0*len], len, relative_position, currentPosition);
//printf("%d %g\n", currentPosition, pos);
  current = data[1*len + currentPosition];
  target = data[1*len + currentPosition + 1];

  if(bmax[currentPosition] > 0.00001) {
    t = pos * te[currentPosition];
    if(t <= tb[currentPosition]) {
      se = 0.5 * bmax[currentPosition] * t * t;
    }
    else if(t <= te[currentPosition] - tb[currentPosition]) {
      se = (vmax[currentPosition] * t - 0.5 * vmax[currentPosition] * vmax[currentPosition] / bmax[currentPosition]);
    }
    else if(t <= te[currentPosition]) {
      se = (vmax[currentPosition] * (te[currentPosition] - tb[currentPosition]) - 0.5 * bmax[currentPosition] * (te[currentPosition] - t) * (te[currentPosition] - t) );
    }
    else {se = fabsf(target - current);}

    current +=  (sgn[currentPosition] * se);
  }

  *( (double*)result) = current;
}

// interpolation preparation
void CSCurve::start()
{
  IPOPRECISION *tes = &data[2*len];
  IPOPRECISION *sgns = &data[3*len];
  IPOPRECISION *tbs = &data[4*len];
  IPOPRECISION *vmaxs = &data[5*len];
  IPOPRECISION *bmaxs = &data[6*len];
  int i;
  float current, target;

  IPOPRECISION TIME = time;//len * global.motion.ipoPause;
  IPOPRECISION VMAX = parameters.vmax;
  IPOPRECISION BMAX = parameters.bmax;

printf("%d %g\n", len, TIME);
  IPOPRECISION vmax, bmax, se;
  long int te, tb, tv, sgn;
  int ipo = global.motion.ipoPause;

  for(i=0; i<len-1; i++) {
    target = data[1*len + i + 1];
    current = data[1*len + i];

    vmax = VMAX;
    bmax = BMAX;
    se = (IPOPRECISION)(target - current);
    if(se < 0) {
      sgn = -1;
      se = -se;
    }
    else { sgn = 1;}

    if(vmax*vmax > bmax * se) {
      vmax =  sqrt(bmax * se);

      tb = ( (long int)(vmax / (bmax * (IPOPRECISION)ipo) ) ) * ipo;
      tv = (long int)(TIME - tb);
    }
    else {
      tb = (long int)(vmax / (bmax * (IPOPRECISION)ipo) + 0.5) * ipo;
      tv = (long int)(TIME - tb);
    }

    if(tb <= 0) {
      tb = 1;
    }
    if(tv <= 0) {
      tv = 1;
    }

    te = tv + tb;
    vmax = se / (IPOPRECISION)tv;
    bmax = vmax / (IPOPRECISION)tb;

    tes[i] = te;
    sgns[i] = sgn;
    tbs[i] = tb;
    vmaxs[i] = vmax;
    bmaxs[i] = bmax;
  }

  tes[len-1] = 0;
  sgns[len-1] = 0;
  tbs[len-1] = 0;
  vmaxs[len-1] = 0;
  bmaxs[len-1] = 0;

  /*
     switch (param)
     {
     case 1:
     // synchronous ptp

     // get max total time
     long int temax = 0;
     for (i=0; i<AX12_COUNT; i++)
          if (tes[i] > temax)
           temax = (long int)tes[i];

     // recalculate max velocity and acceleration times
     for (i=0; i<AX12_COUNT; i++)
     {
       target = writeBuffer[i * AX12_DATA_WRITE + 0] + 256 * writeBuffer[i * AX12_DATA_WRITE + 1];
       current = readBuffer[i * AX12_DATA_WRITE + 0] + 256 * readBuffer[i * AX12_DATA_WRITE + 1];

        se = (target - current);
        if (se < 0)
        {
           se = -se;
        }

        bmax = bmaxs[i];
        vmax = bmax * temax;
        vmax = 0.5 * vmax - sqrt(0.25*vmax*vmax - se * bmax);

        tv = ((long int)( se / (vmax * ipo) + 0.5)) * ipo;
        tb = temax - tv;
        vmax = se / (float)tv;
        bmax = vmax / (float)tb;

        tes[i] = temax;
        tbs[i] = tb;
        vmaxs[i] = vmax;
        bmaxs[i] = bmax;
     }
     break;
     }*/
}

/*
   S-Curve Interpolation Class

   start() - call before getPoint(), calculates acceleration start and end times, max velocity, max speed
 */
CSCurveSine::CSCurveSine()
{
  size = 7; // store x, y, te, sgn, tb, vmax, bmax
  sprintf(name, "PTP (Sinoidal)");
}

void CSCurveSine::getPoint(double relative_position, void*result)
{
  IPOPRECISION se, t, pos;
  float current;
  IPOPRECISION *te = &data[2*len];
  IPOPRECISION *sgn = &data[3*len];
  IPOPRECISION *tb = &data[4*len];
  IPOPRECISION *vmax = &data[5*len];
  IPOPRECISION *bmax = &data[6*len];

  if(currentPosition == len -1) {
    *(double*)result = data[len-1 + 1*len];
    return;
  }

  pos = getRelativePosition(&data[0 + 0*len], len, relative_position, currentPosition);

  current = data[1*len + currentPosition];

  t = pos * te[currentPosition];
  IPOPRECISION tv = te[currentPosition] - tb[currentPosition];
  if(t <= tb[currentPosition]) {
    se = bmax[currentPosition] * (0.25 * t * t + (tb[currentPosition] * tb[currentPosition] / (8.0 * M_PI * M_PI) ) * (cos(2.0*M_PI*t/tb[currentPosition])-1.0) );
  }
  else if(t <= te[currentPosition] - tb[currentPosition]) {
    se = vmax[currentPosition] * (t - 0.5 * tb[currentPosition]);
  }
  else {
    se = 0.5 * bmax[currentPosition] * (te[currentPosition] * (t + tb[currentPosition]) - 0.5*(t*t + te[currentPosition]*te[currentPosition] + 2.0 * tb[currentPosition]*tb[currentPosition]) + (tb[currentPosition]*tb[currentPosition]/(4.0 * M_PI * M_PI) )*(1.0 - cos( (2.0*M_PI/tb[currentPosition])*(t - tv) ) ) );
  }

  current +=  (sgn[currentPosition] * se);

  *( (double*)result) = current;
}

// interpolation preparation
void CSCurveSine::start()
{
   /*  parameters.vmax = global.parameters[100];
     parameters.bmax = global.parameters[101];
     */
     
  IPOPRECISION *tes = &data[2*len];
  IPOPRECISION *sgns = &data[3*len];
  IPOPRECISION *tbs = &data[4*len];
  IPOPRECISION *vmaxs = &data[5*len];
  IPOPRECISION *bmaxs = &data[6*len];
  int i;
  double current, target;

  IPOPRECISION TIME = time;//len * global.motion.ipoPause;
  IPOPRECISION VMAX = parameters.vmax;
  IPOPRECISION BMAX = parameters.bmax;



  IPOPRECISION vmax, bmax, se;
  int te, tb, tv, sgn;
  int ipo = global.motion.ipoPause;

  for(i=0; i<len-1; i++) {
    target = data[1*len + i + 1];
    current = data[1*len + i];

    vmax = VMAX;
    bmax = BMAX;
    se = (target - current);
    if(se < 0) {
      sgn = -1;
      se = -se;
    }
    else { sgn = 1;}

    if(vmax*vmax > bmax * se * 0.5) {
      vmax =  sqrt(bmax * se * 0.5);

      tb = ( (long int)(2.0 * vmax / (bmax * ipo) ) ) * ipo;
      tv = (long int)(TIME - tb);
    }
    else {
      tb = (int)( ( (long int)2.0 * vmax / (bmax * (IPOPRECISION)ipo) + 0.5) * ipo); // cast by mdda
      tv = (long int)(TIME - tb);
    }

    if(tb == 0) {
      tb = 1;
    }
    if(tv == 0) {
      tv = 1;
    }

    te = tv + tb;
    vmax = se / (IPOPRECISION)tv;
    bmax = 2.0 * vmax / (IPOPRECISION)tb;

    printf("tb: %d tv: %d te: %d vmax: %g bmax: %g total: %g\n", tb, tv, te, vmax, bmax, se);

    tes[i] = te;
    sgns[i] = sgn;
    tbs[i] = tb;
    vmaxs[i] = vmax;
    bmaxs[i] = bmax;

    /*
       if (se > 0)
       printf("%g %d %d %d\n", se, tb, tv, ipo);

       if (se > 0)
       printf("te: %g sgn: %g tb: %g v: %g b: %g\n",
       tes[i], sgns[i], tbs[i], vmaxs[i], bmaxs[i]);
     */
  }
  tes[len-1] = 0;
  sgns[len-1] = 0;
  tbs[len-1] = 0;
  vmaxs[len-1] = 0;
  bmaxs[len-1] = 0;
}

