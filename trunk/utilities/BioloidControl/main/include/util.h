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


#ifndef __UTIL

#define __UTIL

#include <stdlib.h>
#include "types.h"
#include "tinyxml.h"
#include "robot.h"
#include "vecmath.h"
#include "cmd.h"


#define ROTX 0.0
#define ROTY 0.0
#define ROTZ 0.0

/*! \brief Mathematical functions
 */
class CMathLib
{
public:
  /*! \brief Calculates dot product of two double vectors of the stated \a size
   */
  static double calcDotProduct(double firstVector[], double secondVector[], int size);

  /*! \brief Calculates vector-matrix product

     \param resultVector Has to be allocated
   */
  static void calcMatrixResult(double matrix[], double vector[], int rows, int columns, double resultVector[]);

  /*! \brief Transforms homogenous matrix into axis-angle representation
   */
  static void getRotationFromMatrix(CMatrix &mat, CVec &axis, float &angle);

  /*! \brief Transposes a matrix

     Set \a resultMatrix to \a matrix if you want to transpose the matrix

     \param resultMatrix Has to be allocated
   */
  static void transposeMatrix(double *matrix, double *resultMatrix, int size);

  static void printMatrix(double *matrix, int rows, int columns);
  /*! \brief Multiplies two matrices

     \param resultMatrix Has to be allocated
   */
  static void multiplyMatrices(double *firstMatrix, double *secondMatrix, int firstRows, int firstColumns, int secondColumns, double *resultMatrix);

  /*! \brief Transforms a homogenous matrix into quaternion representation

     \param quaternion XYZ is the rotation axis part, W the angle part
   */
  static void quaternionFromMatrix(CMatrix &mat, CVec &quaternion);

  /*! \brief Transforms a quaternion into homogenous matrix representation
   */
  static void matrixFromQuaternion(CVec &quaternion, CMatrix &matrix);

  /*! \brief Transforms axis-angle representation into homogenous matrix representation
   */
  static void getMatrixFromRotation(CMatrix &mat, CVec &axis, float angle);

  /*! \brief Transforms homogenous matrix into Euler angle (YZX) representation (two solutions)
   */
  static void getOrientation(CMatrix &mat, CVec &first, CVec &second);

  /*! \brief ransforms Euler angle (YZX) representation into homogenous matrix representation

     \see getRotation
   */
  static void getRotation(CMatrix &mat, CVec &vec);

  /*! \brief Transforms Euler angle (YZX) representation into homogenous matrix representation
   */
  static void getRotation(CMatrix &mat, float x, float y, float z);

  /*! \brief Calculates inverse kinematics of a standard two link leg

     \param leg1 Length of leg connected to base
     \param leg2 Length of leg connected to hand
     \param x Cartesian x position of hand
     \param y Cartesian y position of hand
     \param first Angle between base and leg1
     \param second Angle between leg1 and leg2 (elbow)
   */
  static void calcAngles(float leg1, float leg2, float x, float y, float &first, float &second);

};

/*! \brief Utility functions
 */
class CUtil
{
public:
  /*! \brief Copies \a count bytes from \a source to \a destintation
   */
  static void *copymem(void *destination, const void *source, int count);

  /*! \brief Custom string to double conversion function
   */
  static double strtodouble(char *text, int len, bool*error = NULL);

  /*! \brief Custom truncate (from left) string function
   */
  static char*strtrim(char*text);

  /*! \brief Custom truncate (from right) string function
   */
  static char*strtrimright(char*text);

  /*! \brief Frees a pointer and sets it to NULL
   */
  static void free(void**var);

  /*! \brief Resizes a pointer
   */
  static void*realloc(void*var, size_t size);

  /*! \brief Resynchs the serial communication if error occured
   */
  static void resync();

  /*! \brief Prints colored text to the console
   */
  static void cout(const char*text, int type = TEXT_NORMAL);

  /*! \brief Blocks until the return key is pressed
   */
  static void waitForReturn(); // This is defined in cmd.cpp

  /*! \brief Transforms 10bit angle representation to float representation
   */
  static float wordToAngle(byte lo, byte hi);

  /*! \brief Transforms float angle representation to 10bit representation
   */
  static void angleToWord(float angle, byte &lo, byte &hi);

  /*! \brief Calculates jacobian

     \param targetHand The target pose of the hand
     \param currentLinks The current poses of all links (array)
     \param size Size of \a currentLinks
   */
  static CMatrix getJacobian(CMatrix *currentLinks, int size, CMatrix &targetHand);

};

void runTest(char*params[], int paramcount);

#endif
