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


#ifndef __MATH

#define __MATH

#include <math.h>

#define PRECISION double // todo: use template instead

class CDh;

/*! \brief Homogenous vector
 */
class CVec
{
public:
  PRECISION x, y, z, w;

  CVec();
  CVec(PRECISION x, PRECISION y, PRECISION z);
  void set(PRECISION x, PRECISION y, PRECISION z);

  /*! \brief Prints vector as (x,y,z) to console
   */
  void print();

  /*! \brief Normalizes the vector
   */
  void normalize();
  
  /*! \brief Returns length of vector
   */
  PRECISION length();

  /*! \brief Scalar multiplication
   */
  CVec operator *(PRECISION s) const;

  /*! \brief Assigns values of vector \a v to vector
   */
  CVec&operator =(const CVec&v);

  /*! \brief Array access (read)
   */
  PRECISION operator [](unsigned int i) const;

  /*! \brief Array access (write)
   */
  PRECISION&operator [](unsigned int i);

  /*! \brief Vector addition
   */
  CVec operator +(const CVec&v) const;

  /*! \brief Vector difference
   */
  CVec operator -(const CVec&v) const;

  /*! \brief Vector negation
   */
  CVec operator -() const;

  /*! \brief Dot product
   */
  PRECISION operator |(const CVec&v) const;

  /*! \brief Cross product
   */
  CVec operator ^(const CVec&v) const;

};

/*! \brief Homogenous matrix
 */
class CMatrix
{
public:
  PRECISION a[16];
  CMatrix();
  CMatrix(
    PRECISION a0, PRECISION a4, PRECISION a8,  PRECISION a12,
    PRECISION a1, PRECISION a5, PRECISION a9,  PRECISION a13,
    PRECISION a2, PRECISION a6, PRECISION a10, PRECISION a14,
    PRECISION a3 = 0.0f, PRECISION a7 = 0.0f, PRECISION a11 = 0.0f, PRECISION a15 = 1.0f);
  void set(
    PRECISION a0, PRECISION a4, PRECISION a8,  PRECISION a12,
    PRECISION a1, PRECISION a5, PRECISION a9,  PRECISION a13,
    PRECISION a2, PRECISION a6, PRECISION a10, PRECISION a14,
    PRECISION a3 = 0.0f, PRECISION a7 = 0.0f, PRECISION a11 = 0.0f, PRECISION a15 = 1.0f);

  /*! \brief Trace - sum of diagonal elements - of matrix
   */
  PRECISION trace();

  /*! \brief Transposes the matrix
   */
  void transpose();

  /*! \brief Inverts the matrix (only for homogenous matrices)
   */
  void invert();

  /*! \brief Assigns product of two matrices to matrix
   */
  void mul(const CMatrix &first, const CMatrix &second);

  /*! \brief Prints matrix to console
   */
  void print(bool round = false);

  /*! \brief Creates Denavit Hartenberg matrix
   */
  void setDh(const CDh &dh);

  /*! \brief Updates variable parts of the Denavit Hartenberg matrix
   */
  void updateDh(const CDh &dh);

  /*! \brief Scalar multiplication
   */
  CMatrix operator *(PRECISION f) const;

  /*! \brief Matrix addition
   */
  CMatrix operator +(const CMatrix &b) const;

  /*! \brief Matrix subtraction
   */
  CMatrix operator -(const CMatrix &b) const;

  /*! \brief Matrix multiplication
   */
  CMatrix operator *(const CMatrix &m) const;

  /*! \brief Overwrites matrix with sum of matrix and vector
   */
  CMatrix&operator +=(const CVec &v);

  /*! \brief Matrix vector product
   */
  CVec operator *(const CVec &v) const;

  /*! \brief Array access (read&write)
   */
  const CVec&operator [](int i) const;

  /*! \brief Assigns values of matrix \a v to matrix
   */
  CMatrix&operator =(const CMatrix&v);
  
  PRECISION length();

};

/*! \brief Rounds a value
 */
int rnd(PRECISION value);

#endif

