/**************************************************************************

    Copyright 2007, 2008 Rainer J�kel <rainer.jaekel@googlemail.com>

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


#include <cstdlib>
#include <iostream>
#include <cstring>
#include <math.h>

#define TINY 1.0e-20;
#define SWAP(x, y) tmp = a[x]; a[x]=a[y]; a[y] = tmp;

#include "../include/platform.h"
#include "../include/types.h"
#include "../include/util.h"
#include "../include/motion.h"
#include "../include/robot.h"
#include "../include/cmd.h"
#include "../include/vars.h"



// add text to log  - make text const (mdda)
void CUtil::cout(const char*text, int type) {
  if(global.initialisationComplete) {
    global.console.addLine( (char *)text, type);
    global.console.printLog();
  }
  else {
    std::cout << text;
  }
}

// alloc, realloc memory
void*CUtil::realloc(void*var, size_t size) {
  void*tmp = NULL;

  if(size > 0) {
    if(var == NULL) {
      tmp = malloc(size);
    }
    else {
      tmp = std::realloc(var, size);
    }
  }

  if( (tmp == NULL) && (var != NULL) ) {
    std::free(var);
  }

  return tmp;
}

// free memory
void CUtil::free(void**var) {
  if(*var != NULL) {
    std::free(*var);
  }
  *var = NULL;
}

// word = byte[2] to angle
float CUtil::wordToAngle(byte lo, byte hi) {
  return (512.0 - (float)(lo + 256 * hi) ) * 150.0 / 512.0;
}

void CUtil::angleToWord(float angle, byte &lo, byte &hi) {
  angle = 512.0 - angle * 512.0 / 150.0 + 0.5;
  word tmp = (word)angle;
  lo = (byte)(tmp & 0x00FF);
  hi = (byte)(tmp >> 8);
}

// calcs inertia matrix of aabb 'box'
void calcInertiaMatrix(CBox box, CMatrix &mat, float mass) {
  // b = x c = z a = y
  float tmpf = mass / 12.0;
  float a = box.extents.y * 2.0 / 1000.0;
  float b = box.extents.x * 2.0 / 1000.0;
  float c = box.extents.z * 2.0 / 1000.0;
  CMatrix tmpMatrix, tmpMatrix2;

  tmpMatrix.set(tmpf*(b*b+c*c), 0.0,             0.0,             0.0,
                0.0,            tmpf*(a*a+c*c),  0.0,             0.0,
                0.0,            0.0,             tmpf*(a*a+b*b),  0.0,
                0.0,            0.0,             0.0,             1.0);

  return;

  // steiner -> rotationszentrum != schwerpunkt
  CMatrix steiner;
  a = box.center.x;
  b = box.center.y;
  c = box.center.z;

  steiner.set(b*b+c*c,   -a*b,      -a*c,      0.0,
              -a*b,      a*a+c*c,   -b*c,      0.0,
              -a*c,      -b*c,      a*a+b*b,   0.0,
              0.0,       0.0,       0.0,       1.0);

  tmpMatrix2 = steiner * box.mass;
  mat = tmpMatrix + tmpMatrix2;
}

// calculates rotation matrix from quaternion
// http://www.flipcode.com/documents/matrfaq.html
void CMathLib::matrixFromQuaternion(CVec &quaternion, CMatrix &matrix)
{
  float xx, xy, xz, xw, yy, yz, yw, zz, zw;
  float X = quaternion.x;
  float Y = quaternion.y;
  float Z = quaternion.z;
  float W = quaternion.w;
  xx      = X * X;
  xy      = X * Y;
  xz      = X * Z;
  xw      = X * W;

  yy      = Y * Y;
  yz      = Y * Z;
  yw      = Y * W;

  zz      = Z * Z;
  zw      = Z * W;

  matrix.a[0]  = 1.0 - 2.0 * (yy + zz);
  matrix.a[4]  =     2.0 * (xy - zw);
  matrix.a[8]  =     2.0 * (xz + yw);

  matrix.a[1]  =     2.0 * (xy + zw);
  matrix.a[5]  = 1.0 - 2.0 * (xx + zz);
  matrix.a[9]  =     2.0 * (yz - xw);

  matrix.a[2]  =     2.0 * (xz - yw);
  matrix.a[6]  =     2.0 * (yz + xw);
  matrix.a[10] = 1.0 - 2.0 * (xx + yy);

  matrix.a[3]  = matrix.a[7] = matrix.a[11] = matrix.a[12] = matrix.a[13] = matrix.a[14] = 0.0;
  matrix.a[15] = 1.0;
}

// calculates matrix from rotation axis and rotation angle
// http://www.flipcode.com/documents/matrfaq.html
void CMathLib::getMatrixFromRotation(CMatrix &mat, CVec &axis, float angle)
{
  CVec tmp, quater;
  float sin_a = sin(angle / 2.0);
  float cos_a = cos(angle / 2.0);

  tmp.x = axis.x * sin_a;
  tmp.y = axis.y * sin_a;
  tmp.z = axis.z * sin_a;
  tmp.w = cos_a;

  // normalisieren
  float tmpf = 1.0/sqrt(tmp.x*tmp.x+
                        tmp.y*tmp.y+
                        tmp.z*tmp.z+
                        tmp.w*tmp.w);

  //tmpf = 1.0;

  quater.x = tmp.x * tmpf;
  quater.y = tmp.y * tmpf;
  quater.z = tmp.z * tmpf;
  quater.w = tmp.w * tmpf;

  matrixFromQuaternion(quater, mat);
}

// calculates rotation axis and angle from rotation matrix
// http://www.flipcode.com/documents/matrfaq.html
void CMathLib::getRotationFromMatrix(CMatrix &mat, CVec &result, float &angle)
{
  CVec quat, tmp;
  double rotangle, tmpf, cos_a, sin_a;
  quaternionFromMatrix(mat, tmp);

  // normalisieren
  tmpf = 1.0/sqrt(tmp.x*tmp.x+
                  tmp.y*tmp.y+
                  tmp.z*tmp.z+
                  tmp.w*tmp.w);

  quat = tmp * tmpf;
  quat.w = tmp.w * tmpf;

  cos_a = quat.w;
  rotangle = acos(cos_a) * 2;
  sin_a = sqrt(1.0 - cos_a * cos_a);
  if(fabs(sin_a) < 0.0005) { sin_a = 1;}

  result.x = quat.x / sin_a;
  result.y = quat.y / sin_a;
  result.z = quat.z / sin_a;

  angle = rotangle;

}

// calculates matrix[n x m] * vector [m]
void CMathLib::calcMatrixResult(double a[], double b[], int n, int m, double result[])
{
  int i, j;
  for(i=0; i<n; i++) {
    result[i] = 0.0;
    for(j=0; j<m; j++) {
      result[i] += a[i+j*n]*b[j];
    }
  }
}

// calculates dot product a[n] * b[n]
double CMathLib::calcDotProduct(double a[], double b[], int n)
{
  int i;
  double tmp = a[0]*b[0];
  for(i=1; i<n; i++) {
    tmp += a[i]*b[i];
  }

  return tmp;
}

// calculates quaternion from rotation matrix
// http://www.flipcode.com/documents/matrfaq.html
void CMathLib::quaternionFromMatrix(CMatrix &mat, CVec &quaternion)
{
  float T = 1+mat.trace();
  float S, X, Y, Z, W;

  if(T > 0.00000001) {
    S = 2 * sqrt(T);
    X = (mat.a[6] - mat.a[9])/S;
    Y = (mat.a[8] - mat.a[2])/S;
    Z = (mat.a[1] - mat.a[4])/S;
    W = S*0.25;
  }
  else
  if(mat.a[0] > mat.a[5] && mat.a[0] > mat.a[10]) {   // Column 0:
    S  = sqrt(1.0 + mat.a[0] - mat.a[5] - mat.a[10]) * 2;
    X = 0.25 * S;
    Y = (mat.a[4] + mat.a[1]) / S;
    Z = (mat.a[2] + mat.a[8]) / S;
    W = (mat.a[6] - mat.a[9]) / S;
  }
  else if(mat.a[5] > mat.a[10]) {            // Column 1:
    S  = sqrt(1.0 + mat.a[5] - mat.a[0] - mat.a[10]) * 2;
    X = (mat.a[4] + mat.a[1]) / S;
    Y = 0.25 * S;
    Z = (mat.a[9] + mat.a[6]) / S;
    W = (mat.a[8] - mat.a[2]) / S;
  }
  else {                        // Column 2:
    S  = sqrt(1.0 + mat.a[10] - mat.a[0] - mat.a[5]) * 2;
    X = (mat.a[2] + mat.a[8]) / S;
    Y = (mat.a[9] + mat.a[6]) / S;
    Z = 0.25 * S;
    W = (mat.a[1] - mat.a[4]) / S;
  }

  quaternion.x = X;
  quaternion.y = Y;
  quaternion.z = Z;
  quaternion.w = W;
}

// transposes matrix a[n x n]
void CMathLib::transposeMatrix(double *a, double *result, int n)
{
  int i, j;
  double tmp;
  for(i=0; i<n; i++) {
    for(j=0; j<i; j++) {
      tmp = a[i*n+j];
      result[i*n+j] = a[j*n+i];
      result[j*n+i] = tmp;
    }
  }
}

void CMathLib::printMatrix(double *matrix, int rows, int columns)
{
  char text[1024];
  for(int i=0; i<rows; i++) {
    for(int j=0; j<columns; j++) {
      sprintf(text, "%2.2f ", matrix[i*columns + j]);
      CUtil::cout(text);
    }
    CUtil::cout("\n");
  }
}
// multiplies two matrix a[n x m] * b [m x k]
void CMathLib::multiplyMatrices(double *a, double *b, int n, int m, int k, double *res)
{
  int i, j, l;
  for(i=0; i<n; i++) {
    for(j=0; j<k; j++) {
      res[i*n + j] = 0.0;
      for(l=0; l<m; l++) {
        res[i*n + j] += a[i*n + l] * b[l*m + j];
      }
    }
  }
}

// calculates euler angles representing rotation matrix
void CMathLib::getOrientation(CMatrix &mat, CVec &first, CVec &second)
{
          float a, b, g;
     
     b = atan2(-mat.a[2], sqrt(mat.a[0]*mat.a[0]+mat.a[1]*mat.a[1]));

     if (fabsf(b - M_PI/2.0) < 0.0001)
     {
        a = 0;
        g = atan2(mat.a[4], mat.a[5]);        
     } else if (fabsf(b + M_PI/2.0) < 0.0001)
     {
        a = 0;
        g = -atan2(mat.a[4], mat.a[5]);        
     } else
     {
        a = atan2(mat.a[1]/cos(b), mat.a[0]/cos(b));
        g = atan2(mat.a[6]/cos(b), mat.a[10]/cos(b));   
     }
     
     first.set(g, b, a);
     second = first;
     /*
  float a, b, g;

  b = atan2(-mat.a[4], sqrt(mat.a[5]*mat.a[5]+mat.a[6]*mat.a[6]) );

  if(fabs(b - M_PI/2.0) < 0.0001) {
    g = 0;
    a = atan2(mat.a[9], mat.a[10]);
  }
  else if(fabs(b + M_PI/2.0) < 0.0001) {
    g = 0;
    a = atan2(-mat.a[9], mat.a[10]);
  }
  else {
    a = atan2(mat.a[8]/cos(b), mat.a[0]/cos(b) );
    g = atan2(mat.a[6]/cos(b), mat.a[5]/cos(b) );
  }

  first.set(g, a, b);
  second = first;*/
}

// convenience function
void CMathLib::getRotation(CMatrix &mat, CVec &vec)
{
  getRotation(mat, vec.x, vec.y, vec.z);
}

// calculates rotation matrix representing euler angles
void CMathLib::getRotation(CMatrix &mat, float aX, float aY, float aZ)
{
    float g = aX;
    float b = aY; 
    float a = aZ; 
                                          
    if (fabsf(b - M_PI/2.0) < 0.0001)
    {
        mat.a[0] = 0.0;                          
        mat.a[1] = 0.0;        
        mat.a[2] = -1.0;      
        mat.a[4] = sin(g - a);                                
        mat.a[5] = cos(g - a);                          
        mat.a[6] = 0.0;                           
    
        mat.a[8] = cos(g - a);                           
        mat.a[9] = -sin(g - a);      
        mat.a[10] = 0.0;   
    } else
        if (fabsf(b + M_PI/2.0) < 0.0001)
    {
        mat.a[0] = 0.0;                          
        mat.a[1] = 0.0;        
        mat.a[2] = 1.0;      
        mat.a[4] = -sin(g + a);                                
        mat.a[5] = cos(g + a);                          
        mat.a[6] = 0.0;                           
    
        mat.a[8] = -cos(g + a);                           
        mat.a[9] = -sin(g + a);      
        mat.a[10] = 0.0;  
    } else
    {    
    mat.a[0] = cos(b)*cos(a);                          
    mat.a[1] = sin(a)*cos(b);        
    mat.a[2] = -sin(b);      
    mat.a[4] = cos(a)*sin(b)*sin(g) - sin(a)*cos(g);                                
    mat.a[5] = sin(a)*sin(b)*sin(g) + cos(a)*cos(g);                          
    mat.a[6] = cos(b)*sin(g);                           
    
    mat.a[8] = cos(a)*sin(b)*cos(g)+sin(a)*sin(g);                           
    mat.a[9] = sin(a)*sin(b)*cos(g)-cos(a)*sin(g);      
    mat.a[10] = cos(b)*cos(g); 
    }
     /*
  float a = aX;
  float b = aZ;
  float g = aY;
  mat.a[0] = cos(b)*cos(g);
  mat.a[1] = cos(a)*sin(b)*cos(g) + sin(a)*sin(g);
  mat.a[2] = sin(a)*sin(b)*cos(g)-cos(a)*sin(g);
  mat.a[4] = -sin(b);
  mat.a[5] = cos(a)*cos(b);
  mat.a[6] = sin(a)*cos(b);

  mat.a[8] = cos(b)*sin(g);
  mat.a[9] = cos(a)*sin(b)*sin(g)-sin(a)*cos(g);
  mat.a[10] = sin(a)*sin(b)*sin(g)+cos(a)*cos(g);
  mat.a[12] = 0.0; mat.a[13] = 0.0; mat.a[14] = 0.0;
  mat.a[3] = 0.0; mat.a[7] = 0.0; mat.a[11] = 0.0; mat.a[15] = 1.0;
  */
}

// calculates inverse kinematics of a two bone leg
void CMathLib::calcAngles(float leg1, float leg2, float x, float y, float &first, float &second)
{
  float lambda;

  // normalize x and y
  const float factor = 0.9999;
  if( (x*x +y*y) >= factor*(leg1+leg2)*(leg1+leg2) ) {
    x = factor*x*sqrt( (leg1+leg2)*(leg1+leg2) /(x*x +y*y) );
    y = factor*y*sqrt( (leg1+leg2)*(leg1+leg2) /(x*x +y*y) );
    first = 0.0;
    second = atan2(-x, -y);
    return;
  }

  lambda = leg1;
  leg1 = leg2;
  leg2 = lambda;
  float cosb, sinb, sinab, cosab, cosa, sina;
  lambda = sqrt(x*x+y*y);
  sina = y / lambda;
  cosa = -x / lambda;
  cosb = (leg1*leg1 + lambda*lambda - leg2*leg2) / (2*leg1*lambda);
  sinb = sqrt(1-cosb*cosb);
  sinab = sina*cosb + cosa*sinb;
  cosab = cosa*cosb-sina*sinb;
  second = atan2(sinab, cosab)+0.5*M_PI;

  cosa = (leg1*leg1 + leg2*leg2 - lambda*lambda)/(2*leg1*leg2);
  sina = sqrt(1-cosa*cosa);
  first = 0.5*M_PI-atan2(-cosa, -sina);

  first = atan2(sina, cosa) - M_PI;
}

// resync pc and robot
// robot waits for CMD_COMM = 255 in input stream -> flush line with 0s
void CUtil::resync()
{
#define BUFFERLEN 255
  byte buffer[BUFFERLEN];
  for(int i=0; i<BUFFERLEN; i++) {
    buffer[i] = 0;
  }

  CPlatform::TxD8Buffer(buffer, BUFFERLEN);
  CPlatform::sleep(100);
  CPlatform::clearLine();
}

// converts text to double, error = true if error occured
double CUtil::strtodouble(char *text, int len, bool*error)
{
  int dot;
  int i, start;

  if(error != NULL) {
    *error = false;
  }

  for(dot = 0; dot < len; dot++) {
    if(text[dot] == '.') {
      break;
    }
  }

  if(text[0] == '-') {
    start = 1;
  }
  else {
    start = 0;
  }

  int integer = 0;
  int tmp;
  double real = 0.0;

  for(i = start; i < dot; i++) {
    tmp = (int)text[i] - '0';

    if( (tmp < 0) || (tmp > 9) ) {
      if(error != NULL) {
        *error = true;
      }
      continue;
    }
    integer = integer * 10 + tmp;
  }



  for(i = len - 1; i > dot; i--) {
    tmp = (int)text[i] - '0';

    if( (tmp < 0) || (tmp > 9) ) {
      if(error != NULL) {
        *error = true;
      }
      continue;
    }

    real = real / 10 + tmp;
  }

  real = integer + real / 10;

  if(start > 0) {
    return -real;
  }
  else {
    return real;
  }
}

// trims control characters from start of text
char*CUtil::strtrim(char*text)
{
  byte *c = (byte*)text;

  while(*c != 0)
  {
    if(*c > ' ') {
      return (char *)c;
    }

    c++;
  }

  return (char *)c;
}

// copies count bytes in memory from src to dest
void *CUtil::copymem(void *dest, const void *src, int count)
{
  char *tmp = (char *)dest, *s = (char *)src;

  while(count--) {
    *tmp++ = *s++;
  }

  return dest;
}

// trims control characters from start of text
char*CUtil::strtrimright(char*text)
{
  int len = strlen(text);

  for(int i=len-1; i>=0; i--) {
    if(text[i] > ' ') {
      text[i+1] = '\0';
      return text;
    }
  }

  return text;
}

CMatrix CUtil::getJacobian(CMatrix *c, int size, CMatrix &t)
{
  CMatrix jac;

  CVec target = t[3];
  CVec current[3];
  CVec z(0, 0, 1);
  z.w = 0.0;
  CVec tmp, tmp2, tmp3;

  CMatrix tmpM;

  int i, j;
  for(i=0; i<size; i++) {
    current[i] = c[i][3];
    current[i].print();
    printf("\n");
    tmp = target - current[i];

    //c[i].a[12] = c[i].a[13] = c[i].a[14] = 0.0;
    tmp2 = c[i] * z;
    tmp3 = tmp2 ^ tmp;

    jac.a[4*i+0] = tmp3[0];
    jac.a[4*i+1] = tmp3[1];
    jac.a[4*i+2] = tmp3[2];
  }
  jac.print();
  printf("\n");
}

