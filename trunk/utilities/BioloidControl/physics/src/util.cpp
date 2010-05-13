/**************************************************************************

    Copyright 2008 Martin Andrews <martin.andrew@PLATFORMedia.com>

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

using namespace std;

#include "util.h"


bool strequali(const char*p, const char*tag) {
  while(*p && *tag && tolower(*p) == tolower(*tag) )        {
    ++p;    ++tag;
  }
  return(*p==0 && *tag==0); // Get to the end of both strings...
}

// sleeps 'ms' milliseconds
void sleep_ms(unsigned long ms) {
#ifdef WIN32
  Sleep(ms);
#else
  usleep(1000*ms);
#endif
}

#include <sstream>
#include <iomanip>

string formatted_double(double v, int dp) {
  stringstream ss;
  ss.setf(ios::fixed, ios::floatfield);
  ss.precision(dp);
  ss << v;
  return ss.str();
}

/*
 #include <sys/time.h>

   // gets current tickcount (= elapsed time from system start in milliseconds)
   unsigned long getTickCount() {
 #ifdef WIN32
    return GetTickCount();
 #else
    static struct timeval tstart;
    static struct timezone tz;
    gettimeofday(&tstart, &tz);
    return tstart.tv_sec*1000 + tstart.tv_usec / 1000;
 #endif
   }

   float getTime() { // In seconds (consistent with ODE)
    return (float)(getTickCount()/1000.0f);
   }
 */
