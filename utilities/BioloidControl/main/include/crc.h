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


#include "types.h"

#ifndef CRC_INC
 #define CRC_INC

/*! \brief Cyclic redundancy check.

    Computes 16bit CRC of byte-string.
 */
class CCrc
{
public:
  static word calc(byte *buffer, int len);
  static word calcTable(byte *buffer, int len);

};

#endif
