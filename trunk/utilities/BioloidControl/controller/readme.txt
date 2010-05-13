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

Custom Firmware (CM-5):
-----------------------

Instructions:
-------------
Compile the program (f.e. on Windows platforms with Programmers Notepad 2 and WinAVR) and 
copy the RobotWrapper.hex file to the robot (see Bioloid User Manual if you don't know how).

You can configure the firmware via the following console commands (from ./main/bioloid):
----------------------------------------------------------------------------------------
setrobot 0 x: x is the number of generated  interpolation points
setrobot 1 x: x is the pause in milliseconds between the processing of two interpolation points
setrobot 2 x: x is the interpolation type 
(0: linear (works good on medium loads), 1: ptp (works good on medium loads), 2: simple (works best on heavy loads, f.e. humanoid, reason: you can use higher baudrates and better interpolation techniques on pc))
