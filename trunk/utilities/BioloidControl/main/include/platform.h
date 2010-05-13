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


#ifndef __PLATFORM

#define __PLATFORM

#include "types.h"
#include "robot.h"

/*! \brief Thread identifiers
 */
enum
{
  THREAD_MOTION,
  THREAD_INPUT,
  THREAD_CMD,
  THREAD_INTERPOLATION,
  THREAD_TRAJECTORY,
  THREAD_MOTIONCMD,
  THREADNUM
};

/*! \brief Mutex identifiers
 */
enum
{
  MUTEXCIN,
  MUTEXCOUT,
  MUTEXCOMM,
  MUTEXMOTION,
  MUTEXINPUT,
  MUTEXTRAJECTORY,
  MUTEXINTERPOLATION,
  MUTEXCMDS,
  MUTEXNUM
};

// Macros to lock/unlock mutexes
#define ML(x) CPlatform::mutex_lock(x)
#define MU(x) CPlatform::mutex_unlock(x)

/*! \brief Operation system depending functions

   Functions:
   \li Time
   \li Thread control
   \li Shared Memory
   \li Console
   \li Serial Communication
   \li File Handling
 */
class CPlatform
{
public:
  // General
  static void sleep(unsigned long ms); ///< Suspends current thread for \a ms milliseconds
  static unsigned long getTickCount(); ///< Returns time in milliseconds since system start

  // Thread sync
  static void initThreads(bool rtc); ///< Configures and starts all threads
  static void mutex_lock(int id); ///< Locks the mutex with \a id, \see MUTEX_IDENTIFIERS
  static void mutex_unlock(int id); ///< Unlocks the mutex with \a id, \see MUTEX_IDENTIFIERS
  static void closeThreads(); ///< Ends all threads

  // General
  static void init(); ///< Intializes mutexes, ...
  static void clean(); ///< Deinitialisation

  // Console
  static void clearScreen(); ///< Clears the console screen

  // Serial
  static bool initSerial(char *cPort, unsigned int baudrate = 115200); ///< Inits serial port
  static void clearLine(); ///< Clears the serial line buffer

  // File Handling
  static void getPathFromFilename(char*filename, char*path);
  static void readFromFile(const char*filename, byte*buffer, int &len); ///< Reads data from a file
  static void writeToFile(const char*filename, byte*buffer, int len); ///< Writes data to a file

  //Memory Mapped File (Communication with 3d-Viewer)
  static void openMemoryMappedFile(); ///< Opens shared memory in order to communicate with the opengl-viewer
  static void closeMemoryMappedFile(); ///< Close shared memory
  static void writeToMemoryMappedFile(CRobot *robot); ///< Writes 3d information of current robot pose to shared memory which will be visualized in the opengl-viewer

  //Communication
  static bool TxD8(byte bTxdData); ///< Writes one byte to the serial line
  static byte RxD8(bool*failure = NULL); ///< Reads one byte from the serial line
  static bool RxD8Buffer(byte*buffer, int len); ///< Reads \a len bytes from the serial line
  static bool TxD8Buffer(byte*buffer, int len); ///< Writes \a len bytes to the serial line
  static byte TxPacket(byte bID, byte bInstruction, byte bParameterLength, byte*gbpParameter, byte *gbpTxBuffer);
  static byte RxPacket(byte*gbpRxBuffer, byte bRxPacketLength, byte *gbpTxBuffer);

};

/*! \brief Memory Mapped File Data Container

   The eight corners make up one polyeder (a cube most of the time) which will be colored with the stated rgb values
 */
typedef struct
{
  float corner[8][3];
  unsigned char red, green, blue;
} Polyeder;

#define POLYCOUNT 50 // max number of boxes in shared memory

/*! \brief Size of Memory Mapped File
 */
const int dwMemoryFileSize = POLYCOUNT * sizeof(Polyeder) + 2 * sizeof(int);

#endif
