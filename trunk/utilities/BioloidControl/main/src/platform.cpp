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
#include <string>
#include <stdlib.h>

#include "../include/platform.h"
#include "../include/vecmath.h"
#include "../include/robot.h"
#include "../include/util.h"
#include "../include/cmd.h"
#include "../include/vars.h"

/* Linux FTDI driver for USB2Dynamixel: Still testing*/
#define _USE_FTDI

#ifdef USE_FTDI
// you have to enable #define SERIALSIMPLE below in order to make this work
#include <ftdi.h>
#include <stdio.h>
#include <usb.h>
struct ftdi_context ftdi;
#endif

//  uS
#define SERIALTIMEOUTREAD 1000
// in uS
#define SERIALTIMEOUTWRITE 1000
// dont init serial (remove _)
#define _SERIALDONTINIT
// use readfile/writefile
#define _SERIALSIMPLE

// include tcp serverr
#define _INCLUDETCP

// enable to print packets as hex
#define _SHOWPACKETS

#ifdef INCLUDETCP
 #include "../include/practicalsocket.h" // include tcp server
TCPSocket *pcOut, *pcIn;
#endif

#ifdef WIN32
// windows
#include <process.h>
#include <windows.h>

// com port handle
HANDLE hSerialPort = INVALID_HANDLE_VALUE;

// memory-mapped file name
const LPCTSTR sMemoryFileName = "Bioloid Shared Memory";
// mm: mutex name
const LPCTSTR sMutexFileName = "Bioloid Shared Memory Mutex";
HANDLE hFile;
LPVOID pFile;
HANDLE hMutex;
// mm: maxinum number of milliseconds to wait
const DWORD dwMaxWait = 200;
const DWORD dwMaxWaitMutex = 10000;

// threads & stuff
// windows -> critical sections. less overhead
CRITICAL_SECTION mutexes[MUTEXNUM];
//const LPCTSTR mutexnames[MUTEXNUM] = {"BIOLOID1", "BIOLOID2", "BIOLOID3"};

HANDLE threads[THREADNUM];
int threadids[THREADNUM];
DWORD threadhandles[THREADNUM];
#else
// linux
 #include <stdio.h>   /* Standard input/output definitions */
 #include <string.h>  /* String function definitions */
 #include <unistd.h>  /* UNIX standard function definitions */
 #include <fcntl.h>   /* File control definitions */
 #include <errno.h>   /* Error number definitions */
 #include <termios.h> /* POSIX terminal control definitions */
 #include <pthread.h>
 #include <linux/serial.h>

 #include <sys/types.h>
 #include <sys/ipc.h>
 #include <sys/shm.h>
 #include <sys/sysinfo.h>
 #include <sys/times.h>
 #include <sys/time.h>

 #include <sys/stat.h> /* for stat structure : file sizes */


 #define FTOK_FILE "/home/."

pthread_t threads[THREADNUM];
pthread_mutex_t mutexes[MUTEXNUM] = {
  PTHREAD_MUTEX_INITIALIZER,  PTHREAD_MUTEX_INITIALIZER,
  PTHREAD_MUTEX_INITIALIZER,  PTHREAD_MUTEX_INITIALIZER,
  PTHREAD_MUTEX_INITIALIZER,  PTHREAD_MUTEX_INITIALIZER,
  PTHREAD_MUTEX_INITIALIZER,  PTHREAD_MUTEX_INITIALIZER
};

int threadids[THREADNUM];

int shmId;
void  *shm;

int fdSerialPort; /* File descriptor for the port */
#endif

#define sc(box, r, g, b) { box.red = r; box.green = g; box.blue = b; }

#define MAXSERIALERROR 10
#define STDSERIALLINUX "/dev/ttyUSB0"
#define STDSERIALWINDOWS "COM4"

// general thread callback function
#ifdef WIN32
DWORD WINAPI thread_callback(LPVOID ptr)
#else
void *thread_callback(void *ptr)
#endif
{
  int num = *(int *)ptr; // mdda : point at an integer - don't use the pointer as an integer : it's naughty

  switch(num) {
  case THREAD_CMD:
    break;

  case THREAD_MOTION:
    global.threadMotion();
    break;

  case THREAD_MOTIONCMD:
    global.motion.calcTrajectory();
    break;

  case THREAD_INPUT:
    break;

  case THREAD_INTERPOLATION:
    global.motion.updateRobot();
    break;

  case THREAD_TRAJECTORY:
    global.motion.interpolate();
    break;
  }
// #ifdef WIN32
  return 0;
// #else
//  return (void *)NULL;
// #endif
}

// clean up threads
void CPlatform::closeThreads() {
}

void CPlatform::init() {
#ifdef INCLUDETCP
  pcIn = pcOut = NULL;
#endif

  for(int i=0; i<MUTEXNUM; i++) {
#ifdef WIN32
    InitializeCriticalSection(&mutexes[i]);
#endif
  }
}

// clean up
void CPlatform::clean() {
  int i;
  for(i=0; i<MUTEXNUM; i++) {
#ifdef WIN32
    DeleteCriticalSection(&mutexes[i]);
#else
    pthread_mutex_destroy(&mutexes[i]);
#endif
  }
}

void CPlatform::closeMemoryMappedFile()
{
#ifdef WIN32

#else
  //shmctl(shmId, IPC_RMID, 0);
#endif
}

// init threads
void CPlatform::initThreads(bool rtc)
{
  for(int i=0; i<THREADNUM; i++) {
          threadids[i]=i; // mdda - Needs to be set explicitly with the pointer vs. integer mod
#ifdef WIN32
    threads[i] = CreateThread(NULL, 0, thread_callback, (void*)&threadids[i], 0, &threadhandles[i]);
#else
//        pthread_create(&threads[i], NULL, thread_callback , (void*)i);
    
    pthread_create(&threads[i], NULL, thread_callback, (void*)&threadids[i]); // mdda : point at an integer - don't use the pointer as an integer
#endif
  }

#ifdef WIN32
  SetThreadPriority(threads[THREAD_INTERPOLATION], THREAD_PRIORITY_ABOVE_NORMAL);
#endif

  global.threadCmd();
}

// lock mutex
void CPlatform::mutex_lock(int id)
{
#ifdef WIN32
  EnterCriticalSection(&mutexes[id]);
#else
  pthread_mutex_lock(&mutexes[id]);
#endif
}

// unlock mutex
void CPlatform::mutex_unlock(int id)
{
#ifdef WIN32
  LeaveCriticalSection(&mutexes[id]);
#else
  pthread_mutex_unlock(&mutexes[id]);
#endif
}

// init shared memory section
void CPlatform::openMemoryMappedFile()
{
#ifdef WIN32
  // create mutex
  hMutex = CreateMutex(NULL, FALSE, sMutexFileName);

  if(!hMutex) {
    CUtil::cout("initMemoryMappedFile: CreateMutex() failed.\n", TEXT_ERROR);
    return;
  }

  // create file mapping to store dwMemoryFileSize bytes
  hFile = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, dwMemoryFileSize*sizeof(TCHAR), sMemoryFileName);

  if(!hFile) {
    CUtil::cout("initMemoryMappedFile: CreateFileMapping() failed.\n", TEXT_ERROR);
    return;
  }
  else {
    pFile = MapViewOfFile(hFile, FILE_MAP_ALL_ACCESS, 0, 0, 0);

    if(!pFile) {
      CUtil::cout("initMemoryMappedFile: MapViewOfFile() failed.\n", TEXT_ERROR);
      return;
    }
  }
#else
  key_t key;

  // get key
  key = ftok(FTOK_FILE, 'S');

  // open shared memory segment
  if( (shmId = shmget(key, dwMemoryFileSize, IPC_CREAT|IPC_EXCL|0666) ) == -1) {
    if( (shmId = shmget(key, dwMemoryFileSize, 0) ) == -1) {
      CUtil::cout("initMemoryMappedFile: shmget() failed.\n", TEXT_ERROR);
      return;
    }
  }

  // attach
  if( (shm = shmat(shmId, 0, 0) ) == NULL) {
    CUtil::cout("initMemoryMappedFile: shmat() failed.\n", TEXT_ERROR);
    return;
  }
#endif
}

void transformPolyeder(Polyeder &poly, CBox &box, CMatrix *pMat)
{
  CVec tmp2, tmp;
  tmp2.set(box.extents.x,
           box.extents.y,
           box.extents.z);
  tmp = box.center + tmp2; tmp2  = *pMat * tmp;
  poly.corner[0][0] =  tmp2.x; poly.corner[0][1] =  tmp2.y; poly.corner[0][2] =  tmp2.z;
  //----------
  tmp2.set(-box.extents.x,
           box.extents.y,
           box.extents.z);
  tmp = box.center + tmp2; tmp2  = *pMat * tmp;
  poly.corner[1][0] =  tmp2.x; poly.corner[1][1] =  tmp2.y; poly.corner[1][2] =  tmp2.z;
  //----------
  tmp2.set(box.extents.x,
           -box.extents.y,
           box.extents.z);
  tmp = box.center + tmp2; tmp2  = *pMat * tmp;
  poly.corner[2][0] =  tmp2.x; poly.corner[2][1] =  tmp2.y; poly.corner[2][2] =  tmp2.z;
  //----------
  tmp2.set(-box.extents.x,
           -box.extents.y,
           box.extents.z);
  tmp = box.center + tmp2; tmp2  = *pMat * tmp;
  poly.corner[3][0] =  tmp2.x; poly.corner[3][1] =  tmp2.y; poly.corner[3][2] =  tmp2.z;
  //----------
  tmp2.set(box.extents.x,
           box.extents.y,
           -box.extents.z);
  tmp = box.center + tmp2; tmp2  = *pMat * tmp;
  poly.corner[4][0] =  tmp2.x; poly.corner[4][1] =  tmp2.y; poly.corner[4][2] =  tmp2.z;
  //----------
  tmp2.set(-box.extents.x,
           box.extents.y,
           -box.extents.z);
  tmp = box.center + tmp2; tmp2  = *pMat * tmp;
  poly.corner[5][0] =  tmp2.x; poly.corner[5][1] =  tmp2.y; poly.corner[5][2] =  tmp2.z;
  //----------
  tmp2.set(box.extents.x,
           -box.extents.y,
           -box.extents.z);
  tmp = box.center + tmp2; tmp2  = *pMat * tmp;
  poly.corner[6][0] =  tmp2.x; poly.corner[6][1] =  tmp2.y; poly.corner[6][2] =  tmp2.z;
  //----------
  tmp2.set(-box.extents.x,
           -box.extents.y,
           -box.extents.z);
  tmp = box.center + tmp2; tmp2  = *pMat * tmp;
  poly.corner[7][0] =  tmp2.x; poly.corner[7][1] =  tmp2.y; poly.corner[7][2] =  tmp2.z;

// rotate coordinate system
  for(int j=0; j<8; j++) {
    poly.corner[j][0] =  -poly.corner[j][0];
    float tmpf = poly.corner[j][1];
    poly.corner[j][1] =  poly.corner[j][2];
    poly.corner[j][2] =  tmpf;
  }
}

float getDistanceValue(float distance, float distOffset, float distFactor)
{
  // 3 + EXP(0.4*(d-50)) * 255 = distance
  if(distance <= 3.0) {
    return 0.0;
  }

  return -(log( (distance - 3.0) / 255.0) / 0.4) * distFactor + distOffset;

  // linear:
  //return distOffset + distFactor * ((255.0 - distance));
}

// write to memory mapped file
void CPlatform::writeToMemoryMappedFile(CRobot *robot)
{
  // write text to memory-mapped file
#ifdef WIN32
  if(pFile  &&  hMutex)
#else
  if(shmId != -1)
#endif
  {
    CFrame *frame = NULL;
    CMatrix frameMatrix;
    if( (global.viewType >= 0) && (global.viewType <= global.robotInput.framesCount) ) {
      frame =  global.robotInput.frames[global.viewType];
      frameMatrix = frame->getRelativeToBase();
      frameMatrix.invert();
    }

    Polyeder pos[POLYCOUNT];
    CVec tmp, tmp2;
    CMatrix mat(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);;
    CMatrix *pMat;
    CMatrix matleg, tmp1;
    int i, j;
    float tmpf;


    for(i=0; i<robot->geometry.length; i++) {
      if(robot->geometry.geometry[i].frame != NULL) {
        mat = robot->geometry.geometry[i].frame->getRelativeToBase();
      }
      else {
        mat.set(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      }

      pMat = (CMatrix*)&mat;

      pMat->mul(frameMatrix, *pMat);

      transformPolyeder(pos[i], robot->geometry.geometry[i], pMat);
    }

    // set color
    for(i=0; i<robot->geometry.length; i++) {
      pos[i].red = (unsigned char)(255*robot->geometry.geometry[i].color[0]);
      pos[i].green = (unsigned char)(255*robot->geometry.geometry[i].color[1]);
      pos[i].blue = (unsigned char)(255*robot->geometry.geometry[i].color[2]);
    }

    int sensorCount = 0;
    // test: show distance sensor values as hyperplanes -->

    /*

       i = 0;
       if (robot->geometry.geometry[i].frame != NULL)
       {
           mat = robot->geometry.geometry[i].frame->getRelativeToBase();
       } else
       mat.set(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);

       pMat = (CMatrix*)&mat;

       pMat->mul(frameMatrix, *pMat);

       CBox tmpBox;
       float distFactor = 20.0;
       float distOffset = 10.0;
       float distMin = 3.0;
       // center
       //printf("%d\n", global.robotWrapper.Axs1s[0].getDistLeft());
       if (global.robotWrapper.Axs1s[0].getDistCenter() > distMin)
       {
        tmpBox.center.y = robot->geometry.geometry[i].center.y + getDistanceValue((float)global.robotWrapper.Axs1s[0].getDistCenter(), distOffset, distFactor);
        tmpBox.center.z = robot->geometry.geometry[i].center.z -150;
        tmpBox.center.x = robot->geometry.geometry[i].center.x;
        tmpBox.extents.x = 300;
        tmpBox.extents.y = 10;
        tmpBox.extents.z = 300;

        transformPolyeder(pos[robot->geometry.length + sensorCount], tmpBox, pMat);

        pos[robot->geometry.length + sensorCount].red = 255;
        pos[robot->geometry.length + sensorCount].green = 0;
        pos[robot->geometry.length + sensorCount].blue = 0;

        sensorCount++;
       }

       // left

       //printf("%g\n", getDistanceValue((float)global.robotWrapper.Axs1s[0].getDistLeft(), distOffset, distFactor));
       if (global.robotWrapper.Axs1s[0].getDistLeft() > distMin)
       {
        tmpBox.center.x = robot->geometry.geometry[i].center.x + getDistanceValue((float)global.robotWrapper.Axs1s[0].getDistLeft(), distOffset, distFactor);
        tmpBox.center.z = robot->geometry.geometry[i].center.z -150;
        tmpBox.center.y = robot->geometry.geometry[i].center.y;
        tmpBox.extents.x = 10;
        tmpBox.extents.y = 300;
        tmpBox.extents.z = 300;

        transformPolyeder(pos[robot->geometry.length + sensorCount], tmpBox, pMat);

        pos[robot->geometry.length + sensorCount].red = 255;
        pos[robot->geometry.length + sensorCount].green = 0;
        pos[robot->geometry.length + sensorCount].blue = 0;

        sensorCount++;
       }

       // right
       if (global.robotWrapper.Axs1s[0].getDistRight() > distMin)
       {
        tmpBox.center.x = robot->geometry.geometry[i].center.x - getDistanceValue((float)global.robotWrapper.Axs1s[0].getDistRight(), distOffset, distFactor);
        tmpBox.center.z = robot->geometry.geometry[i].center.z -150;
        tmpBox.center.y = robot->geometry.geometry[i].center.y;
        tmpBox.extents.x = 10;
        tmpBox.extents.y = 300;
        tmpBox.extents.z = 300;

        transformPolyeder(pos[robot->geometry.length + sensorCount], tmpBox, pMat);

        pos[robot->geometry.length + sensorCount].red = 255;
        pos[robot->geometry.length + sensorCount].green = 0;
        pos[robot->geometry.length + sensorCount].blue = 0;

        sensorCount++;
       }
     */
    // <- test



#ifdef WIN32
    if(WaitForSingleObject(hMutex, dwMaxWait) == WAIT_OBJECT_0) {
      int tmp = robot->geometry.length + sensorCount;
      memcpy(pFile, &tmp, sizeof(int) );
      tmp = 1;
      memcpy((void*)( (unsigned char *)pFile+sizeof(int) ), &tmp, sizeof(int) );
      memcpy( (void*)( (unsigned char *)pFile+2*sizeof(int) ), pos, POLYCOUNT * sizeof(Polyeder) );
      ReleaseMutex(hMutex);
    }


#else
    int tmpi = robot->geometry.length;
    unsigned int count = 0;
    memcpy(shm, &tmpi, sizeof(int) ); count += sizeof(int);
    tmpi = 1;
    memcpy( (void*)( (unsigned char *)shm+count), &tmpi, sizeof(int) ); count += sizeof(int);
    memcpy( (void*)( (unsigned char *)shm+count), pos, POLYCOUNT * sizeof(Polyeder) );
#endif
  }
}

// reads data from file 'filename' in 'buffer'
// if len == -1 then the filesize will be retrieved
void CPlatform::readFromFile(const char*filename, byte*buffer, int &len)
{
#ifdef WIN32
  HANDLE hFile;
  BOOL bSuccess = FALSE;
  DWORD dwRead;

  // retrieve file size, only small sized files are supported
  if(len == -1) {
    WIN32_FILE_ATTRIBUTE_DATA fileInfo;
    if(GetFileAttributesEx(filename, GetFileExInfoStandard, (void*)&fileInfo) ) {
      len = fileInfo.nFileSizeLow;
    }
  }

  hFile = CreateFile(filename, GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

  if(hFile != INVALID_HANDLE_VALUE) {
    if(ReadFile(hFile, buffer, len, &dwRead, NULL) ) {
      len = dwRead;
    }
    else {
      len = 0;
    }

    CloseHandle(hFile);
  }
  else { len = 0;}
#else
  int fd = open(filename, O_RDONLY);

  if(fd == -1) {
    len = 0;
    return;
  }

  if(len == -1) {
   len = SSIZE_MAX;
   /*    struct stat finfo;
    // if (stat(filepath, &finfo) < 0) return -1; // Assume file not missing, if we're here
    len=finfo.st_size;
   */
  }
  int n = read(fd, buffer, len);
  len = n;
  close(fd);
#endif
}

void bufferToString(char *strbuffer, byte*data, int len)
{
  const char conv[16] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
  };

  for(int bCount = 0; bCount < len; bCount++) {
    strbuffer[bCount*3 + 0] = conv[(data[bCount] & 0xF0) >> 4];
    strbuffer[bCount*3 + 1] = conv[(data[bCount] & 0x0F)];
    strbuffer[bCount*3 + 2] = ' ';
  }
  strbuffer[len*3-1] = '\0';
}

// writes data to file 'filename'
void CPlatform::writeToFile(const char*filename, byte*buffer, int len)
{
#ifdef WIN32
  HANDLE hFile;
  BOOL bSuccess = FALSE;

  hFile = CreateFile(filename, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if(hFile != INVALID_HANDLE_VALUE) {
    DWORD dwWritten;
    if(WriteFile(hFile, buffer, len, &dwWritten, NULL) ) {
      bSuccess = TRUE;
    }

    CloseHandle(hFile);
  }
#else
  int fd = open(filename, O_CREAT | O_RDWR | O_TRUNC, S_IRWXU);
  write(fd, buffer, len);
  close(fd);
#endif
}

#ifndef WIN32
unsigned int getSpeedConstant(unsigned int baudrate)
{
  switch(baudrate)
  {
  case  50:
    return B50;

  case  75:
    return B75;

  case 110:
    return B110;

  case 134:
    return B134;

  case 150:
    return B150;

  case 200:
    return B200;

  case 300:
    return B300;

  case 600:
    return B600;

  case 1200:
    return B1200;

  case 1800:
    return B1800;

  case 2400:
    return B2400;

  case 4800:
    return B4800;

  case 9600:
    return B9600;

  case 19200:
    return B19200;

  case 38400:
    return B38400;

  case 57600:
    return B57600;

  case 115200:
    return B115200;

  case 230400:
    return B230400;

  case 460800:
    return B460800;

  case 500000:
    return B500000;

  case 576000:
    return B576000;

  case 921600:
    return B921600;

  case 1000000:
    return B1000000;

  case 1152000:
    return B1152000;

  case 1500000:
    return B1500000;

  case 2000000:
    return B2000000;

  case 2500000:
    return B2500000;

  case 3000000:
    return B3000000;

  case 3500000:
    return B3500000;

  case 4000000:
    return B4000000;

  default:
    return B115200;
  }
}

#endif

// init serial communication device
bool CPlatform::initSerial(char*cPort, unsigned int baudrate)
{
// Windows
#ifdef WIN32
  if(hSerialPort != INVALID_HANDLE_VALUE) {
    CloseHandle(hSerialPort);
  }

  if(cPort == NULL) {
    cPort = STDSERIALWINDOWS;
  }

  hSerialPort = ::CreateFile(cPort, GENERIC_READ|GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

  if(hSerialPort == INVALID_HANDLE_VALUE) {
    CUtil::cout("initSerial: CreateFile() failed.\n", TEXT_ERROR);
    return false;
  }
  else {
    CUtil::cout("initSerial: Connected!\n", TEXT_ERROR);
  }

  COMMTIMEOUTS timeouts;
  GetCommTimeouts(hSerialPort, &timeouts);

  int a = 40; //40
  timeouts.ReadIntervalTimeout = a;
  timeouts.ReadTotalTimeoutMultiplier = 20;
  timeouts.ReadTotalTimeoutConstant = a;
  timeouts.WriteTotalTimeoutMultiplier = 20;
  timeouts.WriteTotalTimeoutConstant = a;

  SetCommTimeouts(hSerialPort, &timeouts);

  DCB dcb = {0};
  dcb.DCBlength = sizeof(DCB);

  if(!::GetCommState(hSerialPort, &dcb) ) {
    CUtil::cout("initSerial: GetCommState() failed.\n", TEXT_ERROR);
  }
  else {
    // 8bit, no parity, one stopbit
    dcb.BaudRate  = baudrate;
    dcb.ByteSize = 8;
    dcb.Parity = 0;
    dcb.StopBits = 0;

    if(!::SetCommState(hSerialPort, &dcb) ) {
      CUtil::cout("initSerial: SetCommState() failed.\n", TEXT_ERROR);
    }
  }

  return true;

// LINUX
#else
  if(cPort == NULL) {
    cPort = (char *)STDSERIALLINUX;
  }

#ifdef USE_FTDI
  
  /* Still testing */

  unsigned char  buf [1000];
  
  int a;
  int rsize;
  if (ftdi_init(&ftdi) < 0 )
    return false;
  int ret;
  
  if((ret = ftdi_usb_open(&ftdi, 0x0403, 0x6001)) < 0) {
    printf( "unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdi));
    return EXIT_FAILURE;
  }
  
  // Read out FTDIChip-ID of R type chips
  if (ftdi.type == TYPE_R) {
    unsigned int chipid;
    printf("ftdi_read_chipid: %d\n", ftdi_read_chipid(&ftdi, &chipid));
    printf("FTDI chipid: %X\n", chipid);
  }
  ftdi_read_data_set_chunksize(&ftdi, 4096);
  ftdi_write_data_set_chunksize(&ftdi, 4096);
  ftdi_set_line_property2(&ftdi, BITS_8, STOP_BIT_1, NONE, BREAK_OFF);
  ftdi_usb_reset (&ftdi);
  printf("baudrate: %d\n", ftdi_set_baudrate (&ftdi, baudrate));
  ftdi_setflowctrl(&ftdi, SIO_DISABLE_FLOW_CTRL);
  ftdi_set_latency_timer(&ftdi, 1);
  ftdi_usb_purge_buffers (&ftdi);
  
  printf (" Err : %s \n", ftdi.error_str);
  
  unsigned char nix[150];
  int j = 0;
  int cr;
  for (j = 0; j < 10; j++) {
    cr = ftdi_read_data (&ftdi, &nix[0], 100);
    printf (" %i \n", cr);
  }
  
    
    printf ("Typ: %d\n", ftdi.type);
    printf ("usb_read_timeout: %d\n", ftdi.usb_read_timeout);
    printf ("usb_write_timeout: %d\n", 
	    ftdi.usb_write_timeout);
    printf ("baudrate: %d\n",  ftdi.baudrate);
    printf ("bitbang_enabled: %x\n", ftdi.bitbang_enabled);
    printf ("bitbang_mode: %x\n", ftdi.bitbang_mode);
    
    return true;

    /* close handles */
    ftdi_usb_close(&ftdi);
    ftdi_deinit(&ftdi);
    return false;
#endif
  //system("stty -F /dev/ttyS0 speed 115200 raw cs8");

  fdSerialPort = open(cPort, O_RDWR | O_NOCTTY | O_NDELAY);// | O_NONBLOCK | O_NDELAY);
  if(fdSerialPort == -1) {
    CUtil::cout("initSerial: open() failed.\n", TEXT_ERROR);
    return false;
  }
  else {
    CUtil::cout("initSerial: Connected.\n", TEXT_ERROR);
  }

  clearLine();


 #ifndef SERIALDONTINIT


  //fcntl(fdSerialPort, F_SETFL, FNDELAY);

  struct termios options;

  if(tcgetattr(fdSerialPort, &options) == -1) {
    CUtil::cout("initSerial: tvgetattr() failed.\n", TEXT_ERROR);
  }
  else {
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_cflag = 0;
    options.c_lflag = 0;

    cfsetispeed(&options, getSpeedConstant(baudrate) );
    cfsetospeed(&options, getSpeedConstant(baudrate) );
    cfmakeraw(&options);

    options.c_cflag |= (CLOCAL | CREAD);

    // 8 bit, no parity, 1 stopbit
    options.c_cflag |= CS8;

    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);


    // wait for 1 characters
    options.c_cc[VMIN]  = 1;
    // timeout 1 seconds
    options.c_cc[VTIME] = 1;

    /*
       cfsetispeed(&options, getSpeedConstant(baudrate));
       cfsetospeed(&options, getSpeedConstant(baudrate));

       //options.c_cflag = 0;
       options.c_cflag |= (getSpeedConstant(baudrate) | CLOCAL | CREAD);

       // 8 bit, no parity, 1 stopbit
       options.c_cflag |= CS8;

       options.c_cflag &= ~CRTSCTS;

       //options.c_lflag = 0;
       options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);*/
    // testing -->

    // <-- testing

    /*
       // baudrate >> 1152000

       struct serial_struct nuts;
       int arby;
       if(ioctl(fdSerialPort, TIOCGSERIAL, &nuts) == -1)
       CUtil::cout( "Serial: ioctl(*,TIOCGSERIAL,*) failed.\n");

       nuts.custom_divisor = nuts.baud_base / 500000;

       if (!(nuts.custom_divisor))
       nuts.custom_divisor = 1;

       arby = nuts.baud_base / nuts.custom_divisor;
       nuts.flags &= ~ASYNC_SPD_MASK;
       nuts.flags |= ASYNC_SPD_CUST;

       if (ioctl(fdSerialPort, TIOCSSERIAL, &nuts) == -1)
       CUtil::cout("Serial: ioctl(*, TIOCSSERIAL, *) failed.\n");
       options.c_cflag |= B38400;

       // wait for 1 characters
       options.c_cc[VMIN]  = 1;
       // timeout 1 seconds
       options.c_cc[VTIME] = 1;
     */


    if(tcsetattr(fdSerialPort, TCSANOW, &options) != 0) {
      CUtil::cout("initSerial: tcsetattr() failed.\n", TEXT_ERROR);
    }
  }
 #endif
  return true;

#endif
}

// flush serial line
void CPlatform::clearLine(void) {
  char Rxch;
  unsigned long numread=0;

#ifdef WIN32
  do {
    ReadFile(hSerialPort, &Rxch, 1, &numread, NULL);
  } while(numread!=0);
#else
#ifdef USE_FTDI
  ftdi_usb_purge_buffers (&ftdi);
  return;
#endif
  if(fdSerialPort != -1) {
    tcflush(fdSerialPort, TCIFLUSH);
  }
#endif
}

void unify_tcp_ports(void) {    // We only need to create one socket if ports are the same (mdda)
  if(global.tcpPortIn != global.tcpPortOut) {
    return;
  }

#ifdef INCLUDE_TCP
  // The ports are meant to be the same
  // If one exists, and the other doesn't, then copy the existing one accross
  if( (pcOut == NULL) && (pcIn != NULL) ) {
    pcOut=pcIn;
  }
  if( (pcOut != NULL) && (pcIn == NULL) ) {
    pcIn=pcOut;
  }
#endif
}

//reads data from serial line
bool CPlatform::RxD8Buffer(byte*buffer, int len) {
  bool bResult = false;

#ifdef INCLUDETCP
  if(global.useTcp && !global.useSerial) {
    //CUtil::cout("pcServerReceive() entered\n", TEXT_DEBUG);
    int result = 0;
    // connect to client
    if(pcIn == NULL) {
      try  {
        TCPSocket*newSocket = new TCPSocket(global.tcpHostname, global.tcpPortIn);
        pcIn = newSocket;

        CUtil::cout("pcServerReceive() connected.\n", TEXT_DEBUG);
        unify_tcp_ports(); // mdda

      }
      catch(SocketException &e) {
        CUtil::cout("pcServerSend() failed (not connected).\n", TEXT_DEBUG);
        pcIn = NULL;
        bResult = false;
      }
    }

    if(pcIn != NULL) {
      try {
        result = pcIn->recv(buffer, len);
      }
      catch(SocketException &e) {
        CUtil::cout("pcServerReceive() failed.\n", TEXT_DEBUG);
        pcIn = NULL;
        bResult = false;
      }
      //CUtil::cout("pcServerReceive() returned ok\n", TEXT_DEBUG);
      bResult = (result == len);
    }
  }
#endif

  if(global.useSerial) {
#ifdef WIN32
    DWORD dwBytesRead = 0;
    ReadFile(hSerialPort, buffer, len, &dwBytesRead, NULL);

    if(dwBytesRead != len) {
      //CUtil::cout("RxD8Buffer: ReadFile() failed.\n", TEXT_ERROR);
      bResult = false;
    }
    bResult = true;
#else

#ifdef SERIALSIMPLE

    int counter = 0;
    int retries = 0;

#ifdef USE_FTDI

    /* Still testing */

    while (counter < 2)
      {
	int ftdi_result = ftdi_read_data(&ftdi, &buffer[counter], 1);
	if (ftdi_result < 1)
	  {
	    CUtil::cout("Read 1\n", TEXT_DEBUG);
	    return false;
	  }
	
	if (buffer[counter] == 0xFF)
	  counter++;
	else 
	  {
	    counter--;
	    if (counter < 0)
	      counter = 0;
	    
	    retries++;
	    if (retries > 10)
	      {
		CUtil::cout("Retries\n", TEXT_DEBUG);
		return false;
	      }
	  }
	
      }
    
    int ftdi_result = ftdi_read_data(&ftdi, &buffer[2], len-2) ;
    if (len - 2 != ftdi_result)
      {
	char strbuffer[1024];
	bufferToString(strbuffer, buffer, ftdi_result);
	char strbuffer2[1024];
	sprintf(strbuffer2, "RXD: %s\n", strbuffer);
	CUtil::cout(strbuffer2, TEXT_DEBUG);
    
	printf("rxd: %d (%d)\n", ftdi_result, len);
      }
    return ftdi_result == len-2;
#endif
    // testing -->
    counter = 0;
    retries = 0;
    while (counter < 2)
      {
	if (read(fdSerialPort, &buffer[counter], 1) < 1)
	      {
  CUtil::cout("Read 1\n", TEXT_DEBUG);
	      return false;
	      }

	if (buffer[counter] == 0xFF)
	  counter++;
	else 
	  {
	    counter--;
	    if (counter < 0)
	      counter = 0;

	    retries++;
	    if (retries > 10)
	      {
  CUtil::cout("Retries\n", TEXT_DEBUG);
	      return false;
	      }
	  }
	
      }

    int res = read(fdSerialPort, &buffer[2], len-2);

char strbuffer[1024];
  bufferToString(strbuffer, buffer, len);
  char strbuffer2[1024];
  sprintf(strbuffer2, "RXD: %s\n", strbuffer);
  CUtil::cout(strbuffer2, TEXT_DEBUG);
  return res == len-2;
    //return len == read(fdSerialPort, buffer, len);

    // <-- testing
 #endif


    int n = 0;
    int error = 0;
    byte b;
    int cur;
    int max_fd = fdSerialPort + 1;
    struct timeval timeout;
    fd_set input;
    FD_ZERO(&input);
    FD_SET(fdSerialPort, &input);

    while(n<len && error < MAXSERIALERROR) {
      timeout.tv_sec = 0;
      timeout.tv_usec = SERIALTIMEOUTREAD;
      select(max_fd, &input, NULL, NULL, &timeout);
      if(FD_ISSET(fdSerialPort, &input) ) {
        cur= read(fdSerialPort, buffer, len - n);
        if(cur >= 0) {
          n+=cur;
          buffer+=cur;
        }
        else {
          error++;
        }
        FD_CLR(fdSerialPort, &input);
      }
      else {
        error++;
        FD_ZERO(&input);
        FD_SET(fdSerialPort, &input);
      }
    }

    if(n != len) {
      char str[255];
      sprintf(str, "RxD8Buffer: read() failed (tries: %d).\n", error);
      CUtil::cout(str, TEXT_ERROR);
    }
    bResult = (n == len);
#endif
  }

#ifdef SHOWPACKETS
  char strbuffer[1024];
  bufferToString(strbuffer, buffer, len);
  char strbuffer2[1024];
  sprintf(strbuffer2, "RXD: %s\n", strbuffer);
  CUtil::cout(strbuffer2, TEXT_DEBUG);
#endif

  return bResult;
}

// writes data to serial line
bool CPlatform::TxD8Buffer(byte*buffer, int len) {
#ifdef SHOWPACKETS
  char strbuffer[1024];
  bufferToString(strbuffer, buffer, len);
  char strbuffer2[1024];
  sprintf(strbuffer2, "TXD: %s\n", strbuffer);
  CUtil::cout(strbuffer2, TEXT_DEBUG);
#endif

#ifdef INCLUDETCP
  if(global.useTcp) {
    // connect to client
    if(pcOut == NULL) {
      try {
        // Establish connection with the clients
        TCPSocket*newSocket = new TCPSocket(global.tcpHostname, global.tcpPortOut);
        pcOut = newSocket;

        CUtil::cout("pcServerSend() connected.\n", TEXT_DEBUG);
        unify_tcp_ports(); // mdda

      }
      catch(SocketException &e) {
        CUtil::cout("pcServerSend() failed (not connected).\n", TEXT_DEBUG);
        pcOut = NULL;
      }
    }

    if(pcOut != NULL) {
      try {
        //printf("sending: %d\n", getTickCount());
        pcOut->send(buffer, len);
      }
      catch(SocketException &e) {
        CUtil::cout("pcServerSend() failed.\n", TEXT_DEBUG);
        pcOut = NULL;
      }
    }
  }
#endif
  if(global.useSerial) {
#ifdef WIN32
    DWORD dwBytesWritten;
    ::WriteFile(hSerialPort, buffer, len, &dwBytesWritten, NULL);

    return dwBytesWritten == len;

#else
//	Linux


 #ifdef SERIALSIMPLE
    
    #ifdef USE_FTDI
    //ftdi_usb_purge_buffers (&ftdi);
    int ftdi_result = ftdi_write_data(&ftdi,buffer, len);
    if (ftdi_result != len)
    {
char strbuffer[1024];
  bufferToString(strbuffer, buffer, ftdi_result);
  char strbuffer2[1024];
  sprintf(strbuffer2, "TXD: %s\n", strbuffer);
  CUtil::cout(strbuffer2, TEXT_DEBUG);
    }
    //usleep(1000);
    //printf("txd: %d\n", ftdi_result);
    return ftdi_result == len;
#endif
    // testing -->
    return write(fdSerialPort, buffer, len) == len;

    // <-- testing
 #endif

    int n, error;
    byte b;
    n = error = 0;
    int cur;
    int max_fd = fdSerialPort + 1;
    struct timeval timeout;
    fd_set input;
    FD_ZERO(&input);
    FD_SET(fdSerialPort, &input);

    while(n<len && error < MAXSERIALERROR) {
      timeout.tv_sec = 0;
      timeout.tv_usec = SERIALTIMEOUTWRITE;
      select(max_fd, NULL, &input, NULL, &timeout);
      if(FD_ISSET(fdSerialPort, &input) ) {
        cur= write(fdSerialPort, buffer, len - n);
        if(cur >= 0) {
          n+=cur;
          buffer+=cur;
        }
        else {
          error++;
        }
      }
      else {
        FD_ZERO(&input);
        FD_SET(fdSerialPort, &input);
      }
    }

    if(n != len) {
      char str[255];
      sprintf(str, "TxD8Buffer: write() failed (tries: %d).\n", error);
      CUtil::cout(str, TEXT_ERROR);
      return false;
    }
#endif
  }
  return true;
}

// writes single byte to robot
bool CPlatform::TxD8(byte bTxdData) {
  return TxD8Buffer(&bTxdData, 1);
}

// reads single byte from robot
byte CPlatform::RxD8(bool *failure) {
  byte buffer = 0;
  bool result = RxD8Buffer(&buffer, 1);

  if(failure != NULL) {
    *failure = !result;
  }
  return buffer;
}

// clears console screen
void CPlatform::clearScreen() {
#ifdef WIN32
  COORD coordScreen = { 0, 0 };
  DWORD cCharsWritten;
  CONSOLE_SCREEN_BUFFER_INFO csbi;
  DWORD dwConSize;
  HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

  GetConsoleScreenBufferInfo(hConsole, &csbi);
  dwConSize = csbi.dwSize.X * csbi.dwSize.Y;
  FillConsoleOutputCharacter(hConsole, ' ', dwConSize, coordScreen, &cCharsWritten);
  GetConsoleScreenBufferInfo(hConsole, &csbi);
  FillConsoleOutputAttribute(hConsole, csbi.wAttributes, dwConSize, coordScreen, &cCharsWritten);

  SetConsoleCursorPosition(hConsole, coordScreen);
#else
  printf("\033[2J");
#endif
}

// sleeps 'ms' milliseconds
void CPlatform::sleep(unsigned long ms) {
#ifdef WIN32
  Sleep(ms);
#else
  usleep(1000*ms);
#endif
}

// gets current tickcount (= elapsed time from system start in milliseconds)
unsigned long CPlatform::getTickCount() {
#ifdef WIN32
       
       //Variablen
    LONGLONG g_Frequency, g_CurrentCount;

    //Frequenz holen
    if (!QueryPerformanceFrequency((LARGE_INTEGER*)&g_Frequency))
        std::cout << "Performance Counter nicht vorhanden" << std::endl;

    //1. Messung
    QueryPerformanceCounter((LARGE_INTEGER*)&g_CurrentCount);

    double dTimeDiff = (((double)(g_CurrentCount))/((double)g_Frequency)) * 1000.0; 

    return (unsigned long) dTimeDiff;
  //return GetTickCount();

#else
  static struct timeval tstart;
  static struct timezone tz;
  gettimeofday(&tstart, &tz);
  return tstart.tv_sec*1000 + tstart.tv_usec / 1000;

  /*
     tms tm;
     return times(&tm);
   */
#endif
}

void CPlatform::getPathFromFilename(char*filename, char*path) {
  // quick n dirty
  char buffer[1024];
#ifdef WIN32
  const char separator = '\\';
  sprintf(buffer, filename);
#else
  const char separator = '/';

  if (filename[0] != separator)
    {
      char cwd[1024];
      if (getcwd(cwd, 1024) == NULL)
	sprintf(cwd, "");
      sprintf(buffer, "%s/%s", cwd, filename); 
    }
  else
    sprintf(buffer, filename);
#endif

	    
  // copy each byte and store last location of separator
  char *pF = buffer;
  char *pP = path;
  char *pS = NULL;
  while(*pF != 0) {
    *pP = *pF;

    if(*pF == separator) {
      pS = pP;
    }
    pF++;
    pP++;
  }

  if(pS != NULL) {
    *(++pS) = 0;
    }
}

/*
   TxPacket() send data to RS485.
   TxPacket() needs 3 parameter; ID of Dynamixel, Instruction byte, Length of parameters.
   TxPacket() return length of Return packet from Dynamixel.
 */
byte CPlatform::TxPacket(byte bID, byte bInstruction, byte bParameterLength, byte*gbpParameter, byte *gbpTxBuffer) {
  byte bCount, bCheckSum, bPacketLength;

  gbpTxBuffer[0] = 0xff;
  gbpTxBuffer[1] = 0xff;
  gbpTxBuffer[2] = bID;
  gbpTxBuffer[3] = bParameterLength+2; //Length(Paramter,Instruction,Checksum)
  gbpTxBuffer[4] = bInstruction;

  for(bCount = 0; bCount < bParameterLength; bCount++) {
    gbpTxBuffer[bCount+5] = gbpParameter[bCount];
  }

  bCheckSum = 0;
  bPacketLength = bParameterLength+4+2;

  for(bCount = 2; bCount < bPacketLength-1; bCount++) { //except 0xff,checksum
    bCheckSum += gbpTxBuffer[bCount];
  }
  gbpTxBuffer[bPacketLength-1] = ~bCheckSum; //Writing Checksum with Bit Inversion

  /*
     char str[1024];
     bufferToString(str, gbpTxBuffer, bPacketLength);
     printf("TxD: %s\n", str);
   */

  if(!TxD8Buffer(gbpTxBuffer, bPacketLength) ) {
    return 0;
  }

  return(bPacketLength);
}

/*
   RxPacket() read data from buffer.
   RxPacket() need a Parameter; Total length of Return Packet.
   RxPacket() return Length of Return Packet.
 */

byte CPlatform::RxPacket(byte*gbpRxBuffer, byte bRxPacketLength, byte *gbpTxBuffer) {
  unsigned long ulCounter;
  byte bCount, bLength, bChecksum;
  byte bTimeout;

  if(!RxD8Buffer(gbpRxBuffer, bRxPacketLength) ) {
    return 0;
  }

  bLength = bRxPacketLength;
  bChecksum = 0;

  if(gbpTxBuffer[2] != BROADCASTING_ID) {
    if(bLength > 3) { //checking is available.
      if( (gbpRxBuffer[0] != 0xff) || (gbpRxBuffer[1] != 0xff) ) {
        return 0;
      }

      if(gbpRxBuffer[2] != gbpTxBuffer[2]) {
        return 0;
      }

      if(gbpRxBuffer[3] != bLength-4) {
        return 0;
      }

      for(bCount = 2; bCount < bLength; bCount++) {
        bChecksum += gbpRxBuffer[bCount];
      }

      if(bChecksum != 0xff) {
        return 0;
      }
    }
  }
  return bLength;
}

