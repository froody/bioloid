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

**************************************************************************

    Tutorial Code from nehe.gamedev.net,
    modified by Rainer Jäkel <rainer.jaekel@googlemail.com>

    Credits from Nehe:
    This Code Was Created By Jeff Molofee 2000
    A HUGE Thanks To Fredric Echols For Cleaning Up
    And Optimizing This Code, Making It More Flexible!
    If You've Found This Code Useful, Please Let Me Know.
    Visit My Site At nehe.gamedev.net

**************************************************************************/


#include <windows.h>     // Header File For Windows
#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>           // Header File For The GLu32 Library
#include <math.h>

#define WIDTH 400
#define HEIGHT 300
#define POSX 1020 - WIDTH
#define POSY 20

enum KEY_BINDINGS
{
  ROTXP = 0x57,
  ROTXM = 0x53,
  ROTYP = 0x44,
  ROTYM = 0x41,
  ROTZP = 0x45,
  ROTZM = 0x51,
  TRANSXP = 0x52,
  TRANSXM = 0x46,
  TRANSYP = 0x43,
  TRANSYM = 0x58,
  ZOOMP = 0x32,
  ZOOMM = 0x31
};

typedef struct
{
  float corner[8][3];
  unsigned char red, green, blue;
} Polyeder;

HDC hDC=NULL;       // Private GDI Device Context
HGLRC hRC=NULL;       // Permanent Rendering Context
HWND hWnd=NULL;      // Holds Our Window Handle
HINSTANCE hInstance;      // Holds The Instance Of The Application

bool keys[256];          // Array Used For The Keyboard Routine
bool active=true;        // Window Active Flag Set To true By Default
bool fullscreen=true;    // Fullscreen Flag Set To Fullscreen Mode By Default

GLfloat rtri = 190.0f;              // Angle For The Triangle ( NEW )
GLfloat rquad = 0.0f;               // Angle For The Quad ( NEW )
GLfloat rr = 0.0f;
GLfloat zz = 0.0f;

GLfloat tx = 0.0f;
GLfloat ty = 0.0f;
// number of characters in memory-mapped file
#define POLYCOUNT 50
const DWORD dwMemoryFileSize = POLYCOUNT * sizeof(Polyeder) + 2 * sizeof(int);

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

GLfloat LightAmbient[]= { 0.5f, 0.5f, 0.5f, 1.0f };                 // Ambient Light Values ( NEW )
GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f };              // Diffuse Light Values ( NEW )
GLfloat LightPosition[]= { 0.0f, 0.0f, 2.0f, 1.0f };                 // Light Position ( NEW )

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);   // Declaration For WndProc

Polyeder boxes[POLYCOUNT];
int boxes_len = 0;
 
bool LoadBitmap(LPTSTR szFileName, GLuint &texid)                   // Creates Texture From A Bitmap File
{
  HBITMAP hBMP;                                                       // Handle Of The Bitmap
  BITMAP BMP;                                                        // Bitmap Structure

  glGenTextures(1, &texid);                                           // Create The Texture
  hBMP=(HBITMAP)LoadImage(GetModuleHandle(NULL), szFileName, IMAGE_BITMAP, 0, 0, LR_CREATEDIBSECTION | LR_LOADFROMFILE);

  if(!hBMP) {                                                        // Does The Bitmap Exist?
    return false;                                                   // If Not Return False

  }
  GetObject(hBMP, sizeof(BMP), &BMP);                                 // Get The Object
                                                                      // hBMP:        Handle To Graphics Object
                                                                      // sizeof(BMP): Size Of Buffer For Object Information
                                                                      // &BMP:        Buffer For Object Information

  glPixelStorei(GL_UNPACK_ALIGNMENT, 4);                              // Pixel Storage Mode (Word Alignment / 4 Bytes)

  // Typical Texture Generation Using Data From The Bitmap
  glBindTexture(GL_TEXTURE_2D, texid);                                // Bind To The Texture ID
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);   // Linear Min Filter
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);   // Linear Mag Filter
  glTexImage2D(GL_TEXTURE_2D, 0, 3, BMP.bmWidth, BMP.bmHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, BMP.bmBits);

  DeleteObject(hBMP);                                                 // Delete The Object

  return true;                                                        // Loading Was Successful
}

GLvoid ReSizeGLScene(GLsizei width, GLsizei height)     // Resize And Initialize The GL Window
{
  if(height==0) {                                    // Prevent A Divide By Zero By
    height=1;                                       // Making Height Equal One
  }

  glViewport(0, 0, width, height);                       // Reset The Current Viewport

  glMatrixMode(GL_PROJECTION);                        // Select The Projection Matrix
  glLoadIdentity();                                   // Reset The Projection Matrix

  // Calculate The Aspect Ratio Of The Window
  gluPerspective(45.0f, (GLfloat)width/(GLfloat)height, 0.1f, 100.0f);

  glMatrixMode(GL_MODELVIEW);                         // Select The Modelview Matrix
  glLoadIdentity();                                   // Reset The Modelview Matrix
}

int InitGL(GLvoid)                                      // All Setup For OpenGL Goes Here
{
  glShadeModel(GL_SMOOTH);                            // Enable Smooth Shading
  glClearColor(58.0/255.0, 110.0/255.0, 165.0/255.0, 0.5f);               // Black Background

  glClearDepth(1.0f);                                 // Depth Buffer Setup
  glEnable(GL_DEPTH_TEST);                            // Enables Depth Testing
  glDepthFunc(GL_LEQUAL);                             // The Type Of Depth Testing To Do
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Really Nice Perspective Calculations

  glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);             // Setup The Ambient Light
  glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);             // Setup The Diffuse Light
  glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);            // Position The Light

  return true;                                        // Initialization Went OK
}

void drawQuad(Polyeder &poly, int x, int y, int z, int w, float scale)
{
  glColor3f(poly.red / 255.0, poly.green / 255.0, poly.blue / 255.0);
  glVertex3f(scale*poly.corner[x][0], scale*poly.corner[x][1], scale*poly.corner[x][2]);
  glVertex3f(scale*poly.corner[y][0], scale*poly.corner[y][1], scale*poly.corner[y][2]);
  glVertex3f(scale*poly.corner[z][0], scale*poly.corner[z][1], scale*poly.corner[z][2]);
  glVertex3f(scale*poly.corner[w][0], scale*poly.corner[w][1], scale*poly.corner[w][2]);
}

int DrawGLScene(GLvoid)                                 // Here's Where We Do All The Drawing
{
  Sleep(10);
  if(keys[ROTYP]) {
    rtri += 1.00f;
  }
  else
  if(keys[ROTYM]) {
    rtri -= 1.00f;
  }

  if(keys[ROTXP]) {
    rquad += 1.00f;
  }
  else
  if(keys[ROTXM]) {
    rquad -= 1.00f;
  }

  if(keys[ROTZP]) {
    rr += 1.0f;
  }
  else  if(keys[ROTZM]) {
    rr -= 1.0f;
  }

  if(keys[TRANSYP]) {
    tx += 0.5f;
  }
  else  if(keys[TRANSYM]) {
    tx -= 0.5f;
  }

  if(keys[TRANSXP]) {
    ty += 0.5f;
  }
  else  if(keys[TRANSXM]) {
    ty -= 0.5f;
  }

  if(keys[ZOOMP]) {
    zz += 1.0f;
  }
  else  if(keys[ZOOMM]) {
    zz -= 1.0f;
  }

  int changed = 0;
  int i;

  if(WaitForSingleObject(hMutex, dwMaxWait) == WAIT_OBJECT_0) {
    memcpy(&changed, (void*)( (unsigned char *)pFile + sizeof(int) ), sizeof(int) );
    if (changed != 0)
    {                             
        memcpy(&boxes_len, pFile, sizeof(int) );
        memcpy(boxes, (void*)( (unsigned char *)pFile + 2*sizeof(int) ), dwMemoryFileSize-2*sizeof(int) );
        changed = 0;
        memcpy((void*)( (unsigned char *)pFile + sizeof(int) ), &changed, sizeof(int) );
    }
    ReleaseMutex(hMutex);   
  }
  else { return 0;}

  float scale = 0.04;
  float scale2 = 0.1;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear Screen And Depth Buffer									// Reset The Current Modelview Matrix

  for(i=0; i<boxes_len; i++) {
    glLoadIdentity();

    glTranslatef(tx, ty, -31.0 + zz);

    glRotatef(rtri, 0.0, 1.0, 0.0);
    glRotatef(rquad, 0.0, 0.0, 1.0);
    glRotatef(rr, 1.0, 0.0, 0.0);



    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 0.0f);          // Set The Color To Green
    drawQuad(boxes[i], 0, 1, 3, 2, scale); // top
    glColor3f(1.0f, 0.5f, 0.0f);          // Set The Color To Orange
    drawQuad(boxes[i], 4, 0, 2, 6, scale); // right
    glColor3f(1.0f, 0.0f, 0.0f);          // Set The Color To Red
    drawQuad(boxes[i], 5, 4, 6, 7, scale); // bottom
    glColor3f(1.0f, 1.0f, 0.0f);          // Set The Color To Yellow
    drawQuad(boxes[i], 1, 5, 7, 3, scale); // left
    glColor3f(0.0f, 0.0f, 1.0f);          // Set The Color To Blue
    drawQuad(boxes[i], 4, 5, 1, 0, scale); // behind
    glColor3f(1.0f, 0.0f, 1.0f);           // Set The Color To Pink
    drawQuad(boxes[i], 6, 7, 3, 2, scale); // front

    glEnd();
  }
  return true;                                        // Keep Going
}

GLvoid KillGLWindow(GLvoid)                             // Properly Kill The Window
{
  if(hFile) {
    if(pFile) {
      UnmapViewOfFile(pFile);
    }

    CloseHandle(hFile);
  }

  if(hMutex) {
    CloseHandle(hMutex);
  }

  if(fullscreen) {                                   // Are We In Fullscreen Mode?
    ChangeDisplaySettings(NULL, 0);                  // If So Switch Back To The Desktop
    ShowCursor(true);                               // Show Mouse Pointer
  }

  if(hRC) {                                          // Do We Have A Rendering Context?
    if(!wglMakeCurrent(NULL, NULL) ) {               // Are We Able To Release The DC And RC Contexts?
      MessageBox(NULL, "Release Of DC And RC Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
    }

    if(!wglDeleteContext(hRC) ) {                   // Are We Able To Delete The RC?
      MessageBox(NULL, "Release Rendering Context Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
    }
    hRC=NULL;                                       // Set RC To NULL
  }

  if(hDC && !ReleaseDC(hWnd, hDC) ) {                  // Are We Able To Release The DC
    MessageBox(NULL, "Release Device Context Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
    hDC=NULL;                                       // Set DC To NULL
  }

  if(hWnd && !DestroyWindow(hWnd) ) {                 // Are We Able To Destroy The Window?
    MessageBox(NULL, "Could Not Release hWnd.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
    hWnd=NULL;                                      // Set hWnd To NULL
  }

  if(!UnregisterClass("OpenGL", hInstance) ) {          // Are We Able To Unregister Class
    MessageBox(NULL, "Could Not Unregister Class.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
    hInstance=NULL;                                 // Set hInstance To NULL
  }
}

/*	This Code Creates Our OpenGL Window.  Parameters Are:					*
 *	title			- Title To Appear At The Top Of The Window				*
 *	width			- Width Of The GL Window Or Fullscreen Mode				*
 *	height			- Height Of The GL Window Or Fullscreen Mode			*
 *	bits			- Number Of Bits To Use For Color (8/16/24/32)			*
 *	fullscreenflag	- Use Fullscreen Mode (true) Or Windowed Mode (false)	*/

BOOL CreateGLWindow(char*title, int width, int height, int bits, bool fullscreenflag)
{
  GLuint PixelFormat;            // Holds The Results After Searching For A Match
  WNDCLASS wc;                     // Windows Class Structure
  DWORD dwExStyle;              // Window Extended Style
  DWORD dwStyle;                // Window Style
  RECT WindowRect;             // Grabs Rectangle Upper Left / Lower Right Values
  WindowRect.left=(long)0;            // Set Left Value To 0
  WindowRect.right=(long)width;       // Set Right Value To Requested Width
  WindowRect.top=(long)0;             // Set Top Value To 0
  WindowRect.bottom=(long)height;     // Set Bottom Value To Requested Height

  fullscreen=fullscreenflag;          // Set The Global Fullscreen Flag

  hInstance           = GetModuleHandle(NULL);                // Grab An Instance For Our Window
  wc.style            = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;   // Redraw On Size, And Own DC For Window.
  wc.lpfnWndProc      = (WNDPROC)WndProc;                    // WndProc Handles Messages
  wc.cbClsExtra       = 0;                                    // No Extra Window Data
  wc.cbWndExtra       = 0;                                    // No Extra Window Data
  wc.hInstance        = hInstance;                            // Set The Instance
  wc.hIcon            = LoadIcon(NULL, IDI_WINLOGO);          // Load The Default Icon
  wc.hCursor          = LoadCursor(NULL, IDC_ARROW);          // Load The Arrow Pointer
  wc.hbrBackground    = NULL;                                 // No Background Required For GL
  wc.lpszMenuName     = NULL;                                 // We Don't Want A Menu
  wc.lpszClassName    = "OpenGL";                              // Set The Class Name

  if(!RegisterClass(&wc) ) {                                  // Attempt To Register The Window Class
    MessageBox(NULL, "Failed To Register The Window Class.", "ERROR", MB_OK|MB_ICONEXCLAMATION);
    return false;                                           // Return false
  }

  if(fullscreen) {                                           // Attempt Fullscreen Mode?
    DEVMODE dmScreenSettings;                               // Device Mode
    memset(&dmScreenSettings, 0, sizeof(dmScreenSettings) );   // Makes Sure Memory's Cleared
    dmScreenSettings.dmSize=sizeof(dmScreenSettings);       // Size Of The Devmode Structure
    dmScreenSettings.dmPelsWidth    = width;                // Selected Screen Width
    dmScreenSettings.dmPelsHeight   = height;               // Selected Screen Height
    dmScreenSettings.dmBitsPerPel   = bits;                 // Selected Bits Per Pixel
    dmScreenSettings.dmFields=DM_BITSPERPEL|DM_PELSWIDTH|DM_PELSHEIGHT;

    // Try To Set Selected Mode And Get Results.  NOTE: CDS_FULLSCREEN Gets Rid Of Start Bar.
    if(ChangeDisplaySettings(&dmScreenSettings, CDS_FULLSCREEN)!=DISP_CHANGE_SUCCESSFUL) {
      // If The Mode Fails, Offer Two Options.  Quit Or Use Windowed Mode.
      if(MessageBox(NULL, "The Requested Fullscreen Mode Is Not Supported By\nYour Video Card. Use Windowed Mode Instead?", "NeHe GL", MB_YESNO|MB_ICONEXCLAMATION)==IDYES) {
        fullscreen=false;       // Windowed Mode Selected.  Fullscreen = false
      }
      else {
        // Pop Up A Message Box Letting User Know The Program Is Closing.
        MessageBox(NULL, "Program Will Now Close.", "ERROR", MB_OK|MB_ICONSTOP);
        return false;                                   // Return false
      }
    }
  }

  if(fullscreen) {                                           // Are We Still In Fullscreen Mode?
    dwExStyle=WS_EX_APPWINDOW;                              // Window Extended Style
    dwStyle=WS_POPUP;                                       // Windows Style
    ShowCursor(false);                                      // Hide Mouse Pointer
  }
  else {
    dwExStyle=WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;           // Window Extended Style
    dwStyle=WS_OVERLAPPEDWINDOW;                            // Windows Style
  }

  AdjustWindowRectEx(&WindowRect, dwStyle, false, dwExStyle);     // Adjust Window To True Requested Size

  // Create The Window
  if(!(hWnd=CreateWindowEx(dwExStyle | WS_EX_TOPMOST,                          // Extended Style For The Window
                           "OpenGL",                            // Class Name
                           title,                              // Window Title
                           dwStyle |                           // Defined Window Style
                           WS_CLIPSIBLINGS |                   // Required Window Style
                           WS_CLIPCHILDREN,                    // Required Window Style
                           POSX, POSY,                             // Window Position
                           WindowRect.right-WindowRect.left,   // Calculate Window Width
                           WindowRect.bottom-WindowRect.top,   // Calculate Window Height
                           NULL,                               // No Parent Window
                           NULL,                               // No Menu
                           hInstance,                          // Instance
                           NULL) ) ) {                           // Dont Pass Anything To WM_CREATE
    KillGLWindow();                             // Reset The Display
    MessageBox(NULL, "Window Creation Error.", "ERROR", MB_OK|MB_ICONEXCLAMATION);
    return false;                               // Return false
  }

  static PIXELFORMATDESCRIPTOR pfd=              // pfd Tells Windows How We Want Things To Be
  {
    sizeof(PIXELFORMATDESCRIPTOR),              // Size Of This Pixel Format Descriptor
    1,                                          // Version Number
    PFD_DRAW_TO_WINDOW |                        // Format Must Support Window
    PFD_SUPPORT_OPENGL |                        // Format Must Support OpenGL
    PFD_DOUBLEBUFFER,                           // Must Support Double Buffering
    PFD_TYPE_RGBA,                              // Request An RGBA Format
    bits,                                       // Select Our Color Depth
    0, 0, 0, 0, 0, 0,                           // Color Bits Ignored
    0,                                          // No Alpha Buffer
    0,                                          // Shift Bit Ignored
    0,                                          // No Accumulation Buffer
    0, 0, 0, 0,                                 // Accumulation Bits Ignored
    16,                                         // 16Bit Z-Buffer (Depth Buffer)
    0,                                          // No Stencil Buffer
    0,                                          // No Auxiliary Buffer
    PFD_MAIN_PLANE,                             // Main Drawing Layer
    0,                                          // Reserved
    0, 0, 0                                     // Layer Masks Ignored
  };

  if(!(hDC=GetDC(hWnd) ) ) {                       // Did We Get A Device Context?
    KillGLWindow();                             // Reset The Display
    MessageBox(NULL, "Can't Create A GL Device Context.", "ERROR", MB_OK|MB_ICONEXCLAMATION);
    return false;                               // Return false
  }

  if(!(PixelFormat=ChoosePixelFormat(hDC, &pfd) ) ) { // Did Windows Find A Matching Pixel Format?
    KillGLWindow();                             // Reset The Display
    MessageBox(NULL, "Can't Find A Suitable PixelFormat.", "ERROR", MB_OK|MB_ICONEXCLAMATION);
    return false;                               // Return false
  }

  if(!SetPixelFormat(hDC, PixelFormat, &pfd) ) {     // Are We Able To Set The Pixel Format?
    KillGLWindow();                             // Reset The Display
    MessageBox(NULL, "Can't Set The PixelFormat.", "ERROR", MB_OK|MB_ICONEXCLAMATION);
    return false;                               // Return false
  }

  if(!(hRC=wglCreateContext(hDC) ) ) {             // Are We Able To Get A Rendering Context?
    KillGLWindow();                             // Reset The Display
    MessageBox(NULL, "Can't Create A GL Rendering Context.", "ERROR", MB_OK|MB_ICONEXCLAMATION);
    return false;                               // Return false
  }

  if(!wglMakeCurrent(hDC, hRC) ) {                  // Try To Activate The Rendering Context
    KillGLWindow();                             // Reset The Display
    MessageBox(NULL, "Can't Activate The GL Rendering Context.", "ERROR", MB_OK|MB_ICONEXCLAMATION);
    return false;                               // Return false
  }

  ShowWindow(hWnd, SW_SHOW);                       // Show The Window
  SetForegroundWindow(hWnd);                      // Slightly Higher Priority
  SetFocus(hWnd);                                 // Sets Keyboard Focus To The Window
  ReSizeGLScene(width, height);                   // Set Up Our Perspective GL Screen

  if(!InitGL() ) {                                // Initialize Our Newly Created GL Window
    KillGLWindow();                             // Reset The Display
    MessageBox(NULL, "Initialization Failed.", "ERROR", MB_OK|MB_ICONEXCLAMATION);
    return false;                               // Return false
  }


  // create mutex
  hMutex = CreateMutex(NULL, false, sMutexFileName);

  if(!hMutex) {
    MessageBox(NULL, "initMemoryMappedFile: CreateMutex() failed.\n", "ERROR", MB_OK|MB_ICONEXCLAMATION);
    return false;
  }

  // create file mapping to store dwMemoryFileSize bytes
  hFile = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, dwMemoryFileSize*sizeof(TCHAR), sMemoryFileName);

  if(!hFile) {
    MessageBox(NULL, "initMemoryMappedFile: CreateFileMapping() failed.\n", "ERROR", MB_OK|MB_ICONEXCLAMATION);
    return false;
  }
  else {
    pFile = MapViewOfFile(hFile, FILE_MAP_ALL_ACCESS, 0, 0, 0);

    if(!pFile) {
      MessageBox(NULL, "initMemoryMappedFile: MapViewOfFile() failed.\n", "ERROR", MB_OK|MB_ICONEXCLAMATION);
      return false;
    }
  }

  return true;                                    // Success
}

LRESULT CALLBACK WndProc(HWND hWnd,           // Handle For This Window
                         UINT uMsg,           // Message For This Window
                         WPARAM wParam,         // Additional Message Information
                         LPARAM lParam)         // Additional Message Information
{
  switch(uMsg)                                   // Check For Windows Messages
  {
  case WM_ACTIVATE:                           // Watch For Window Activate Message
  {
    if(!HIWORD(wParam) ) {                  // Check Minimization State
      active=true;                        // Program Is Active
    }
    else {
      active=false;                       // Program Is No Longer Active
    }

    return 0;                               // Return To The Message Loop
  }

  case WM_SYSCOMMAND:                         // Intercept System Commands
  {
    switch(wParam)                         // Check System Calls
    {
    case SC_SCREENSAVE:                 // Screensaver Trying To Start?
    case SC_MONITORPOWER:               // Monitor Trying To Enter Powersave?
      return 0;                           // Prevent From Happening
    }
    break;                                  // Exit
  }

  case WM_CLOSE:                              // Did We Receive A Close Message?
  {
    PostQuitMessage(0);                     // Send A Quit Message
    return 0;                               // Jump Back
  }

  case WM_KEYDOWN:                            // Is A Key Being Held Down?
  {
    keys[wParam] = true;                    // If So, Mark It As true
    return 0;                               // Jump Back
  }

  case WM_KEYUP:                              // Has A Key Been Released?
  {
    keys[wParam] = false;                   // If So, Mark It As false
    return 0;                               // Jump Back
  }

  case WM_SIZE:                               // Resize The OpenGL Window
  {
    ReSizeGLScene(LOWORD(lParam), HIWORD(lParam) );  // LoWord=Width, HiWord=Height
    return 0;                               // Jump Back
  }
  }
  // Pass All Unhandled Messages To DefWindowProc
  return DefWindowProc(hWnd, uMsg, wParam, lParam);
}

int WINAPI WinMain(HINSTANCE hInstance,          // Instance
                   HINSTANCE hPrevInstance,      // Previous Instance
                   LPSTR lpCmdLine,          // Command Line Parameters
                   int nCmdShow)           // Window Show State
{
  MSG msg;                                    // Windows Message Structure
  BOOL done=false;                             // Bool Variable To Exit Loop

  fullscreen=false;                           // Windowed Mode

  // Create Our OpenGL Window
  if(!CreateGLWindow("Bioloid Simulation", WIDTH, HEIGHT, 16, fullscreen) ) {
    return 0;                                   // Quit If Window Was Not Created
  }

  while(!done)                                    // Loop That Runs While done=false
  {
    if(PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) ) { // Is There A Message Waiting?
      if(msg.message==WM_QUIT) {             // Have We Received A Quit Message?
        done=true;                          // If So done=true
      }
      else {                                  // If Not, Deal With Window Messages
        TranslateMessage(&msg);             // Translate The Message
        DispatchMessage(&msg);              // Dispatch The Message
      }
    }
    else {                                      // If There Are No Messages
      // Draw The Scene.  Watch For ESC Key And Quit Messages From DrawGLScene()
      if(active) {                           // Program Active?
        if(keys[VK_ESCAPE]) {              // Was ESC Pressed?
          done=true;                      // ESC Signalled A Quit
        }
        else {                              // Not Time To Quit, Update Screen
          DrawGLScene();                  // Draw The Scene
          SwapBuffers(hDC);               // Swap Buffers (Double Buffering)
        }
      }

      if(keys[VK_F1]) {                      // Is F1 Being Pressed?
        keys[VK_F1]=false;                  // If So Make Key false
        KillGLWindow();                     // Kill Our Current Window
        fullscreen=!fullscreen;             // Toggle Fullscreen / Windowed Mode
        // Recreate Our OpenGL Window
        if(!CreateGLWindow("Bioloid Simulation", WIDTH, HEIGHT, 16, fullscreen) ) {
          return 0;                       // Quit If Window Was Not Created
        }
      }
    }
  }

  // Shutdown
  KillGLWindow();                                 // Kill The Window
  return(msg.wParam);                            // Exit The Program
}

