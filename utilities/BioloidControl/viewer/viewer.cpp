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

**************************************************************************

    Tutorial Code from nehe.gamedev.net,
    modified by Rainer J�kel <rainer.jaekel@googlemail.com>

    Credits from Nehe:
    This code was created by Jeff Molofee '99 (ported to Linux/GLUT by Richard Campbell '99)
    If you've found this code useful, please let me know.
    Visit me at www.demonews.com/hosted/nehe
    (email Richard Campbell at ulmont@bellsouth.net)
**************************************************************************/


#include <GL/glut.h>    // Header File For The GLUT Library
#include <GL/gl.h>   // Header File For The OpenGL32 Library
#include <GL/glu.h>  // Header File For The GLu32 Library
#include <unistd.h>     // Header file for sleeping.

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */

#include <cstdlib>
#include <iostream>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>

using namespace std;

typedef struct
{
  float corner[8][3];
  unsigned char red, green, blue;
} Polyeder;

enum KEY_BINDINGS
{
  ROTXP = 'w',
  ROTXM = 's',
  ROTYP = 'd',
  ROTYM = 'a',
  ROTZP = 'e',
  ROTZM = 'q',
  TRANSYP = 'c',
  TRANSYM = 'x',
  TRANSXP = 'r',
  TRANSXM = 'f',
  ZOOMP = '2',
  ZOOMM = '1'
};

GLfloat rtri = 190.0f;              // Angle For The Triangle ( NEW )
GLfloat rquad = 0.0f;               // Angle For The Quad ( NEW )

GLfloat tx = 0.0f;
GLfloat ty = 0.0f;
GLfloat zz = 0.0f;
GLfloat rr = 0.0f;

int shmid;
void  *segptr;


// number of characters in memory-mapped file
#define POLYCOUNT 50
const int dwMemoryFileSize = POLYCOUNT * sizeof(Polyeder) + 2 * sizeof(int);

typedef struct
{
    int boxes_len;
    int changed;
    Polyeder boxes[POLYCOUNT];
} Message;


Message msg;

GLfloat LightAmbient[]= { 0.5f, 0.5f, 0.5f, 1.0f };                 // Ambient Light Values ( NEW )
GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f };              // Diffuse Light Values ( NEW )
GLfloat LightPosition[]= { 0.0f, 0.0f, 2.0f, 1.0f };                 // Light Position ( NEW )

/* The number of our GLUT window */
int window;

void drawQuad(Polyeder &poly, int x, int y, int z, int w, float scale) {
  glColor3f(poly.red / 255.0, poly.green / 255.0, poly.blue / 255.0);
  glVertex3f(scale*poly.corner[x][0], scale*poly.corner[x][1], scale*poly.corner[x][2]);
  glVertex3f(scale*poly.corner[y][0], scale*poly.corner[y][1], scale*poly.corner[y][2]);
  glVertex3f(scale*poly.corner[z][0], scale*poly.corner[z][1], scale*poly.corner[z][2]);
  glVertex3f(scale*poly.corner[w][0], scale*poly.corner[w][1], scale*poly.corner[w][2]);
}

void DrawGLScene(void)                                  // Here's Where We Do All The Drawing
{
  usleep(10*1000);

  int changed = 0;
  int i, j;
  
  
  // get exclusive access to common memory region
  memcpy(&changed, (void*)( (unsigned char *)segptr+sizeof(int) ), sizeof(int) );
  if (changed != 0)
  {
      memcpy(&msg, segptr, sizeof(Message));
      changed = 0;
      memcpy((void*)( (unsigned char *)segptr+sizeof(int) ), &changed, sizeof(int) );
  }

  {
    float scale = 0.04;
    float scale2 = 0.1;
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear Screen And Depth Buffer
    
    glLoadIdentity();
    
    glTranslatef(tx, ty, -31.0 + zz);
    
    glRotatef(rtri, 0.0, 1.0, 0.0);
    glRotatef(rquad, 0.0, 0.0, 1.0);
    glRotatef(rr, 1.0, 0.0, 0.0);
    
    for(i=0; i<msg.boxes_len; i++) {
        glBegin(GL_QUADS);
        drawQuad(msg.boxes[i], 0, 1, 3, 2, scale); // top
        drawQuad(msg.boxes[i], 4, 0, 2, 6, scale); // right
        drawQuad(msg.boxes[i], 5, 4, 6, 7, scale); // bottom
        drawQuad(msg.boxes[i], 1, 5, 7, 3, scale); // left
        drawQuad(msg.boxes[i], 4, 5, 1, 0, scale); // behind
        drawQuad(msg.boxes[i], 6, 7, 3, 2, scale); // front
        glEnd();
    }

  }
  glutSwapBuffers();
  // Keep Going
}

/* A general OpenGL initialization function.  Sets all of the initial parameters. */
void InitGL(int Width, int Height)          // We call this right after our OpenGL window is created.
{
  glClearColor(58.0/255.0, 110.0/255.0, 165.0/255.0, 0.5f);

  glClearDepth(1.0);                // Enables Clearing Of The Depth Buffer
  glDepthFunc(GL_LESS);             // The Type Of Depth Test To Do
  glEnable(GL_DEPTH_TEST);          // Enables Depth Testing
  glShadeModel(GL_SMOOTH);          // Enables Smooth Color Shading

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();             // Reset The Projection Matrix

  gluPerspective(45.0f, (GLfloat)Width/(GLfloat)Height, 0.1f, 100.0f);  // Calculate The Aspect Ratio Of The Window

  glMatrixMode(GL_MODELVIEW);

  glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);             // Setup The Ambient Light
  glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);             // Setup The Diffuse Light
  glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);            // Position The Light

}

/* The function called when our window is resized (which shouldn't happen, because we're fullscreen) */
void ReSizeGLScene(int Width, int Height)
{
  if(Height==0) {              // Prevent A Divide By Zero If The Window Is Too Small
    Height=1;
  }

  glViewport(0, 0, Width, Height);      // Reset The Current Viewport And Perspective Transformation

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(45.0f, (GLfloat)Width/(GLfloat)Height, 0.1f, 100.0f);
  glMatrixMode(GL_MODELVIEW);
}

/* The function called whenever a key is pressed. */
#define AMOUNT 3.00
void keyPressed(unsigned char key, int x, int y)
{
  /* avoid thrashing this procedure */
  usleep(100);

  if(key == ROTYP) {
    rtri += 1.00f;
  }
  else
  if(key == ROTYM) {
    rtri -= 1.00f;
  }

  if(key == ROTXP) {
    rquad += 1.00f;
  }
  else
  if(key == ROTXM) {
    rquad -= 1.00f;
  }

  if(key == ROTZP) {
    rr += 1.0f;
  }
  else  if(key == ROTZM) {
    rr -= 1.0f;
  }

  if(key == TRANSYP) {
    tx += 0.5f;
  }
  else  if(key == TRANSYM) {
    tx -= 0.5f;
  }

  if(key == TRANSXP) {
    ty += 0.5f;
  }
  else  if(key == TRANSXM) {
    ty -= 0.5f;
  }

  if(key == ZOOMP) {
    zz += 1.0f;
  }
  else  if(key == ZOOMM) {
    zz -= 1.0f;
  }

  /* If escape is pressed, kill everything. */
  if(key == 0x1B) {
    /* shut down our window */
    glutDestroyWindow(window);

    /* exit the program...normal termination. */
  }
}

void platform_initMemoryMappedFile()
{
  // number of characters in memory-mapped file
  key_t key;
  int cntr;

  /* Create unique key via call to ftok() */
  key = ftok("/home/.", 'S');

  /* Open the shared memory segment - create if necessary */
  if( (shmid = shmget(key, dwMemoryFileSize, IPC_CREAT|IPC_EXCL|0666) ) == -1) {
    printf("Shared memory segment exists - opening as client\n");

    /* Segment probably already exists - try as a client */
    if( (shmid = shmget(key, dwMemoryFileSize, 0) ) == -1) {
      perror("shmget");
      return;
    }
  }
  else {
    printf("Creating new shared memory segment\n");
  }

  /* Attach (map) the shared memory segment into the current process */
  if( (segptr = shmat(shmid, 0, 0) ) == NULL) {
    perror("shmat");
    return;
  }
}

int main(int argc, char **argv) {
    
    msg.boxes_len = 0;
    
  bool command_line_geometry=false;
  for(int i=0; i<argc; i++) {
//		cout << "argv[" << i << "] = " << argv[i] << endl;
    if( (strcmp(argv[i], "--help")==0) || (strcmp(argv[i], "-help")==0) ) {
      cout << "Command line options : " << endl
           << "  -geometry  WxH+X+Y" << endl;
      return(1);
    }
    if(strcmp(argv[i], "-geometry")==0) {
      cout << "Got Geometry" << endl;
      command_line_geometry=true;
    }
  }

  platform_initMemoryMappedFile();

  /* Initialize GLUT state - glut will take any command line arguments that pertain to it or
     X Windows - look at its documentation at http://reality.sgi.com/mjk/spec3/spec3.html
         more Recent : http://www.opengl.org/resources/libraries/glut/spec3/spec3.html
   */
  glutInit(&argc, argv);

  /* Select type of Display mode:
     Double buffer
     RGBA color
     Alpha components supported
     Depth buffer */
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

  if(!command_line_geometry) {
    /* default to a 640 x 480 window */
    glutInitWindowSize(640, 480);
    //glutInitWindowSize(1024, 768);

    /* default is window starts at the upper left corner of the screen */
    glutInitWindowPosition(0, 0);
  }

  /* Open a window */
  window = glutCreateWindow("Bioloid Simulation");

  /* Register the function to do all our OpenGL drawing. */
  glutDisplayFunc(&DrawGLScene);

  /* Go fullscreen.  This is as soon as possible. */
  //glutFullScreen();

  /* Even if there are no events, redraw our gl scene. */
  glutIdleFunc(&DrawGLScene);

  /* Register the function called when our window is resized. */
  glutReshapeFunc(&ReSizeGLScene);

  /* Register the function called when the keyboard is pressed. */
  glutKeyboardFunc(&keyPressed);

  /* Initialize our window. */
  InitGL(640, 480);

  /* Start Event Processing Engine */
  glutMainLoop();

  return 1;
}

