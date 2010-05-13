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

#include <ode/ode.h>
#include "drawstuff/drawstuff.h"

// #include "main.h"
#include "world.h"
#include "scene.h"

World *world_in_scene;
WorldState *world_state_store_scene;

void before_simulations() {
// From Ctrl+V : Viewpoint = (1.5924,0.5740,1.0300,-160.5000,-21.0000,0.0000)
//	static float xyz[3] = {1.5924f,0.5740f,1.0300f};  // mdda - nicer angle
//	static float hpr[3] = {-160.5000f,-21.0000f,0.0000f};

// From Ctrl+V : (0.7274,0.3451,0.4000,-152.5000,-14.0000,0.0000)
//	static float xyz[3] = {0.7274f,0.3451f,0.4000f};  // mdda - nicer angle
//	static float hpr[3] = {-152.5000f,-14.0000f,0.0000f};

  // From Ctrl+V : Viewpoint = (0.5327,0.1731,0.2900,-160.5000,-4.5000,0.0000)
//  static float xyz[3] = {0.5327, 0.1731, 0.2900};  // mdda - nicer angle
//  static float hpr[3] = {-160.5000, -4.5000, 0.0000};

  // Side View Viewpoint = (0.0006,0.5254,0.3300,-87.0000,-14.5000,0.0000)
//  static float xyz[3] = {0.0006,0.5254,0.3300};
//  static float hpr[3] = {-87.0000,-14.5000,0.0000};

  // Similar to controller viewpoint : (0.5369,-0.1739,0.2900,161.5000,-14.5000,0.0000)
  static float xyz[3] = {0.5369, -0.1739, 0.2900};
  static float hpr[3] = {161.5000, -14.5000, 0.0000};


  dsSetViewpoint(xyz, hpr);

  world_in_scene->resetTime();
}

class GraphicsGeom {
public:
  int type;
  dReal position[3];
  dReal rotation[12];
  float r, g, b; // RGB

// for type==dBoxClass
  dVector3 sides;

  GraphicsGeom(Geom &geom);
};

GraphicsGeom::GraphicsGeom(Geom &geom) {
  int i;
  dGeomID _id=geom.id;

  const dReal *_pos = dGeomGetPosition(_id);
  for(i=0; i<3; i++) {
    position[i]=_pos[i];
  }

  const dReal *_R = dGeomGetRotation(_id);
  for(i=0; i<12; i++) {
    rotation[i]=_R[i];
  }

  r=geom.r;
  g=geom.g;
  b=geom.b;

  type = dGeomGetClass(_id);
  switch(type) {
  case dBoxClass:
    dGeomBoxGetLengths(_id, sides);
    break;

  default:
    cerr << "Unknown dClass for geom '" << geom.name << "'\n";
  }
}

void every_display_frame(int pause) {
  // In a multi-threaded world, don't create physics action because of the GUI ...
  // we must sample the state of the world_in_scene physics, in an instant when
  // it's not updating - and save that into our own buffer

//	world_in_scene->simulateToMatchCurrentWallClock(&nearCallback);

  vector <GraphicsGeom> graphics_geom;

  if(1) {  // This is to scope the mutex easily
    // Wait for World mutex to be free
    boost::mutex::scoped_lock scoped_lock(world_in_scene->guard_ode_data);

    // Copy data into a vector of our own for display
    for(vector<Geom>::iterator geom=world_in_scene->geom_list.begin(); geom!=world_in_scene->geom_list.end(); ++geom) {
      GraphicsGeom gg(*geom);
      graphics_geom.push_back(gg);
    }
    // Release the mutex - as scoped_lock goes out of scope...
  }

  // Now display the data - free of thread-safe worries
  for(vector<GraphicsGeom>::iterator gg=graphics_geom.begin(); gg!=graphics_geom.end(); ++gg) {
    dsSetColor(gg->r, gg->g, gg->b);
    if(gg->type == dBoxClass) {
      dsSetTexture(DS_WOOD);
      dsDrawBoxReal(gg->position, gg->rotation, gg->sides);
//			cout << "Draw box for '" << geom->name << "' at (" << pos[0] << "," << pos[1] << "," << pos[2] << "),";
//      cout << " sized :(" << sides[0] << "x" << sides[1] << "x" << sides[2] << ")\n";
    }
  }
}

void key_pressed_in_visualization_display(int cmd) { // This is asynchronos
  if(cmd == 'r') {
    world_state_store_scene->restore(); // Revert to original
    world_in_scene->resetTime();
  }
  else {
    dsPrint( (char *)"Received command %d (`%c') in simulation window\n", cmd, cmd);
  }
}

static void after_simulations() {
  dsPrint( (char *)"\nAbort GUI (simulation may still be running)...\n");
}

void draw_scene(World *w, int textures, int x_size, int y_size) {
  world_in_scene=w;

  world_state_store_scene=new WorldState(world_in_scene);
  world_state_store_scene->save();

  // setup pointers to callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start   = &before_simulations;
//	fn.step    = &every_simulation_loop;
  fn.step    = &every_display_frame;
  fn.command = &key_pressed_in_visualization_display;
  fn.stop    = &after_simulations;
//	fn.path_to_textures = 0;	// uses default
  fn.path_to_textures = (char *)"../../physics/ode/drawstuff/textures";

  // run simulation
  int ds_argc=1;
  char *ds_argv[10]={ (char *)"physics_drawstuff"};
  if(!textures) {
    ds_argv[ds_argc++]=(char *)"-notex";
  }
  if(0) {
    ds_argv[ds_argc++]=(char *)"-noshadows";
  }

  dsSimulationLoop(ds_argc, ds_argv, x_size, y_size, &fn);
//	dsSimulationLoop(0,0, 400,300, &fn);

  // This runs until the viewpoint closes...

//	delete world_state_store_scene;
}

