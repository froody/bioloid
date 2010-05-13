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

#include "main.h"
#include "util.h"
#include "world.h"
#include "scene.h"

#include "bioloid.h"

// # http://www.boost.org/
#include <boost/program_options.hpp>
// -lboost_program_options
namespace po=boost::program_options;

// This is for the asynchronous GUI process
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <iterator>
#include <fstream>

int main(int ac, char *av[]) {
  int gui, gui_texture, gui_width, gui_height;
  int port_enable, port;

  float simulation_time_end, simulation_time_dilation, simulation_time_step, simulation_gravity;
  float geometry_variability_factor;

  string robot_file;
  int load_debug; // Amount of debug information to display during robot load

  vector<string> files;

  po::options_description desc("Bioloid simulation options");
  desc.add_options()
  ("help", "produce help message")
  ("gui.enable",  po::value<int>(&gui)->default_value(1),          "display gui (0=no)")
  ("gui.texture", po::value<int>(&gui_texture)->default_value(1),  "display gui textures (0=no)")
  ("gui.width",   po::value<int>(&gui_width)->default_value(800),  "gui width")
  ("gui.height",  po::value<int>(&gui_height)->default_value(600), "gui height")
//  ("config-path,I", po::value< vector<string> >(), "config files path")
//  ("config", po::value< vector<string> >(), "config file")
  ("interface.enable", po::value<int>(&port_enable)->default_value(1), "enable the Bioloid bus simulation")
  ("interface.port", po::value<int>(&port)->default_value(7777), "interface port")
  ("simulation.gravity", po::value<float>(&simulation_gravity)->default_value(9.81f), "Gravity in m/s/s")
  ("simulation.time_dilation", po::value<float>(&simulation_time_dilation)->default_value(1.0f), "Ratio of wall time to simulation time\n   (0 for as-quick-as-possible)")
  ("simulation.time_end", po::value<float>(&simulation_time_end)->default_value(20.0f), "Time simulation should run (0 for never-ending)")
  ("simulation.time_step", po::value<float>(&simulation_time_step)->default_value(0.01f), "Timestep used for simulations")
  ("config", po::value< vector<string> >(&files), "config file file to load")
  ("world.robot_file", po::value<string>(&robot_file)->default_value("../../physics/physics_humanoid.xml"), "robot definition file")
  ("world.geometry_variability", po::value<float>(&geometry_variability_factor)->default_value(0.01f), "(random) variability to subtract from geometry in robot file")
  ("world.load_debug", po::value<int>(&load_debug)->default_value(0), "debug information while loading world")
  ;

  po::positional_options_description p;
  p.add("config", -1);

  po::variables_map vm;

  // This one doesn't handle positional parameters
  // po::store(po::parse_command_line(ac, av, desc), vm);

  //  Do all the command line processing
  po::store(po::command_line_parser(ac, av).options(desc).positional(p).run(), vm);

  po::notify(vm);

  if(files.size()==0) {
    cout << "Using default configuration file '../../physics/physics.conf'" << endl;
    files.push_back("../physics/physics.conf");
  }

  for(vector<string>::iterator f=files.begin(); f!=files.end(); ++f) {
    ifstream config;
    config.open(f->c_str() ); // , ios_base::in
    if(config.is_open() ) {
      cout << "Config file '" << *f << "'" << endl;
      po::store(po::parse_config_file(config, desc), vm);
    }
    else {
      cerr << "Config file '" << *f << "' not found" << endl;
    }
  }
  po::notify(vm);

  if(vm.count("help") ) {
    cout << desc << endl;
    return 1;
  }

  if(1) {
    cout << "GUI ";
    if(gui) {
      cout << "Enabled : (" << gui_width << "x" << gui_height << ")" << endl;
    }
    else {
      cout << "Disabled" << endl;
    }
  }

  if(1) {
    string port_str=(port_enable>0) ? "enabled" : "disabled";
    cout << "Interface port " <<  port << " " << port_str << "\n";
  }

  cout << "Loading World" << endl;

  dInitODE();

  World w;
  w.simulation_time_end      = simulation_time_end;
  w.simulation_time_dilation = simulation_time_dilation;
  w.setGravity(simulation_gravity);
  w.setTimestep(simulation_time_step);
  w.load_debug=load_debug;
  w.setGeometryVariabilityFactor(geometry_variability_factor);

  if(!w.loadXML( (char *)robot_file.c_str() ) ) {
    cerr << "Unable to read in robot definition" << endl;
    return(0);
  }

  cout << "Creating Bus (with electronics)" << endl;

  // This creates the Bioloid bus object
  BBus bus(port_enable ? port : -1);

  // Go through the World structure, looking for servos which are AX12 devices
  // and then create then in the BBus structure
  for(vector<Servo>::iterator servo=w.servo_list.begin(); servo!=w.servo_list.end(); servo++) {
    int id=servo->id;
    if( (id>0) && strequali(servo->type.c_str(), "ax12") ) {
      BDevice *dev=new BDeviceAX12(id, &*servo);
      bus.connect_device(dev);
    }
  }

  for(vector<Sensor>::iterator sensor=w.sensor_list.begin(); sensor!=w.sensor_list.end(); sensor++) {
    int id=sensor->id;
    if( (id>0) && strequali(sensor->type.c_str(), "imu") && strequali(sensor->version.c_str(), "ideal") ) {
      cout << "Adding IMU to bus : " << id << endl;

      BDevice *dev=new BDeviceIMUideal(id, &*sensor);
      bus.connect_device(dev);
    }
  }

  if(gui) { // Start up a GUI thread that monitors the World asynchronously
    cout << "---------------------------\n" << "Starting GUI" << endl;
    boost::thread gui_thread(
      boost::bind(draw_scene, &w, gui_texture, gui_width, gui_height)
      );
  }

  cout << "---------------------------\n" << "Starting physics simulation" << endl;

  float time_display=0.0, time_display_step=1.0;
  w.resetTime();
  while( (w.simulation_time_end<=0) || (w.simulation_time < w.simulation_time_end) ) { // Simulate as long as required
    float simulated_until=w.simulation_time*w.simulation_time_dilation;

    if(simulated_until>time_display-simulation_time_step/2.0) {
      cout << "Simulation time : " << formatted_double(simulated_until, 2) << endl;

/*
            const dReal *pos=dBodyGetPosition(w.body_list[0].id);
            const dReal *vel=dBodyGetLinearVel(w.body_list[0].id);
            cout << "Simulation time : " << simulated_until << " Position : " << pos[2] << " Velocity : " << vel[2] << endl;
 */
      time_display+=time_display_step;
    }

    float time_until_next_step = simulated_until-w.elapsedTime();
    if(time_until_next_step>0) { //sleep until that time
      sleep_ms(time_until_next_step * 1000.0);
    }
    w.simulateSingleStep(&nearCallback);
  }

  dCloseODE();
  return 0;
}

/* Other examples of configuration options usage */

/*
    if(vm.count("compression")) {
        cout << "Compression level was set to " << vm["compression"].as<int>() << ".\n";
    }
    else {
        cout << "Compression level was not set.\n";
    }
 */

/*
    po::options_description config("Configuration");
    config.add_options()
        ("include-path,I", po::value< vector<string> >()->composing(), "include path")
    ;
   //Note the call to the composing method in the declaration
    of the "include-path" option. It tells the library that values
    from different sources should be composed together, as we'll see shortly.
 */

