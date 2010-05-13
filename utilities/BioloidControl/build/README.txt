This is a build directory for /experimental/ cmake building of the bioloidcontrol project

In the current configuration, this compiles 'physics', 'viewer' and 'main'.
This has been tested on Linux (Fedora).
Testing on Vista (DevCPP) coming soon.

To do a build :
1) type :
   cmake ..
   
2) Then you can type :
   make
   
3) and the targets will appear in sub-directories of this one...

4) Running the programs :
4a) cd physics; ./physics # will run the simulator
4b) cd main; ./bioloid    # will run the controller
4c) cd viewer;./viewer    # will run the simulator viewer
