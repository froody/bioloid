# [ROS](http://code.google.com/p/bioloid/source/browse/#svn/trunk/utilities/ROS/awesomeo_controller) #

I'm in the process of learning ROS, so I started applying it to my Bioloid robot. The code does a fair bit right now - an IK solver on a simulated robot that looks somewhat like a TypeA Bioloid. As I progress in my understanding of ROS I'll add more features including an accurate model and communication with an actual robot!

# [Bioloid Control](http://code.google.com/p/bioloid/source/browse/#svn/trunk/utilities/BioloidControl) #

An integrated package of a physics simulator, CM-5 firmware and a console app which talks to the CM-5 over serial. Runs on Windows & Linux, was originally hosted [here](https://sourceforge.net/projects/bioloidcontrol/), documentation will be moved across shortly.

# [Dynamixel Monitor](http://code.google.com/p/bioloid/source/browse/#svn/trunk/utilities/DynamixelMonitor) #

A small OS X app which talks over the Dynamixel bus (via USB2Dynamixel) to your robot, which means you can use it without touching the firmware on your controller. It can control individual servos by dragging sliders around, view all the properties of servos in real time, and has work-in-progress support for playing back motion files.

# [CM-510 firmware](http://code.google.com/p/bioloid/source/browse/#svn/trunk/firmware/motion) #

This is something I hacked up from different bits of code I found on the net. It's rather hacky, but it runs on the CM-510 and plays motion files (kinda) with linear interpolation for joint speeds. There's relatively very little code, so it's good as a starting point to learn about how the CM-510 works.