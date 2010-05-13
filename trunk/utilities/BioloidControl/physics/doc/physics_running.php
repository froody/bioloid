<?
$result = '
<h2>Running the bioloidcontrol Physics Simulator</h2>
<p>
A simple run of the simulator (using the default configuration file in <code>bioloidcontrol/physics/physics.conf</code>) :
</p>
<code>
<pre>
cd bioloidcontrol/build/physics
make && ./physics
</pre>
</code>
<p>
In <code>physics.conf</code>, there are a number of user-settable parameters.  
Most of them (with the default values) are in the configuration file, but commented out just to illustrate what&quot;s there.
In addition, any of the options in <code>physics.conf</code> can be overridden on the <code>./physics</code> command line.  
</p>
<p>
Drag (while clicking) the mouse on the GUI to change viewpoint.  Press &quot;r&quot; to respawn the robot (if it falls over, for instance).
</p>

<h3>Running the bioloidcontrol Physics Simulator with the Controller</h3>
<p>
To produce the videos on YouTube, the following set-up was used :
</p>
<code>
<pre>
# STEP 1 - Terminal Session 1
cd bioloidcontrol/build/viewer
./viewer -geometry 320x480+0+0 &
</pre>
</code>
<code>
<pre>
# STEP 2 - Terminal Session 2
cd bioloidcontrol/build/physics
make && ./physics --simulation.time_end=0 --gui.width=320 --gui.height=480 
</pre>
</code>
<p>
Starting the controller part needs the physics simulation running first (since it connects to the physics electronics as a client) :
</p>
<code>
<pre>
# STEP 3 - Terminal Session 1
cd bioloidcontrol/build/main
./bioloid -config ../../main/config_physics.xml 
</pre>
</code>
<p>
From the simulator command-line, one can then run (as a sample) :
<ul>
<li>playshow</li>
<li>test 0.05 5 0 0</li>
<li>...</li>
</ul>
</p>

<h3>Running the bioloidcontrol Physics Simulator from a Perl controller</h3>
<p>
For people who are familiar with perl (great for quickly hacking something together : see C++ for something diametrically opposite): 
</p>
<code>
<pre>
cd bioloidcontrol/physics/misc
perl Bioloid.pl
</pre>
</code>
<p>
This chunk of code reads from the &quot;IMU&quot; continuously, and looks for key-presses.  Try tapping some of the number keys.  
And then look at the code to find out what&quot;s going on.
</p>

<h3>Running the bioloidcontrol Physics Simulator : Next Steps</h3>
<p>
The physics module is built so that its objects can be used independently.  There&quot;s no need to run the GUI.  
If your &quot;robot brain&quot; just wants to simulate the physics, then there&quot;s no need to run the bioloid bus - just control the servos directly.
</p>
<p>
Most likely, I&quot;ll be embedding a way of manipulating the physics objects within perl - 
developement is just so much quicker in a higher level langauge like perl :-)
</p>

<h3>Adapting the bioloidcontrol Physics Simulator : Robot Definition XML</h3>
<p>
Have a look at the Excel file in <code>&quot;bioloidcontrol/physics/xml-generation&quot;</code>.  
</p>
<p>
This file contains the source of all the parameters for the robot that&quot;s being simulated - 
and defines the relationship between all the parts.  Also, it collects together these relationships in the last sheet, 
of which the coloured background part can be copy-pasted into the <code>&quot;PortugeseHumanoid.txt&quot;</code> file.
From there, the following updates the humanoid definition file :
</p>
<code>
<pre>
cd bioloidcontrol/physics/xml-generation
perl perl-to-xml.pl > ../physics_humanoid.xml
</pre>
</code>
<p>
The idea of creating the robot this way was to ensure that people could see (and understand) exactly the steps required to get to the 
end XML definition of the robot.  
</p>
<p>
Ideally, there would be the one extra step of converting the joint definitions into the 
DH-parameterization used by the <code>bioloidcontrol/main/bioloid</code> program : 
which could then be made completely compatible end-to-end : all driven by one parameterization.  
</p>

<h3>Taking a video of the bioloidcontrol Physics Simulator</h3>
<p>
There are a couple of useful things in the <code>bioloidcontrol/physics/frame</code> folder which were used to produce the YouTube videos.
</p>
';
return $result;
?>
