<?
$result = '
<h2>About the bioloidcontrol Physics Simulator</h2>
<h3>What</h3>
<p>
The physics simulator now in the bioloidcontrol project is a close-to-realistic simulation of the
bioloid humanoid robot.  Here are some key features :
<ul>
<li>Written in C++ : with a decent object hierarchy (IMHO)</li>
<li>Complete bioloid bus simulator drivable via TCP (all commands)</li>
<li>Realistic physics : dimensions, masses, inertias taken from Portugeuse research group</li>
<li>Robot parameters created from Excel file (via a perl-to-XML translator)</li>
<li>Bioloid components (servos, IMUs, etc) react to bioloid message packets realistically</li>
<li>Code is readable and extensible (and open source)</li>
</ul>
</p>
<p>
There are videos - via YouTube: 
</p>
<h4>Physics simulation controlled by Controller - playshow</h4>
<p>
This has the two parts of bioloidcontrol communicating with each other (and executing &quot;playshow&quot; in the controller main window) :
</p>
<object width="425" height="344"><param name="movie" value="http://www.youtube.com/v/b5-BC4XJdVk&hl=en&fs=1"></param><param name="allowFullScreen" value="true"></param><embed src="http://www.youtube.com/v/b5-BC4XJdVk&hl=en&fs=1" type="application/x-shockwave-flash" allowfullscreen="true" width="425" height="344"></embed></object>
<h4>Physics simulation controlled by Controller - walking</h4>
<p>
This has the two parts of bioloidcontrol communicating with each other (and executing &quot;test 0.05 5 0 0&quot; in the controller main window) :
</p>
<object width="425" height="344"><param name="movie" value="http://www.youtube.com/v/r1LLr6wxt8Q&hl=en&fs=1"></param><param name="allowFullScreen" value="true"></param><embed src="http://www.youtube.com/v/r1LLr6wxt8Q&hl=en&fs=1" type="application/x-shockwave-flash" allowfullscreen="true" width="425" height="344"></embed></object>
<h4>Physics simulation controlled by perl - passive-balancing</h4>
<p>
This is just the physics simulator being controlled by the perl snippet in misc/ (via a TCP socket).  The robot&quot;s
movements are just in response to me pressing the &quot;4&quot; key (and then respawning it in the GUI with &quot;r&quot;) :
</p>
<object width="425" height="344"><param name="movie" value="http://www.youtube.com/v/zVjzq-suE0M&hl=en&fs=1"></param><param name="allowFullScreen" value="true"></param><embed src="http://www.youtube.com/v/zVjzq-suE0M&hl=en&fs=1" type="application/x-shockwave-flash" allowfullscreen="true" width="425" height="344"></embed></object>
<h3>Why</h3>
<p>
For me : Because I didn&quot;t want to get bogged down in hardware details before I had a chance to see whether my
controller ideas worked.  And I didn&quot;t want to pay for WebBots, and player/stage/gazebo wouldn&quot;t compile on my
non-GPU-accelerated laptop.
</p>
<p>
For you : Because you might want to test stuff without wearing out your physical robot.
</p>
<h3>How</h3>
<p>
The physics simulator &quot;ODE&quot; does most of the heavy lifting for the simulation.  Of course, there&quot;s a lot to 
do in between receiving a message on the bioloid bus to seeing whether the robot topples over in 3-D...
</p>
<h3>Where</h3>
<p>
Available via anonymous SVN on bioloidcontrol.sourceforge.net (all under the GPL2).
</p>
<h3>Who</h3>
<p>
The main bioloidcontrol &quot;controller&quot; was written by Rainer Jaekel (a.k.a. &quot;cosa&quot;).  
This new physics module was added by Martin Andrews (mostly appearing as &quot;mdda&quot;).
</p>
<h3>When</h3>
<p>
<ul>
<li><b>Right now</b>;  </li>
<li>Comments welcome;  </li>
<li>Code even more welcome...  </li>
</ul>
</p>
';
return $result;
?>
