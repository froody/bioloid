<?
$result = '
<h2>Building &quot;bioloidcontrol&quot; physics</h2>
<p>
My build environment is Fedora-centric, though these instructions should be applicable to any *NIX environment.
</p>
<p>
Also, because I&quot;ve used the &quot;cmake&quot; machinery, it should be buildable on Windows using a number of different compilers (please let me know if anyone has any problems/insight into doing a Windows build).
</p>

<h3>Installing Prerequisites for &quot;bioloidcontrol&quot;</h3>
Before these packages can be built on Fedora, there are a few prerequisites (Fedora uses &quot;yum&quot;, other Linuxes may use &quot;apt get&quot;, for example) :
<ul>
<li>main/bioloid requires <code>&quot;yum install ncurses-devel&quot;</code></li>
<li>viewer/viewer requires <code>&quot;yum install freeglut freeglut-devel mesa-libGLU-devel&quot;</code></li>
</ul>
<p>
Building the physics module is more involved, since the physics simulation has a dependencies on :
<ul>
<li>ODE - physics library</li>
<li>STL - Standard Template Libary (for </li>
<li>Boost - C++ template library (for command-line parsing, threading, ...)</li>
<li>asio - for TCP sockets (soon to be incorporated in Boost)</li>
</ul>
</p>

<h4>General installation requirements for Fedora</h4>
<p>
Fedora installation :
<code>
<pre>
# yum install svn cmake gcc-c++ 
# yum install automake autoconf libtool
# yum install mesa-libGLU-devel freeglut-devel libXmu-devel libXi-devel
# yum install boost boost-devel asio-devel 
</pre>
</code>
</p>

<h4>Installation of &quot;asio&quot; libraries - Easy way</h4>
<p>
In boost versions before 1.35, there is no boost-asio library (for the TCPIP sockets) so the asio-devel rpm is required from the fedora repositories : 
<code>&quot;yum install asio-devel&quot;</code>.
</p>

<h4>Installation of &quot;asio&quot; libraries - Long way</h4>
<p>
Alternatively, one can build the asio library from scratch :
</p>
<code>
<pre>
curl -O http://internap.dl.sourceforge.net/sourceforge/asio/asio-1.0.0.tar.gz
tar -xzf asio-1.0.0.tar.gz
cd asio-1.0.0
./configure
make
make install # (this step as root)
</pre>
</code>

<h4>Installation of &quot;ODE&quot; libraries</h4>
<p>
Although the ode libraries are installable via &quot;yum&quot; (and Fedora now has version 0.10.1 already), the physics library makes use of its graphics sub-library.  So the source much be installed anyway, and compiled :
</p>
<code>
<pre>
curl -O http://voxel.dl.sourceforge.net/sourceforge/opende/ode-0.10.1.tar.gz
tar -xzf ode-0.10.1.tar.gz
cd ode-0.10.1
./autogen.sh
./configure
make
</pre>
</code>
<p>
To test that the build of ODE works, within the ode-0.10.1 directory : 
</p>
<code>
<pre>
cd drawstuff/dstest
./dstest  
</pre>
</code>
<p>
Then, so that the code can link in correctly (without installing ODE machine-wide), create a link in the physics directory to the ode-0.10.1 directory.
</p>
<code>
<pre>
cd bioloidcontrol/physics
ln -s PATH-TO-DIRECTORY/ode-0.10.1 ode
</pre>
</code>

<h4>Enabling the perl test-harness</h4>
<p>
This is only required if you want to play around with the &quot;perl Bioloid.pl&quot; tester in the misc/ directory - choose one of :
<ul>
<li>yum install perl-TermReadKey</li>
<li>perl -MCPAN -e "install Term::ReadKey"</li>
<li>cpan Term::ReadKey</li>
</ul>
</p>

<h3>Building the &quot;bioloidcontrol&quot; makefiles/project files</h3>
<p>
Using &quot;cmake&quot; means that the build directory is separate from the code.
So the build sequence is : 
<code>
cd bioloidcontrol/build
cmake ..
</code>
which builds all the compilation environment.
</p>
<p>
Then, the actual files (for the viewer, controller and the physics simulator) can be built in one pass (from within the bioloidcontrol/build directory) with :
<code>
make
</code>
also, &quot;make&quot; can be run in the build subdirectories, to remake individual parts of the project - have a look at the &quot;running&quot; pages for details.
</p>
';
return $result;
?>
