<h1>Stanley algorithm package</h1>
<br>
<h2>Installation instructions</h2>
<br>
<p><li>The driver has been tested with ROS Kinetic on Ubuntu 16.04 64-bit.</li>
<li>stdr_simulator</li>
<a>http://wiki.ros.org/stdr_simulator/Tutorials/Set%20up%20STDR%20Simulator</a></p>
<br>
<h2>Steps required to run simulation:</h2>
<br>
<pre>1.cd catkin_ws</pre>
<br>
<pre>2.roslaunch stanley full.launch</pre>
<br>



<h3>full.launch contains:</h3>

  <ol>
  <li>1.stanley.launch(running stanley algoritham)</li>
  <br>
     <li>2.simulator.launch(running my.launch and rviz.launch)</li> 
      <br>
     <ul> <li>   2.1. my.launch(2D graphical view robot on map)</li> 
        <br>
        <li>   2.2. rviz.launch(3D graphical view with possibility to subscribe to topics that you inerest in)</li></ul>
        
  <li >3.publish.launch (contains three testing paths:circular ,8 path, linear(8 path and linear are commented) you can only choose
  one(other must be commented))</li></ol>
