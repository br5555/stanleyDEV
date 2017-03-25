"# Stanley" 
<br>
<h2>Steps required to run simulation:</h2>
<br>
<pre>1.cd catkin_ws</pre>
<br>
<pre>2.roslaunch stanley full.launch</pre>
<br>



<h4>full.launch contains:</h4>

  <p>
  <pre>1.stanley.launch(running stanley algoritham)</pre>
  <br>
     <pre>2.simulator.launch(running my.launch and rviz.launch)</pre> 
      <br>
      <pre>   2.1. my.launch(2D graphical view robot on map)</pre>  
        <br>
        <pre>   2.2. rviz.launch(3D graphical view with possibility to subscribe to topics that you inerest in)</pre>
        <br>
  <pre >3.publish.launch (contains three testing paths:circular ,8 path, linear(8 path and linear are commented) you can only choose
  one(other must be commented))</pre></p>
