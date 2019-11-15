
### Nicole Graf nmg63, Ellis Saupe ems236, George Vo gbv6 lab 6

Our launch file as the same as the lab 3 launch file.  
Gazebo is incredibly slow to launch, so rather than sleep our robot at launch, we require that osrf_gear sample_environment.launch be launced before our node.

roslaunch osrf_gear sample_environment.launch
roslaunch ariac_runner grading.launch

We had some trouble getting the arm to start nicely.  It usually hits the bin.  After that, it moves pretty nicely, but sometimes decides to spin in the middle of the trajectory.  We set exactly 2 trajectory points, and both seem to have reasonable positions when we print them out.  Our heuristics bound the first 3 joint angles between pi and 2pi.  We legitimately tried a lot of configurations, and this one worked best.  

## Demo Video
https://drive.google.com/file/d/1pk9Ku779mlocpwflMJb36cVpTizCYQ76/view?usp=sharing