##Ellis Saupe ems236 George Vo Nicole Graf Lab 6

Our launch file as the same as the lab 3 launch file.  
Gazebo is incredibly slow to launch, so rather than sleep our robot at launch, we require that osrf_gear sample_environment.launch be launced before our node.

roslaunch ariac_runner grading.launch