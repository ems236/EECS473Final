
### Nicole Graf nmg63, Ellis Saupe ems236, George Vo gbv6 Final Project

Our launch file as the same as the labs 3 and 6 launch file.  
Gazebo is incredibly slow to launch, so rather than sleep our robot at launch, we require that osrf_gear sample_environment.launch be launced before our node.

roslaunch osrf_gear sample_environment.launch
roslaunch ariac_runner grading.launch

Our arm can complete phase 2 pretty effectively.  It can grab the correct number of parts for an order, place them in the correct position on the tray [not the correct orientation], and submit the order.  It should reasonably work for any number of orders or any degree of order complexity, but it currently can only look in the bin that's right in front of it.  

#### Basic state transitions
go home
wait for orders
for each desired part
    move on a 3 point trajectory home to 0.3ish units above the part to a very small distance above the desired part
    activate gripper
    move on a 2 point trajectory from the part to 0.3ish units above the part
    if gripper is not attached
        repeat
    
    move on a 3 point trajectory from the current position to a hardcoded orientation where the arm is parallel to the linear track so it won't hit the bar to a linear movement to AGV1
    move on a 3 point trajectory from the dropoff position to 0.1 units above the center of the AGV tray to 0.1 units above the gripped parts target location
    deactivate gripper
    move on a 4 point trajectory from the current position to the dropoff orientation, to a linear movement home, to the home position

submit order

## Demo Video
https://drive.google.com/file/d/12dg71DvT7-Uaz-hzrZQ3EDBa-Dg09SZk/view?usp=sharing