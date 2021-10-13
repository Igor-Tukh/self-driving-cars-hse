## Instructions:
* Start ROS Master: `roscore`;
* Run `turtlesim_node`: `rosrun turtlesim turtlesim_node`;
* Spawn an additional ("destination") turtle: `rosservice call /spawn "{x: 10.0, y: 10.0, theta: 3.1415, name: 'destination'}"`;
* Run the node responsible for the movement `rosrun hw01 turtle_navigator.py`.
