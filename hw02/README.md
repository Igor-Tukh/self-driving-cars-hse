## Instructions:
* Start ROS Master: `roscore`;
* Run `rosrun rviz rviz -d config.rviz` (to load UI settings I used);
* Replay topics record: `rosbag play 2011-01-25-06-29-26.bag` (however, I omitted the dataset file in repo);
* Run the implemented node: `rosrun hw02 filter.py`.
### Note:
In case you receive a message `[rospack] Error: package 'hw02' not found`, the following actions are required additionally:
* `catkin_make` (in this directory);
* `source devel/setup.bash`.
