# Agile Robotics Industrial Automation Competition : Person avoidance and Sensor Blackout

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Videos

[Video 1](https://youtu.be/DbxpUw12As4) </br>
[Video 2](https://youtu.be/JfT_rkOfotc) </br>
## Team members
1. Pradeep Gopal
2. Rajesh 
3. Govind
4. Dakota Abernathy
5. Cheng Chen

## Steps to Run the package

1. Copy and paste the package in the /ariac_ws/src/rwa3_group1 directory
2. Open a terminal and type the following commands
3. cd /ariac_ws
4. catkin build <your_package_name>
5. source devel/setup.bash
6. roslaunch <your_package_name> rwa5.launch load_moveit:=true

Wait till the terminal says "you can start planning now"

7. Open a new terminal and enter the following command to run the node.
8. cd /ariac_ws
9. source devel/setup.bash
10. rosrun <your_package_name> rwa5_node
