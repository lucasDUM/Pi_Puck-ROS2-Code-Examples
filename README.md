# Pi_Puck-ROS2-Code-Examples
This repository is intended for educational use. 
Containing information of Pi-Puck simulation and ROS2 examples.
## Introduction
The Pi-puck is a [Raspberry Pi](https://www.raspberrypi.org) extension board for the [e-puck](http://www.gctronic.com/doc/index.php?title=E-Puck) and [e-puck2](http://www.gctronic.com/doc/index.php?title=e-puck2) robot platforms, designed and built as a collaboration between the [University of York](https://www.york.ac.uk/robot-lab/) and [GCtronic](http://www.gctronic.com).

[Webots](https://cyberbotics.com/) is an open source, professional mobile robot simulation software package. It can be used for a variety of applications.
I used it for rapid prototyping and carrying out swarm experiments.

This repository is a set of code examples and useful links to other ROS2 code.

For more information about the Pi-puck and Webots, see:
- GCtronic wiki page - http://www.gctronic.com/doc/index.php?title=Pi-puck
- Pi-puck on the YRL website - https://www.york.ac.uk/robot-lab/pi-puck/
- IROS 2017 paper - https://eprints.whiterose.ac.uk/120310/
- ANTS 2020 paper-https://eprints.whiterose.ac.uk/170703/1/ANTS_2020_Pi_puck_paper.pdf
- Webots ROS2 GitHub - https://github.com/cyberbotics/webots_ros2
- Webots documentation - https://cyberbotics.com/doc/guide/index
- Webots paper - https://link.springer.com/chapter/10.1007/978-3-540-78317-6_23

![](Assets/Pi-Puck-Sim.JPG)

The sections go as follows:
- Webots
- ROS2
  - Frontier detection
  - Path finding
  - Data collection
- Links to good tutorials and code bases

TF tree nd other important standards in REPs [103](https://www.ros.org/reps/rep-0103.html) and [105](https://www.ros.org/reps/rep-0105.html)
## Webots Setup
Webots has support for the [epuck2](https://www.gctronic.com/doc/index.php/e-puck2) robot however, this includes support for the Pi-Puck.
This is an early version of the Pi-Puck so a few modifications had to be made. 

I had to add the extra time of flight sensors (tof). So, I defined a proto file (This is just a ["node"](https://www.cyberbotics.com/doc/reference/nodes-and-functions) or obeject you can intereact with). I then added them into the [Webots_Ros2_Epuck Driver](https://github.com/lucas-d87u/Pi_Puck-ROS2-Code-Examples/blob/main/Webots-Modifications/driver.py). I also enabled the full use of the [IMU](https://en.wikipedia.org/wiki/Inertial_measurement_unit) sensor. After adding the component, this is handled automatically by Webots in the [Device Manager](https://github.com/cyberbotics/webots_ros2/blob/3a91326c3df2597a1a217d82ab1a60cdd7ce976b/webots_ros2_core/webots_ros2_core/devices/device_manager.py#L31).

The last change I made was to the launch files. In the [webots_ros2_core](https://github.com/cyberbotics/webots_ros2/tree/3a91326c3df2597a1a217d82ab1a60cdd7ce976b/webots_ros2_core/launch) file there is [robot_launch.py](https://github.com/cyberbotics/webots_ros2/blob/3a91326c3df2597a1a217d82ab1a60cdd7ce976b/webots_ros2_core/launch/robot_launch.py) I have replaced this with [robot_launch_multi.py](https://github.com/lucas-d87u/Pi_Puck-ROS2-Code-Examples/blob/main/Webots-Modifications/launch-files/robot_launch_multi.py) and this can be found in this repository. I have also ammended the corresponding launch file in [webots_ros2_epuck](https://github.com/cyberbotics/webots_ros2/tree/3a91326c3df2597a1a217d82ab1a60cdd7ce976b/webots_ros2_epuck) calling it [robot_multi_launch.py](Examples/blob/main/Webots-Modifications/launch-files/robot_multi_launch.py). 

These are effectively the same, except I added a for loop to append more controllers. These have to be set manually, though it should be simple to add a paramter to control this process.

Assuming you have set up a ROS2 [work space](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) and set up the [Webots ROS2 package](https://github.com/cyberbotics/webots_ros2). You should be able to (after replacing or adding these files) to call the launch file like this.

```
cd to your work space
source /opt/ros/foxy/setup.bash
. install/setup.bash
ros2 launch webots_ros2_epuck robot_multi_launch.py
```
This will launch the Pi-Puck in a maze world. 
I have also added the world and driver for the Pi-Puck with multiple small lidars as an example of a proto-type.2

## ROS2
## Frontier detection
This part consists of a OpenCV-based frontier detector node, a filter node and an assigner node. It is based partially on [this](https://github.com/hasauino/rrt_exploration) implementaion in ROS. So, please check that out.

The OpenCV-based frontier detector node is designed to run indivually on each robot, this is a local algorithmn. Make parameter for topic to easily assign name.

**Subscribed Topics**
 - The map (Topic name is defined by the ```~map_topic``` parameter) ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
 - The odometry (Topic name is defined by the ```~odom_topic``` parameter) ([nav_msgs/Odometry](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg))


**Published Topics**
 - The map (Topic name is defined by the ```~map_topic``` parameter) ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
 - The odometry (Topic name is defined by the ```~odom_topic``` parameter) ([nav_msgs/Odometry](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg))

## Path finding
To follow the swarm methodoloy I implemented a local pathfinder. There is a [navgating stack](https://github.com/ros-planning/navigation2) in ROS2 for this exact purpose but this is quite heavy weight (computaitaionaly expensive). 

The Pi-Puck is a differetial drive robot so, the ododmetry data byitself is unstable and prone to errors over time, this may be do to surface friction or uneven surfaces. Though, this can also be due to bumps and getting stuck (The wheels will keep spinning). 

It also has access to the IMU, this is anotehr source of odometry this tends to decrease ina ccurayc over time

Scan mathcing is a thing extarpoltion of data (fake data) lidar mode or writting own software.

If you are looking for error or calibration values for the marix see [this](https://github.com/yorkrobotlab/pi-puck-ros) the ROS driver for the Pi-Puck robot

**Subscribed Topics**
 - The map (Topic name is defined by the ```~map_topic``` parameter) ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
 - The odometry (Topic name is defined by the ```~odom_topic``` parameter) ([nav_msgs/Odometry](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg))

**Published Topics**
 - The map (Topic name is defined by the ```~map_topic``` parameter) ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
 - The odometry (Topic name is defined by the ```~odom_topic``` parameter) ([nav_msgs/Odometry](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg))

## Data Collection
I employed some basic data collection where I subscribed to some topics **(see below)** and manually saved the data from text files. 
I then compiled this data in python in [Jupyter notebook](https://jupyter.org/). I no longer have the code for this, but [this](https://www.geeksforgeeks.org/graph-plotting-in-python-set-1/) link is to a basic python tuturial for plotting graphs.

I first created a ground truth map, this was in a simulation so I could model a perfect map easily. In a real enviroment, I would suggest either doing this from an image or using a more complex robot that is known to have great accuracy to do acheive this.

I then compared my generated map to this map to produce some statstics. 
I also measured the odometry to use as a comparasion. See mt undergraduate dissertstaion to look at the graphs. 

**Subscribed Topics**
 - The map (Topic name is defined by the ```~map_topic``` parameter) ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
 - The odometry (Topic name is defined by the ```~odom_topic``` parameter) ([nav_msgs/Odometry](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg))

My topic names are arbitray and won't be correct, so you need to either change the names or create a parameter to allow name changing on the fly. 


**See** this [paper](https://www.mdpi.com/2218-6581/6/3/21) by Z. Yan et al for an indeapth discussion of data collection methods.
## Links to good tutorial and code bases

- [Occupancy Grid Mapping with Webots and ROS2](https://towardsdatascience.com/occupancy-grid-mapping-with-webots-and-ros2-a6162a644fab)
- [https://automaticaddison.com/](https://automaticaddison.com/) This whole website is very useful
- [ROS2 cookbook](https://github.com/mikeferguson/ros2_cookbook)
- [Basic video series about ROS2](https://www.reddit.com/r/ROS/comments/jipc4v/webots_ros2_tutorial_series/)
- [Colcon build cheat sheet](https://github.com/ubuntu-robotics/ros2_cheats_sheet)
