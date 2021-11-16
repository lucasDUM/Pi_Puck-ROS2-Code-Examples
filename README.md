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

**The sections go as follows:**
- Webots
- ROS2
  - Frontier detection
  - Path finding
  - Data collection
- Link to good tutorials and code bases

Before I get into the code I followed or mainly followed some standard practises see REPs [103](https://www.ros.org/reps/rep-0103.html) and [105](https://www.ros.org/reps/rep-0105.html). This should also explain a bit about how TF-Trees work in ROS/ROS2.

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
All the code is made using Python and ROS2 libraries.

## Frontier detection
In order to carry out frontier detecion in ROS2 I adpated partially a [previous implementaion](https://github.com/hasauino/rrt_exploration) in ROS. 

My frontier detection consists of an OpenCV-based frontier detector node and a filter node with accompanying methods.
**Frontier-detection node**
The OpenCV-based frontier detector node is designed to run individually on each robot, it takes a local map defaulting to ```/epuck0/map/``` and produces a set of "goal" points. As well as some marker points for visual debugging. This then passes goals to the filter node.

The ```map_topic``` currently does not have a parameter so this has to be changed manually or simply create a parameter (CP). This is an occuring problem within the code base so I will label it as CP.

**Subscribed Topics**
 - The map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
 - The clock ([rosgraph_msgs/Clock](http://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Clock.html))

**Published Topics**
 - PointStamped ([geometry_msgs/PointStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html))
 - The marker ([visualization_msg/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))
 
**The filter node**
This also runs on each robot because this is a swarm system so, it is local information only. If you did want to use it in a centralised manner you would have to make it subscribe to all the ```goal_topics```. (CP)

The filter node involves deleting pointless goal points, this extends to old points (points now in occupied space) and invalid points.
It subscribes to a ```goal_topic``` and produced a ``` filtered goal_topic```. (CP)

**Subscribed Topics**
 - The map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
 - The clock ([rosgraph_msgs/Clock](http://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Clock.html))
 - PointStamped ([geometry_msgs/PointStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html))

**Published Topics**
 - PointArray ([geometry_msgs/PointStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html))
 - The marker ([An array of points](https://github.com/hasauino/rrt_exploration/blob/master/msg/PointArray.msg)) I made my own custom workspace just for messages. I would advise the same thing. However, the link points to the original ROS implementaion.
 
## Path finding
To follow the swarm methodoloy I implemented a simple local pathfinder based on the "Bug2 motion planning algorihtmn". A good implementaion of this algorithm is [this](https://automaticaddison.com/the-bug2-algorithm-for-robot-motion-planning/) which also goes on to explain how to create your own [Extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter). 

There is also a [Navagation stack (Nav2)](https://github.com/ros-planning/navigation2) in ROS2 for pathfinding purposes but this is quite heavy weight. [Here](https://github.com/SteveMacenski/nav2_rosdevday_2021?ref=pythonrepo.com) is an example of this running on another robot using Python.

My code was unstable at times so instead of a full implementaion I've provided some methods and examples. Communication was realtively easy to do in the simulation however a lot harder in reality. Since it doesn't work, I haven't added it. In simple terms I created a gloabl script (which can run on every robot) that calulated the distances between there estimated positions. This can be very inaccurate in real life so this was the main challenge. Below is some short steps to try and mitigate this.

The Pi-Puck is a differetial drive robot so, the odometry data by itself is unstable and prone to errors over time, this may be do to surface friction or uneven surfaces. Though, this can also be due to bumps and getting stuck (The wheels will keep spinning). 

It also has access to an IMU, this acts as another source of odometry but this tends to decrease in accuracy over time.

I used the [robot localization package](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) to merge these two sources of odometry to create a better estimation. If you do use this method, make sure to use the ROS2 version and create the [Yaml file](https://roboticsbackend.com/ros2-yaml-params/) to the ROS2 specification.
In regards to this method if you are looking for any error or calibration values see the [ROS driver](https://github.com/yorkrobotlab/pi-puck-ros) for the Pi-Puck robot

Alternatively a lot of [simultaneous localisation and mapping (SLAM) methods](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) have their own scan-matching algorihtms that estimate the global position within a map. 

The Pi-Puck doesn't have enough scan data to take advantage of this however there are three areas to experiment in:
- In the [ROS driver](https://github.com/yorkrobotlab/pi-puck-ros) for the Pi-Puck they use extrapolation to make fake laser data to increase the scan data. 
- You could also run the tof sensors multiple times creating a "fake" lidar
- Lastly, you could write your own scan matcher optimised for the Pi-Puck

## Data Collection
I employed some basic data collection where I subscribed to some topics **(see below)** and manually saved the data into text files. 
I then compiled this data in python in [Jupyter notebook](https://jupyter.org/). I no longer have the code for this, but [this](https://www.geeksforgeeks.org/graph-plotting-in-python-set-1/) link is to a basic python tuturial for plotting graphs.

I first created a ground truth map, this was in a simulation so I could model a perfect map easily. In a real enviroment, I would suggest either doing this from an image or using a more complex robot, that is known to have high accuracy.

I then compared my generated map to this map to produce some statistics. 
I also measured the odometry for each robot to use as a comparison baseline. 

One note when comparing map quality: make sure the maps are aligned properly otherwise this won't work.
**Subscribed Topics**
 - The map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))
 - The odometry ([nav_msgs/Odometry](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg))

My topic names are arbitray and won't be correct, so you need to either change the names or create a parameter to allow name changing on the fly. (CP) 

**See** this [paper](https://www.mdpi.com/2218-6581/6/3/21) by Z. Yan et al for a detailed discussion of data collection methods.
## Links to good tutorial and code bases

- [Occupancy Grid Mapping with Webots and ROS2](https://towardsdatascience.com/occupancy-grid-mapping-with-webots-and-ros2-a6162a644fab)
- [https://automaticaddison.com/](https://automaticaddison.com/) This whole website is very useful
- [ROS2 cookbook](https://github.com/mikeferguson/ros2_cookbook)
- [Basic video series about ROS2](https://www.reddit.com/r/ROS/comments/jipc4v/webots_ros2_tutorial_series/)
- [Colcon build cheat sheet](https://github.com/ubuntu-robotics/ros2_cheats_sheet)
- [More ROS2 examples](https://github.com/alsora/ros2-code-examples)
- [Nav2 example](https://github.com/SteveMacenski/nav2_rosdevday_2021?ref=pythonrepo.com)
