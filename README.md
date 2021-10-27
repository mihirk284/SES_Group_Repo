# SES_Group_Repo

## About:




## Prerequisites:
1) [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) on Ubuntu 18.04 or [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) on Ubuntu 16.04. 
2) [Python catkin-tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
3) [OpenCV](https://opencv.org/) - Computer Vision library 
4) [Boost](https://www.boost.org/) - C++ library




## Set up
1) Create a ROS workspace 
```
mkdir -p ~/catkin_ws/src
cd catkin_ws
catkin_init_workspace
```

2) Clone this repository in your ROS workspace recursively along with the submodules
```
cd ~/catkin_ws/src
git clone --recurse-submodules -j8 https://github.com/mihirk284/SES_Group_Repo
```

3) Build your packages
```
cd ~/catkin_ws
catkin build
```

## Usage 
### 1) Test the installation
  Try launching a sample test launch file
   ```
   roslaunch rotors_gazebo mav_hovering_example.launch 
   ```
   You should see a drone hovering some distance above the ground in your Gazebo simulator.
### 2) YOLO Implementation
   Launch your drone in a Gazebo environment
   ```
   roslaunch rotors_gazebo mav_hovering_example.launch 
   ```
   Visualise the camera feed in RViz
   ```
   rosrun rviz rviz
   ```
   Click on "Add" and add the camera topic, you'll see a camera feed of the environment in which the drone is.
   Launch the YOLO object detector.
   ```
   roslaunch darknet_ros yolo_v3.launch 
   ```
   Try placing some objects like the aeroplane, table, turtlebot in the Gazebo environment in front of the camera and you'll see YOLO        detecting it with a bounding box around that object.
***
## Contributors:
1) [Mihir Kulkarni](https://github.com/mihirk284)
2) [SriSreyas Sundaresan](https://github.com/SriSreyas) 
3) [Shivangi Gupta](https://github.com/shivangixgupta)
4) [Vishal Singh](https://github.com/vishalbhsc)
5) [Aditya Bidwai](https://github.com/adbidwai)
