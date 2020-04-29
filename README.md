# SES_Group_Repo
# SES_Group_Repo

## About:




## Prerequisites:
1) [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) on Ubuntu 18.04 or [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) on Ubuntu 16.04. 
2) [Python catkin-tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)




## Usage 
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


## Contributors:
1) [Mihir Kulkarni](https://github.com/mihirk284)
2) [SriSreyas Sundaresan](https://github.com/SriSreyas) 
3) Shivangi Gupta
4) [Vishal Singh] (https://github.com/vishalbhsc)
5) [Aditya Bidwai](https://github.com/adbidwai)
