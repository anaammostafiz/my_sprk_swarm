# sphero_swarm
Code primarily written by Anaam Mostafiz for the Sphero Swarm Arena being developed at the Arizona State University IgnitED Labs. 

<!-- GETTING STARTED -->
## Getting Started
   
To get a local copy up and running follow these simple steps.

### Prerequisites

- Ubuntu 20.04
- ROS Noetic
- OpenCV, Numpy
- USB Webcam - I am using the Logitech C270
- [sphero_driver_v2](https://github.com/larics/sphero_robot/tree/master/sphero_driver_v2) and/or [sphero_sprk_ros](https://github.com/antonellabarisic/sphero_sprk_ros/tree/noetic-devel)

### Installation

1. [Setup your USB Webcam in ROS](https://msadowski.github.io/ros-web-tutorial-pt2-cameras/)
2. Clone the repo into your catkin workspace
   ```shell script
   $ cd ~/<name_of_your_catkin_ws>/src
   $ git clone https://github.com/anaammostafiz/sphero_swarm.git
   $ cd ~/<name_of_your_catkin_ws>
   $ catkin_make
   ```

<!-- USAGE EXAMPLES -->
## Usage

Here's an example on how to run a leader-follower swarm of 3 Sphero's.

1. Launch the usb webcam with ROS. Assuming you followed the above webcam setup link,
   ```
   roslaunch my_camera elp.launch
   ```
2. Connect to 3 Sphero's via Bluetooth
   ```
   rosrun sphero_driver_v2 drivers.launch.py 3
   ```
   or, if only using Sphero SPRK+ robots,
   ```
   roslaunch sphero_sprk_ros drivers.launch num_of_robots:=3
   ```
3. Change the colors of Sphero's 0, 1, and 2 to green, red, and blue respectively, and run the color trackers. 
   ```
   roslaunch sphero_swarm rgb_track.launch g_num:=0 r_num:=1 b_num:=2
   ```
4. Tell Sphero 1 and 2 to follow 0.
   ```
   rosrun sphero_swarm follow_lead.py 1 0
   ```
   ```
   rosrun sphero_swarm follow_lead.py 2 0
   ```
5. Move Sphero 0.
   ```
   rostopic pub sphero_0/cmd_vel geometry_msgs/Twist -r 5 '[1,0,0]' '[0,0,0]' 
   ```
***Note***: In the ```utils``` folder, ```color_track.py``` should be edited for your arena size and color values. The color values can be tuned with ```color_tune.py```.

<!-- ROADMAP -->
## Roadmap

| Feature    | Current Status |
|------------|----------------|
|Tool: Color Tracking| <ul><li>- [x] completed</li><li>- [ ] in progress</li><li>- [ ] to do</li></ul>
|Tool: Gazebo Sim| <ul><li>- [x] completed</li><li>- [ ] in progress</li><li>- [ ] to do</li></ul>
|Tool: Depth Sensing (for object detection in dark)| <ul><li>- [ ] completed</li><li>- [ ] in progress</li><li>- [x] to do</li></ul>
|Swarm: Leader-Follower| <ul><li>- [x] completed</li><li>- [ ] in progress</li><li>- [ ] to do</li></ul>
|Swarm: Light-Synchronization| <ul><li>- [ ] completed</li><li>- [x] in progress</li><li>- [ ] to do</li></ul>
|Swarm: Object Behaviors (like collective transport)| <ul><li>- [ ] completed</li><li>- [ ] in progress</li><li>- [x] to do</li></ul>


