# CSE598 Perception in Robotics (Spring 2020) Project 3
## Pursuit Evasion Game
This repository contains the package(s) for project 3.

It depends and follows similar paradigms to the upstream turtlebot3 package from ROBOTIS.

## File Structure
For convenience,the upstream turtlebot3 and turtlebo3_msgs packages from ROBOTIS-GIT/turtlebot3 are bundled.

```
src/
├── project3a
│   └── pursuitevasion
├── project3b
│   ├── human_description
│   ├── pursuit_evasion
│   └── pursuit_evasion_description
├── turtlebot3
│   ├── turtlebot3
│   ├── turtlebot3_bringup
│   ├── turtlebot3_description
│   ├── turtlebot3_example
│   ├── turtlebot3_navigation
│   ├── turtlebot3_slam
│   └── turtlebot3_teleop
└── turtlebot3_msgs
    └── msg

```

### Building
Clone to the `src/` directory of your catkin workspace, then run `catkin_make` from the latter.

### Running
`source devel/setup.bash`

`export TURTLEBOT3_MODEL=burger`

To run the AMCL node manually: `roslaunch pursuit_evasion robot_amcl.launch map_file:='$(find pursuit_evasion)/maps/world1.yaml' world_index:=1`, Replacing _1_ with the world index in the map file and world_index

To run the pursuer node manually: `roslaunch pursuit_evasion detect_pursuer.launch` and `roslaunch pursuit_evasion move_pursuer.launch`

To run a demo: `roslaunch pursuit_evasion pursuit_evasion_demo.launch world_index:=1` and `roslaunch pursuit_evasion move_evader.launch world_index:=1`, replacing _1_ with the desired world index`

## Node Interface
###  pursuit_evasion/pursuer_detector.py
_Subscribes_ to the following topics:
  * image (default: '/camera/rgb/image_raw') sensor_msgs/Image
    * input image from the pursuer
  * depth_image (default: '/camera/depth/image_raw') sensor_msgs/Image
    * input depth image from the pursuer

_Publishes_ the following topics:
  * detections (default: '/pursuer_detections') sensor_msgs/RegionOfInterest
    * region of the detected evader on the image
  * distance (default: '/pursuer_distance') std_msgs/Float32
    * approximage distance to detected evader in world units
  * detection_image (default: '/pursuer_detection_image') sensor_msgs/Image
    * input image with detection rectangle overlaid
  * detection_performance (default: '/pursuer_detection_performance') std_msgs/Float32
    * detector performance (num detections over num images recieved)

Topics are configurable by ros param, e.g. `rosrun pursuit_evasion pursuer_detector.py _image:=/my_image_topic _depth_image:=/my_depth_image`
The node can be launched using the `detect_pursuer.launch` launch file
