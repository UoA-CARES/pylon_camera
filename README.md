# Pylon Camera

ROS package for publishing images from Basler (Pylon) cameras. Provides hardware trigger configuration based on the exposure pin.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
1) ROS Neotic - works in Melodic as well

2) pylon 6.2.0 Camera Software Suite Linux x86 (64 Bit): https://www.baslerweb.com/en/sales-support/downloads/software-downloads/#version=6.2.0;os=linuxx8664bit

3) Pull master version of cares_msgs
    a) cd ~/catkin_ws/src
    b) git clone https://github.com/UoA-CARES/cares_msgs.git

4) Compile both libraries
    a) cd ~/catkin_ws
    b) catkin_make
```

### Installing

A step by step series of examples that tell you how to get a development env running

Clone the package into the catkin directory you are using, presumned here to be "~/catkin_ws"

```
cd ~/catkin_ws/src
git clone https://github.com/UoA-CARES/pylon_camera.git
```

Build the package with catkin_make in the source directory

```
cd ~/catkin_src/
catkin_make
```

## Running the tests

Tests to be added

## Basler GPIO

Pin out on Basler camera USB 3.0 GPIO.\
Pin 5 is 12 o'clock with USB input at 6 o'clock, numbers then rotate clockwise.

```
  Pin 6 -> DCon GND
  Pin 5 -> Opto GND
  Pin 4 -> Line 2 : Opto
  Pin 3 -> Line 4 : Dcon Output
  Pin 2 -> Line 1 : Opto
  Pin 1 -> Line 3 : Dcon Input
```

![Alt text](docs/basler-pin-out.png?raw=true "Pin-out")

## Applications

### Pylon Stereo Node

This node will run two basler cameras with given names (default left/right) in a stereo pair with the desired sync mode.\
There are three possible modes that the stereo cameras can operate in.

#### Pylon Camera Async Mode (0)
Asynchronous mode will run both cameras via the software triggers independently without the need for hardware synchronization.

#### Pylon Camera Exposure Sync Mode (1)
When the left camera is ready to capture (ready to expose) a pin is set high triggering the right camera.\
Similar to the Pin trigger but the cameras are capturing continuously and self triggering via the exposure pin.

The trigger output on the left camera is wired into the trigger input on the right camera.\
Trigger input is Pin 1 on the right camera.\
Trigger output is Pin 1 on the left camera.

![Alt text](docs/sync-mode-1.png?raw=true "Pin-out")

#### Pylon Camera Pin Mode (2)
Pin mode sets a pin on each camera as an input that will trigger the cameras on a rising edge. 
The left camera has a pin setup that can be toggled via software to trigger the capture on each camera.\
The left camera toggles the trigger pin to trigger both cameras.
 
The trigger output on the left camera is wired into the trigger input on the left and right cameras.\
Trigger input is Pin 1 on the left and right camera.\
Trigger output is Pin 3 on the left camera.

![Alt text](docs/sync-mode-2.png?raw=true "Pin-out")

#### Subscribed Topics
No Subscribed topics

#### Published Topics
The base topic name is the default name (ns=stereo_pair), it can be changed via setting parameters in the launch file via "ns".

* Image
  * stereo_pair/left/image_color
  * stereo_pair/right/image_color

If a calibration file is provided then these topics will be published as well.

* Image
  * stereo_pair/left/image_rect_color
  * stereo_pair/right/image_rect_color
* sensor_msgs::CameraInfo
  * stereo_pair/left/camera_info
  * stereo_pair/right/camera_info
* cares_msgs/StereoCameraInfo
  * stereo_pair/stereo_info

##### pylon_camera_node_calibrated.launch
Run using launch file as below.

```
roslaunch pylon_camera pylon_camera_node_calibrated.launch
```

Launch file can change the name of the left and right stereo cameras, default is cameras named "left" and "right" (set via pylonviewver).\
If calibration is empty the node will not publish camera or stereo information.\
The trigger mode can be selected with each mode, and cable setup, described above.\
Display can be set if you wish to have OpenCV display the images as they are captured off the camera.

```xml
<?xml version="1.0"?>
<launch>
  <arg name="ns" default="stereo_pair"/>
  <arg name="loop_rate" default="5"/>
  <arg name="display" default="true"/>
  <arg name="camera_left"  default="left"/>
  <arg name="camera_right" default="right"/>
  <!--Trigger modes-->
  <!--0 Aysnc-->
  <!--1 Exposure Based-->
  <!--2 Toggle Pin-->
  <arg name="trigger_mode" default="1"/>
  <arg name="calibration"  default="$(find pylon_camera)/config/calibration.json"/>

  <node name="pylon_stereo_node" pkg="pylon_camera" type="pylon_stereo_node" output="screen" ns="$(arg ns)">
    <param name ="loop_rate"    value="$(arg loop_rate)"/>
    <param name ="display"      value="$(arg display)"/>
    <param name ="camera_left"  value="$(arg camera_left)"/>
    <param name ="camera_right" value="$(arg camera_right)"/>
    <param name ="trigger_mode" value="$(arg trigger_mode)"/>
    <param name ="calibration"  value="$(arg calibration)"/>
  </node>
</launch>

```

Camera left and right arguements are the IDs of the cameras you wish to connect to - default is "left" and "right".

```xml
<arg name="camera_left"  default="left"/>
<arg name="camera_right" default="right"/>

```

## Version

Version 1.0

## Authors

* **Henry Williams**

## License

TBD

## Acknowledgments

* Matthew Seabright for finding the configuration for the exposure mode


