# Pylon Camera

ROS package for publishing images from Basler (Pylon) cameras. Provides two different hardware trigger configurations, and a master/slave trigger for multiple PC setups.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
1) ROS Kenitic (no known reason why future versions will not work)

2) pylon 5.1.0 Camera Software Suite Linux ARM 64 bit: https://www.baslerweb.com/en/sales-support/downloads/software-downloads/

3) Pull master version of maara_lib
    a) cd ~/catkin_ws/src
    b) git clone https://github.com/maraatech/maara_lib.git

4) Pull master version of maara_msgs
    a) cd ~/catkin_ws/src
    b) git clone https://github.com/maraatech/maara_msgs.git

5) Compile both libraries
    a) cd ~/catkin_ws
    b) catkin_make
```

### Installing

A step by step series of examples that tell you how to get a development env running

Clone the package into the catkin directory you are using, presumned here to be "~/catkin_ws"

```
cd ~/catkin_ws/src
git clone https://github.com/maraatech/pylon_camera.git
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

## Applications

#### Pylon Camera Trigger Sync

Stereo pair capture images when their hardware trigger input pin is gets a high input.\
The trigger output is set as a pin on the right camera, which when set high triggers both cameras.\

The trigger output needs to be wired into each cameras trigger input.\
Trigger input is Pin 1 -> Line 3 on each camera.\
Triggering pin is Pin 3 -> Line 4 on the right camera.

###### Launch File

Run using launch file as below.

```
roslaunch pylon_camera pylon_camera_node_trigger.launch
```

Launch file can change the name of the left and right cameras, default is cameras named "left" and "right".

```xml
<?xml version="1.0"?>
<launch>
    <node name="pylon_camera_trigger" pkg="pylon_camera" type="pylon_node_trigger_sync">
		<param name ="left_camera"    value="left"/>
		<param name ="right_camera"   value="right"/>
    </node>
</launch>
```

#### Pylon Camera Exposure Sync

When the right camera is ready to capture (ready to expose) a pin is set high triggering the left camera.\
Similar to the hardware trigger but the cameras are capturing continuously and self triggering.

The trigger output on the right camera is wired into the trigger input on the left camera.\
Trigger input is Pin 1 -> Line 3 on the left camera.\
Triggering pin is Pin 1 -> Line 3 on the right camera.

###### Launch File

Run using launch file as below.

```
roslaunch pylon_camera pylon_camera_node_exposure.launch
```

Launch file can change the name of the left and right cameras, default is cameras named "left" and "right".

```xml
<?xml version="1.0"?>
<launch>
    <node name="pylon_camera_exposure" pkg="pylon_camera" type="pylon_node_exposure_sync" output="screen">
    	<param name ="left_camera"    value="left"/>
		<param name ="right_camera"   value="right"/>
    </node>
</launch>
```

#### Pylon Master/Salve configuration

Pylon master and slave enable hardware triggering across a number of stereo pairs on multiple PC's.\
Trigger is the same as the exposure sync, but tied to all slave cameras from a single master camera.

Master camera captures images when the software trigger is sent, the exposure pin goes high and triggers the slave cameras.\
The slave cameras are then called via a rosservice with the time-stamp from when the master triggered, and they then publish their respective images.

###### Launch File

Master launch file

```
roslaunch pylon_camera master.launch
```

Master launch file, set whether the "Left" or "Right" camera in the stereo pair.\
You can also set the left and right camera names.

```xml
<?xml version="1.0"?>
<launch>
	<arg name="camera"     default="Right"/>

	<node name="pylon_camera_master" pkg="pylon_camera" type="pylon_master">
    	<param name ="camera_lr"      value="$(arg camera)"/>
    	<param name ="left_camera"    value="left"/>
      <param name ="right_camera"   value="right"/>
	</node>
</launch>
```

Slave launch file

```
roslaunch pylon_camera slave.launch
```

Set whether the slave is a "Left" or "Right" camera in the stereo pair.\
You can also set the left and right camera names.

```xml
<?xml version="1.0"?>
<launch>
 	<arg name="camera"     default="Left"/>

	<node name="$(anon pylon_camera_slave)" pkg="pylon_camera" type="pylon_server">
     <param name ="camera_lr"      value="$(arg camera)"/>
		 <param name ="left_camera"    value="left"/>
		 <param name ="right_camera"   value="right"/>
	</node>
</launch>
```

## Version

Version 1.0

## Authors

* **Henry Williams**

## License

TBD

## Acknowledgments

* 


