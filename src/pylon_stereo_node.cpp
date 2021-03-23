//
// Created by henry on 20/07/17.
//

//
// Created by anyone on 9/12/16.
//
#include <ros/ros.h>
#include <ros/package.h>

#include "camera.h"
#include "parameters.h"

#include <cares_msgs/StereoCameraInfo.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace cv_bridge;

cares_msgs::StereoCameraInfo loadCameraInfo(std::string camera_name){
  cares_msgs::StereoCameraInfo stereo_camera_info;
  //Load camera info from "config" folder here
  return stereo_camera_info;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pylon_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  //GPIO
  //Pins
  //6 DCon GND
  //5 Opto GND
  //4 Line 2 : Opto
  //3 Line 4 : Dcon Output Trigger
  //2 Line 1 : Opto
  //1 Line 3 : Dcon Input Trigger

  // Before using any pylon methods, the pylon runtime must be initialized
  PylonInitialize();
  image_transport::ImageTransport it(nh);

  ROS_INFO("Setting up cameras");
  //Left Camera
  std::string camera_left;
  if(nh_private.hasParam(CARES::Pylon::CAMERA_LEFT_S))
    nh_private.getParam(CARES::Pylon::CAMERA_LEFT_S, camera_left);
  Camera left_camera(camera_left, false);

  image_transport::Publisher pub_left_image = it.advertise(camera_left+"/image_raw", 1);
  ros::Publisher pub_left_info = nh.advertise<sensor_msgs::CameraInfo>(camera_left+"/camera_info", 100);

  //Right Camera
  std::string camera_right;
  if(nh_private.hasParam(CARES::Pylon::CAMERA_RIGHT_S))
    nh_private.getParam(CARES::Pylon::CAMERA_RIGHT_S, camera_right);
  Camera right_camera(camera_right, true);

  image_transport::Publisher pub_right_image = it.advertise(camera_right+"/image_raw", 1);
  ros::Publisher pub_right_info = nh.advertise<sensor_msgs::CameraInfo>(camera_right+"/camera_info", 100);

  //Stereo Information
  cares_msgs::StereoCameraInfo stereo_camera_info = loadCameraInfo("stereo_info.yaml");
  ros::Publisher pub_stereo_info = nh.advertise<cares_msgs::StereoCameraInfo>(camera_left+"_"+camera_right+"/stereo_info", 100);

  //Primary Loop
  ROS_INFO("Running cameras");
  int loop_rate = 20;
  if(nh_private.hasParam("loop_rate"))
    nh_private.getParam("loop_rate", loop_rate);

  ros::Rate rate(loop_rate);
  ROS_INFO("Publishing at %i Hz", loop_rate);
  while(ros::ok()){
    //Fire trigger pin on master camera
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    right_camera.trigger();
    header.frame_id = camera_left;

    //Get images
    cv::Mat left_image  = left_camera.getImage();
    cv::Mat right_image = right_camera.getImage();

    sensor_msgs::ImagePtr msg_left_image = cv_bridge::CvImage(header, "bgr8", left_image).toImageMsg();
    msg_left_image->header = header;
    sensor_msgs::ImagePtr msg_right_image = cv_bridge::CvImage(header, "bgr8", right_image).toImageMsg();
    msg_right_image->header = header;

    stereo_camera_info.header = header;
    stereo_camera_info.left_info.header = header;
    //Make sure the right image has frame id as camera_right frame
    stereo_camera_info.right_info.header = header;
    stereo_camera_info.right_info.header.frame_id = camera_right;

    //Publish the images
    pub_left_image.publish(msg_left_image);
    pub_left_info.publish(stereo_camera_info.left_info);

    pub_right_image.publish(msg_right_image);
    pub_right_info.publish(stereo_camera_info.right_info);

    pub_stereo_info.publish(stereo_camera_info);

    rate.sleep();
  }
  return 0;
}
