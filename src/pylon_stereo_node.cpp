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

void setCameraInfo(std::string camera_name, cv::Mat image_size, sensor_msgs::CameraInfo &camera_info){
  camera_info.header.frame_id = camera_name;
  camera_info.width  = image_size.at<double>(0,0);
  camera_info.height = image_size.at<double>(0,1);
  camera_info.binning_x;
  camera_info.binning_y;
  camera_info.D;
  camera_info.K;
  camera_info.P;
  camera_info.R;
  camera_info.distortion_model;
  camera_info.roi;
}

cares_msgs::StereoCameraInfo loadCameraInfo(std::string camera_left, std::string camera_right){
  cares_msgs::StereoCameraInfo stereo_camera_info;
  //Load camera info from "config" folder here

  std::string camera_name = "/home/henry/catkin_ws/src/pylon_camera/config/calibration_opencv.json";
  cv::FileStorage fs(camera_name, cv::FileStorage::READ);
  if(!fs.isOpened()) {
    ROS_ERROR("Failed to open file: %s", camera_name.c_str());
  }

  cv::Mat image_size;
  cv::Mat K1;
  cv::Mat dist1;
  cv::Mat K2;
  cv::Mat dist2;
  cv::Mat R;
  cv::Mat T;
  fs["imageSize"] >> image_size;
  fs["K1"] >> K1;
  fs["dist1"] >> dist1;
  fs["K2"] >> K2;
  fs["dist2"] >> dist2;
  fs["R"] >> R;
  fs["T"] >> T;

  sensor_msgs::CameraInfo left_info;
  setCameraInfo(camera_left, image_size, left_info);
  stereo_camera_info.left_info  = left_info;

  sensor_msgs::CameraInfo right_info;
  setCameraInfo(camera_right, image_size, right_info);
  stereo_camera_info.right_info = right_info;

  stereo_camera_info.header.frame_id = camera_left;
  stereo_camera_info.Q;
  stereo_camera_info.R_left_right;
  stereo_camera_info.T_left_right;

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

  //TODO Debugging delete
  loadCameraInfo("left", "right");

  // Before using any pylon methods, the pylon runtime must be initialized
  PylonInitialize();
  image_transport::ImageTransport it(nh);

  ROS_INFO("Setting up cameras");
  //Left Camera
  std::string camera_left;
  if(nh_private.hasParam(CARES::Pylon::CAMERA_LEFT_S))
    nh_private.getParam(CARES::Pylon::CAMERA_LEFT_S, camera_left);
  Camera left_camera(camera_left, true);

  image_transport::Publisher pub_left_image = it.advertise(camera_left+"/image_raw", 1);
  ros::Publisher pub_left_info = nh.advertise<sensor_msgs::CameraInfo>(camera_left+"/camera_info", 100);

  //Right Camera
  std::string camera_right;
  if(nh_private.hasParam(CARES::Pylon::CAMERA_RIGHT_S))
    nh_private.getParam(CARES::Pylon::CAMERA_RIGHT_S, camera_right);
  Camera right_camera(camera_right, false);

  image_transport::Publisher pub_right_image = it.advertise(camera_right+"/image_raw", 1);
  ros::Publisher pub_right_info = nh.advertise<sensor_msgs::CameraInfo>(camera_right+"/camera_info", 100);

  //Stereo Information
  cares_msgs::StereoCameraInfo stereo_camera_info = loadCameraInfo(camera_left, camera_right);
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
    ros::Time trigger_stamp = ros::Time::now();
    right_camera.trigger();

    //Get images
    cv::Mat left_image  = left_camera.getImage();
    cv::Mat right_image = right_camera.getImage();

    std_msgs::Header image_header;
    image_header.stamp = trigger_stamp;

    sensor_msgs::ImagePtr msg_left_image = cv_bridge::CvImage(image_header, "bgr8", left_image).toImageMsg();
    msg_left_image->header.frame_id = camera_left;
    sensor_msgs::ImagePtr msg_right_image = cv_bridge::CvImage(image_header, "bgr8", right_image).toImageMsg();
    msg_right_image->header.frame_id = camera_right;

    stereo_camera_info.header.stamp = trigger_stamp;

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
