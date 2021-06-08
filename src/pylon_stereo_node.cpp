//
// Created by henry on 20/07/17.
//
#include <ros/ros.h>
#include <ros/package.h>

#include "camera.h"
#include "parameters.h"

#include <cares_msgs/StereoCameraInfo.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace cv_bridge;

struct CameraInfo{
  cv::Mat K;
  cv::Mat D;
  cv::Mat P;
  cv::Mat R;
};

struct StereoInfo{
    CameraInfo left;
    CameraInfo right;
    cv::Mat Q;
    cv::Mat R;
    cv::Mat T;
    cv::Size size;
};

void setCameraInfo(Camera &camera, cv::Size image_size, cv::Mat K, cv::Mat D, cv::Mat P, cv::Mat R, sensor_msgs::CameraInfo &camera_info, std::string ns){
  //Frame ID is named after the camera name
  ROS_INFO("Reading name");
  camera_info.header.frame_id = ns+"/"+camera.name();

  //Size of image at calibration
  camera_info.width  = image_size.width;
  camera_info.height = image_size.height;

  std::vector<double>K_v(K.begin<double>(), K.end<double>());
  std::vector<double>P_v(P.begin<double>(), P.end<double>());
  std::vector<double>R_v(R.begin<double>(), R.end<double>());

  //Intrinsic parameters
  camera_info.D = D;//dist
  for (int i=0; i<K_v.size(); i++)camera_info.K[i] = (K_v[i]);
  for (int i=0; i<P_v.size(); i++)camera_info.P[i] = (P_v[i]);
  for (int i=0; i<R_v.size(); i++)camera_info.R[i] = (R_v[i]);

  //Default is plump bob
  //TODO add this to calibration file and read from there
  camera_info.distortion_model = "plumb_bob";

  ROS_INFO("Reading binning");
  camera_info.binning_x = camera.getBinningX();//width
  camera_info.binning_y = camera.getBinningY();//height
  ROS_INFO("Read binning");

  ///<Set the published image ROI based on the camera's internal settings - this can be different to the settings used during calibration
  //The default setting of roi (all values 0) is considered the same as full resolution (roi.width = width, roi.height = height)
  // See camera_info documentation http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  sensor_msgs::RegionOfInterest roi;
  //If the published image width or height is different then set to the published ROI otherwise set to 0 as per camera_info docs
  roi.height     = camera.getImageHeight() == camera_info.height ? 0 : camera.getImageHeight();
  roi.width      = camera.getImageWidth()  == camera_info.width  ? 0 : camera.getImageWidth();
  roi.x_offset   = camera.getOffsetX();
  roi.y_offset   = camera.getOffsetY();
  //If the size of the image being published is not the same as the calibration image size then recalibration is required
  roi.do_rectify = camera_info.width != camera.getImageWidth() || camera_info.height != camera.getImageHeight();
  camera_info.roi = roi;
}

void loadCameraInfo(Camera& camera_left,
                    Camera& camera_right,
                    std::string calibration_file,
                    std::string ns,
                    StereoInfo &stereo_info,
                    cares_msgs::StereoCameraInfo &stereo_camera_info){
  //Load camera info from "config" folder here
  ROS_INFO("Loading stereo information - %s", calibration_file.c_str());
  cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
  if(!fs.isOpened()) {
    ROS_ERROR("Failed to open file: %s - please check file location", calibration_file.c_str());
    PylonTerminate();
    exit(1);
  }

  cv::Size image_size;
  fs["imageSize"] >> image_size;
  stereo_info.size = image_size;

  //Left parameters
  cv::Mat K1;
  cv::Mat D1;
  cv::Mat P1;
  cv::Mat R1;
  fs["K1"] >> K1;
  fs["D1"] >> D1;
  fs["P1"] >> P1;
  fs["R1"] >> R1;

  stereo_info.left.K = K1;
  stereo_info.left.D = D1;
  stereo_info.left.P = P1;
  stereo_info.left.R = R1;

  sensor_msgs::CameraInfo left_info;
  setCameraInfo(camera_left, image_size, K1, D1, P1, R1, left_info, ns);
  stereo_camera_info.left_info  = left_info;

  //Right parameters
  cv::Mat K2;
  cv::Mat D2;
  cv::Mat P2;
  cv::Mat R2;
  fs["K2"] >> K2;
  fs["D2"] >> D2;
  fs["P2"] >> P2;
  fs["R2"] >> R2;

  stereo_info.right.K = K2;
  stereo_info.right.D = D2;
  stereo_info.right.P = P2;
  stereo_info.right.R = R2;

  sensor_msgs::CameraInfo right_info;
  setCameraInfo(camera_right, image_size, K2, D2, P2, R2, right_info, ns);
  stereo_camera_info.right_info = right_info;

  //Stereo parameters
  cv::Mat Q;
  cv::Mat R;
  cv::Mat T;
  fs["Q"] >> Q;
  fs["R"] >> R;
  fs["T"] >> T;

  stereo_info.Q = Q;
  stereo_info.R = R;
  stereo_info.T = T;

  std::vector<double>Q_v(Q.begin<double>(), Q.end<double>());
  std::vector<double>R_v(R.begin<double>(), R.end<double>());
  std::vector<double>T_v(T.begin<double>(), T.end<double>());

  stereo_camera_info.header.frame_id = ns+"/"+camera_left.name();
  for (int i=0; i<Q_v.size(); i++)stereo_camera_info.Q[i] = Q_v[i];
  for (int i=0; i<R_v.size(); i++)stereo_camera_info.R_left_right[i] = R_v[i];
  for (int i=0; i<T_v.size(); i++)stereo_camera_info.T_left_right[i] = T_v[i];
}

void displayPairs(cv::Mat image_left, std::string left_window, cv::Mat image_right, std::string right_window){
  cv::namedWindow(left_window, cv::WINDOW_NORMAL);
  cv::resizeWindow(left_window, 600, 600);
  cv::imshow(left_window, image_left);

  cv::namedWindow(right_window, cv::WINDOW_NORMAL);
  cv::resizeWindow(right_window, 600, 600);
  cv::imshow(right_window, image_right);
  cv::waitKey(10);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pylon_stereo_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string ns = ros::this_node::getNamespace();
  ROS_INFO("Operating under namespace: %s", ns.c_str());
  // Before using any pylon methods, the pylon runtime must be initialized
  PylonInitialize();

  ROS_INFO("Setting up cameras");
  bool display;
  nh_private.param(CARES::Pylon::DISPLAYP_B, display, true);

  //Get trigger mode
  Camera::TriggerMode trigger_mode;
  int mode;
  nh_private.param(CARES::Pylon::TRIGGER_MODE_I, mode, 0);
  trigger_mode = static_cast<Camera::TriggerMode >(mode);
  ROS_INFO("Trigger Mode: %i", trigger_mode);

  //Left Camera
  std::string camera_left_id;
  if(!nh_private.getParam(CARES::Pylon::CAMERA_LEFT_S, camera_left_id)){
    ROS_ERROR((CARES::Pylon::CAMERA_LEFT_S+ " not set.").c_str());
    return 0;
  }
  std::string camera_left_name = "left";
  Camera camera_left(camera_left_id, camera_left_name, trigger_mode, true);

  //Right Camera
  std::string camera_right_id;
  if(!nh_private.getParam(CARES::Pylon::CAMERA_RIGHT_S, camera_right_id)){
    ROS_ERROR((CARES::Pylon::CAMERA_RIGHT_S+ " not set.").c_str());
    return 0;
  }
  std::string camera_right_name = "right";
  Camera camera_right(camera_right_id, camera_right_name, trigger_mode, false);

  //Setup publishers
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_image_left  = it.advertise(camera_left_name  + "/image_color", 1);
  image_transport::Publisher pub_image_right = it.advertise(camera_right_name + "/image_color", 1);
  image_transport::Publisher pub_image_left_rec;
  image_transport::Publisher pub_image_right_rec;

  ros::Publisher pub_left_info;
  ros::Publisher pub_right_info;
  ros::Publisher pub_stereo_info;

  StereoInfo stereo_info;
  cares_msgs::StereoCameraInfo stereo_camera_info;
  cv::Mat left_map_1, left_map_2;
  cv::Mat right_map_1, right_map_2;

  //Load Stereo and Camera Information
  std::string calibration_file = "";
  if(nh_private.getParam(CARES::Pylon::CALIBRATION_S, calibration_file)){
    pub_image_left_rec  = it.advertise(camera_left_name  + "/image_rect_color", 1);
    pub_image_right_rec = it.advertise(camera_right_name + "/image_rect_color", 1);

    pub_left_info = nh.advertise<sensor_msgs::CameraInfo>(camera_left_name   + "/camera_info", 100);
    pub_right_info = nh.advertise<sensor_msgs::CameraInfo>(camera_right_name + "/camera_info", 100);

    loadCameraInfo(camera_left, camera_right, calibration_file, ns, stereo_info, stereo_camera_info);
    pub_stereo_info = nh.advertise<cares_msgs::StereoCameraInfo>("stereo_info", 100);

    cv::initUndistortRectifyMap(stereo_info.left.K, stereo_info.left.D, stereo_info.left.R, stereo_info.left.P, stereo_info.size, CV_32FC1, left_map_1, left_map_2);
    cv::initUndistortRectifyMap(stereo_info.right.K, stereo_info.right.D, stereo_info.right.R, stereo_info.right.P, stereo_info.size, CV_32FC1, right_map_1, right_map_2);
  }else{
    ROS_WARN("No calibration file found for location: %s - will not publish camera or stereo information", calibration_file.c_str());
  }

  //Primary Loop
  int loop_rate = 20;
  nh_private.param(CARES::Pylon::LOOP_RATE_I, loop_rate, loop_rate);

  ros::Rate rate(loop_rate);
  ROS_INFO("Publishing at %i Hz", loop_rate);
  while(ros::ok()){
    //Fire trigger pin on master camera
    std_msgs::Header header_image;
    ros::Time stamp_trigger = ros::Time::now();
    header_image.stamp = stamp_trigger;//Set timestamp

    camera_left.trigger();
    camera_right.trigger();

    //Get images
    cv::Mat image_left  = camera_left.getImage();
    cv::Mat image_right = camera_right.getImage();

    if(display){
      std::string left_window = camera_left.name();
      std::string right_window = camera_right.name();
      displayPairs(image_left, left_window, image_right, right_window);
    }

    //Convert images to ROS messages
    sensor_msgs::ImagePtr msg_image_left = cv_bridge::CvImage(header_image, "bgr8", image_left).toImageMsg();
    msg_image_left->header.frame_id = ns+"/"+camera_left_name;
    sensor_msgs::ImagePtr msg_image_right = cv_bridge::CvImage(header_image, "bgr8", image_right).toImageMsg();
    msg_image_right->header.frame_id = ns+"/"+camera_right_name;

    //Publish the images
    pub_image_left.publish(msg_image_left);
    pub_image_right.publish(msg_image_right);

    //Publish stereo and camera info if loaded
    if(calibration_file != ""){
      stereo_camera_info.header.stamp = stamp_trigger;//Set timestamp
      pub_left_info.publish(stereo_camera_info.left_info);
      pub_right_info.publish(stereo_camera_info.right_info);
      pub_stereo_info.publish(stereo_camera_info);

      cv::Mat image_left_rec, image_right_rec;
      cv::remap(image_left, image_left_rec, left_map_1, left_map_2, CV_INTER_LINEAR);
      cv::remap(image_right, image_right_rec, right_map_1, right_map_2, CV_INTER_LINEAR);

      if(display){
        std::string left_window = camera_left.name()+"_rec";
        std::string right_window = camera_right.name()+"_rec";
        displayPairs(image_left_rec, left_window, image_right_rec, right_window);
      }

      //Convert images to ROS messages
      sensor_msgs::ImagePtr msg_image_left_rec = cv_bridge::CvImage(header_image, "bgr8", image_left_rec).toImageMsg();
      msg_image_left_rec->header.frame_id = ns+"/"+camera_left_name+"_optical_frame";
      sensor_msgs::ImagePtr msg_image_right_rec = cv_bridge::CvImage(header_image, "bgr8", image_right_rec).toImageMsg();
      msg_image_right_rec->header.frame_id = ns+"/"+camera_right_name+"_optical_frame";

      pub_image_left_rec.publish(msg_image_left_rec);
      pub_image_right_rec.publish(msg_image_right_rec);
    }
    rate.sleep();
  }
  return 0;
}
