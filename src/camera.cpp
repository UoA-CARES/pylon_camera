#include "camera.h"

cv::Mat Camera::getImage(int time_out){
  //Grab image from Baslar camera via Pylon, code from Pylon Examples
  CGrabResultPtr ptrGrabResult;
  CPylonImage image;
  CImageFormatConverter fc;
  fc.OutputPixelFormat = PixelType_BGR8packed;
  //Get the image from the camera's buffer
  this->camera.RetrieveResult(time_out, ptrGrabResult, TimeoutHandling_ThrowException);

  // Image grabbed successfully?
  cv::Mat cv_image;
  if (ptrGrabResult->GrabSucceeded()){
    fc.Convert(image, ptrGrabResult);
    //Create a Mat structure for the image to be stored
    cv::Mat frame(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)image.GetBuffer());
    frame.copyTo(cv_image);
  } else{
    ROS_ERROR("Camera %s failed to grab!", this->camera_name.c_str());
    exit(1);
  }
  return cv_image;
}


void Camera::setSlaveTrigger() {
  try
  {
    //Set pin that will trigger the camera
    this->camera.TriggerMode.SetValue(TriggerMode_On);
    this->camera.TriggerSource.SetValue(TriggerSource_Line3);

    //Set exposure mode
    this->camera.ExposureMode.SetValue(ExposureMode_Timed);
    //camera->ExposureTime.SetValue(400.0);

    //Set camera to start grabbing
    this->camera.StartGrabbing();
  }
  catch (const GenericException &e)
  {
    ROS_ERROR("Camera crashed! %s", e.GetDescription());
    exit(1);
  }
  return;
}

void Camera::setMasterTrigger() {
  try
  {
    //Set exposure pin to go high when ready
    this->camera.LineSelector.SetValue(LineSelector_Line3);
    this->camera.LineMode.SetValue(LineMode_Output);
    this->camera.LineSource.SetValue(LineSource_ExposureActive);

    //Set software trigger to fire camera
    this->camera.AcquisitionMode.SetValue(AcquisitionMode_Continuous);
    this->camera.TriggerMode.SetValue(TriggerMode_On);
    this->camera.TriggerSource.SetValue(TriggerSource_Software);

    //Set exposure mode
    this->camera.ExposureMode.SetValue(ExposureMode_Timed);
    //camera->ExposureTime.SetValue(400.0);

    //Set camera to start grabbing
    this->camera.StartGrabbing();
  }
  catch (const GenericException &e)
  {
    ROS_ERROR("Camera crashed! %s", e.GetDescription());

    exit(1);
  }
  return;
}

void Camera::createCamera(Camera_t &current_camera, String_t camera_name){

  // Get the transport layer factory.
  CTlFactory& tlFactory = CTlFactory::GetInstance();

  // Get all attached devices and exit application if no device is found
  DeviceInfoList_t devices;
  tlFactory.EnumerateDevices(devices);

  //Create a camera instance for the camera matching the relevant serial number
  CDeviceInfo curCameraInfo;
  for (int i = 0; i < devices.size(); ++i) {
    if(!std::strcmp(devices[i].GetUserDefinedName(), camera_name)){
        ROS_INFO("Camera: %s", devices[i].GetUserDefinedName().c_str());
        curCameraInfo = devices[i];
    }
  }
  current_camera.Attach( CTlFactory::GetInstance().CreateDevice(curCameraInfo));
  current_camera.MaxNumBuffer = 1;
  current_camera.Open();
}

void Camera::trigger(){
  this->camera.ExecuteSoftwareTrigger();
}

// std_msgs::Header softwareTrigger(Camera_t *camera){
//   int exitCode = 0;
//   std_msgs::Header header;
//   try
//   {
//     //Use lineselector to select a GPIO port.
//     camera->LineSelector.SetValue(LineSelector_Line4);
//     //Use linemode to set the GPIO port to input or output
//     camera->LineMode.SetValue(LineMode_Output);
//     // Use the line source to choose a UserOutput channel (essentially connecting a Line (GPIO port) to a UserOutput channel).
//     camera->LineSource.SetValue(LineSource_UserOutput3);
//     //Use the UserOutputSelector to select the userOutput channel selected above
//     camera->UserOutputSelector.SetValue(UserOutputSelector_UserOutput3);
//     //Set the value of the user output channel. True and false enable a rising edge.
//     if(camera->LineStatus.GetValue()){
//       //Need to set high and low otherwise get caught in a loop waiting for the camera to retrieve the result
//       camera->UserOutputValue.SetValue(false);
//       usleep(10);
//       header.stamp = ros::Time::now();
//       camera->UserOutputValue.SetValue(true);
//       usleep(10);
//       camera->UserOutputValue.SetValue(false);
//     }else if (!camera->LineStatus.GetValue()){
//       header.stamp = ros::Time::now();
//       camera->UserOutputValue.SetValue(true);
//       usleep(10);
//       camera->UserOutputValue.SetValue(false);
//     }
//   }
//   catch (const GenericException &e)
//   {
//     ROS_ERROR("Error: %s", e.GetDescription());
//     exitCode = 1;
//     PylonTerminate();
//     exit(exitCode);
//   }
//   return header;
// }

// void Camera::setSlaveTrigger(){
//   try{
//     //Set pin that will trigger the camera
//     this->camera.TriggerMode.SetValue(TriggerMode_On);
//     this->camera.TriggerSource.SetValue(TriggerSource_Line3);

//     //Set exposure mode
//     this->camera.ExposureMode.SetValue(ExposureMode_Timed);
//     //camera->ExposureTime.SetValue(400.0);

//     //Set camera to start grabbing
//     this->camera.StartGrabbing();
//   }
//   catch (const GenericException &e){
//     ROS_ERROR("Camera crashed! %s", e.GetDescription());

//     exit(1);
//   }
// }

// void Camera::setMasterTrigger(){
//   try{
//     //Set exposure pin to go high when ready
//     this->camera.LineSelector.SetValue(LineSelector_Line3);
//     this->camera.LineMode.SetValue(LineMode_Output);
//     this->camera.LineSource.SetValue(LineSource_ExposureActive);

//     //Set software trigger to fire camera
//     this->camera.AcquisitionMode.SetValue(AcquisitionMode_Continuous);
//     this->camera.TriggerMode.SetValue(TriggerMode_On);
//     this->camera.TriggerSource.SetValue(TriggerSource_Software);

//     //Set exposure mode
//     this->camera.ExposureMode.SetValue(ExposureMode_Timed);
//     //camera->ExposureTime.SetValue(400.0);

//     //Set camera to start grabbing
//     this->camera.StartGrabbing();
//   }
//   catch (const GenericException &e){
//     ROS_ERROR("Camera crashed! %s", e.GetDescription());
//     exit(1);
//   }
// }