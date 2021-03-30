#ifndef PYLON_CAMERA_H
#define PYLON_CAMERA_H

#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
typedef Pylon::CBaslerUsbInstantCamera Camera_t;
using namespace Basler_UsbCameraParams;

using namespace Pylon;

class Camera{

  private:
    Camera_t camera;
    std::string camera_name;

    void createCamera(Camera_t &current_camera, String_t camera_name);

    void setSlaveTrigger();

    void setMasterTrigger();

  public:
    Camera(std::string camera_name, bool master=false){
      this->camera_name = camera_name;
      this->createCamera(this->camera, this->camera_name.c_str());
      if(master)
        this->setMasterTrigger();
      else
        this->setSlaveTrigger();

      this->camera.AcquisitionStart.Execute();
    }

    cv::Mat getImage(int timeout=1000);

    void trigger();

    std::string name();
    int getBinningX();
    int getBinningY();
    int getOffsetX();
    int getOffsetY();
    int getImageWidth();
    int getImageHeight();

};

#endif