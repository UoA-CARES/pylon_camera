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
  public:
    enum TriggerMode {
        TRIGGER_MODE_ASYNC,
        TRIGGER_MODE_EXPOSURE,
        TRIGGER_MODE_PIN};

  private:
    Camera_t camera;
    std::string camera_id;
    std::string name;
    TriggerMode trigger_mode;
    bool master;

    void createCamera(Camera_t &current_camera, String_t camera_name);

    void setExposureTrigger();
    void setExposureMaster();
    void setPinTrigger();

    void toggleTriggerPin();

  public:
    Camera(std::string camera_id, std::string name, TriggerMode trigger_mode, bool master=false){
      this->name = name;
      this->camera_id = camera_id;
      this->trigger_mode = trigger_mode;
      this->master = master;
      this->createCamera(this->camera, this->camera_id.c_str());

//      INodeMap* pNodeMap = this->camera.GetNodeMap();
//      this->camera.ExpertFeatureEnable.SetValue(true);
      //Set trigger configuration
      switch(this->trigger_mode){
        case TRIGGER_MODE_ASYNC:
          this->setExposureMaster();
          break;
        case TRIGGER_MODE_EXPOSURE:
          if(this->master)
            this->setExposureMaster();
          else
            this->setExposureTrigger();
          break;
        case TRIGGER_MODE_PIN:
          this->setPinTrigger();
          break;
      }
      this->camera.AcquisitionStart.Execute();
    }

    cv::Mat getImage(int timeout=1000);

    void trigger();

    std::string getName();
    int getBinningX();
    int getBinningY();
    int getOffsetX();
    int getOffsetY();
    int getImageWidth();
    int getImageHeight();

    void setAutoFunctions();

};

#endif