#ifndef PYLON_PARAMETERS_H
#define PYLON_PARAMETERS_H

#include <ros/package.h>

namespace CARES {
    namespace Pylon{
        const std::string DISPLAYP_B = "display";
        const std::string TRIGGER_MODE_I = "trigger_mode";
        const std::string CAMERA_LEFT_S  = "camera_left";
        const std::string CAMERA_RIGHT_S = "camera_right";
        const std::string CALIBRATION_S = "calibration";
        const std::string LOOP_RATE_I = "loop_rate";
    }
}

#endif //PYLON_PARAMETERS_H