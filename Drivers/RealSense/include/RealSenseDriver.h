//
// Created by root on 2021/1/21.
//

#ifndef INFANTRY_REALSENSEDRIVER_H
#define INFANTRY_REALSENSEDRIVER_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include "mydefine.h"
#include "Driver.h"

#define  FPS           60

using namespace rs2;
using namespace cv;

/**
 * @brief camera driver for Intel 435D
 * @param none
 * @details the api provided by intel is simple and easy to use, which forms a sharp contrast with dahua's api
 */
class RealSenseDriver: public Driver
{
public:
    bool InitCam() override;
    bool StartGrab() override;
    bool SetCam() override;
    bool Grab(Mat& src) override;
    bool StopGrab() override;
private:
    rs2::config *config;
    context ctx;
    rs2::pipeline pipe;

    rs2::frame frame_;
    cv::Mat src_;
    rs2::frameset data;
};
#endif //INFANTRY_REALSENSEDRIVER_H
