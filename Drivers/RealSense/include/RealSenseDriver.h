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

#define  FPS           60

using namespace rs2;
using namespace cv;

class RealSenseDriver
{
public:
    void Init();
    void Start();
    void Grab(Mat& src);
private:
    rs2::config *config;
    context ctx;
    rs2::pipeline pipe;

    rs2::frame frame_;
    cv::Mat src_;
    rs2::frameset data;
};

#endif //INFANTRY_REALSENSEDRIVER_H
