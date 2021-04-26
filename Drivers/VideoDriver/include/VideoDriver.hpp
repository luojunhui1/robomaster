//
// Created by root on 2021/3/6.
//

#ifndef HERO_VIDEODRIVER_H
#define HERO_VIDEODRIVER_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "Driver.h"
#include "mydefine.h"

using namespace cv;

class VideoDriver: public Driver
{
private:
    VideoCapture capture;
public:
    bool InitCam() override;
    bool StartGrab() override;
    bool SetCam() override;
    bool Grab(Mat& src) override;
    bool StopGrab() override;
};
#endif //HERO_VIDEODRIVER_H
