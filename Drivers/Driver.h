//
// Created by root on 2021/3/5.
//

#ifndef HERO_DRIVER_H
#define HERO_DRIVER_H
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace cv;

class Driver
{
public:
    virtual bool InitCam() = 0 ;
    virtual bool StartGrab() = 0;
    virtual bool SetCam() = 0;
    virtual bool Grab(Mat& src) = 0;
    virtual bool StopGrab() = 0;

    virtual ~Driver() = default;;
};

#endif //HERO_DRIVER_H
