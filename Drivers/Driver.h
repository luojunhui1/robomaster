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
    virtual bool InitCam(){
        return false;
    };
    virtual bool StartGrab(){
        return false;
    };
    virtual int SetCam(){
        return 0;
    };
    virtual bool Grab(Mat& src){
        return false;
    };
};

#endif //HERO_DRIVER_H
