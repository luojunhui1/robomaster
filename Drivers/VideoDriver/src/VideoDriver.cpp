//
// Created by root on 2021/3/6.
//
#include "VideoDriver.hpp"

bool VideoDriver::InitCam()
{
    capture.open(videoPath);
    if(!capture.isOpened())return false;
    return true;
}
bool VideoDriver::StartGrab()
{

}
int VideoDriver::SetCam()
{

}
bool VideoDriver::Grab(Mat& src)
{
    capture.read(src);
}

