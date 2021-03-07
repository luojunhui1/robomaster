//
// Created by root on 2021/3/6.
//
#include "VideoDriver.hpp"

bool VideoDriver::InitCam()
{
    capture.open(videoPath);
    if(!capture.isOpened())
    {
        perror("Video Open Failed!\n");
        return false;
    }
    return true;
}
bool VideoDriver::StartGrab()
{
    return true;
}
int VideoDriver::SetCam()
{
    return true;
}
bool VideoDriver::Grab(Mat& src)
{
    //printf("Video Capture\n");
    capture.read(src);
    return !src.empty();
}

