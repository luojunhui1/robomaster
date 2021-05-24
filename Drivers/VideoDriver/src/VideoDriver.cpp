//
// Created by root on 2021/3/6.
//
#include "VideoDriver.hpp"

bool VideoDriver::InitCam()
{
    if(carName == INFANTRY_MELEE || carName == INFANTRY_TRACK)
        capture = VideoCapture(cameraIndex);
    else
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
bool VideoDriver::SetCam()
{
    if(carName == INFANTRY_MELEE || carName == INFANTRY_TRACK)
    {
        capture.set(CAP_PROP_FRAME_WIDTH, IMAGEWIDTH); //帧宽

        capture.set(CAP_PROP_FRAME_HEIGHT, IMAGEHEIGHT);//帧高

        capture.set(CAP_PROP_FPS, 100);//帧率

        capture.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));//视频流格式

    }
    return true;
}
bool VideoDriver::Grab(Mat& src)
{
    //printf("Video Capture\n");
    capture.read(src);

    return !src.empty();
}

bool VideoDriver::StopGrab()
{

}
