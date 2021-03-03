//
// Created by root on 2021/1/21.
//

#include "RealSenseDriver.h"

void RealSenseDriver::Init()
{
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        perror("No device detected. Stopped!");
    device dev = list.front();

    config = new rs2::config();
    //Add desired streams to configuration
    config->enable_stream(RS2_STREAM_COLOR,IMAGEWIDTH,IMAGEHEIGHT,RS2_FORMAT_BGR8,FPS);
    config->enable_stream(RS2_STREAM_DEPTH, IMAGEWIDTH, IMAGEHEIGHT, RS2_FORMAT_Z16,FPS);
    config->enable_stream(RS2_STREAM_INFRARED, 1, IMAGEWIDTH, IMAGEHEIGHT, RS2_FORMAT_Y8, FPS);
    config->enable_stream(RS2_STREAM_INFRARED, 2, IMAGEWIDTH, IMAGEHEIGHT, RS2_FORMAT_Y8, FPS);
}

void RealSenseDriver::Start()
{
    pipe.start(*config);
}

void RealSenseDriver::Grab(Mat& src)
{
    data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
    frame_ = data.get_color_frame();

    src_ =  Mat(Size(IMAGEWIDTH,IMAGEHEIGHT), CV_8UC3,(void*)frame_.get_data(),Mat::AUTO_STEP);

    flip(src_,src,-1);
}