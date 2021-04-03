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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     These parameters are reconfigurable                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEPTH_FORMAT          RS2_FORMAT_Z16    // rs2_format identifies how binary data is encoded within a frame//
#define COLOR_FORMAT          RS2_FORMAT_BGR8    // rs2_format identifies how binary data is encoded within a frame//
#define FPS             60                // Defines the rate of frames per second                                //
#define STREAM_INDEX    0           // Defines the stream index, used for multiple streams of the same type //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

    float GetArmorDepth(Rect& rect);
private:
    rs2_error* e;
    rs2_context* ctx;

    rs2_device* dev;

    rs2_pipeline* pipeline;
    rs2_config* config;

    rs2_pipeline_profile* pipeline_profile;
    rs2_frame* frames;

    int num_of_frames;

    rs2_frame* depthFrame;
    float dist2Armor;
};
#endif //INFANTRY_REALSENSEDRIVER_H
