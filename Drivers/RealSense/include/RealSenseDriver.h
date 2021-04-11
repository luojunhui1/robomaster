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
    bool StopGrab() override;
    bool Grab(Mat& src) override;

    void measure(Rect rect);

    Mat align_Depth2Color(Mat depth,Mat color,rs2::pipeline_profile profile);
    void measure_distance(Mat &color,Mat depth,rs2::pipeline_profile profile,Rect rect);
    float get_depth_scale(rs2::device dev);

private:
    rs2::colorizer c;
    rs2::config *config;
    context ctx;
    rs2::pipeline pipe;
    rs2::pipeline_profile profile;
    rs2::sensor sensor;

    rs2::frame frame_color;
    rs2::frame frame_depth;
    rs2::frame frame_show;
    cv::Mat src_color;
    cv::Mat src_depth;
    cv::Mat src_4_show;
    rs2::frameset data;
public:
    float dist2Armor;
};
#endif //INFANTRY_REALSENSEDRIVER_H