//
// Created by root on 2021/1/19.
//

#ifndef CAMERANEW_V4L2KAS_H
#define CAMERANEW_V4L2KAS_H

#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>

#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "mydefine.h"
#include "Driver.h"

using namespace std;
using namespace cv;

namespace V4L2KAS
{
    struct buffer
    {
        void * start;
        unsigned int length;
    };

    /**
     * @brief camera driver for all camera support VUC protocol
     * @param none
     */
    class V4L2Driver: public Driver
    {
        char devicePath[30];
        int      fd;
        struct   v4l2_capability   cap;
        struct v4l2_fmtdesc fmtdesc;
        struct v4l2_format fmt,fmtack;
        struct v4l2_streamparm setfps;
        struct v4l2_requestbuffers req;
        struct v4l2_buffer buf;
        enum v4l2_buf_type type;
        //unsigned char frame_buffer[FRAMEHEIGHT*FRAMEWIDTH*3];
        struct buffer *buffers;
        uchar *b_Buffer;
    public:
        bool InitCam() override;
        bool StartGrab() override;
        bool Info();
        bool  SetCam() override;
        bool RequireBuffer();
        bool Grab(Mat& src) override;
        bool StopGrab() override;
    };
}
#endif //CAMERANEW_V4L2KAS_H
