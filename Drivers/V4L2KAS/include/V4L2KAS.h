//
// Created by root on 2021/1/19.
//

#ifndef CAMERANEW_V4L2KAS_H
#define CAMERANEW_V4L2KAS_H

#include <iostream>
#include <string>
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

#define FILE_VIDEO     "/dev/video0"

using namespace cv;

namespace V4L2KAS
{
    struct buffer
    {
        void * start;
        unsigned int length;
    };


    class V4L2Driver
    {
        int      fd;
        struct   v4l2_capability   cap;
        struct v4l2_fmtdesc fmtdesc;
        struct v4l2_format fmt,fmtack;
        struct v4l2_streamparm setfps;
        struct v4l2_requestbuffers req;
        struct v4l2_buffer buf;
        enum v4l2_buf_type type;
        unsigned char frame_buffer[IMAGEWIDTH*IMAGEHEIGHT*3];
        struct buffer *buffers;

    public:
        bool Init();
        bool Info();
        bool SetFormat();
        bool RequireBuffer();
        bool Grab(Mat& src);

    };
}
#endif //CAMERANEW_V4L2KAS_H