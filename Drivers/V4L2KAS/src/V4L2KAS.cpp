//
// Created by root on 2021/1/19.
//
#include "V4L2KAS.h"

namespace V4L2KAS
{
    bool V4L2Driver::InitCam()
    {
        /**
        * Open the Video Device
        */
        memset(devicePath,'\0',30);
        strcpy(devicePath,"/dev/video");
        strcat(devicePath,(to_string(cameraIndex)).c_str());

        printf("device path: %s\n",devicePath);

        if ((fd = open(devicePath, O_RDWR)) == -1)
        {
            printf("Error opening V4L interface\n");
            return false;
        }
        b_Buffer =  (uchar*)malloc(IMAGEWIDTH * IMAGEHEIGHT * 2);
        return true;
    }

    bool V4L2Driver::Info()
    {
    /**
     * Read the information in video_capability, including the driver, card, bus information, version and etc.
     */
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
        {
            printf("Error opening device %s: unable to query device.\n",devicePath);
            return (false);
        }
        else
        {
            printf("driver:\t\t%s\n",cap.driver);
            printf("card:\t\t%s\n",cap.card);
            printf("bus_info:\t%s\n",cap.bus_info);
            printf("version:\t%d\n",cap.version);
            printf("capabilities:\t%x\n",cap.capabilities);

            if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE)
            {
                printf("Device %s: supports capture.\n",devicePath);
            }

            if ((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING)
            {
                printf("Device %s: supports streaming.\n",devicePath);
            }
        }

        /**
         * List all the image formats that the current video device supported
         */
        fmtdesc.index=0;
        fmtdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        printf("Support format:\n");
        while(ioctl(fd,VIDIOC_ENUM_FMT,&fmtdesc)!=-1)
        {
            printf("\t%d.%s\n",fmtdesc.index+1,fmtdesc.description);
            fmtdesc.index++;
        }
        return true;
    }

    bool V4L2Driver::SetCam()
    {
    /**
     * Set the image format as MJPEG
     */
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.height = IMAGEHEIGHT;
        fmt.fmt.pix.width = IMAGEWIDTH;
        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        if(ioctl(fd, VIDIOC_S_FMT, &fmt) == -1)
        {
            printf("Unable to set format\n");
            return false;
        }

        if(ioctl(fd, VIDIOC_G_FMT, &fmt) == -1)
        {
            printf("Unable to get format\n");
            return false;
        }
        {
            printf("fmt.type:\t\t%d\n",fmt.type);
            printf("pix.pixelformat:\t%c%c%c%c\n",fmt.fmt.pix.pixelformat & 0xFF, (fmt.fmt.pix.pixelformat >> 8) & 0xFF,(fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
            printf("pix.height:\t\t%d\n",fmt.fmt.pix.height);
            printf("pix.width:\t\t%d\n",fmt.fmt.pix.width);
            printf("pix.field:\t\t%d\n",fmt.fmt.pix.field);
        }

        if(!RequireBuffer())
        {
            printf("RequireBuffer Failed!\n");
            return false;
        }
        return true;
    }

    /*For simplicity of this project, we call this function at the end of SerCam function, so you should not use this
     * function independently unless you are sure what you are doing*/
    bool V4L2Driver::RequireBuffer()
    {
        /**
         * Require for image buffer
         */
        req.count=3;
        req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory=V4L2_MEMORY_MMAP;
        if(ioctl(fd,VIDIOC_REQBUFS,&req)==-1)
        {
            printf("request for buffers error\n");
        }

        /**
         *mmap the information transmitted by video device from fd to buffer, and store the add in the buffers
         */

        buffers = static_cast<buffer *>(malloc(req.count * sizeof(struct buffer)));
        if (!buffers)
        {
            printf ("Out of memory\n");
            return (false);
        }
        return true;
    }

    bool V4L2Driver::Grab(Mat& src)
    {
        for (int n_buffers = 0; n_buffers < req.count; n_buffers++)
        {
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = n_buffers;
            //query buffers
            try {
                if(ioctl (fd, VIDIOC_QUERYBUF, &buf) == -1)
                {
                    throw exception();
                }

            } catch (exception e) {
                printf("QUERY BUFFER ERROR: %s\n",to_string(errno).c_str());
                close(fd);
                InitCam();
            }
//            if (ioctl (fd, VIDIOC_QUERYBUF, &buf) == -1)
//            {
//                printf("query buffer error\n");
//                return (false);
//            }

            buffers[n_buffers].length = buf.length;
            //map
            buffers[n_buffers].start = mmap(NULL,buf.length,PROT_READ |PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
            if (buffers[n_buffers].start == MAP_FAILED)
            {
                printf("buffer map error\n");
                return (false);
            }
        }

        for (int n_buffers = 0; n_buffers < req.count; n_buffers++)
        {
            buf.index = n_buffers;
            ioctl(fd, VIDIOC_QBUF, &buf);
        }


        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl (fd, VIDIOC_STREAMON, &type);
        ioctl(fd, VIDIOC_DQBUF, &buf);

        //cv::Size szSize(IMAGEWIDTH,IMAGEHEIGHT);
        cv::Mat mSrc(IMAGEHEIGHT,IMAGEWIDTH,CV_8UC2, b_Buffer);

        memcpy((char*)b_Buffer,buffers[0].start, sizeof(uchar)*IMAGEHEIGHT*IMAGEWIDTH*2);

        cvtColor(mSrc, src, COLOR_YUV2BGR_YUYV);

        //muanmp,  release memory, https://blog.csdn.net/wenrenhua08/article/details/40044495
        for (int n_buffers = 0; n_buffers < req.count; n_buffers++)
        {
            munmap(buffers[n_buffers].start,buffers[n_buffers].length);
        }

        return true;
    }
    bool V4L2Driver::StartGrab()
    {
        return true;
    }

    bool V4L2Driver::StopGrab()
    {
        delete b_Buffer;
        delete buffers;
    }

}
