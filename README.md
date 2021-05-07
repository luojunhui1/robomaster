# 工程相关库

**主要是大华相机的驱动，V4L2KAS相机的驱动只需要V4L2即可，Realsense相机的驱动只需在Intel Realsense的官方github仓库下载安装即可。**
1. **V4L2KAS**
支持UVC协议的摄像头应该只需要V4L2驱动设置好即可，关于其驱动的撰写可以参考如下博客或直接使用Driver/V4L2KAS文件夹下的源文件驱动USB摄像头：
参考博客：
- https://www.cnblogs.com/surpassal/archive/2012/12/19/zed_webcam_lab1.html
- https://www.cnblogs.com/lovebay/p/13553842.html

2. **Realsense**
按官方仓库的指示按需求安装即可，为正常使用本工程，至少需要安装librealsense2-dev和librealsense2-dbg
官方仓库：
- https://github.com/IntelRealSense/librealsense

3. **DAHUA**
大华相机的驱动和动态库封装得一言难尽，有需求可以在其官方网站上下载SDK并参考SDK安装目录下的例程学习使用其SDK。本仓库下的大华驱动库并不完整，使用时将对应平台下的库文件全部复制到Drivers/DAHUA/lib文件夹中，并复制到可执行文件所在文件夹中。

官方SDK：
- http://download.huaraytech.com/pub/sdk/
