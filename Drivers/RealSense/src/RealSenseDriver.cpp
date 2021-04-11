//
// Created by root on 2021/1/21.
//

#include "RealSenseDriver.h"

/* Function calls to librealsense may raise errors of type rs_error*/
void check_error(rs2_error* e)
{
    if (e)
    {
        printf("rs_error was raised when calling %s(%s):\n", rs2_get_failed_function(e), rs2_get_failed_args(e));
        printf("    %s\n", rs2_get_error_message(e));
        exit(EXIT_FAILURE);
    }
}

void print_device_info(rs2_device* dev)
{
    rs2_error* e = 0;
    printf("\nUsing device 0, an %s\n", rs2_get_device_info(dev, RS2_CAMERA_INFO_NAME, &e));
    check_error(e);
    printf("    Serial number: %s\n", rs2_get_device_info(dev, RS2_CAMERA_INFO_SERIAL_NUMBER, &e));
    check_error(e);
    printf("    Firmware version: %s\n\n", rs2_get_device_info(dev, RS2_CAMERA_INFO_FIRMWARE_VERSION, &e));
    check_error(e);
}

static std::string get_sensor_name(const rs2::sensor& sensor)
{
    // Sensors support additional information, such as a human readable name
    if (sensor.supports(RS2_CAMERA_INFO_NAME))
        return sensor.get_info(RS2_CAMERA_INFO_NAME);
    else
        return "Unknown Sensor";
}

static rs2::sensor get_a_sensor_from_a_device(const rs2::device& dev)
{
    // A rs2::device is a container of rs2::sensors that have some correlation between them.
    // For example:
    //    * A device where all sensors are on a single board
    //    * A Robot with mounted sensors that share calibration information

    // Given a device, we can query its sensors using:
    std::vector<rs2::sensor> sensors = dev.query_sensors();

    std::cout << "Device consists of " << sensors.size() << " sensors:\n" << std::endl;
    int index = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors)
    {
        std::cout << "  " << index++ << " : " << get_sensor_name(sensor) << std::endl;
    }

    //uint32_t selected_sensor_index = get_user_selection("Select a sensor by index: ");
    uint32_t selected_sensor_index = 1;
    // The second way is using the subscript ("[]") operator:
    if (selected_sensor_index >= sensors.size())
    {
        throw std::out_of_range("Selected sensor index is out of range");
    }

    return  sensors[selected_sensor_index];
}

bool RealSenseDriver::InitCam()
{
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        perror("No device detected. Stopped!");
    device dev = list.front();
    sensor = get_a_sensor_from_a_device(dev);

    config = new rs2::config();
    //Add desired streams to configuration
    config->enable_stream(RS2_STREAM_COLOR,IMAGEWIDTH,IMAGEHEIGHT,RS2_FORMAT_BGR8,FPS);
    config->enable_stream(RS2_STREAM_DEPTH, IMAGEWIDTH, IMAGEHEIGHT, RS2_FORMAT_Z16,FPS);

    return true;
}

bool RealSenseDriver::StartGrab()
{
    profile = pipe.start(*config);
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    return true;
}

bool RealSenseDriver::SetCam()
{
    rs2::option_range range = sensor.get_option_range(RS2_OPTION_EXPOSURE);
    float default_value = range.def;
    float maximum_supported_value = range.max;
    float minimum_supported_value = range.min;
    float difference_to_next_value = range.step;
    std::cout << "  Min Value     : " << minimum_supported_value << std::endl;
    std::cout << "  Max Value     : " << maximum_supported_value << std::endl;
    std::cout << "  Default Value : " << default_value << std::endl;
    std::cout << "  Step          : " << difference_to_next_value << std::endl;

    sensor.set_option(RS2_OPTION_EXPOSURE,60);

    return true;
}

bool RealSenseDriver::Grab(Mat& src)
{
    data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
    frame_color = data.get_color_frame();
    frame_depth = data.get_depth_frame();
    frame_show = data.get_depth_frame().apply_filter(c);

    src_depth = Mat(Size(IMAGEWIDTH,IMAGEHEIGHT), CV_16U,(void*)frame_depth.get_data(),Mat::AUTO_STEP);
    src_color =  Mat(Size(IMAGEWIDTH,IMAGEHEIGHT), CV_8UC3,(void*)frame_color.get_data(),Mat::AUTO_STEP);

    flip(src_color,src,-1);
    if(src_color.empty())
        return false;
    return true;
}

void RealSenseDriver::measure(Rect rect)
{
    Mat result = align_Depth2Color(src_depth,src_color,profile);
    measure_distance(src_color,result,profile,rect);
}

//获取深度像素对应长度单位（米）的换算比例
float RealSenseDriver::get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}
//深度图对齐到彩色图函数
Mat RealSenseDriver::align_Depth2Color(Mat depth, Mat color, rs2::pipeline_profile profile)
{
    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取内参
    const auto intrinDepth=depth_stream.get_intrinsics();
    const auto intrinColor=color_stream.get_intrinsics();

    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    //auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    rs2_extrinsics  extrinDepth2Color;
    rs2_error *error;
    rs2_get_extrinsics(depth_stream,color_stream,&extrinDepth2Color,&error);

    //平面点定义
    float pd_uv[2],pc_uv[2];
    //空间点定义
    float Pdc3[3],Pcc3[3];

    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    int y=0,x=0;
    //初始化结果
    //Mat result=Mat(color.rows,color.cols,CV_8UC3,Scalar(0,0,0));
    Mat result=Mat(color.rows,color.cols,CV_16U,Scalar(0));
    //对深度图像遍历
    for(int row=0;row<depth.rows;row++)
    {
        for(int col=0;col<depth.cols;col++)
        {
            //将当前的(x,y)放入数组pd_uv，表示当前深度图的点
            pd_uv[0]=col;
            pd_uv[1]=row;
            //取当前点对应的深度值
            uint16_t depth_value=depth.at<uint16_t>(row,col);
            //换算到米
            float depth_m=depth_value*depth_scale;
            //将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_m);
            //将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
            //将彩色摄像头坐标系下的深度三维点映射到二维平面上
            rs2_project_point_to_pixel(pc_uv,&intrinColor,Pcc3);

            //取得映射后的（u,v)
            x=(int)pc_uv[0];
            y=(int)pc_uv[1];
//            if(x<0||x>color.cols)
//                continue;
//            if(y<0||y>color.rows)
//                continue;
            //最值限定
            x=x<0? 0:x;
            x=x>depth.cols-1 ? depth.cols-1:x;
            y=y<0? 0:y;
            y=y>depth.rows-1 ? depth.rows-1:y;

            result.at<uint16_t>(y,x)=depth_value;
        }
    }
    //返回一个与彩色图对齐了的深度信息图像
    return result;
}

void RealSenseDriver::measure_distance(Mat &color, Mat depth, rs2::pipeline_profile profile, Rect rect)
{
    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    //定义图像中心点
    cv::Point center(color.cols/2,color.rows/2);
    //定义计算距离的范围
    //cv::Rect RectRange(center.x-range.width/2,center.y-range.height/2,range.width,range.height);
    //遍历该范围
    float distance_sum=0;
    int effective_pixel=0;
    for(int y=rect.y;y<rect.y+rect.height;y++)
    {
        for(int x=rect.x;x<rect.x+rect.width;x++)
        {
            //如果深度图下该点像素不为0，表示有距离信息
            if(depth.at<uint16_t>(y,x))
            {
                distance_sum+=depth_scale*depth.at<uint16_t>(y,x);
                effective_pixel++;
            }
        }
    }

    dist2Armor = distance_sum/effective_pixel;

}

bool RealSenseDriver::StopGrab()
{
    return true;
}
