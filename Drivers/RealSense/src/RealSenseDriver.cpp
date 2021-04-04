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

bool RealSenseDriver::InitCam()
{
    // Create a context object. This object owns the handles to all connected realsense devices.
    // The returned object should be released with rs2_delete_context(...)
    ctx = rs2_create_context(RS2_API_VERSION, &e);
    check_error(e);

    /* Get a list of all the connected devices. */
    // The returned object should be released with rs2_delete_device_list(...)
    rs2_device_list* device_list = rs2_query_devices(ctx, &e);
    check_error(e);

    int dev_count = rs2_get_device_count(device_list, &e);
    check_error(e);
    printf("There are %d connected RealSense devices.\n", dev_count);
    if (0 == dev_count)
        return EXIT_FAILURE;
    // Get the first connected device
    // The returned object should be released with rs2_delete_device(...)
    dev = rs2_create_device(device_list, 0, &e);
    check_error(e);

    print_device_info(dev);

    return true;
}

bool RealSenseDriver::StartGrab()
{
    pipeline_profile = rs2_pipeline_start_with_config(pipeline, config, &e);
    return true;
}

bool RealSenseDriver::SetCam()
{
    // Create a pipeline to configure, start and stop camera streaming
    // The returned object should be released with rs2_delete_pipeline(...)
    pipeline =  rs2_create_pipeline(ctx, &e);
    check_error(e);

    config = rs2_create_config(&e);
    check_error(e);

    // Request a specific configuration
    rs2_config_enable_stream(config, RS2_STREAM_COLOR, STREAM_INDEX, FRAMEWIDTH, FRAMEHEIGHT, COLOR_FORMAT, FPS, &e);
    rs2_config_enable_stream(config, RS2_STREAM_DEPTH, STREAM_INDEX, FRAMEWIDTH, FRAMEHEIGHT, DEPTH_FORMAT, FPS, &e);
    check_error(e);

    return true;
}

bool RealSenseDriver::Grab(Mat& src)
{
    frames = rs2_pipeline_wait_for_frames(pipeline, RS2_DEFAULT_TIMEOUT, &e);

    // Returns the number of frames embedded within the composite frame
    if((num_of_frames = rs2_embedded_frames_count(frames, &e)) == 0)
    {
        return false;
    }


    int i;
    for (i = 0; i < num_of_frames; ++i)
    {
        // The retunred object should be released with rs2_release_frame(...)
        rs2_frame* frame = rs2_extract_frame(frames, i, &e);
        check_error(e);

        // Check if the given frame can be extended to depth frame interface
        // Accept only depth frames and skip other frames
        if (0 != rs2_is_frame_extendable_to(frame, RS2_EXTENSION_DEPTH_FRAME, &e))
        {
            //depthFrame can Not be released until GetArmorDepth function completed
            if(depthFrame != nullptr)
                rs2_release_frame(depthFrame);
            depthFrame = frame;
            check_error(e);
            continue;
        }

        src = Mat(Size(FRAMEWIDTH,FRAMEHEIGHT), CV_8UC3, (void*)rs2_get_frame_data(frame, &e),Mat::AUTO_STEP);

        if(src.empty())
        {
            rs2_release_frame(frames);
            return false;
        }

        flip(src,src,-1);

        rs2_release_frame(frame);
    }

    rs2_release_frame(frames);

    return true;
}

float RealSenseDriver::GetArmorDepth(Rect& rect)
{
    if(depthFrame == nullptr)
        return 0;
    float sum = 0,count = 0;
    for(int i = rect.x; i < rect.x + rect.width; i++)
    {
        for(int j = rect.y; j < rect.y + rect.height;j++)
        {
            dist2Armor = rs2_depth_frame_get_distance(depthFrame, i,j, &e);
            sum += (dist2Armor != 0)?(count++,dist2Armor):(0);
        }
    }

    rs2_release_frame(depthFrame);

    dist2Armor = (count > 0)?(sum/count):(0);
    return dist2Armor;
}
bool RealSenseDriver::StopGrab()
{
    rs2_pipeline_stop(pipeline, &e);
    check_error(e);

    // Release resources
    rs2_delete_pipeline_profile(pipeline_profile);
    rs2_delete_config(config);
    rs2_delete_pipeline(pipeline);
    rs2_delete_device(dev);
    rs2_delete_context(ctx);

    return EXIT_SUCCESS;

}
