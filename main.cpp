//
// Created by luojunhui on 1/28/20.
//
#include<thread>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "preoptions.h"
#include "ArmorDetector.hpp"
#include "SerialPort.hpp"
#include "Media/RMDriver.h"
#include "SolveAngle.hpp"
#define CVUI_IMPLEMENTATION
#include "RMTools.hpp"
#include "SerialPort.hpp"
#define USECAM 1

using namespace std;
using namespace rm;

int main(int argc, char** argv)
{
    VideoCapture cap;
    RMDriver videoDriver;
    Mat src;
    ArmorDetector AD;
    SolveAngle sA;
    Serial serial;
    float yaw,pitch,dis;
    RNG rng;

    const int stateNum = 4;
    const int measureNum = 2;
    const int winHeight = (int)src.rows;
    const int winWidth = (int)src.cols;
    Mat prediction;
    Point predict_pt;

    KalmanFilter KF(stateNum, measureNum, 0);

    KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 10, 0, 0, 1, 0, 10, 0, 0, 1, 0, 0, 0, 0, 1);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(1));
    rng.fill(KF.statePost, RNG::UNIFORM, 0, winHeight > winWidth ? winWidth : winHeight);
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
    Point2f pret;

    //Visualize
    int value1, value;
    Mat src1 = Mat(780, 1080, CV_8UC3, Scalar(255, 255, 255));
    Mat src2 = Mat(640, 720, CV_8UC3, Scalar(255, 255, 255));
    auto *a = new RMTools::DisPlayWaveCLASS(src1, &value, &value1,"Display Window", 100);

    Mat DebugWindow = Mat::zeros(480,640,CV_8UC3);
//    Mat checkBG = Mat(100,150,CV_8UC3,Scalar(255,255,255));
//    char OSName[20];
//    int16_t count = 0;
//    char current_str[100], current_str_win[100];
//    string checkWindow = "ChecBox";
//    cv::namedWindow(checkWindow);
//    cvui::init(checkWindow);

#if USECAM==1
    videoDriver.InitCam();
    videoDriver.StartGrab();
    CThread::sleep(100);
    if(videoDriver.SetCam() == -1)
    {
        cout<<"Set Camera Failed!"<<endl;
        return 0;
    }
    videoDriver.Grab(src);
    //printf("whatever");
#elif USECAM==0
    cap.open("/home/ljh/视频/Videos/test1.mp4");
    cap.read(src);
#endif

    AD.Init();
    serial.InitPort();
    while (1)
    {
        if(!src.empty())
        {
            //imwrite("src.jpg",src);
            if(AD.ArmorDetectTask(src))
            {
                sA.GetPose(AD.lastArmor.rect,20,yaw,pitch,dis,AD.IsSmall());
                prediction = KF.predict();
                predict_pt = Point2f(prediction.at<float>(0), prediction.at<float>(1));   //Ô¤²âÖµ(x',y')
                //3.update measurement
                measurement.at<float>(0) = (float)AD.lastArmor.rect.x;
                measurement.at<float>(1) = (float)AD.lastArmor.rect.y;
                KF.correct(measurement);
                value = (int)(pret.x/1000+200);
                value1 = (int)(predict_pt.x/1000+200);
                serial.pack(pitch,yaw,dis,1,1,0);
            }else
            {
                value = (int)(pret.x/100+200);
                value1 = (int)(predict_pt.x/100+200);
            	serial.pack(0,0,dis,1,1,0);
            }
            serial.WriteData();

////            a->DisplayWave2();
////            imshow("src",src);
//            cout<<"pitch: "<<pitch<<" "<<"yaw: "<<yaw<<" "<<"distance: "<<dis<<" SMALL?: "<<AD.IsSmall()<<" ArmorX: "<<AD.lastArmor.center.x<<" ArmorY: "<<AD.lastArmor.center.y<<endl;
////            if (cvui::button(checkBG,  10, 20, "GRAB Image"))
////            {
////                sprintf(current_str, "%s_PIC_%d.jpg",OSName,count++);
////                imwrite(current_str, src);
////            }
////            imshow(checkWindow, checkBG);
        }

#if USECAM==1
        videoDriver.Grab(src);
#elif USECAM==0
        cap.read(src);
#endif
        if(waitKey(20)==27)break;
    }

    return 0;
}


