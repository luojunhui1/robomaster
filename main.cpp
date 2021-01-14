
#include<thread>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ArmorDetector.hpp"

#include "SerialPort.hpp"

#include "Media/RMDriver.h"
#include "RMVideoCapture.hpp"

#include "SolveAngle.hpp"

#include "RMTools.hpp"

#include "mydefine.h"

#define USECAM 0
#define USEDAHUA 1
#define USEXAVIER 1
#define DEBUGMODE 0

using namespace std;
using namespace rm;
using namespace cv;

int main(int argc, char** argv)
{
    redTarget = true;
    showArmorBoxes = true;
    Mat src;
    ArmorDetector AD;
    SolveAngle sA;
    Serial serial;
    float yaw,pitch,dis;
    RNG rng;

    freopen("log.txt","w",stdout);

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

#if DEBUGMODE == 1
    int value1, value;
    Mat src1 = Mat(780, 1080, CV_8UC3, Scalar(255, 255, 255));
    Mat src2 = Mat(640, 720, CV_8UC3, Scalar(255, 255, 255));
    auto *a = new RMTools::DisPlayWaveCLASS(src1, &value, &value1,"Display Window", 200);

    Mat DebugWindow = Mat::zeros(480,640,CV_8UC3);
#endif

#if USECAM==1
    #if USEDAHUA==1
        RMDriver videoDriver;
        videoDriver.InitCam();
        videoDriver.StartGrab();
        CThread::sleep(100);
        if(videoDriver.SetCam() == -1)
        {
            cout<<"Set Camera Failed!"<<endl;
            return 0;
        }
        videoDriver.Grab(src);
    #else
        RMVideoCapture videoDriver(2);
        if(!videoDriver.isOpened())
        {
            cout<<"Open Camera Failed!"<<endl;
            return 0;
        }
        videoDriver.startStream();
        videoDriver.info();
    #endif
#elif USECAM==0
    VideoCapture cap;
    cap.open("/home/ljh/视频/Videos/LINUX_Video_0.avi");
    cap.read(src);
#endif

    AD.Init();
    //serial.InitPort();

    while (1)
    {
        if(!src.empty())
        {
            //imwrite("src.jpg",src);
            if(AD.ArmorDetectTask(src))
            {
                sA.GetPoseV(AD.targetArmor.pts,20,yaw,pitch,dis,AD.IsSmall());
                prediction = KF.predict();
                predict_pt = Point2f(prediction.at<float>(0), prediction.at<float>(1));
                //3.update measurement
                measurement.at<float>(0) = (float)yaw;
                measurement.at<float>(1) = (float)pitch;

                yaw = yaw - 10;
                //KF.correct(measurement);
#if DEBUGMODE == 1
                if(abs(yaw - sA.averageY) < 60)
                    value = yaw + 200;
                else
                    value = sA.averageY + 200;
                //value1 = (int)(predict_pt.x+200);
                //value = AD.targetArmor.rect.y/10;
//                cout<<"Armor Left Top X:"<<AD.targetArmor.pts[0].x<<" Armor Left Top Y:"<<AD.targetArmor.pts[0].y<<endl;
//                cout<<"Armor Right Top X:"<<AD.targetArmor.pts[1].x<<" Armor Right Top Y:"<<AD.targetArmor.pts[1].y<<endl;
//                cout<<"Armor Right Bottom X:"<<AD.targetArmor.pts[2].x<<" Armor Right Bottom Y:"<<AD.targetArmor.pts[2].y<<endl;
//                cout<<"Armor Left Bottom X:"<<AD.targetArmor.pts[3].x<<" Armor Left Bottom Y:"<<AD.targetArmor.pts[3].y<<endl;
//                cout<<"-------------------------------------------------------------------------------------------"<<endl;
                //value = value + 200;
                cout<<yaw<<endl;
#endif
                serial.pack(pitch,yaw,dis,1,1,0);
            }else
            {
#if DEBUGMODE == 1
                value1 = (int)(predict_pt.x + 200);
#endif
                serial.pack(0,0,dis,1,1,0);
            }
            serial.WriteData();
#if DEBUGMODE == 1
            a->DisplayWave();
#endif
            imshow("src",src);
        }
#if USECAM==1
    #if USEDAHUA==1
        videoDriver.Grab(src);
    #else
        videoDriver>>src;
    #endif
#elif USECAM==0
        cap.read(src);
#endif
        if(waitKey(20)==27)break;
    }
    return 0;
}


