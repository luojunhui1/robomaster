//
// Created by luojunhui on 1/28/20.
//

#include <csignal>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <chrono>
#include <iostream>
#include <memory>
#include "MyThread.hpp"

using namespace std;
using namespace cv;

namespace rm
{
    bool ImgProdCons::quitFlag = false;

    bool produceMission = false;
    bool detectMission = false;
    bool energyMission = false;
    bool feedbackMission = false;

    int8_t curControlState = AUTO_SHOOT_STATE; //current control mode
    uint8_t curDetectMode = SEARCH_MODE; //tracking or searching
    uint8_t direction = false;
    u_int8_t clearFilter = false;

    RealSenseDriver intelCapture;
#if DEBUG == 1
    double time;
    int frequency;
    Mat debugWindowCanvas = Mat(300,500,CV_8UC1,Scalar(0));
#endif

    static void sleep_ms(unsigned int secs)
    {
        struct timeval tval;

        tval.tv_sec=secs/1000;

        tval.tv_usec=(secs*1000)%1000000;

        select(0,NULL,NULL,NULL,&tval);
    }

    void ImgProdCons::SignalHandler(int)
    {
        perror("sigaction error:");
        ImgProdCons::quitFlag = true;
        exit(-1);
    }


    void ImgProdCons::InitSignals()
    {
        ImgProdCons::quitFlag = false;
        struct sigaction sigact{};
        sigact.sa_handler = SignalHandler;
        sigemptyset(&sigact.sa_mask);
        sigact.sa_flags = 0;
        /*if interrupt occurs,set _quit_flag as true*/
        sigaction(SIGINT, &sigact, (struct sigaction *)nullptr);
    }


    ImgProdCons::ImgProdCons():
            serialPtr(unique_ptr<Serial>(new Serial())),
            solverPtr(unique_ptr<SolveAngle>(new SolveAngle())),
            armorDetectorPtr(unique_ptr<ArmorDetector>(new ArmorDetector())),
            kalman(unique_ptr<Kalman>(new Kalman())),
            armorType(BIG_ARMOR),
            driver()
    {
    }

    void ImgProdCons::Init()
    {
        /*initialize signal*/
        InitSignals();

        /*initialize camera*/

        switch (carName) {
            case HERO:
                driver = &intelCapture;
                break;
            case INFANTRY_MELEE:
                driver = &v4l2Capture;
                break;
            case INFANTRY_TRACK:
                driver = &v4l2Capture;
                break;
            case SENTRY:
                driver = &dahuaCapture;
                break;
            case UAV:
                driver = &v4l2Capture;
                break;
            case VIDEO:
                driver = &videoCapture;
                break;
            case NOTDEFINED:
                driver = &videoCapture;
                break;
        }

        Mat curImage;
        driver->InitCam();
        LOGM("Camera Initialized\n");
        driver->SetCam();
        LOGM("Camera Set\n");
        driver->StartGrab();
        LOGM("Camera Start to Grab\n");

        do
        {
            if(driver->Grab(curImage))
            {
                FRAMEWIDTH = curImage.cols;
                FRAMEHEIGHT = curImage.rows;
                armorDetectorPtr->Init();
                break;
            }
        }while(true);

        LOGM("Initialization Completed\n");
    }

    void ImgProdCons::Produce()
    {
        static uint32_t seq = 0;

        int missCount = 0;

        while(!ImgProdCons::quitFlag)
        {
            unique_lock<mutex> lock(writeLock);

            writeCon.wait(lock,[]{ return !produceMission;});
#if DEBUG == 1
            debugWindowCanvas.zeros(Size(300,500),CV_8UC1);
            line(debugWindowCanvas,Point(300,0),Point(300,299),Scalar(255),2,LINE_4);
            putText(debugWindowCanvas,"Produce  Thread",Point(310,30),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

            putText(debugWindowCanvas,"SEARCH   MODE",Point(310,230),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
            putText(debugWindowCanvas,"TRACKING MODE",Point(310,260),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
            time = (double)getTickCount();
#endif
            if (!driver->Grab(frame) || frame.rows != FRAMEHEIGHT || frame.cols != FRAMEWIDTH)
            {
                missCount++;
                if(missCount > 10)
                {
                    quitFlag = true;
                    driver->StopGrab();
                    //readCon.notify_all();
                    LOGM("Process Exited");
                    exit(-1);
                }
                continue;
            }

            missCount = 0;

            detectFrame = frame.clone();

            detectMission  = energyMission = feedbackMission = false;
            produceMission = true;
            readCon.notify_all();
        }
    }

    void ImgProdCons::Detect()
    {
        do
        {
            unique_lock<mutex> lock(detectLock);

            readCon.wait(lock,[]{ return !detectMission&&produceMission;});
#if DEBUG == 1
            putText(debugWindowCanvas,"Detect   Thread",Point(310,60),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
            line(debugWindowCanvas,Point(000,200),Point(499,200),Scalar(255),2,LINE_4);
#endif
            switch (curDetectMode)
            {
                case SEARCH_MODE:
                {
#if DEBUG == 1
                    circle(debugWindowCanvas,Point(480,225),8,Scalar(255),-1);
#endif
                    if (armorDetectorPtr->ArmorDetectTask(detectFrame))
                    {
                        armorDetectorPtr->tracker = TrackerKCF::create();
                        armorDetectorPtr->tracker->init(detectFrame, armorDetectorPtr->targetArmor.rect);
                        curDetectMode = SEARCH_MODE;
                    }
                    else
                    {
                        curDetectMode = SEARCH_MODE;
                    }
                    //如果成功找到，将roi区域视为tracking区域,下一次进入tracking模式
                }
                break;
                case TRACKING_MODE:
                {
#if DEBUG == 1
                    circle(debugWindowCanvas,Point(480,255),8,Scalar(255),-1);
#endif
                    //这里一开始就是trakcking，若追踪到目标，继续，未追踪到，进入searching,进入tracking函数时要判断分类器是否可用
                    if (armorDetectorPtr->trackingTarget(detectFrame, armorDetectorPtr->targetArmor.rect))
                    {
                        curDetectMode = TRACKING_MODE;
                    }
                    else
                    {
                        curDetectMode = SEARCH_MODE;
                    }
                }
                break;
            }

            detectMission = true;

            writeCon.notify_all();

            feedbackCon.notify_all();

        } while (!ImgProdCons::quitFlag);
    }

    void ImgProdCons::Energy()
    {
        do {
            unique_lock<mutex> lock(energyLock);
            readCon.wait(lock,[]{ return !energyMission&&produceMission;});
#if DEBUG == 1
            putText(debugWindowCanvas,"Energy   Thread",Point(310,90),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
#endif
            if(curControlState == BIG_ENERGY_STATE || curControlState == SMALL_ENERGY_STATE)
            {
                /*do energy detection*/
            }

            energyMission = true;
            feedbackCon.notify_all();
        }while(!ImgProdCons::quitFlag);
    }

    void ImgProdCons::Feedback()
    {
        int count_test = 0;
        do {
            unique_lock<mutex> lock(feedbackLock);
            feedbackCon.wait(lock,[]{ return !feedbackMission&&detectMission&&energyMission;});
#if DEBUG == 1
            putText(debugWindowCanvas,"FeedBack Thread",Point(310,120),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
#endif
            if(curControlState == AUTO_SHOOT_STATE) {
                if (armorDetectorPtr->findState) {
                    solverPtr->GetPoseV(kalman->SetKF(armorDetectorPtr->targetArmor.center,clearFilter),
                                        armorDetectorPtr->targetArmor.pts,
                                        15, armorDetectorPtr->IsSmall());
                }
#if DEBUG == 1
    frequency = getTickFrequency()/((double)getTickCount() - time);

    debugWindowCanvas.colRange(0,299).setTo(0);
    putText(debugWindowCanvas,"Yaw: ",Point(10,30),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
    putText(debugWindowCanvas,to_string(solverPtr->yaw).substr(0,5),Point(100,30),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

    putText(debugWindowCanvas,"Pitch: ",Point(10,60),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
    putText(debugWindowCanvas,to_string(receiveData.pitchAngle).substr(0,5),Point(100,60),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

    putText(debugWindowCanvas,"Dist: ",Point(10,90),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
    if(carName != HERO)
        putText(debugWindowCanvas,to_string(solverPtr->dist).substr(0,5),Point(100,90),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
    else
        putText(debugWindowCanvas,to_string(dynamic_cast<RealSenseDriver*>(driver)->dist2Armor).substr(0,5),Point(100,90),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
    putText(debugWindowCanvas,"Shoot: ",Point(10,120),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
    if(solverPtr->shoot)
        circle(debugWindowCanvas,Point(100,115),8,Scalar(255),-1);

    putText(debugWindowCanvas,"Num: ",Point(10,150),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
    putText(debugWindowCanvas,to_string(armorDetectorPtr->armorNumber),Point(100,150),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

    putText(debugWindowCanvas,"Fre: ",Point(10,180),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
    putText(debugWindowCanvas,to_string(frequency),Point(100,180),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

    if(armorDetectorPtr->findState)
        rectangle(debugWindowCanvas,Rect(10,225,50,50),Scalar(255),-1);

    if(armorDetectorPtr->IsSmall())
        putText(debugWindowCanvas,"S",Point(110,255),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
    else
        putText(debugWindowCanvas,"B",Point(110,255),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

    if(curControlState)
        putText(debugWindowCanvas,"B",Point(210,255),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
    else
        putText(debugWindowCanvas,"R",Point(210,255),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

    imshow("DEBUG",debugWindowCanvas);
#endif

                if(!armorDetectorPtr->findState)
                {
                    solverPtr->yaw /= 2;
                    solverPtr->pitch /= 2;
                    kalman->SetKF(Point(0,0),true);
                }

                if(showOrigin)
                {
                    circle(detectFrame,Point(kalman->p_predictx,kalman->p_predicty),5,Scalar(255,255,255),-1);

                    if(FRAMEHEIGHT > 1000)
                    {
                        //pyrDown(detectFrame.img,detectFrame.img);
                        pyrDown(detectFrame,detectFrame);
                    }
                    imshow("detect",detectFrame);

                    waitKey(30);
                }
                if(carName != HERO)
                    serialPtr->pack(receiveData.yawAngle + feedbackDelta*solverPtr->yaw,receiveData.pitchAngle + feedbackDelta*solverPtr->pitch, solverPtr->dist, solverPtr->shoot,
                                armorDetectorPtr->findState, AUTO_SHOOT_STATE,0);
                else
                {
                    dynamic_cast<RealSenseDriver*>(driver)->GetArmorDepth(armorDetectorPtr->targetArmor.rect);
                    serialPtr->pack(receiveData.yawAngle + feedbackDelta*solverPtr->yaw,receiveData.pitchAngle + feedbackDelta*solverPtr->pitch,1000*static_cast<RealSenseDriver*>(driver)->dist2Armor, solverPtr->shoot,
                                    armorDetectorPtr->findState, AUTO_SHOOT_STATE,0);
                }

                serialPtr->WriteData();
            }
            else
            {
                /*do energy things*/
            }

            /*Receive Data*/
            serialPtr->ReadData(receiveData);

            /*update states*/
            /**if receive data failed, the most reasonable decision may be just keep the status as the last time**/
            curControlState = receiveData.targetMode;
            blueTarget = receiveData.targetColor;
            clearFilter  = direction ^ receiveData.direction;
            direction = receiveData.direction;
            /*update condition variables*/
            produceMission = false;
            feedbackMission = true;

            /*wake up threads blocked by writeCon*/
            writeCon.notify_all();
        }while(!ImgProdCons::quitFlag);
    }

    void ImgProdCons::Receive()
    {
        do
        {
            /*Receive Data*/
            sleep_ms(2);
            unique_lock<mutex> lock1(receiveLock);
            serialPtr->ReadData(receiveData);
        }while(!ImgProdCons::quitFlag);
    }
}
