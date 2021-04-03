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

    FILE *fp;

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
        printf("Camera Initialized\n");
        driver->SetCam();
        printf("Camera Set\n");
        driver->StartGrab();
        printf("Camera Start to Grab\n");

#if RECORD == 1
        fp = fopen("log.txt","w");
#endif
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

        printf("Initialization Completed\n");
    }

    void ImgProdCons::Produce()
    {
        static uint32_t seq = 0;
        Mat newImg;

        int Misscount = 0;

        while(!ImgProdCons::quitFlag)
        {
            unique_lock<mutex> lock(writeLock);

            writeCon.wait(lock,[]{ return !produceMission;});

            if (!driver->Grab(newImg) || newImg.empty())
            {
                Misscount++;
                if(Misscount > 50)
                {
                    quitFlag = true;
                    driver->StopGrab();
                    readCon.notify_all();
                    exit(-1);
                }
                continue;
            }

            Misscount = 0;

            frame = Frame{newImg, seq};

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

            switch (curDetectMode)
            {
                case SEARCH_MODE:
                {
                    if (armorDetectorPtr->ArmorDetectTask(detectFrame.img))
                    {
                        armorDetectorPtr->tracker = TrackerKCF::create();
                        armorDetectorPtr->tracker->init(detectFrame.img, armorDetectorPtr->targetArmor.rect);
                        curDetectMode = TRACKING_MODE;
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
                    //这里一开始就是trakcking，若追踪到目标，继续，未追踪到，进入searching,进入tracking函数时要判断分类器是否可用
                    if (armorDetectorPtr->trackingTarget(detectFrame.img, armorDetectorPtr->targetArmor.rect))
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

            if(showOrigin)
            {
                if(FRAMEHEIGHT > 1000)
                {
                    pyrDown(detectFrame.img,detectFrame.img);
                }

                imshow("detect",detectFrame.img);

                waitKey(30);
            }

            detectMission = true;

            produceMission = false;
            writeCon.notify_all();

            feedbackCon.notify_all();

        } while (!ImgProdCons::quitFlag);
    }

    void ImgProdCons::Energy()
    {
        do {
            unique_lock<mutex> lock(energyLock);
            readCon.wait(lock,[]{ return !energyMission&&produceMission;});

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
            if(curControlState == AUTO_SHOOT_STATE) {
                if (armorDetectorPtr->findState) {
                    cout<<count_test++<<endl;
                    solverPtr->GetPoseV(kalman->SetKF(armorDetectorPtr->targetArmor.center),
                                        armorDetectorPtr->targetArmor.pts,
                                        15, armorDetectorPtr->IsSmall());
                }
#if RECORD == 1
    fprintf(fp,"YAW: %f,PITCH: %f,DIST: %f,SHOOT: %d,SHOOT STATE: %d\n"
            ,solverPtr->yaw,solverPtr->pitch,solverPtr->dist,solverPtr->shoot,AUTO_SHOOT_STATE);
#endif
                if(armorDetectorPtr->findState)
                serialPtr->pack(receiveData.yawAngle + feedbackDelta*solverPtr->yaw,receiveData.pitchAngle + feedbackDelta*solverPtr->pitch, solverPtr->dist, solverPtr->shoot,
                                1, AUTO_SHOOT_STATE,0);
                else
                {
                    solverPtr->yaw /= 2;
                    solverPtr->pitch /= 2;
                    /*The sending Angle drops twice each time until the offset angle are both small than 1*/
                    if(solverPtr->yaw < 1 && solverPtr->pitch < 1)
                        serialPtr->pack(receiveData.yawAngle + feedbackDelta*solverPtr->yaw, receiveData.pitchAngle + feedbackDelta*solverPtr->pitch, solverPtr->dist, 0,
                                        0, AUTO_SHOOT_STATE,0);
                    else
                    serialPtr->pack(receiveData.yawAngle + feedbackDelta*solverPtr->yaw, receiveData.pitchAngle + feedbackDelta*solverPtr->pitch, solverPtr->dist, 0,
                                    1, AUTO_SHOOT_STATE,0);
                }
                serialPtr->WriteData();
            }else
            {
                /*do energy things*/
            }

            /*Receive Data*/
            unique_lock<mutex> lock1(receiveLock);
            //serialPtr->ReadData(receiveData);

            curControlState = receiveData.targetMode;

            produceMission = false;
            feedbackMission = true;

            writeCon.notify_all();
        }while(!ImgProdCons::quitFlag);
    }

    void ImgProdCons::Receive()
    {
        do
        {
            /*Receive Data*/
            sleep_ms(30);
            unique_lock<mutex> lock1(receiveLock);
            serialPtr->ReadData(receiveData);
        }while(!ImgProdCons::quitFlag);
    }
}
