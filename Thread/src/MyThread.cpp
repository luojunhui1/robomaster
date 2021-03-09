//
// Created by luojunhui on 1/28/20.
//

#include <csignal>
#include <opencv2/opencv.hpp>
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
    bool compareMission = false;
    bool detectMission = false;
    bool energyMission = false;
    bool feedbackMission = false;

    int8_t curControlState = AUTO_SHOOT_STATE; //current control mode

    uint8_t curDetectMode = SEARCH_MODE; //tracking or searching

    auto startTime_ = chrono::high_resolution_clock::now();

    void ImgProdCons::SignalHandler(int)
    {
        perror("sigaction error:");
        ImgProdCons::quitFlag = true;
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
            armorComparePtr(unique_ptr<ArmorCompare>(new ArmorCompare())),
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
            case INFANTRY:
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
        driver->SetCam();
        driver->StartGrab();
        do
        {
            if(driver->Grab(curImage))
            {
                FRAMEWIDTH = curImage.cols;
                FRAMEHEIGHT = curImage.rows;
                armorDetectorPtr->Init();
                armorComparePtr->InitCompare();
                break;
            }
        }while(true);


    }

    void ImgProdCons::Produce()
    {
        //auto startTime = chrono::high_resolution_clock::now();
        static uint32_t seq = 0;
        Mat newImg;
        long long timeStamp;
        struct timeb curTime{};

        while(!ImgProdCons::quitFlag)
        {
            startTime_ = chrono::high_resolution_clock::now();
            unique_lock<mutex> lock(writeLock);
            //cout<<"Get write Lock"<<endl;
            writeCon.wait(lock,[]{ return !produceMission;});

            //cout<<"Enter Produce Main Loop"<<endl;
            if (!driver->Grab(newImg))
                continue;
            //cout<<"Produce Captured Image"<<endl;

            ftime(&curTime);
            timeStamp = (curTime.time*1000 + curTime.millitm - 1610812848148);

            frame = Frame{newImg, seq, timeStamp};

            detectFrame = frame.clone();
            compareFrame = frame.clone();

            //cout<<"PRODUCE MISSION!"<<endl;
            detectMission = compareMission = energyMission = feedbackMission = false;
            produceMission = true;
            readCon.notify_all();
        }
    }

    void ImgProdCons::Compare()
    {
        do {
            unique_lock<mutex> lock(compareLock);
            readCon.wait(lock,[]{ return !compareMission&&produceMission;});

            armorComparePtr->DetectArmor(compareFrame.img);

            //cout<<"COMPARE MISSION! ===="<<endl;

            compareMission = true;
            feedbackCon.notify_all();
        }while(!ImgProdCons::quitFlag);
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
                imshow("src",detectFrame.img);
                waitKey(30);
            }
            //cout<<"DETECT MISSION! ============"<<endl;
            detectMission = true;
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
            //cout<<"ENERGY MISSION ======"<<endl;
            energyMission = true;
            feedbackCon.notify_all();
        }while(!ImgProdCons::quitFlag);
    }

    void ImgProdCons::Feedback()
    {
        do {
            unique_lock<mutex> lock(feedbackLock);
            feedbackCon.wait(lock,[]{ return !feedbackMission&&detectMission&&compareMission&&energyMission;});
            if(curControlState == AUTO_SHOOT_STATE) {
                if (armorDetectorPtr->findState) {
                    solverPtr->GetPoseV(kalman->SetKF(armorDetectorPtr->targetArmor.center),
                                        armorDetectorPtr->targetArmor.pts,
                                        15, armorDetectorPtr->IsSmall());
                } else if (armorComparePtr->findState) {
                    solverPtr->GetPoseV(kalman->SetKF(armorComparePtr->targetArmor.center),
                                        armorComparePtr->targetArmor.pts,
                                        15, armorComparePtr->IsSmall());
                }

                serialPtr->pack(solverPtr->yaw, solverPtr->pitch, solverPtr->dist, solverPtr->shoot,
                                1, AUTO_SHOOT_STATE,frame.timeStamp);
                serialPtr->WriteData();

                //cout<<"YAW: "<<solverPtr->yaw<<"PITCH: "<<solverPtr->pitch<<"DIS: "<<solverPtr->dist<<endl;
            }else
            {
                /*do energy things*/
            }
            //cout<<"FEEDBACK MISSION ================"<<endl;
            produceMission = false;
            feedbackMission = true;
            auto time = (static_cast<chrono::duration<double, std::milli>>(chrono::high_resolution_clock::now() -startTime_)).count();
            cout<<"FREQUENCY: "<< 1000.0/time<<endl;
            writeCon.notify_all();
        }while(!ImgProdCons::quitFlag);
    }
} // namespace rm
