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
    bool compareMission = false;
    bool detectMission = false;
    bool energyMission = false;
    bool feedbackMission = false;

    int8_t curControlState = AUTO_SHOOT_STATE; //current control mode

    uint8_t curDetectMode = SEARCH_MODE; //tracking or searching

    FILE *fp;

    double time;

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
	        //cout<<"Init FRAME FORMAT"<<endl;
            if(driver->Grab(curImage))
            {
                FRAMEWIDTH = curImage.cols;
                FRAMEHEIGHT = curImage.rows;
                armorDetectorPtr->Init();
                armorComparePtr->InitCompare();
                break;
            }
        }while(true);
        //cout<<"Process Intialized"<<endl;

        //Protect process
//        pid_t pid;
//        int i;
//        pid=fork();        //创建第一子进程
//        if(pid<0) exit(1);//创建失败退出
//        if(pid>0) exit(0);//父进程退出
//        setsid();         //第一子进程成为领头进程，脱离终端
//        pid=fork();   //第一子进程生成第二子进程
//        if(pid<0) exit(1);//创建失败退出
//        if(pid>0) exit(0);//第一子进程退出
//        chdir("/home");//切换目录
//        umask(0);               //改变文件创建掩码
//
//        int fdTableSize = getdtablesize();
//
//        for(i=0;i<fdTableSize;i++)  //关闭文件流
//            close(i);

    }

    void ImgProdCons::Produce()
    {
        //auto startTime = chrono::high_resolution_clock::now();
        static uint32_t seq = 0;
        Mat newImg;
        long long timeStamp;
        struct timeb curTime{};

        int Misscount = 0;
        time = (double)getTickCount();

        while(!ImgProdCons::quitFlag)
        {
            //time = (double)getTickCount();
            unique_lock<mutex> lock(writeLock);
            //cout<<"Get write Lock"<<endl;
            writeCon.wait(lock,[]{ return !produceMission;});

            //cout<<"Enter Produce Main Loop"<<endl;
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
            //cout<<"Produce Captured Image"<<endl;

            Misscount = 0;

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

            if(showOrigin)
            {
                imshow("compare",compareFrame.img);
            }

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
#if GPUMODE == 1
                    if (armorDetectorPtr->ArmorDetectTaskGPU(detectFrame.img))
#elif GPUMODE == 0
                    if (armorDetectorPtr->ArmorDetectTask(detectFrame.img))
#endif
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
                imshow("detect",detectFrame.img);
                waitKey(30);
            }
            //cout<<"DETECT MISSION! ============"<<endl;

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
#if RECORD == 1
    fprintf(fp,"YAW: %f,PITCH: %f,DIST: %f,SHOOT: %d,SHOOT STATE: %d,TIME: %lld\n"
            ,solverPtr->yaw,solverPtr->pitch,solverPtr->dist,solverPtr->shoot,AUTO_SHOOT_STATE,frame.timeStamp);
#endif
                if(armorDetectorPtr->findState|| armorComparePtr->findState)
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

                //cout<<"YAW: "<<solverPtr->yaw<<"PITCH: "<<solverPtr->pitch<<"DIS: "<<solverPtr->dist<<endl;
            }else
            {
                /*do energy things*/
            }
            //cout<<"FEEDBACK MISSION ================"<<endl;

            /*Receive Data*/
            unique_lock<mutex> lock1(receiveLock);
            serialPtr->ReadData(receiveData);


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
} // namespace rm
