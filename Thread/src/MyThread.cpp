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

extern pthread_t producePThreadHandler;
extern pthread_t detectPThreadHandler;
extern pthread_t energyPThreadHandler;
extern pthread_t feedbackPThreadHandler;

#if SAVE_VIDEO == 1
extern int video_save_count;
    VideoWriter videowriter;
#endif

#if SAVE_LOG == 1
std::ofstream logWrite("../Log/log.txt",ios::out);

#endif

namespace rm
{
    bool ImgProdCons::quitFlag = false;// quit flag, all threads would jump out from loop when quitFlag is true

    bool produceMission = false;//when produce mission completed, produceMission is true, when feedback mission completed, produceMission is false
    bool detectMission = false;//when detect mission completed, detectMission is true, when produce mission completed, detectMission is false
    bool energyMission = false;//when energy mission completed, energyMission is true, when produce mission completed, energyMission is false
    bool feedbackMission = false;//when feedback mission completed, feedbackMission is true, when produce mission completed, feedbackMission is false

    int8_t curControlState = AUTO_SHOOT_STATE; //current control mode
    uint8_t curDetectMode = SEARCH_MODE; //tracking or searching

    RealSenseDriver intelCapture;// Intel D435 Camera Driver, when it is declared in class ImgProdCons, there are a segment fault when program runs
    float coordinateBias;// the bias between armor center point and image center

    bool pauseFlag = false;

#if DEBUG == 1
    int predictX,originalX;
    Mat WaveBackground = Mat(480,640,CV_8UC3,Scalar(0,0,0));
    auto *waveWindowPanel =new DisPlayWaveCLASS(WaveBackground, &originalX,&predictX);



    Mat debugWindowCanvas = Mat(300,500,CV_8UC1,Scalar(0));
#endif

#if DEBUG == 1 || SAVE_LOG == 1
    double timeFlag, taskTime;
    double frequency;
#endif

    float yawTran = 0;
    float pitchTran = 0;

#define YAW_LIST_LEN 15
    double yawList[YAW_LIST_LEN] = {0};
    double yawDeviation = 0;
    int yawListCount = 0;

    void ImgProdCons::SignalHandler(int)
    {
        LOGE("Process Shut Down By SIGINT\n");
        ImgProdCons::quitFlag = true;

#if SAVE_VIDEO == 1
        videowriter.release();
#endif

#if SAVE_LOG == 1
        logWrite.close();
#endif
        if(pthread_kill(producePThreadHandler,0) == ESRCH)
        {
            LOGW("Child Thread Produce Thread Close Failed\n");
        }

        if(pthread_kill(detectPThreadHandler,0) == ESRCH)
        {
            LOGW("Child Thread Detect Thread Close Failed\n");
        }

        if(pthread_kill(energyPThreadHandler,0) == ESRCH)
        {
            LOGW("Child Thread EnergyDetector Thread Close Failed\n");
        }

        if(pthread_kill(feedbackPThreadHandler,0) == ESRCH)
        {
            LOGW("Child Thread Feedback Thread Close Failed\n");
        }

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
            energyPtr(unique_ptr<EnergyDetector>(new EnergyDetector())),
            armorType(BIG_ARMOR),
            driver(),
            missCount(0)
    {
//#if SAVE_LOG == 1
//        logWrite<<"Find    "<<"TARGET X    "<<"TARGET Y    "<<"TARGET HEIGHT    "<<"TARGET WIDTH    "<<"YAW    "<<"PITCH    "\
//    <<"SHOOT    "<<endl;
//#endif
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
        if((driver->InitCam() && driver->SetCam() && driver->StartGrab()))
        {
            LOGM("Camera Initialized\n");
            LOGM("Camera Set Down\n");
            LOGM("Camera Start to Grab Frames\n");
        }
        else
        {
            driver->StopGrab();
            LOGW("Camera Resource Released\n");
            exit(-1);
        }

        do
        {
            if(driver->Grab(curImage))
            {
                FRAMEWIDTH = curImage.cols;
                FRAMEHEIGHT = curImage.rows;
                armorDetectorPtr->Init();
                break;
            }
            missCount++;
            if(missCount > 5)
            {
                driver->StopGrab();
                exit(-1);
            }
        }while(true);
        missCount = 0;

#if SAVE_VIDEO == 1
        string video_save_path = "../Log/video_sichuan_SWMU_" + std::to_string(video_save_count) + ".avi";
        videowriter = VideoWriter(video_save_path,cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),60,Size(FRAMEWIDTH,FRAMEHEIGHT));
#endif
        LOGM("Initialization Completed\n");
    }

    void ImgProdCons::Produce()
    {
        static uint32_t seq = 0;

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
#endif

#if SAVE_LOG == 1 || DEBUG == 1
            timeFlag = (double)getTickCount();
#endif

            if (!driver->Grab(frame) || frame.rows != FRAMEHEIGHT || frame.cols != FRAMEWIDTH)
            {
                missCount++;
                if(missCount > 5)
                {
                    driver->StopGrab();
                    raise(SIGINT);
                }
                continue;
            }

            missCount = 0;

            detectFrame = frame.clone();
            energyFrame = frame.clone();

#if DEBUG_MSG == 1
            LOGM("Produce Thread Completed\n");
#endif

#if SAVE_VIDEO == 1
            videowriter.write(detectFrame);
#endif

#if SAVE_LOG == 1
            logWrite<<"Produce Time Consume : "<<((double)getTickCount() - timeFlag)/getTickFrequency()<<endl;
#endif

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

#if SAVE_LOG == 1
            taskTime = (double)getTickCount();
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
                        //armorDetectorPtr->tracker = TrackerKCF::create();
                        //armorDetectorPtr->tracker->init(detectFrame, armorDetectorPtr->targetArmor.rect);
                        if(armorDetectorPtr->armorNumber > 0 && armorDetectorPtr->armorNumber < 6)
                            curDetectMode = SEARCH_MODE;
                    }
                    else
                    {
                        curDetectMode = SEARCH_MODE;
                    }
                }
                break;
                case TRACKING_MODE:
                {
#if DEBUG == 1
                    circle(debugWindowCanvas,Point(480,255),8,Scalar(255),-1);
#endif
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
                default:
                    /**do some thing about serial to correct the transmission task**/
                    break;
            }

            detectMission = true;

#if DEBUG_MSG == 1
            LOGM("Detect Thread Completed\n");
#endif

#if SAVE_LOG == 1
            logWrite<<"Detect Time Consume : "<<((double)getTickCount() - taskTime)/getTickFrequency()<<endl;
#endif

            feedbackCon.notify_all();

        } while (!ImgProdCons::quitFlag);
    }

    void ImgProdCons::Energy()
    {
        do {
            unique_lock<mutex> lock(energyLock);
            readCon.wait(lock,[]{ return !energyMission&&produceMission;});
#if DEBUG == 1
            putText(debugWindowCanvas,"EnergyDetector   Thread",Point(310,90),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
#endif
            if(curControlState == BIG_ENERGY_STATE || curControlState == SMALL_ENERGY_STATE)
            {
                /*do energy detection*/
                energyPtr->EnergyTask(energyFrame, curControlState == BIG_ENERGY_STATE);
            }

            energyMission = true;
            feedbackCon.notify_all();
        }while(!ImgProdCons::quitFlag);
    }

    void ImgProdCons::Feedback()
    {
        do {
            unique_lock<mutex> lock(feedbackLock);
            feedbackCon.wait(lock,[]{ return !feedbackMission&&detectMission&&energyMission;});

#if DEBUG == 1
            putText(debugWindowCanvas,"FeedBack Thread",Point(310,120),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
#endif
#if SAVE_LOG == 1
            taskTime = (double)getTickCount();
#endif
            if(curControlState == AUTO_SHOOT_STATE) {
                if (armorDetectorPtr->findState)
                {

#if DEBUG_MSG == 1
                    LOGW("Target Armor Founded!\n");
#endif
                    /**call solvePnp algorithm function to get the yaw, pitch and distance data**/
                    solverPtr->GetPoseV(Point2f(0, 0),
                                        armorDetectorPtr->targetArmor.pts,
                                        15, armorDetectorPtr->IsSmall());

                }
                else
                {
                    coordinateBias = 0;
                }
#if DEBUG == 1
                frequency = getTickFrequency()/((double)getTickCount() - timeFlag);

                debugWindowCanvas.colRange(0,299).setTo(0);
                putText(debugWindowCanvas,"Yaw: ",Point(10,30),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                putText(debugWindowCanvas,to_string(yawTran).substr(0,5),Point(100,30),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

                putText(debugWindowCanvas,"Pitch: ",Point(10,60),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                putText(debugWindowCanvas,to_string(pitchTran).substr(0,5),Point(100,60),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

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

                predictX = kalman->p_predictx/5;
                originalX = armorDetectorPtr->targetArmor.center.x/5;

//                printf("Original X:%d\t",originalX);
//                printf("prediect X:%d\n",predictX);
//                pyrDown(debugWindowCanvas,debugWindowCanvas);

                imshow("DEBUG",debugWindowCanvas);

//                waveWindowPanel->DisplayWave2();
#endif

//#if SAVE_LOG == 1
//                logWrite<<armorDetectorPtr->findState<<"   "<<armorDetectorPtr->targetArmor.rect.x<<"    "<<armorDetectorPtr->targetArmor.rect.y\
//    <<"    "<<armorDetectorPtr->targetArmor.rect.height<<"    "<<armorDetectorPtr->targetArmor.rect.width<<"    "\
//    <<solverPtr->yaw<<"    "<<solverPtr->pitch<<"    "<<solverPtr->shoot<<endl;
//#endif

                /*******************************************************************************************************
                 * when the armor-detector has not detected target armor successfully, that may be caused by the suddenly
                 * movement of robots(myself and the opposite target  robot), but the target armor is still in the view
                 * scoop, we still need to instruct the movement of the holder instead of releasing it to the cruise mode
                 * ****************************************************************************************************/

                if(showOrigin)
                {
                    circle(detectFrame,Point(FRAMEWIDTH/2, FRAMEHEIGHT/2),5,Scalar(255,255,255),-1);

                    if(FRAMEHEIGHT > 1000)
                    {
                        pyrDown(detectFrame,detectFrame);
                        pyrDown(detectFrame,detectFrame);
                    }
                    imshow("detect",detectFrame);
                }

                /****************************************Control Auxiliary**********************************************
                 1. Adjust the target Angle of the holder according to the distance D from the center of the mounting deck
                 to the center of the image. When D is large, set the target Angle of the holder to be larger; when D is
                 small, set the target Angle of the holder to be smaller, so as to indirectly control the rotation speed
                 of the holder.

                2. Through the retreat algorithm, when the assembly deck is not found, the current offset Angle is set to
                 1/M of the previous frame offset Angle, which can prevent the jitter of the holder caused by intermittent
                 unrecognition to a certain extent.

                 3. Judge whether the holder dither on the YAW axis by the dispersion of the deflection Angle of the holder's
                 YAW axis. If so, reduce the target Angle of the holder and indirectly slow down the rotation speed of the
                 holder.
                 *******************************************************************************************************/

                {
                    /** use feedbackDelta to adjust the speed for holder to follow the armor**/
                    //coordinateBias = fabs((armorDetectorPtr->targetArmor.center.x/(FRAMEWIDTH/2)) - 1);
                    yawDeviation = stdDeviation(yawList, YAW_LIST_LEN);
                    float cur = fabs((armorDetectorPtr->targetArmor.center.x - FRAMEWIDTH/2)/64) + 1;
                    feedbackDelta = 1 + log(cur);
                    //cout<<"cur : "<<cur<<"    "<<"delta : "<<feedbackDelta<<endl;
                }s

                if(!armorDetectorPtr->findState)
                {
                    yawTran /= 1.2;
                    pitchTran /= 1.2;

                    /** reset kalman filter **/
                    kalman->SetKF(Point(0,0),true);

                    if(fabs(yawTran) > 0.1 && fabs(pitchTran) > 0.1)
                        armorDetectorPtr->findState = true;
                }
                else
                {
                    yawTran = solverPtr->yaw - 22;
                    pitchTran = solverPtr->pitch + 17.5;
                }

                /** update yaw list and yawListCount **/
                yawList[yawListCount++] = yawTran;
                yawListCount = yawListCount%YAW_LIST_LEN;

                kalman->SetKF(Point2f(yawTran, pitchTran) , false);

                /** package data and prepare for sending data to lower-machine **/
                if(carName != HERO)
                    serialPtr->pack(receiveData.yawAngle + kalman->p_predictx,receiveData.pitchAngle + pitchTran, solverPtr->dist, solverPtr->shoot,
                                    armorDetectorPtr->findState, AUTO_SHOOT_STATE,0);
                else
                {
                    dynamic_cast<RealSenseDriver*>(driver)->measure(armorDetectorPtr->targetArmor.rect);
                    serialPtr->pack(receiveData.yawAngle + feedbackDelta*yawTran,receiveData.pitchAngle + pitchTran,1000*static_cast<RealSenseDriver*>(driver)->dist2Armor, solverPtr->shoot,
                                    armorDetectorPtr->findState, AUTO_SHOOT_STATE,0);
                }

#if DEBUG_MSG == 1
                LOGM("Write Data\n");
#endif
            }
            else
            {
                /*do energy things*/
                solverPtr->GetPoseV(Point2f(0, 0),
                                    energyPtr->pts,
                                    15, false);

                serialPtr->pack(receiveData.yawAngle + solverPtr->yaw,receiveData.pitchAngle + solverPtr->pitch, solverPtr->dist, solverPtr->shoot,
                                true, AUTO_SHOOT_STATE,0);
            }

            /**press key 'p' to pause or continue task**/
            if(DEBUG || showOrigin || showEnergy)
            {
                if(!pauseFlag && waitKey(30) == 'p'){pauseFlag = true;}

                if(pauseFlag)
                {
                    while(waitKey() != 'p'){}
                    pauseFlag = false;
                }
            }

            /** send data from host to low-end machine to instruct holder's movement **/
            serialPtr->WriteData();

            /**Receive data from low-end machine to update parameters(the color of robot, the task mode, etc)**/
            if(serialPtr->ReadData(receiveData))
            {
#if DEBUG_MSG == 1
                LOGM("Receive Data\n");
#endif
                /**Update task mode, if receiving data failed, the most reasonable decision may be just keep the status
                 * as the last time**/
                curControlState = receiveData.targetMode;

                /**because the logic in armor detection task need the color of enemy, so we need to negate to color variable
                 * received, receiveData.targetColor means the color of OUR robot, but not the enemy's**/
                blueTarget = (receiveData.targetColor) == 0;

#if DEBUG_MSG == 1
                LOGM("BlueTarget: %d\n",(int)blueTarget);
#endif
            }

            /**update condition variables**/
            produceMission = false;
            feedbackMission = true;

#if DEBUG_MSG == 1
            LOGM("Feedback Thread Completed\n");
#endif

#if SAVE_LOG == 1
            logWrite<<"Feedback Time Consume : "<<((double)getTickCount() - taskTime)/getTickFrequency()<<endl;
            logWrite<<"Total Time Consume : "<<((double)getTickCount() - timeFlag)/getTickFrequency()<<endl;
#endif

            /**wake up threads blocked by writeCon**/
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