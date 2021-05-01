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

    double time;
    int frequency;
    Mat debugWindowCanvas = Mat(300,500,CV_8UC1,Scalar(0));
#endif

    float yawTran = 0;
    float pitchTran = 0;


/**
 * 求平均值
 */
    double average1(double *x, int len)
    {
        double sum = 0;
        for (int i = 0; i < len; i++) // 求和
            sum += x[i];
        return sum/len; // 得到平均值
    }

/**
 * 求方差
 */
    double variance(double *x, int len)
    {
        double sum = 0;
        double average = average1(x, len);
        for (int i = 0; i < len; i++) // 求和
            sum += pow(x[i] - average, 2);
        return sum/len; // 得到平均值
    }
/**
 * 求标准差
 */
    double stdDeviation(double *x, int len)
    {
        double variance1 = variance(x, len);
        return sqrt(variance1); // 得到标准差
    }

#define YAW_LIST_LEN 15
    double yawList[YAW_LIST_LEN] = {0};
    double stdDeviation1 = false;
    int yawListCount = 0;

    static void sleep_ms(unsigned int secs)
    {
        struct timeval tval;

        tval.tv_sec=secs/1000;

        tval.tv_usec=(secs*1000)%1000000;

        select(0,NULL,NULL,NULL,&tval);
    }

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
            LOGW("Child Thread Energy Thread Close Failed\n");
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
            armorType(BIG_ARMOR),
            driver(),
            missCount(0)
    {
#if SAVE_LOG == 1
        logWrite<<"Find    "<<"TARGET X    "<<"TARGET Y    "<<"TARGET HEIGHT    "<<"TARGET WIDTH    "<<"YAW    "<<"PITCH    "\
    <<"SHOOT    "<<endl;
#endif
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
            time = (double)getTickCount();
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

#if DEBUG_MSG == 1
            LOGM("Produce Thread Completed\n");
#endif

#if SAVE_VIDEO == 1
            videowriter.write(detectFrame);
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
                        if(armorDetectorPtr->armorNumber > 0 && armorDetectorPtr->armorNumber < 6)
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
                default:
                    /**do some thing about serial to correct the transmission task**/
                    break;
            }

            detectMission = true;

#if DEBUG_MSG == 1
            LOGM("Detect Thread Completed\n");
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

#if DEBUG_MSG == 1
                    LOGW("Target Armor Founded!\n");
#endif
                    /**call solvePnp algorithm function to get the yaw, pitch and distance data**/
                    solverPtr->GetPoseV(Point2f(0, 0),
                                        armorDetectorPtr->targetArmor.pts,
                                        15, armorDetectorPtr->IsSmall());

                }else
                {
                    coordinateBias = 0;
                }
#if DEBUG == 1
                frequency = getTickFrequency()/((double)getTickCount() - time);

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

//    predictX = kalman->p_predictx/5;
//    originalX = armorDetectorPtr->targetArmor.center.x/5;
//
//    printf("Original X:%d\t",originalX);
//    printf("prediect X:%d\n",predictX);
                //pyrDown(debugWindowCanvas,debugWindowCanvas);
                imshow("DEBUG",debugWindowCanvas);

                //waveWindowPanel->DisplayWave2();
#endif

#if SAVE_LOG == 1
                logWrite<<armorDetectorPtr->findState<<"   "<<armorDetectorPtr->targetArmor.rect.x<<"    "<<armorDetectorPtr->targetArmor.rect.y\
    <<"    "<<armorDetectorPtr->targetArmor.rect.height<<"    "<<armorDetectorPtr->targetArmor.rect.width<<"    "\
    <<solverPtr->yaw<<"    "<<solverPtr->pitch<<"    "<<solverPtr->shoot<<endl;
#endif

                /**when the armor-detector has not detected target armor successfully, that may be caused by the suddenly movement
                 * of robots(myself and the opposite target  robot), but the target armor is still in the view scoop, we still need
                 * to instruct the movement of the holder instead of releasing it to the cruise mode**/

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

                /**press key 'p' to pause or continue task**/
                if(DEBUG || showOrigin)
                {
                    if(!pauseFlag && waitKey(30) == 'p'){pauseFlag = true;}

                    if(pauseFlag)
                    {
                        while(waitKey() != 'p'){}
                        pauseFlag = false;
                    }
                }

                /**Control Auxiliary**/
                {
                    /** use feedbackDelta to adjust the speed for holder to follow the armor**/
                    coordinateBias = abs((armorDetectorPtr->targetArmor.center.x/(FRAMEWIDTH/2)) - 1);
                    stdDeviation1 = stdDeviation(yawList, YAW_LIST_LEN);
                    if(stdDeviation1 < 1)
                        feedbackDelta = 2.0 + 1.5*coordinateBias;
                    else
                        feedbackDelta = 0.5 + 0.5*coordinateBias;
                }

                if(!armorDetectorPtr->findState)
                {
                    yawTran /= 1.2;
                    pitchTran /= 1.2;
                    kalman->SetKF(Point(0,0),true);
                    if(fabs(yawTran) > 0.1 && fabs(pitchTran) > 0.1)
                        armorDetectorPtr->findState = true;
                }
                else
                {
                    yawTran = solverPtr->yaw - 22;
                    pitchTran = solverPtr->pitch + 17.5;
                }

                yawList[yawListCount++] = yawTran;
                yawListCount = yawListCount%YAW_LIST_LEN;
                //cout<<"stdDeviation: "<<stdDeviation1<<endl;

                if(carName != HERO)
                    serialPtr->pack(receiveData.yawAngle + feedbackDelta*yawTran,receiveData.pitchAngle + pitchTran, solverPtr->dist, solverPtr->shoot,
                                    armorDetectorPtr->findState, AUTO_SHOOT_STATE,0);
                else
                {
                    dynamic_cast<RealSenseDriver*>(driver)->measure(armorDetectorPtr->targetArmor.rect);
                    serialPtr->pack(receiveData.yawAngle + feedbackDelta*yawTran,receiveData.pitchAngle + feedbackDelta*pitchTran,1000*static_cast<RealSenseDriver*>(driver)->dist2Armor, solverPtr->shoot,
                                    armorDetectorPtr->findState, AUTO_SHOOT_STATE,0);
                }

                /**send data from host to low-end machine to instruct holder's movement**/
                serialPtr->WriteData();

#if DEBUG_MSG == 1
                LOGM("Write Data\n");
#endif

            }
            else
            {
                /*do energy things*/
            }

            /**Receive data from low-end machine to update parameters(the color of
             * robot, the task mode, etc)**/
            if(serialPtr->ReadData(receiveData))
            {
#if DEBUG_MSG == 1
                LOGM("Receive Data\n");
#endif
                /**Update task mode, if receiving data failed, the most reasonable decision may be just keep the status as the last time**/
                curControlState = receiveData.targetMode;

                /**because the logic in armor detection task need the color of enemy, so we need to negate to color variable received,
                 * receiveData.targetColor means the color of OUR robot, but not the enemy's**/
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