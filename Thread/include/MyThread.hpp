//
// Created by root on 2021/1/14.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <mutex>
#include <memory>
#include <shared_mutex>
#include <condition_variable>
#include <sys/timeb.h>

#include "ArmorDetector.hpp"
#include "SerialPort.hpp"
#include "SolveAngle.hpp"
#include "preoptions.h"
#include "mydefine.h"
#include "Filter.h"

#include "RealSenseDriver.h"
#include "Media/RMDriver.h"
#include "V4L2KAS.h"
#include "VideoDriver.hpp"

using namespace std;
using namespace V4L2KAS;

namespace rm
{

    /*min unit to describe figure*/
    struct Frame
    {
        cv::Mat img;
        uint32_t seq;         //count from 0

        Frame()=default;

        Frame(const Mat& img_, uint32_t seq_)
        {
            img = img_.clone();
            seq = seq_;
        }

        Frame clone() const
        {
            return Frame(img,seq);
        }

    };

    class ImgProdCons
    {
    public:
        ImgProdCons();

        ~ImgProdCons() {};

        /*initialize*/
        void Init();

        /*
        * @Brief: Receive self state from the serail port, update task mode if commanded
        */
        void Feedback();

        /*
        * @Brief: keep reading image from the camera into the buffer
        */
        void Produce();


        /*
         * @Brief: detect
         */

        void Detect();

        /*
         * @Brief: operations need two image
         */
        void Compare();

        void Energy();

        /*
         * @Brief: derivation
         */
        void Receive();

    private:
        /*
        * To prevent camera from dying!
        */
        static bool quitFlag;
        static void SignalHandler(int);
        static void InitSignals(void);

        /*Camera Driver Instances*/
        RMDriver dahuaCapture;
        V4L2Driver v4l2Capture;
        RealSenseDriver intelCapture;
        VideoDriver videoCapture;

        /* Camera */
        Driver *driver;

        /* Serial */
        std::unique_ptr<Serial> serialPtr;

        /* Angle solver */
        std::unique_ptr<SolveAngle> solverPtr;

        /* Armor detector */
        std::unique_ptr<ArmorDetector> armorDetectorPtr;

        std::unique_ptr<Kalman> kalman;

        Frame frame;
        Frame detectFrame;

        mutex detectLock;
        mutex writeLock;
        mutex energyLock;
        mutex feedbackLock;
        mutex receiveLock;


        condition_variable writeCon;
        condition_variable readCon;
        condition_variable feedbackCon;
        condition_variable energyCon;

        struct ReceiveData receiveData;
        int armorType;
    };

}
