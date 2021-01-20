
#include <unistd.h>
#include <cerrno>
#include <pthread.h>
#include <mutex>
#include <utility>
#include <thread>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

namespace RMTools
{


/**
 * @brief this class is used to display a waveform figure
 * @param src_ background image
 * @param value_ the value to display as wave
 * @param wn_ window name
 * @param stand_ the standard line to paint
 * @param lc_ the standard line color
 * @param wc_ the wave line color
 */
    class DisPlayWaveCLASS
    {
    private:
        uint8_t count = 0;
        bool m_isExit;
        std::thread m_thread;

        Mat src;
        int *value;
        uint16_t lastValue = 0;

        int *value1{};
        uint16_t lastValue1 = 0;

        string wn;
        int stand = 0;
        int spacing = 10;
        Scalar lc = Scalar(0,0,255);
        Scalar wc = Scalar(0,0,0);
        Mat left,copy;
    public:

        DisPlayWaveCLASS(Mat src_,int* value_): m_isExit(false),src(std::move(src_)),value(value_)
        {
            copy = src.clone();
        };
        DisPlayWaveCLASS(Mat src_,int* value_,int* value1_): m_isExit(false),src(std::move(src_)),value(value_),value1(value1_)
        {
            copy = src.clone();
        };
        DisPlayWaveCLASS(Mat src_,int* value_,string wn_ ,int stand_ = 0,Scalar lc_ = Scalar(0,0,255),\
                    Scalar wc_ = Scalar(0,0,0)): m_isExit(false),src(std::move(src_)),value(value_),wn(std::move(wn_)),stand(stand_),\
                    lc(std::move(lc_)),wc(std::move(wc_))
        {
             copy = src.clone();
        };
        DisPlayWaveCLASS(Mat src_,int* value_,int* value1_,string wn_ ,int stand_ = 0,Scalar lc_ = Scalar(0,0,255),\
                    Scalar wc_ = Scalar(0,0,0)): m_isExit(false),src(std::move(src_)),value(value_),wn(std::move(wn_)),stand(stand_),\
                    lc(std::move(lc_)),wc(std::move(wc_)),value1(value1_)
        {
            copy = src.clone();
        };

        void DisplayWave()
        {
            if(*value >= src.rows || stand >= src.rows)
            {
                perror("Value exceeds the src rows");
                return;
            }
            if ((src.cols/spacing) > count)
            {
                line(copy,Point2d((count -1)*spacing,lastValue),Point2d(count*spacing,*(value)),wc);
                lastValue = *(value);
                count++;
            }
            else
            {
                copy.colRange(spacing,(count - 1)*spacing + 1).copyTo(left);
                copy.setTo(0);
                left.copyTo(copy.colRange(0,(count - 2)*spacing + 1));
                line(copy,Point2d((count - 2)*spacing,lastValue),Point2d((count - 1)*spacing,*(value)),wc);
                lastValue = *(value);
            }
            line(copy,Point2d(0,stand),Point2d(copy.cols-1,stand),lc);
            flip(copy,src,0);
            imshow(wn,src);
        }
        void DisplayWave2()
        {
            if(*value >= src.rows || stand >= src.rows || *value1 >= src.rows)perror("Value exceeds the src rows");
            if ((src.cols/spacing) > count)
            {
                line(copy,Point2d((count -1)*spacing,lastValue),Point2d(count*spacing,*(value)),wc,1);
                line(copy,Point2d((count -1)*spacing,lastValue1),Point2d(count*spacing,*(value1)),Scalar(0,255,0),1);
                lastValue = *(value);
                lastValue1 = *(value1);
                count++;
            }
            else
            {
                copy.colRange(spacing,(count - 1)*spacing + 1).copyTo(left);
                copy.setTo(255);
                left.copyTo(copy.colRange(0,(count - 2)*spacing + 1));
                line(copy,Point2d((count - 2)*spacing,lastValue),Point2d((count - 1)*spacing,*(value)),wc,1);
                line(copy,Point2d((count - 2)*spacing,lastValue1),Point2d((count - 1)*spacing,*(value1)),Scalar(0,255,0),1);
                lastValue = *(value);
                lastValue1 = *(value1);
            }
            line(copy,Point2d(0,stand),Point2d(copy.cols-1,stand),lc);
            flip(copy,src,0);
            imshow(wn,src);
        }
    };


/**
 * @brief evey time execute this function will update the waveform figure, and you should add this function in your loop.
 * @return none
 */
//    void DisPlayWaveCLASS::DisplayWave()
//    {
//        if(*value >= src.rows || stand >= src.rows)perror("Value exceeds the src rows");
//        if ((src.cols/spacing) > count)
//        {
//            line(copy,Point2d((count -1)*spacing,lastValue),Point2d(count*spacing,*(value)),wc);
//            lastValue = *(value);
//            count++;
//        }
//        else
//        {
//            copy.colRange(spacing,(count - 1)*spacing + 1).copyTo(left);
//            copy.setTo(255);
//            left.copyTo(copy.colRange(0,(count - 2)*spacing + 1));
//            line(copy,Point2d((count - 2)*spacing,lastValue),Point2d((count - 1)*spacing,*(value)),wc);
//            lastValue = *(value);
//        }
//        line(copy,Point2d(0,stand),Point2d(copy.cols-1,stand),lc);
//        flip(copy,src,0);
//        imshow(wn,src);
//    }
//
//
//    void DisPlayWaveCLASS::DisplayWave2()
//    {
//        if(*value >= src.rows || stand >= src.rows || *value1 >= src.rows)perror("Value exceeds the src rows");
//        if ((src.cols/spacing) > count)
//        {
//            line(copy,Point2d((count -1)*spacing,lastValue),Point2d(count*spacing,*(value)),wc,1);
//            line(copy,Point2d((count -1)*spacing,lastValue1),Point2d(count*spacing,*(value1)),Scalar(0,255,0),1);
//            lastValue = *(value);
//            lastValue1 = *(value1);
//            count++;
//        }
//        else
//        {
//            copy.colRange(spacing,(count - 1)*spacing + 1).copyTo(left);
//            copy.setTo(255);
//            left.copyTo(copy.colRange(0,(count - 2)*spacing + 1));
//            line(copy,Point2d((count - 2)*spacing,lastValue),Point2d((count - 1)*spacing,*(value)),wc,1);
//            line(copy,Point2d((count - 2)*spacing,lastValue1),Point2d((count - 1)*spacing,*(value1)),Scalar(0,255,0),1);
//            lastValue = *(value);
//            lastValue1 = *(value1);
//        }
//        line(copy,Point2d(0,stand),Point2d(copy.cols-1,stand),lc);
//        flip(copy,src,0);
//        imshow(wn,src);
//    }
}