#ifndef _RMTOOLS__
#define _RMTOOLS__

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

#define __MYSCALAR__(i) ((i%3==0)?Scalar(i*16.7,i*8,0):((i%3==1)?Scalar(0,i*16.7,255 - i*5):Scalar(255 - i*2,0,i*16.7)))

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
        int stand = 64;
        int spacing = 10;
        Scalar lc = Scalar(0,0,255);
        Scalar wc = Scalar(0,146,125);
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
                copy.setTo(0);
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
 * 求平均值
 */
    inline double average(const double *x, int len)
    {
        double sum = 0;
        for (int i = 0; i < len; i++) // 求和
            sum += x[i];
        return sum/len; // 得到平均值
    }

/**
 * 求方差
 */
    inline double variance(double *x, int len)
    {
        double sum = 0;
        double average1 = average(x, len);
        for (int i = 0; i < len; i++) // 求和
            sum += pow(x[i] - average1, 2);
        return sum/len; // 得到平均值
    }
/**
 * 求标准差
 */
    inline double stdDeviation(double *x, int len)
    {
        double variance1 = variance(x, len);
        return sqrt(variance1); // 得到标准差
    }

    /**
 *  the descriptor of the point in the route, including the color, location, velocity and the situation of point.
 */
    class RoutePoint
    {
    public:
        RoutePoint(const Point& p_,Scalar color_,int vx_,int vy_):p(p_),color(std::move(color_)),vx(vx_),vy(vy_){};
        Point p;
        Scalar color;
        int vx;
        int vy;
        bool used = false;
    };

/**
 * @brief this class is used to track the feature points that user provided and display the route of these feature points
 *        with unique color.
 * @param src the image that the points drawing on
 * @param pSize the size of the points, which reflect the route
 */
    class FeatureRoute
    {
    private:
        int pSize;
        int count;
        int expirectl;
        Mat origin;
        Mat srcs[5];
        vector<RoutePoint> lastPts;

        const int wd = 20;
    public:
        explicit FeatureRoute(const Mat& src,int pSize);
        void DisplayRoute(const vector<Point>& pts);
        void FeatureRouteDrawPoint(const Point& p,const Scalar& color,int expirectl);
    };

    inline FeatureRoute::FeatureRoute(const Mat& src,int pSize = 10)
    {
        this->pSize = pSize;
        count = 1;
        this->origin = src;
        expirectl = 0;
        for(auto & i : srcs)
        {
            src.copyTo(i);
        }
    }

    inline void FeatureRoute::FeatureRouteDrawPoint(const Point& p_, const Scalar& color_, int expirectl_)
    {
        for(int i = 0;i < 4;i++)
        {
            circle(srcs[expirectl_],p_,pSize,color_,-1);
            expirectl_ = (expirectl_ + 1)%5;
        }
        srcs[expirectl_].setTo(255);
    }

    inline void FeatureRoute::DisplayRoute(const vector<Point>& pts)
    {
        int i = 0;
        vector<RoutePoint> cur;
        if(lastPts.empty())
        {
            for(const auto& p:pts)
            {
                if(p.x<0||p.x>origin.cols||p.y<0||p.y>origin.rows)
                {
                    printf("Point exceed the limitation of window");
                    continue;
                }
                cur.emplace_back(p,__MYSCALAR__(i),0,0);
                FeatureRouteDrawPoint(p,__MYSCALAR__(i),expirectl);
                i = i + 1;
            }
        }
        else
        {
            vector<Point> lt;
            vector<Point> rb;
            count = pts.size();
            int curError;
            int selectedIndex;
            int error;
            for(const auto& p:pts)
            {
                lt.emplace_back(Point(lastPts[i].p.x - wd,lastPts[i].p.y - wd));
                rb.emplace_back( Point(lastPts[i].p.x + wd,lastPts[i].p.y + wd));
            }
            for(const auto& p:pts)
            {
                error = numeric_limits<int>::max();
                selectedIndex = -1;
                for(i = 0;i<lastPts.size();i++)
                {
                    if((p.x>lt[i].x&&p.y>lt[i].y&&p.x<rb[i].x&&p.y<rb[i].y)
                       &&!lastPts[i].used
                       ||(lastPts[i].vx!=0&&(lastPts[i].p.x - p.x) != 0
                          &&(lastPts[i].vy/lastPts[i].vx > 0) == ((lastPts[i].p.y - p.y)/(lastPts[i].p.x - p.x) > 0)
                          &&(abs(lastPts[i].vy*3)>abs((lastPts[i].p.y - p.y))&&abs(lastPts[i].vx*3)>abs((lastPts[i].p.x - p.x)))
                       )
                            )
                    {
                        curError = abs(lastPts[i].p.y - p.y) + abs(lastPts[i].p.x - p.x);
                        error = (error>curError)?(selectedIndex = i,curError):(error);
                    }
                }
                if(selectedIndex != -1)
                {
                    cur.emplace_back(RoutePoint(p,lastPts[selectedIndex].color,(p.x - lastPts[i].p.x),(p.y - lastPts[i].p.y)));
                    lastPts[selectedIndex].used = true;
                }
                else
                {
                    cur.emplace_back(RoutePoint(p, __MYSCALAR__(count),0,0));
                    count = (count + 1)%18;
                }
            }
            for(const auto& p:cur)
            {
                if(p.p.x<0||p.p.x>origin.cols||p.p.y<0||p.p.y>origin.rows)
                {
                    printf("Point exceed the limitation of window");
                    continue;
                }
                FeatureRouteDrawPoint(p.p,p.color,expirectl);
                //circle(origin,p.p,pSize,p.color,-1);
            }
        }
        imshow("FeatureRoute",srcs[expirectl]);
        expirectl = (expirectl + 1)%5;

        lastPts = cur;
    }

    static void sleep_ms(unsigned int secs)
    {
        struct timeval tval;

        tval.tv_sec=secs/1000;

        tval.tv_usec=(secs*1000)%1000000;

        select(0,NULL,NULL,NULL,&tval);
    }


}

#endif