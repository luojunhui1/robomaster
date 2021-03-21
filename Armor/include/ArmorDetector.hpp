/**********************************************************************************************************************
 * this file is mainly used for detecting the lamps and matching them into series of armors. First, by preprocessing the
 * image captured from video or camera, we get a binary image, in which we set the target lamps' pixel value as 255. Second,
 * recognizing all the possible lamps in the image by a set of rules made in advance, for example, in our source code,
 * we use 5 parameters to recognize lamps, they are
 * 1. the maximum possible angle of lamp.
 * 2. the minimum possible area of lamp.
 * 3. the maximum possible area of lamp.
 * 4. the maximum possible value of the ratio of lamp's height to lamp's width.
 * 5. the minimum possible value of the ratio of lamp's height to lamp's width.
 * The parameters above are particular for each project, they are related the the ratio of your image, the condition for
 * application ,etc.
 * Third, matching these lamps to a series of armors by some parameters and matching algorithm. And the whole code logic
 * and process you can see at https://github.com/luojunhui1/robomaster, for more detail, you can contact with the active
 * team members of Robomaster in SWJTU.
**********************************************************************************************************************/
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>

#include <iostream>
#include <cmath>
#include <ctime>
#include <utility>
#include <vector>
#include <cstring>

#include "log.h"
#include "mydefine.h"

using namespace std;
using namespace cv;


namespace rm
{
    /**
     * define the paramters for lamp recognizing and lamp matching.
     */
    struct ArmorParam
    {

        float maxArmorAngle = 40;
        int   maxAngleError = 60;
        float maxLengthError = 0.70;
        float maxYDiff = 2;
        float maxRatio = 66;
        float minRatio = 0.6;

        float minLightArea = 10;
        float maxLightArea = 2500;
        float maxLightAngle = 60;
        float minLightW2H = 0.2;
        float maxLightW2H = 40;
    };
    /**
     * the structure to describe the matched lamps
     */
    typedef struct MatchLight {
        bool used = false;
        unsigned int matchIndex1 = -1;
        unsigned int matchIndex2 = -1;
        float matchFactor = 10000;

        explicit MatchLight(bool used_ = false, unsigned int match1 = 0, unsigned int match2 = 0, float fractor = 1000) {
            used = used_;
            matchIndex1 = match1;
            matchIndex2 = match2;
            matchFactor = fractor;
        }
    }MatchLight;

    /**
     * define a structure to describe parameters for matching lamps
     */
    typedef struct MatchParam
    {
        double dAngle{};
        double conLen1{};
        double conLen2{};
        double ratio{};
        double nAngle{};
        double yDiff{};
        double dBright{};
        double sizeRatio{};

        MatchParam(){};
        MatchParam(double dAngle_, double conLen1_, double conLen2_, double ratio_, double nAngle_, double yDiff_
                , double dBright_, double sizeRatio_)
        {
            dAngle = dAngle_;
            conLen1 = conLen1_;
            conLen2 = conLen2_;
            ratio = ratio_;
            nAngle = nAngle_;
            yDiff = yDiff_;
            dBright = dBright_;
            sizeRatio = sizeRatio_;
        }
    }MatchParam;

    /**
     * the class to describe the lamp, including a rotated rectangle to describe the lamp's  geometry information and an
     * angle value of the lamp that is more intuitive than the angle member variable in RotateRect. For detail, you can
     * see it at https://blog.csdn.net/xueluowutong/article/details/86069814?utm_medium=distribute.pc_relevant.none-task-blog-title-2&spm=1001.2101.3001.4242
     */
    class LEDStick {
    public:
        LEDStick() : lightAngle(0),avgBrightness(0), size(0)
        {
        }

        LEDStick(RotatedRect bar, float angle, float avgB, float size_) : rect(std::move(bar)), lightAngle(angle),
                                                                          avgBrightness(avgB),size(size_) {
        }

        RotatedRect rect;

        float lightAngle;
        float avgBrightness;
        float size;
    };

    /**
     * the class to describe the armor, including the differ between the angle of two lamps(errorAngle), the rect to
     * represent the armor(rect), the width of the armor(armorWidth) and the height of the armor(armorWidth), the type
     * of the armor(armorType), the priority of the armor to be attacked(priority)
     */
    class Armor {
    public:
        Armor() : errorAngle(0), armorWidth(0), armorHeight(0), armorType(BIG_ARMOR), priority(10000) {
        }

        Armor(const LEDStick &L1, const LEDStick &L2, double priority_);

        void init();

        float errorAngle;
        Point2i center;
        Rect rect;
        vector<Point2f> pts;
        float armorWidth;
        float armorHeight;
        int armorType;
        double priority;
    };

    /**
     * the tool class for functions, including the functions of preprocessing, lamp recognizing, lamp matching, ROI
     * updating, tracking and etc.
     */
    class ArmorDetector {
    public:
        ArmorDetector();

        ~ArmorDetector() = default;

        /*core functions*/
        void Init();

        bool ArmorDetectTask(Mat &img);
        bool ArmorDetectTaskGPU(Mat &img);

        void GetRoi(Mat &img);

        virtual bool DetectArmor(Mat &img);
        virtual bool DetectArmorGPU(Mat &img);

        virtual void Preprocess(Mat &img);
        virtual void PreprocessGPU(cuda::GpuMat& img);

        void MaxMatch(vector<LEDStick> lights);

        vector<LEDStick> LightDetection(Mat& img);

        /*tool functions*/
        static bool MakeRectSafe(cv::Rect &rect, const cv::Size &size);

        Rect GetArmorRect() const;

        bool IsSmall() const;

        void saveMatchParam(FILE* fileP,int selectedIndex1,int selectedIndex2);

        /*state member variables*/
    public:
        /*possible armors*/
        vector<Armor> possibleArmors;

        /*the final armor selected to be attacked*/
        Armor targetArmor;

        /*current find state*/
        bool findState;

        /*current armor type*/
        bool isSmall;

        /*ROI rect in image*/
        Rect roiRect;

        /* variables would be used in functions*/
    public:

        /*a gray image, the difference between rSubB and bSubR*/
        Mat_<int> colorMap;

        /*a gray image, the pixel's value is the difference between red channel and blue channel*/
        Mat_<int> rSubB;

        /*a gray image, the pixel's value is the difference between blue channel and red channel*/
        Mat_<int> bSubR;

        /*a mask image used to calculate the sum of the values in a region*/
        Mat mask;

        /*ROI image*/
        Mat imgRoi;

        /*a binary image*/
        Mat thresholdMap;

        /*the frequency information*/
    public:
        /*the number of frames that program don't get a target armor constantly*/
        int lostCnt = 130;

        /*the number of frames that program get a target armor constantly*/
        int detectCnt = 0;

        /*parameters used for lamps recognizing and lamps matching*/
    public:

        /*parameters used for lamps recognizing and lamps matching*/
        struct ArmorParam param;
        //Net net;

        /*tracking member variables */
    public:

        /*an instance of tracker*/
        cv::Ptr<cv::Tracker> tracker;

        /*if the tracer found the target, return true, otherwise return false*/
        bool trackingTarget(Mat &src, Rect2d target);

    private:
        cuda::Stream stream;
        cuda::GpuMat gpuImg,gpuRoiImg,gpuRoiImgFloat;
        vector<cuda::GpuMat> gpuRoiImgVector;

        cuda::GpuMat gpuGray,gpuBlur,gpuBright;
        cuda::GpuMat gpuBSubR;
        cuda::GpuMat gpuRSubB;

        Ptr<cuda::Filter> gauss = cuda::createGaussianFilter(CV_32F, -1, Size(5, 5), 3);
    };

    class ArmorCompare: public ArmorDetector
    {
    private:
        Mat lastBright;
        Mat dBright;
    public:
        void InitCompare();
        void Preprocess(Mat &img) override;
        bool DetectArmor(Mat &img) override;
    };

    /**
     * @param a an instance of a pair of matched lamps
     * @param b another instance of a pair of matched lamps
     * @return if the match factor of b is larger than a, return true, otherwise return false.
     */
    bool compMatchFactor(const MatchLight a, const MatchLight b);

    /**
     * @param a an instance of armor
     * @param b another instance of armor
     * @return if the priority of b is larger than a, return true, otherwise return false.
     * @note not used anymore
     */
    bool CompArmorPriority(const Armor &a, const Armor &b);
}
