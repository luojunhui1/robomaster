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
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/dnn.hpp>

#include <iostream>
#include <cmath>
#include <ctime>
#include <utility>
#include <vector>
#include <cstring>
#include <omp.h>
#include "log.h"
#include "mydefine.h"

using namespace std;
using namespace cv;
using namespace ml;
using namespace cv::dnn;

namespace rm
{
    /**
     * define the parameters for lamp recognizing and lamp matching.
     */
    struct ArmorParam
    {
        float maxArmorAngle = 45;
        float maxAngleError = 6;
        float maxLengthError = 0.70;
        float maxDeviationAngle = 50;
        float maxYDiff = 2;
        float maxRatio = 20;
        float minRatio = 0.6;

        float minLightArea = 45;
        float maxLightArea = 2500;
        float maxLightAngle = 60;
        float minLightW2H = 0.5;
        float maxLightW2H = 40;
    };

    /**
     * the structure to describe the matched lamps
     */
    typedef struct MatchLight {
        float lampHeight;
        unsigned int matchIndex1 = -1;
        unsigned int matchIndex2 = -1;
        float matchFactor = 10000;

        explicit MatchLight(unsigned int match1 = 0, unsigned int match2 = 0, float fractor = 1000, float lampHeight_ = 0) {
            matchIndex1 = match1;
            matchIndex2 = match2;
            matchFactor = fractor;
            lampHeight = lampHeight_;
        }
    }MatchLight;

    /**
     * define a structure to describe parameters for matching lamps
     */
    typedef struct MatchParam_Tag
    {
        double dAngle{};
        double conLen1{};
        double conLen2{};
        double ratio{};
        double nAngle{};
        double yDiff{};
        double dBright{};
        double sizeRatio{};

        MatchParam_Tag()= default;;
        MatchParam_Tag(double dAngle_, double conLen1_, double conLen2_, double ratio_, double nAngle_, double yDiff_
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
    class Lamp {
    public:
        Lamp() : lightAngle(0), avgRSubBVal(0)
        {
        }

        Lamp(RotatedRect bar, float angle, float avgB) :lightAngle(angle),avgRSubBVal(avgB)
        {
            cv::Size exLSize(int(bar.size.width), int(bar.size.height * 2));
            rect = cv::RotatedRect(bar.center, exLSize, bar.angle);
        }

        RotatedRect rect;

        float lightAngle;
        float avgRSubBVal;
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

        Armor(Lamp L1, Lamp L2, double priority_);
        explicit Armor(Rect &rect);
        void init();

        float errorAngle;
        Point2i center;
        Rect rect;
        vector<Point2f> pts;
        float armorWidth;
        float armorHeight;
        int armorType;
        double priority;
        float avgRSubBVal{};
    };

    /**
     * the tool class for functions, including the functions of preprocessing, lamp recognizing, lamp matching, ROI
     * updating, tracking and etc.
     */
    class ArmorDetector {
    public:
        ArmorDetector();

        ~ArmorDetector() = default;

        /**core functions**/
        void Init();

        bool ArmorDetectTask(Mat &img);

        void GetRoi();

        bool DetectArmor();

        void Preprocess(Mat &img);

        void MaxMatch(vector<Lamp> &lights);

        vector<Lamp> LightDetection(Mat& img);

        void LoadSvmModel(const char *model_path, const Size& armorImgSize = Size(40, 40));

        void SetSVMRectPoints(Point2f& lt, Point2f& rt, Point2f& lb, Point2f& rb);
        void SetSVMRectPoints(Point2f&& lt, Rect& rectArea);


        int GetArmorNumber();

        /**tool functions**/

        Rect GetArmorRect() const;

        bool IsSmall() const;

        static void saveMatchParam(FILE* fileP,int selectedIndex1,int selectedIndex2);

        /**state member variables**/
    public:

        /*the final armor selected to be attacked*/
        Armor targetArmor;

        /*current find state*/
        bool findState;

        /*current armor type*/
        bool isSmall;

        /*ROI rect in image*/
        Rect roiRect;

        /*target armor number*/
        int armorNumber;

        /*clone of image passed in*/
        Mat img;

        /*loss cnt*/
        int lossCnt;

        /** variables would be used in functions**/
    private:

        /*a gray image, the difference between rSubB and bSubR*/
        Mat_<int> colorMap;

        /*a gray image, the pixel's value is the difference between red channel and blue channel*/
        Mat rSubB;

        /*a gray image, the pixel's value is the difference between blue channel and red channel*/
        Mat bSubR;

        /*a mask image used to calculate the sum of the values in a region*/
        Mat mask;

        /*ROI image*/
        Mat imgRoi;

        /*a binary image*/
        Mat thresholdMap;

        /*last target armor*/
        Armor lastTarget;

        /**the frequency information**/
    private:
        /*the number of frames that program don't get a target armor constantly*/
        int lostCnt = 130;

        /*the number of frames that program get a target armor constantly*/
        int detectCnt = 0;

        /*the counter for successfully detection of target armor*/
        int armorFoundCounter = 0;

        /**parameters used for lamps recognizing and lamps matching**/
    private:

        struct ArmorParam param;

        float averageRSubBVal = 0;
        /**tracking member variables **/
    public:
        /*an instance of tracker*/
        cv::Ptr<cv::Tracker> tracker;

        /*if the tracer found the target, return true, otherwise return false*/
        bool trackingTarget(Mat &src, Rect2d target);

        /*armor number recogniztion*/
    private:
        Ptr<SVM>svm;  //svm model svm模型
        Size svmArmorSize;
        Mat svmBinaryImage;
        Mat svmParamMatrix;		//preRecoginze matrix for svm 载入到SVM中识别的矩阵
        Mat warpPerspective_dst;//warpPerspective dstImage   透射变换生成的目标图
        Mat warpPerspective_mat; //warpPerspective transform matrix 透射变换的变换矩阵
        Point2f srcPoints[4];   //warpPerspective srcPoints		透射变换的原图上的目标点 tl->tr->br->bl  左上 右上 右下 左下
        Point2f dstPoints[4];	//warpPerspective dstPoints     透射变换的目标图中的点   tl->tr->br->bl  左上 右上 右下 左下

    //deep learning
    private:
        String cfgPath;
        String weightPath;
        Net net;
        std::vector<String> outNames;
        std::vector<Mat> outs;

        Mat inputBlob;
        double confidence;
        Point classIdPoint;
        vector<Rect> boxes;
        vector<int> classIds;
        vector<float> confidences;
        vector<int> indices;

        bool find_not_engineer;
        int dis2LastCenter;
    public:
        bool ModelDetectTask(Mat &frame);
    };

    /**
     * @param a an instance of a pair of matched lamps
     * @param b another instance of a pair of matched lamps
     * @return if the match factor of b is larger than a, return true, otherwise return false.
     */
    bool compMatchFactor(const MatchLight a, const MatchLight b);

    bool MakeRectSafe(cv::Rect &rect, const cv::Size& size);
}
