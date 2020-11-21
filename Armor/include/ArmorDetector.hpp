#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>
#include <cmath>
#include <ctime>
#include <utility>
#include <vector>
#include <cstring>
#include "mydefine.h"
using namespace std;
using namespace cv;
using namespace Eigen;
namespace rm
{
    struct ArmorParam
    {
        /*Armor paramter*/
        float maxArmorAngle = 30;
        int maxAngleError = 10;
        float maxLengthError = 0.30;
        float maxYDiff = 20;
        float maxRatio = 66;
        float minRatio = 0.8;

        /*LEDStick paramter*/
        float minLightArea = 30;
        float maxLightArea = 500;
        float maxLightAngle = 40;
        float minLightW2H = 1;
        float maxLightW2H = 40;
    };

    typedef struct MatchLight
    {
        bool used = false;
        unsigned int match_index1 = -1;
        unsigned int match_index2 = -1;
        float match_factor = 10000;
    } MatchLight;

    /*match LED stick*/
    bool compMatchFactor(const MatchLight& a, const MatchLight& b);

    class LEDStick
    {
    public:
        LEDStick()
        {
        }
        LEDStick(RotatedRect bar, float angle) : rect(std::move(bar)), light_angle(angle)
        {
        }
        RotatedRect rect;

        float light_angle;
    };

    class Armor
    {
    public:
        Armor() : errorAngle(0), armorWidth(0), armorHeight(0), armorType(BIG_ARMOR), priority(10000)
        {
        }
        Armor(const LEDStick& L1, const LEDStick& L2);
        void init();
        float errorAngle;  /*the differ between the angle of two sticks*/
        Point2i center;		/*center of the armor*/
        Rect rect;			/*the rect to represent the armor*/
        float armorWidth;  /*width*/
        float armorHeight; /*height*/
        int armorType;		/*small armor or big armor*/
        double priority;		/*priority to attack*/
    };

    /*server for sorting the priority of armors*/
    bool CompArmorPriority(const Armor& a, const Armor& b);


    class ArmorDetector
    {
    public:
        ArmorDetector();
        ~ArmorDetector() {}

        void Init();
        bool ArmorDetectTask(Mat& img);
        void GetRoi(Mat& img);
        bool DetectArmor(Mat& img);
        void Preprocess(Mat& img);
        int GetAverageIntensity(Mat& img);
        vector<LEDStick> LightDetection(Mat& img);
        void MaxMatch(vector<LEDStick> lights);
        static bool MakeRectSafe(cv::Rect& rect, cv::Size size);
        Rect GetArmorRect() const
        {
            return lastArmor.rect;
        }
        bool IsSmall()
        {
            if (lastArmor.armorType == SMALL_ARMOR)
                return true;
            else
            {
                return false;
            }
        }
        void Clear()
        {
            history_.clear();
        }

    private:
        vector<Armor> finalArmors; /*possible armors*/
        vector<Armor> history;		/* the armors have being selected*/
        std::list<bool> history_;   /*the armor find state histort*/
        bool findState;			/*current find state*/
        bool isSmall;				/*small armor or big armor*/
    private:
        // parameter
        int color{};
        int capMode{};

    private:
        Rect roiRect;//last roi
        int averageIntensity{};
        Mat_<int> colorMap;
        Mat thresholdMap;
        Mat_<int> rSubB;
        Mat_<int> bSubR;
        Mat mask;
        Mat imgRoi;

    public:
        int lostCnt = 130;
        int detectCnt = 0;
        Armor lastArmor;			/*last armor*/
    public:
        struct ArmorParam param;

    public:
        typedef cv::TrackerKCF TrackerToUse; // Tracker??Ͷ??
        cv::Ptr<cv::Tracker> tracker;
        bool trackingTarget(Mat& src, Rect2d target);

    public:
        bool blueTarget = false;
        bool showBianryImg = true;
        bool showLightBlobs = true;
        bool showArmorBox = false;
        bool showArmorBoxes = true;
    };

} // namespace rm