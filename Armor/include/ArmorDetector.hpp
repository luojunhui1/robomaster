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
    struct PARAM
    {
        /*Armor paramter*/
        float max_armor_angle = 20;
        int max_angle_error = 5;
        float max_length_error = 0.30;
        float max_yDiff = 2;
        float max_ratio = 6;
        float min_ratio = 0.8;

        /*LED_Stick paramter*/
        float min_bar_area = 30;
        float max_bar_area = 500;
        float max_bar_angle = 30;
        float min_bar_height2width = 1;
        float max_bar_height2width = 12;
    };

    typedef struct matchlight
    {
        bool used = false;
        unsigned int match_index1 = -1;
        unsigned int match_index2 = -1;
        float match_factor = 10000;
    } MATCHLIGHT;

    /*match LED stick*/
    bool compMatchFactor(const MATCHLIGHT& a, const MATCHLIGHT& b);

    class LED_Stick
    {
    public:
        LED_Stick()
        {
        }
        LED_Stick(RotatedRect bar, float angle) : rect(std::move(bar)), light_angle(angle)
        {
        }
        RotatedRect rect;
        /*??????ǶȺͳ??ȣ?ֱ?Ӹ?otatedRect????????????*/
        float light_angle;
    };

    class Armor
    {
    public:
        Armor() : error_angle(0), armor_width(0), armor_height(0), armor_type(BIG_ARMOR), priority(10000)
        {
        }
        Armor(const LED_Stick& L1, const LED_Stick& L2);
        void init();
        float error_angle;  /*the differ between the angle of two sticks*/
        Point2i center;		/*center of the armor*/
        Rect rect;			/*the rect to represent the armor*/
        float armor_width;  /*width*/
        float armor_height; /*height*/
        int armor_type;		/*small armor or big armor*/
        double priority;		/*priority to attack*/
    };

    /*server for sorting the priority of armors*/
    bool compArmorPriority(const Armor& a, const Armor& b);


    class ArmorDetector
    {
    public:
        ArmorDetector();
        ~ArmorDetector() {}

        void init();
        bool ArmorDetectTask(Mat& img);
        void GetRoi(Mat& img);
        bool DetectArmor(Mat& img);
        void preprocess(Mat& img);
        int get_average_intensity(Mat& img); /*?????װ?oiƽ??ɫ????ȣ???ɸѡװ?װ???е??*/
        vector<LED_Stick> light_detection(Mat& img);
        void max_match(vector<LED_Stick> lights);
        bool makeRectSafe(cv::Rect& rect, cv::Size size);
        void getAngle(float& yaw, float& pitch) const
        {
            yaw = angle_x_;
            pitch = angle_y_;
        }
        Rect getArmorRect() const
        {
            return last_armor.rect;
        }
        bool isSmall()
        {
            if (last_armor.armor_type = SMALL_ARMOR)
                return true;
            else
            {
                return false;
            }
        }
        void setFilter(int filter_size)
        {
            filter_size_ = filter_size;
        }
        void clear()
        {
            history_.clear();
        }

    private:
        vector<Armor> final_armors; /*possible armors*/
        vector<Armor> history;		/* the armors have being selected*/
        std::list<bool> history_;   /*the armor find state histort*/
        Armor last_armor;			/*last armor*/
        bool find_state_;			/*current find state*/
        bool is_small_;				/*small armor or big armor*/
    private:
        // parameter
        int color_{};
        int cap_mode_{};
    public:
        // pnp paramter
        int km_Qp_ = 1000;
        int km_Qv_ = 1;
        int km_Rp_ = 1;
        int km_Rv_ = 1;
        int km_t_ = 1;
        int km_pt_ = 60;
        float last_angle = 0;
        float last_v = 0;
        float last_last_v = 0;
        int filter_size_ = 5;

    public:

        int short_offset_x_ = 100;
        int short_offset_y_ = 100;
        int long_offset_x_ = 100;
        int long_offset_y_ = 100;
        int color_th_ = 16;
        int gray_th_ = 60;
        float bright_alpha = 1;
        float bright_belta = 70;

    private:
        Rect roi_rect;//last roi
        int average_intensity{};
        Mat_<int> color_map;
        Mat threshold_map;
        Mat_<int> r_b;
        Mat_<int> b_r;
        Mat mask;
        Mat img_roi;
    private:
        float dist_ = 3000;		// ͨ??????????
        float r_ = 0.5;			// ?????? (0-1)
        int update_cap_cnt = 0; // ??ǿ?????????Ƶ?
    public:
        int lost_cnt_ = 130;
        int detect_cnt_ = 0;
        float distance_ = 0;
        float last_distance = 0;
        float angle_x_ = 0;
        float angle_y_ = 0;
        vector<Point2f> points_2d_;

    public:
        struct PARAM param_;

    public:
        typedef cv::TrackerKCF TrackerToUse; // Tracker??Ͷ??
        cv::Ptr<cv::Tracker> tracker;
        bool trackingTarget(Mat& src, Rect2d target);

    public:
        bool blue_target = false;
        bool show_bianryimg = true;
        bool show_light_blobs = true;
        bool show_armor_box = false;
        bool show_armor_boxes = true;
    };

} // namespace rm