#include "ArmorDetector.hpp"
#include <sstream>
#include <algorithm>

namespace rm
{

    /**
    * @brief Armor constructor
    * @param [L1] one lamp of the armor
    * @param [L2] another lamp of the armor
    * @return none
    * @details none
    */
    Armor::Armor(const LEDStick &L1, const LEDStick &L2)
    {
        errorAngle = fabs(L1.lightAngle - L2.lightAngle);
        armorWidth = fabs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
        armorHeight = fabs(static_cast<int>((L1.rect.size.height + L1.rect.size.height) / 2));
        center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x) / 2);
        center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y) / 2);
        rect = Rect(center - Point2i(armorWidth / 2, armorHeight / 2), Size(armorWidth, armorHeight));
        armorType = (armorWidth / armorHeight > 4) ? (BIG_ARMOR) : (SMALL_ARMOR);
        priority = fabs(center.x - 320)+fabs(center.y - 240);

        //need to make sure how to set values to the points
        pts.resize(4);
        Point2f pts_[4];
        if(L1.rect.center.x < L2.rect.center.x)
        {
            L1.rect.points(pts_);
            if(L1.lightAngle < 0)
            {
                pts[0] = Point2f((pts_[0] + pts_[3])/2);
                pts[3] = Point2f(pts_[1]/2 + pts_[2]/2);
            }
            else
            {
                pts[3] = Point2f((pts_[0] + pts_[3])/2);
                pts[0] = Point2f((pts_[1] + pts_[2])/2);
            }

            L2.rect.points(pts_);
            if(L2.lightAngle < 0)
            {
                pts[1] = Point2f((pts_[0] + pts_[3])/2);
                pts[2] = Point2f(pts_[1]/2 + pts_[2]/2);
            }
            else
            {
                pts[2] = Point2f((pts_[0] + pts_[3])/2);
                pts[1] = Point2f(pts_[1]/2 + pts_[2]/2);
            }

        }else
        {
            L2.rect.points(pts_);
            if(L2.lightAngle < 0)
            {
                pts[0] = Point2f((pts_[0] + pts_[3])/2);
                pts[3] = Point2f(pts_[1]/2 + pts_[2]/2);
            }
            else
            {
                pts[3] = Point2f((pts_[0] + pts_[3])/2);
                pts[0] = Point2f(pts_[1]/2 + pts_[2]/2);
            }

            L1.rect.points(pts_);
            if(L1.lightAngle < 0)
            {
                pts[1] = Point2f((pts_[0] + pts_[3])/2);
                pts[2] = Point2f(pts_[1]/2 + pts_[2]/2);
            }
            else
            {
                pts[2] = Point2f((pts_[0] + pts_[3])/2);
                pts[1] = Point2f(pts_[1]/2 + pts_[2]/2);
            }
        }
    }

    /**
    * @brief ArmorDetector constructor
    * @param none
    * @return none
    */
    ArmorDetector::ArmorDetector()
    {
        findState = false;
        isSmall = false;
    }

    /**
     * @brief function used to initialize the armor instance
     * @param none
     * @return none
    */
    void Armor::init()
    {
        errorAngle = 0;
        armorWidth = 0;
        armorHeight = 0;
        armorType = BIG_ARMOR;
        priority = 10000;
    }

    /**
     * @brief function used to initialize the Armor detector instance
     * @param none
     * @return none
    */
    void ArmorDetector::Init()
    {
        possibleArmors.clear();
        history.clear();
        history_.clear();
        targetArmor.init();
        roiRect = Rect(0, 0, 0, 0);
        findState = false;
        detectCnt = 0;
        lostCnt = 120;
    }

    /**
    * @brief: top level detection task
    * @param [img] the image from camera or video that to be processed
    * @return: if ever found armors in this image, return true, otherwise return false
    * @details: none
    */
    bool ArmorDetector::ArmorDetectTask(Mat &img)
    {
        GetRoi(img); //get roi
        return DetectArmor(img);
    }

    /**
    * @brief: detect possible armors
    * @param [img]  the image from camera or video that to be processed
    * @return: if ever found armors in this image, return true, otherwise return false
    * @details: none
    */
    bool ArmorDetector::DetectArmor(Mat &img)
    {
        findState = false;
        vector<LEDStick> lights;

        imgRoi = img(roiRect);

        Preprocess(imgRoi);

        lights = LightDetection(thresholdMap);

        if (showLamps)
        {
            for(auto & light : lights) {
                Point2f rect_point[4]; //
                light.rect.points(rect_point);
                for (int j = 0; j < 4; j++) {
                    line(imgRoi, rect_point[j], rect_point[(j + 1) % 4], Scalar(255, 0, 255), 1);
                }
            }
        }

        MaxMatch(lights);

        if (findState)
        {
            detectCnt++;
            lostCnt = 0;

            targetArmor = possibleArmors[0];

            MakeRectSafe(targetArmor.rect, img.size());

            targetArmor.rect  = targetArmor.rect + Point(roiRect.x, roiRect.y);

            for(int i = 0; i< 4;i++)
            {
                targetArmor.pts[i] = targetArmor.pts[i] + Point2f(roiRect.x, roiRect.y);
            }
            //cout<<"targetArmor: "<<"("<<targetArmor.rect.x<<","<<targetArmor.rect.y<<")"<<" w:"<<targetArmor.rect.width
            // <<" h:"<<targetArmor.rect.height<<endl;
            //cout<<"roiRect"<<"("<<roiRect.x<<","<<roiRect.y<<")"<<" w:"<<roiRect.width<<" h:"<<roiRect.height<<endl;

            if (showArmorBox)
            {
                rectangle(img, targetArmor.rect, Scalar(0, 125, 255), 2);
            }

            if (showArmorBoxes)
            {
                for (auto & final_armor : possibleArmors)
                {
                    final_armor.rect = final_armor.rect + Point(roiRect.x, roiRect.y);
                    MakeRectSafe(final_armor.rect, img.size());
                    //MakeRectSafe(final_armor.rect, imgRoi.size());
                    rectangle(img, final_armor.rect, Scalar(0, 225, 255), 2);
                }
            }

            roiRect = targetArmor.rect;
            possibleArmors.clear();
            return true;
        }
        else
        {
            detectCnt = 0;
            lostCnt++;

            possibleArmors.clear();

            return false;
        }
    }

    /**
    * @brief get the Rect instance that describe the target armor's geometry information
    * @param none
    * @return the Rect instance that describe the target armor's geometry information
    * @details none
    */
    Rect ArmorDetector::GetArmorRect() const
    {
        return targetArmor.rect;
    }

    /**
    * @brief judge wheter the target armor is small armor or big armor
    * @param none
    * @return if the target is small armor then return true, otherwise return false
    * @details this function just used for simplifying the using of targetArmor.armorType
    */
    bool ArmorDetector::IsSmall() const
    {
        return (targetArmor.armorType == SMALL_ARMOR);
    }

    /**
    * @brief match lamps to series of armors
    * @param [lights] a vector of all the possible lamps in the image
    * @return none
    * @details none
    */
    void ArmorDetector::MaxMatch(vector<LEDStick> lights)
    {
        static float yDiff, xDiff;
        static float nL,nW;
        static float dAngle;
        static float contourLen1;
        static float contourLen2;
        static float ratio;
        static float nAngle;
        vector<MatchLight> matchLights;
        float match_factor_;

        if (lights.size() < 2)
            return;

        for (unsigned int i = 0; i < lights.size() - 1; i++)
        {
            for (unsigned int j = i + 1; j < lights.size(); j++)
            {
                /*the difference between to angles*/
                dAngle = fabs(lights[i].lightAngle - lights[j].lightAngle);
                if(dAngle > param.maxAngleError)continue;
                //cout<<"dangle:"<<dAngle<<endl;

                /*the difference ratio of the two lights' height*/
                contourLen1 = abs(lights[i].rect.size.height - lights[j].rect.size.height) / max(lights[i].rect.size.height, lights[j].rect.size.height);
                if(contourLen1 > param.maxLengthError)continue;
                //cout<<"contourLen1:"<<contourLen1<<endl;

                /*the difference ratio of the two lights' width*/
                //contourLen2 = abs(lights[i].rect.size.width - lights[i].rect.size.width) / max(lights[i].rect.size.width, lights[j].rect.size.width);
                //cout<<"contourLen2:"<<contourLen2<<endl;

                /*the average height of two lights(also the height of the armor defined by these two lights)*/
                nW = (lights[i].rect.size.height + lights[j].rect.size.height) / 2;
                //cout<<"nW:"<<nW<<endl;

                /*the width between the center points of two lights*/
                nL = fabs(lights[i].rect.center.x - lights[j].rect.center.x);
                //cout<<"nL:"<<nL<<endl;

                /*anyway, the difference of the lights' angle is tiny,so anyone of them can be the angle of the armor*/
                nAngle = fabs(lights[i].lightAngle);
                if(nAngle > param.maxArmorAngle)continue;
                //cout<<"nAngle:"<<nAngle<<endl;

                /*the ddifference of the y coordinate of the two center points*/
                yDiff = abs(lights[i].rect.center.y - lights[j].rect.center.y) / nW;
                if(yDiff > param.maxYDiff)continue;
                //cout<<"yDiff:"<<yDiff<<endl;

                /*the ratio of the width and the height, must larger than 1 */
                ratio = nL / nW;
                if(ratio > param.maxRatio || ratio < param.minRatio)continue;

                /*the match factor is still rough now, but it can help filter  the most possible target from  the detected armors*/
                match_factor_ = dAngle + contourLen1 + contourLen2 + yDiff + MIN(fabs(ratio - 1.5), fabs(ratio - 3.5));

                if (!findState)
                    findState = true;
                matchLights.push_back(MatchLight {false, i, j, match_factor_});
            }
        }
        /*sort these pairs of lamps by match factor*/
        sort(matchLights.begin(), matchLights.end(), compMatchFactor);

        /*matching the lights by the priority factors*/
        for (int i = 0; i < matchLights.size(); i++)
        {

            if (!matchLights[i].used)
            {
                possibleArmors.emplace_back(lights[matchLights[i].matchIndex1], lights[matchLights[i].matchIndex2]);

                for (int j = i; j < matchLights.size(); j++)
                {
                    if (matchLights[j].used)
                        continue;
                    if (matchLights[j].matchIndex1 == matchLights[i].matchIndex1 ||
                        matchLights[j].matchIndex1 == matchLights[i].matchIndex2 ||
                        matchLights[j].matchIndex2 == matchLights[i].matchIndex1 ||
                        matchLights[j].matchIndex2 == matchLights[i].matchIndex2)
                    {
                        matchLights[j].used = true;
                    }
                }
            }
        }

        if (possibleArmors.empty())
            findState = false;
    }

    /**
    * @brief pre-procession of an image captured
    * @param [img] the ROI image that clipped by the GetRIO function
    * @return none
    * @details if average value in a region of the colorMap is larger than 0, then we can inference that in this region
    * the light is more possible to be red
    */
    void ArmorDetector::Preprocess(Mat &img)
    {
        Mat bright;
        vector<Mat> channels;
        split(img,channels);
        cvtColor(img,bright,CV_BGR2GRAY);//0,2,1
        //Attention!!!if the caculate result is small than 0, because the mat format is CV_UC3, it will be set as 0.
        bSubR = Mat(channels[0] - channels[2]);
        rSubB = Mat(channels[2] - channels[0]);
        threshold(bright, thresholdMap, 180, 255, CV_MINMAX);
        colorMap = rSubB - bSubR ;
    }

    /**
    * @brief detect and filter lights in img
    * @param img
    * @return a vector of possible led object
    **/
    vector<LEDStick> ArmorDetector::LightDetection(Mat &img)
    {
        const Mat &roi_image = img;
        Mat hsv_binary;
        float angle_ = 0;
        Scalar_<double> avg;
        float stick_area;

        vector<LEDStick> LED_Stick_v;

        if (showBianryImg)
        {
            imshow("binary_brightness_img", thresholdMap);
        }

        vector<vector<Point>> contours_light;


        findContours(thresholdMap, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        for (auto & i : contours_light)
        {
            if (i.size() < 5)
                continue;

            double length = arcLength(i, true);

            if (length > 10 && length < 4000)
            {
                //cout<<"contours_brightness: "<<contours_brightness[ii].size()<<endl;
                RotatedRect Likely_stick = fitEllipse(i);
                stick_area = Likely_stick.size.width * Likely_stick.size.height;
                if((stick_area > param.maxLightArea) || (stick_area < param.minLightArea))continue;
                float rate_height2width = Likely_stick.size.height / Likely_stick.size.width;
                if((rate_height2width < param.minLightW2H) || (rate_height2width > param.maxLightW2H))continue;

                angle_ = (Likely_stick.angle > 90.0f) ? (Likely_stick.angle - 180.0f) : (Likely_stick.angle);

                if(fabs(angle_) >= param.maxLightAngle)continue;

                mask = Mat::zeros(img.rows,img.cols,CV_8UC1);
                rectangle(mask,Likely_stick.boundingRect(),Scalar(1),-1);
                avg = mean(colorMap, mask);

                if((blueTarget && avg[0] < 0) || (!blueTarget && avg[0] > 0))
                {
                    LEDStick Build_stick_information(Likely_stick, angle_);
                    LED_Stick_v.push_back(Build_stick_information);
                }
            }
        }
        return LED_Stick_v;
    }

    /**
    * @brief get he region of interest
    * @param [img] the image from camera or video that to be processed
    * @return none
    * @details none
    */
    void ArmorDetector::GetRoi(Mat &img)
    {
        Size img_size = img.size();
        Rect rect_tmp = roiRect;
        if (lostCnt>5||rect_tmp.width == 0|| rect_tmp.height == 0)
        {
            roiRect = Rect(0, 0, img_size.width, img_size.height);
        }
        else if(detectCnt>0)
        {
            float scale = 3;
            int w = int(rect_tmp.width * scale);
            int h = int(rect_tmp.height * scale);
            int x = int(rect_tmp.x - (w - rect_tmp.width) * 0.5);
            int y = int(rect_tmp.y - (h - rect_tmp.height) * 0.5);

            roiRect = Rect(x, y, w, h);
            //MakeRectSafe(roiRect, img.size());
            if (!MakeRectSafe(roiRect, img_size))
            {
                roiRect = Rect(0, 0, img_size.width, img_size.height);
            }
        }
    }

    /**
    * @brief make the rect safe
    * @param [rect] a rect may be not safe
    * @param [size] the image size, the biggest rect size
    * @return it will never be false
    * @details none
    */
    inline bool ArmorDetector::MakeRectSafe(cv::Rect &rect, const cv::Size& size)
    {
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x ;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y ;
        return !(rect.width <= 0 || rect.height <= 0);
    }


    /**
    * @brief track a armor
    * @param [src] the image from camera or video that to be processed
    * @param [target] tracker will be updated by this region
    * @return if tracker ever found armors, return true, otherwise return false
    * @details none
    */
    bool ArmorDetector::trackingTarget(Mat &src, Rect2d target)
    {
        auto pos = target;
        if (!tracker->update(src, target)) 
        {
            detectCnt = 0;
            return false;
        }
        if ((pos & cv::Rect2d(0, 0, 640, 480)) != pos)
        {
            detectCnt = 0;
            return false;
        }
        roiRect = Rect(pos.x - pos.width / 2.0, pos.y - pos.height / 2.0, pos.height * 2, pos.width * 2); //tracker��⵽Ŀ�꣬����roi
        MakeRectSafe(roiRect, Size(640, 480));
        detectCnt++;

        if (DetectArmor(src)) 
        {
            targetArmor.rect.x += roiRect.x;
            targetArmor.rect.y += roiRect.y;
            tracker = cv::TrackerKCF::create();
            tracker->init(src, targetArmor.rect);
            return true;
        }
        else
        {
            detectCnt = 0;
            return false;
        }
    }


    bool CompArmorPriority(const Armor &a, const Armor &b)
    {
        return a.priority < b.priority;
    }

    bool compMatchFactor(const MatchLight &a, const MatchLight &b)
    {
        return a.matchFactor < b.matchFactor;
    }
}
/*
 bool ArmorDetector::getTypeResult(bool is_small)
 {
	 if (history_.size() < filter_size_) {
		 history_.push_back(is_small);
	 }
	 else {
		 history_.push_back(is_small);
		 history_.pop_front();
	 }

 }

 */
/*

 // װ�װ������߼� �ⲿ���Ȳ���
 void ArmorFinder::run(cv::Mat& src) {
	 getsystime(frame_time); //����ȡ��ǰ֡ʱ��(�����㹻��ȷ)
 //    stateSearchingTarget(src);                    // for debug
 //    goto end;
	 switch (state) {
	 case SEARCHING_STATE:
		 if (stateSearchingTarget(src)) {
			 if ((target_box.rect & cv::Rect2d(0, 0, 640, 480)) == target_box.rect) { // �ж�װ�װ������Ƿ�����ͼ������
				 if (!classifier) {                                         ///* ���������������
					 cv::Mat roi = src(target_box.rect).clone(), roi_gray;  //        bool showLightBlobs = true;
/* ��ʹ��װ�������������ж��Ƿ����
					 cv::cvtColor(roi, roi_gray, CV_RGB2GRAY);
					 cv::threshold(roi_gray, roi_gray, 180, 255, cv::THRESH_BINARY);
					 contour_area = cv::countNonZero(roi_gray);
				 }
				 tracker = TrackerToUse::create();                       // �ɹ���Ѱ��װ�װ壬����tracker����
				 tracker->Init(src, target_box.rect);
				 state = TRACKING_STATE;
				 tracking_cnt = 0;
				 LOGM(STR_CTR(WORD_LIGHT_CYAN, "into track"));
			 }
		 }
		 break;
	 case TRACKING_STATE:
		 if (!stateTrackingTarget(src) || ++tracking_cnt > 100) {    // ���׷��100֡ͼ��
			 state = SEARCHING_STATE;
			 LOGM(STR_CTR(WORD_LIGHT_YELLOW, "into search!"));
		 }
		 break;
	 case STANDBY_STATE:
	 default:
		 stateStandBy(); // currently meaningless
	 }
 end:
	 if (is_anti_top) { // �жϵ�ǰ�Ƿ�Ϊ������ģʽ
		 antiTop();
	 }
	 else if (target_box.rect != cv::Rect2d()) {
		 anti_top_cnt = 0;
		 time_seq.clear();
		 angle_seq.Clear();
		 sendBoxPosition(0);
	 }

	 if (target_box.rect != cv::Rect2d()) {
		 last_box = target_box;
	 }

	 if (showArmorBox) {                 // ����������ʾ��ǰĿ��װ�װ�
		 showArmorBox("box", src, target_box);
		 cv::waitKey(1);
	 }
 }
 */
