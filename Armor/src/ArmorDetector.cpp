#include "ArmorDetector.hpp"
#include <sstream>
#include <algorithm>
namespace rm
{
/**
	 * @brief: Armor definition
	 * @param {LedStick} [L1][led stick]
	 * @param {LedStick} [L2][led stick]
	 * @return: none
	 * @details:
*/
    Armor::Armor(const LEDStick &L1, const LEDStick &L2)
    {
        errorAngle = fabs(L1.light_angle - L2.light_angle);
        armorWidth = fabs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
        armorHeight = fabs(static_cast<int>((L1.rect.size.height + L1.rect.size.height) / 2));
        center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x) / 2);
        center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y) / 2);
        rect = Rect(center - Point2i(armorWidth / 2, armorHeight / 2), Size(armorWidth, armorHeight));
        armorType = (armorWidth / armorHeight > 4) ? (BIG_ARMOR) : (SMALL_ARMOR);
        priority = fabs(center.x - 320)+fabs(center.y - 240);
    }

/**
	 * @brief: ArmorDetector definition
*/
    ArmorDetector::ArmorDetector()
    {
        findState = false;
        isSmall = false;
    }

/**
	Armor initialization
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
	ArmorDector initialization
*/
    void ArmorDetector::Init()
    {
        finalArmors.clear();
        history.clear();
        history_.clear();
        lastArmor.init();
        roiRect = Rect(0, 0, 0, 0);
        findState = false;
        detectCnt = 0;
        lostCnt = 120;
        averageIntensity = 0;

    }

/**
	 * @brief: Top level detection task
	 * @param {Mat} [img][the image from camera or video]
	 * @return: whether founf armor or not
	 * @details:
*/
    bool ArmorDetector::ArmorDetectTask(Mat &img)
    {
        GetRoi(img); //get roi
        return DetectArmor(img);
    }

/**
	* @brief: detect possible armors
	* @param {Mat} [img][]
	* @param {Rect} [roiRect][]
	* @return:
	* @details:
*/
    bool ArmorDetector::DetectArmor(Mat &img)
    {
        findState = false;
        vector<LEDStick> lights;
        imgRoi = img(roiRect);
        //GetAverageIntensity(imgRoi);
        Preprocess(imgRoi);
        lights = LightDetection(thresholdMap);
        //cout<<"light"<<lights.size()<<endl;
        if (showLightBlobs)
        {
            for(int i=0;i<lights.size();i++) {
                Point2f rect_point[4]; //
                lights[i].rect.points(rect_point);
                for (int j = 0; j < 4; j++) {
                    line(imgRoi, rect_point[j], rect_point[(j + 1) % 4], Scalar(255, 0, 255), 1);
                }
            }
        }
        imshow("roi", imgRoi);
        MaxMatch(lights);
        if (findState)
        {
            //cout << history.size() << endl;
            detectCnt++;
            lostCnt = 0;
            //sort(finalArmors.begin(), finalArmors.end(), CompArmorPriority);
            lastArmor = finalArmors[0];
            //		if (detectCnt > 5)
            //		{
            //			//history = AA.rect;
            //			lostCnt = 0;
            //		}
            MakeRectSafe(lastArmor.rect, img.size());
            lastArmor.rect  = lastArmor.rect + Point(roiRect.x, roiRect.y);
            //cout<<"lastArmor: "<<"("<<lastArmor.rect.x<<","<<lastArmor.rect.y<<")"<<" w:"<<lastArmor.rect.width<<" h:"<<lastArmor.rect.height<<endl;
            //cout<<"roiRect"<<"("<<roiRect.x<<","<<roiRect.y<<")"<<" w:"<<roiRect.width<<" h:"<<roiRect.height<<endl;
            if (showArmorBox)
            {
                rectangle(img, lastArmor.rect, Scalar(0, 125, 255), 2);
            }
            if (showArmorBoxes)
            {
                for (auto & final_armor : finalArmors)
                {
                    final_armor.rect = final_armor.rect + Point(roiRect.x, roiRect.y);
                    MakeRectSafe(final_armor.rect, img.size());
                    //MakeRectSafe(final_armor.rect, imgRoi.size());
                    rectangle(img, final_armor.rect, Scalar(0, 225, 255), 2);
                }
            }
            roiRect = lastArmor.rect;
            finalArmors.clear();
            return true;
        }
        else
        {
            detectCnt = 0;
            lostCnt++;
            finalArmors.clear();
            return false;
        }
    }

/**
	   * @brief:get the average intensity of roi image
	   * @param {Mat} [img][]
	   * @return: the average intensity of roi image
	   * @details:
	   */
    int ArmorDetector::GetAverageIntensity(Mat &img)
    {
        Mat dst;
        cvtColor(img, dst, COLOR_BGR2GRAY);
        averageIntensity = static_cast<int>(mean(dst).val[0]);
        return averageIntensity;
    }

/**
	 * @brief: match led sticks
	 * @param {vector<Light_Stick>} [LED][]
	 * @param {sizet_t} [i][]
	 * @param {sizet_t} [j][]
	 * @return: none
	 * @details:
	 */
    void ArmorDetector::MaxMatch(vector<LEDStick> lights)
    {
        static float yDiff, xDiff;
        static float nL, nW;
        static float dAngle;
        static float contourLen1;
        static float contourLen2;
        static float ratio;
        static float nAngle;
        vector<MatchLight> matchLights;
        float match_factor_;

        finalArmors.clear();

        if (lights.size() < 2)
            return;
        for (unsigned int i = 0; i < lights.size() - 1; i++)
        {
            for (unsigned int j = i + 1; j < lights.size(); j++)
            {
                //the difference between to angles
                dAngle = fabs(lights[i].light_angle - lights[j].light_angle);
                if(dAngle > param.maxAngleError)continue;
                //cout<<"dangle:"<<dAngle<<endl;
                //the difference ratio of the two lights' height 
                contourLen1 = abs(lights[i].rect.size.height - lights[j].rect.size.height) / max(lights[i].rect.size.height, lights[j].rect.size.height);
                if(contourLen1 > param.maxLengthError)continue;
                //cout<<"contourLen1:"<<contourLen1<<endl;
                //the difference ratio of the two lights' width 
                //contourLen2 = abs(lights[i].rect.size.width - lights[i].rect.size.width) / max(lights[i].rect.size.width, lights[j].rect.size.width);
                //cout<<"contourLen2:"<<contourLen2<<endl;
                //the average height of two lights(also the height of the armor defined by these two lights)
                nW = (lights[i].rect.size.height + lights[j].rect.size.height) / 2; //װ�׵ĸ߶�
                //cout<<"nW:"<<nW<<endl;
                //the width between the center points of two lights
                nL = fabs(lights[i].rect.center.x - lights[j].rect.center.x);
                //cout<<"nL:"<<nL<<endl;
                //anyway, the difference of the lights' angle is tiny,so anyone of them can be the angle of the armor 
                nAngle = fabs(lights[i].light_angle);
                if(nAngle > param.maxArmorAngle)continue;
                //cout<<"nAngle:"<<nAngle<<endl;
                //the ddifference of the y coordinate of the two center points  
                yDiff = abs(lights[i].rect.center.y - lights[j].rect.center.y) / nW;
                if(yDiff > param.maxYDiff)continue;
                //cout<<"yDiff:"<<yDiff<<endl;
                //the ratio of the width and the height, must larger than 1 
                ratio = nL / nW;
                if(ratio > param.maxRatio || ratio < param.minRatio)continue;
                //the match factor is still rough now, but it can help filter  the most possible target from  the detected armors 
                match_factor_ = dAngle + contourLen1 + contourLen2 + yDiff + MIN(fabs(ratio - 1.5), fabs(ratio - 3.5));
//			    if (dAngle <= param.maxAngleError && contourLen1 < param.maxLengthError  && yDiff < param.maxYDiff && ratio < param.maxRatio && ratio > param.minRatio && nAngle < param.maxArmorAngle)
//			    {
                if (!findState)
                    findState = true;
                matchLights.push_back(MatchLight {false, i, j, match_factor_});
//              priority_=dAngle+contourLen1+contourLen2+yDiff+MIN(abs(ratio-1.5),abs(ratio-3.5));
//				armor = Armor(lights[i],lights[j]);
//				finalArmors.push_back(armor);
            }
//		}
        }
        sort(matchLights.begin(), matchLights.end(), compMatchFactor);
        //matching the lights by the priority fractors
        for (int i = 0; i < matchLights.size(); i++)
        {

            if (!matchLights[i].used)
            {
                finalArmors.emplace_back(lights[matchLights[i].match_index1], lights[matchLights[i].match_index2]);

                for (int j = i; j < matchLights.size(); j++)
                {
                    if (matchLights[j].used)
                        continue;
                    if (matchLights[j].match_index1 == matchLights[i].match_index1 ||
                        matchLights[j].match_index1 == matchLights[i].match_index2 ||
                        matchLights[j].match_index2 == matchLights[i].match_index1 ||
                        matchLights[j].match_index2 == matchLights[i].match_index2)
                    {
                        matchLights[j].used = true;
                    }
                }
            }
        }
        if (finalArmors.empty())
            findState = false;
    }

/**
	 * @brief: preprocession
	 * @param {Mat} [img][roi image]
	 * @return:
	 * @details:if average value in a region of the colorMap is larger than 0, then we can inference that in this region the light is more possible to be red
*/
    void ArmorDetector::Preprocess(Mat &img)
    {
        //gamma tranform
        Mat bright;
        vector<Mat> channels;
        split(img,channels);
        cvtColor(img,bright,CV_BGR2GRAY);//0,2,1
        //Attention!!!if the caculate result is small than 0, because the mat format is CV_UC3, it will be set as 0.
        bSubR = Mat(channels[0] - channels[2]);
        rSubB = Mat(channels[2] - channels[0]);
        threshold(bright, thresholdMap, 180, 255, CV_MINMAX);
        colorMap = rSubB - bSubR ;
        //LUT(img, img);
    }

/**
	 * @brief: detect and filter lights in img
	 * @param :img
	 * @return: a vector of possible led object
**/
    vector<LEDStick> ArmorDetector::LightDetection(Mat &img)
    {
        const Mat &roi_image = img;
        Mat hsv_binary;
        float angle_ = 0;
        Scalar_<double> avg;
        float stick_area;
    //	if (hsvMode)
    //	{
    //
    //		if (blueTarget)
    //		{
    //			cv::Mat img_hsv_blue, img_threshold_blue;
    //			cvtColor(roi_image, img_hsv_blue, COLOR_BGR2HSV);
    //			cv::Mat blue_low(cv::Scalar(90, 150, 46));
    //			cv::Mat blue_higher(cv::Scalar(140, 255, 255));
    //			cv::inRange(img_hsv_blue, blue_low, blue_higher, img_threshold_blue);
    //			Mat element = getStructuringElement(MORPH_RECT, Size(1, 1));
    //			morphologyEx(img_threshold_blue, hsv_binary, MORPH_OPEN, element);
    //		}
    //		else
    //		{
    //			cv::Mat img_hsv_red1, img_hsv_red2, img_threshold_red, img_threshold_red1, img_threshold_red2;
    //			cvtColor(roi_image, img_hsv_red1, COLOR_BGR2HSV);
    //			cvtColor(roi_image, img_hsv_red2, COLOR_BGR2HSV);
    //			cv::Mat red1_low(cv::Scalar(0, 43, 46));
    //			cv::Mat red1_higher(cv::Scalar(3, 255, 255));
    //			cv::Mat red2_low(cv::Scalar(170, 43, 46));
    //			cv::Mat red2_higher(cv::Scalar(180, 255, 255));
    //			cv::inRange(img_hsv_red1, red1_low, red1_higher, img_threshold_red1);
    //			cv::inRange(img_hsv_red2, red2_low, red2_higher, img_threshold_red2);
    //			img_threshold_red = img_threshold_red1 | img_threshold_red2;
    //			Mat element = getStructuringElement(MORPH_RECT, Size(1, 1));
    //			morphologyEx(img_threshold_red, hsv_binary, MORPH_OPEN, element);
    //		}
    //	}
        vector<LEDStick> LED_Stick_v;

    //	Mat brightness, gray, binary_brightness_img, binary_color_img, process_img;
    //	vector<Mat> BGRSplit; //channels subtract
    //	split(roi_image, BGRSplit);
    //	if (blueTarget)
    //	{
    //		BGRSplit[2] = BGRSplit[0] - BGRSplit[2];
    //		Mat dst1[] = {BGRSplit[0] * 2, BGRSplit[1], BGRSplit[0]}; //����ͨ��λ�ã��Ҷ�ת��greenͨ�����,red���
    //		merge(dst1, 3, process_img);
    //		cvtColor(process_img, gray, COLOR_BGR2GRAY);
    //	}
    //	else
    //	{
    //		BGRSplit[2] = BGRSplit[2] - BGRSplit[0];
    //		Mat dst1[] = {BGRSplit[2] * 5, BGRSplit[2] + 50, BGRSplit[1]}; //����ͨ��λ�ã��Ҷ�ת��greenͨ�����,red���
    //		merge(dst1, 3, process_img);
    //		cvtColor(process_img, gray, COLOR_BGR2GRAY);
    //	}
    //	//cvtColor(roi_image, gray, COLOR_BGR2GRAY);
    //	//Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    //	//vector<cv::Mat> bgr;
    //	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    //
    //	//GaussianBlur(gray, gray, Size(3, 3),1,1);
    //	threshold(gray, brightness, 100, 255, THRESH_BINARY);
    //	//dilate(brightness, binary_brightness_img, kernel1, Point(-1, -1), 1);
    //	morphologyEx(binary_brightness_img, binary_brightness_img, MORPH_CLOSE, kernel);
    //	morphologyEx(hsv_binary, binary_color_img, MORPH_CLOSE, kernel);

    //    printf("bin_th = %d, color_th = %d\r\n", show_bin_th, show_color_th);
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
                //
                //cout<<"contours_brightness: "<<contours_brightness[ii].size()<<endl;
                RotatedRect Likely_stick = fitEllipse(i);
                stick_area = Likely_stick.size.width * Likely_stick.size.height;
                if((stick_area > param.maxLightArea) || (stick_area < param.minLightArea))continue;
                float rate_height2width = Likely_stick.size.height / Likely_stick.size.width;
                if((rate_height2width < param.minLightW2H) || (rate_height2width > param.maxLightW2H))continue;
                angle_ = (Likely_stick.angle > 90.0f) ? (Likely_stick.angle - 180.0f) : (Likely_stick.angle);
                if(fabs(angle_) >= param.maxLightAngle)continue;
                //height
//            float rate_height2width = Likely_stick.size.height / Likely_stick.size.width;
//            if((rate_height2width > param.minLightW2H) && (rate_height2width < param.maxLightW2H))
                //cout<<"stick_area: "<<stick_area<<endl;
                //cout<<"rate_width_height: "<<rate_width_height<<endl;
//            if ((fabs(angle_) <= param.maxLightAngle) && ((stick_area < param.maxLightArea) && (stick_area > param.minLightArea)) && (rate_height2width > param.minLightW2H) && (rate_height2width < param.maxLightW2H))
//            {
                mask = Mat::zeros(img.rows,img.cols,CV_8UC1);
                rectangle(mask,Likely_stick.boundingRect(),Scalar(1),-1);
                avg = mean(colorMap, mask);
                if((blueTarget && avg[0] < 0) || (!blueTarget && avg[0] > 0))
                {
                    LEDStick Build_stick_information(Likely_stick, angle_);
                    LED_Stick_v.push_back(Build_stick_information);
                }
            }
//        }
        }
        return LED_Stick_v;
    }

/**
	 * @brief: get he region of interest
	 * @param {Mat} [img][]
	 * @return: none
	 * @details:
*/
    void ArmorDetector::GetRoi(Mat &img)
    {
        Size img_size = img.size();
        Rect rect_tmp = roiRect;//replace this
        if (detectCnt == 0 || rect_tmp.width == 0 || rect_tmp.height == 0 || lostCnt >= 120)
        {
            roiRect = Rect(0, 0, img_size.width, img_size.height);
        }
        else if ((detectCnt > 0))
        {
            float scale = 8;
//		if (lostCnt == 30)
//		{
//			scale = 3;
//		}
//		else if(lostCnt == 60)
//		{
//			scale = 4;
//		}
            int w = int(rect_tmp.width * scale);
            int h = int(rect_tmp.height * scale);
            int x = int(rect_tmp.x - (w - rect_tmp.width) * 0.5);
            int y = int(rect_tmp.y - (h - rect_tmp.height) * 0.5);
            //
            roiRect = Rect(x, y, w, h);
            //MakeRectSafe(roiRect, img.size());
            if (!MakeRectSafe(roiRect, img_size))
            {
                roiRect = Rect(0, 0, img_size.width, img_size.height);
            }
        }
    }

/**
	 * @brief: make the rect safe
	 * @param {Rect} [rect][a rect may be not safe]
	 *        {Size} [size][the image size, the biggest rect size]
	 * @return:
 */
    inline bool ArmorDetector::MakeRectSafe(cv::Rect &rect, cv::Size size)
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
	 * @brief: track a armor
	 * @param {Mat} [src][NULL]
	 *        {Rect2d} [target][tracker will be updated by this region]
	 * @return: if tracking action succeseful
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
            lastArmor.rect.x += roiRect.x;
            lastArmor.rect.y += roiRect.y;
            tracker = TrackerToUse::create();
            tracker->init(src, lastArmor.rect);
            return true;
        }
        else
        {
            detectCnt = 0;
            return false;
        }
    }


/*for sort*/
    bool CompArmorPriority(const Armor &a, const Armor &b)
    {
        return a.priority < b.priority;
    }

/*for sort*/
    bool compMatchFactor(const MatchLight &a, const MatchLight &b)
    {
        return a.match_factor < b.match_factor;
    }
} // namespace rm

/**
 * @brief: װ�װ����ͱ���
 * @param {bool} [is_small][�Ƿ�ΪСװ�װ�]
 * @return:
 * @details: װ�װ����ͱ��档
 */
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
