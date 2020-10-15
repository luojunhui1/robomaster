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
    Armor::Armor(const LED_Stick &L1, const LED_Stick &L2)
    {
        error_angle = fabs(L1.light_angle - L2.light_angle);
        armor_width = fabs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
        armor_height = fabs(static_cast<int>((L1.rect.size.height + L1.rect.size.height) / 2));
        center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x) / 2);
        center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y) / 2);
        rect = Rect(center - Point2i(armor_width / 2, armor_height / 2), Size(armor_width, armor_height));
        armor_type = (armor_width / armor_height > 4) ? (BIG_ARMOR) : (SMALL_ARMOR);
        priority = fabs(center.x - 320)+fabs(center.y - 240);
    }

/**
	 * @brief: ArmorDetector definition
*/
    ArmorDetector::ArmorDetector()
    {
        find_state_ = false;
        is_small_ = false;
    }

/**
	Armor initialization
*/
    void Armor::init()
    {
        error_angle = 0;
        armor_width = 0;
        armor_height = 0;
        armor_type = BIG_ARMOR;
        priority = 10000;
    }

/**
	ArmorDector initialization
*/
    void ArmorDetector::init()
    {
        final_armors.clear();
        history.clear();
        history_.clear();
        last_armor.init();
        roi_rect = Rect(0,0,0,0);
        find_state_ = false;
        detect_cnt_ = 0;
        lost_cnt_ = 120;
        average_intensity = 0;
        distance_ = 0;
        last_distance = 0;
        angle_x_ = 0;
        angle_y_ = 0;
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
	* @param {Rect} [roi_rect][]
	* @return:
	* @details:
*/
    bool ArmorDetector::DetectArmor(Mat &img)
    {
        find_state_ = false;
        vector<LED_Stick> lights;
        img_roi = img(roi_rect);
        //get_average_intensity(img_roi);
        preprocess(img_roi);
        lights = light_detection(threshold_map);
        //cout<<"light"<<lights.size()<<endl;
        if (show_light_blobs)
        {
            for(int i=0;i<lights.size();i++) {
                Point2f rect_point[4]; //
                lights[i].rect.points(rect_point);
                for (int j = 0; j < 4; j++) {
                    line(img_roi, rect_point[j], rect_point[(j + 1) % 4], Scalar(255, 0, 255), 1);
                }
            }
        }
        imshow("roi",img_roi);
        max_match(lights);
        if (find_state_)
        {
            //cout << history.size() << endl;
            detect_cnt_++;
            lost_cnt_ = 0;
            sort(final_armors.begin(), final_armors.end(), compArmorPriority);
            last_armor = final_armors[0];
            //		if (detect_cnt_ > 5)
            //		{
            //			//history = AA.rect;
            //			lost_cnt_ = 0;
            //		}
            makeRectSafe(last_armor.rect, img.size());
            last_armor.rect  = last_armor.rect + Point(roi_rect.x,roi_rect.y);
            //cout<<"last_armor: "<<"("<<last_armor.rect.x<<","<<last_armor.rect.y<<")"<<" w:"<<last_armor.rect.width<<" h:"<<last_armor.rect.height<<endl;
            //cout<<"roi_rect"<<"("<<roi_rect.x<<","<<roi_rect.y<<")"<<" w:"<<roi_rect.width<<" h:"<<roi_rect.height<<endl;
            if (show_armor_box)
            {
                rectangle(img, last_armor.rect, Scalar(0, 125, 255), 2);
            }
            if (show_armor_boxes)
            {
                for (auto & final_armor : final_armors)
                {
                    final_armor.rect = final_armor.rect + Point(roi_rect.x,roi_rect.y);
                    makeRectSafe(final_armor.rect, img.size());
                    //makeRectSafe(final_armor.rect, img_roi.size());
                    rectangle(img, final_armor.rect, Scalar(0, 225, 255), 2);
                }
            }
            roi_rect = last_armor.rect;
            final_armors.clear();
            return true;
        }
        else
        {
            detect_cnt_ = 0;
            lost_cnt_++;
            final_armors.clear();
            return false;
        }
    }

/**
	   * @brief:get the average intensity of roi image
	   * @param {Mat} [img][]
	   * @return: the average intensity of roi image
	   * @details:
	   */
    int ArmorDetector::get_average_intensity(Mat &img)
    {
        Mat dst(img.size(), CV_8UC1);
        cvtColor(img, dst, COLOR_BGR2GRAY);
        average_intensity = static_cast<int>(mean(dst).val[0]);
        return average_intensity;
    }

/**
	 * @brief: match led sticks
	 * @param {vector<Light_Stick>} [LED][]
	 * @param {sizet_t} [i][]
	 * @param {sizet_t} [j][]
	 * @return: none
	 * @details:
	 */
    void ArmorDetector::max_match(vector<LED_Stick> lights)
    {
        static float yDiff, xDiff;
        static float nL, nW;
        static float dAngle;
        static float Contour_Len1;
        static float Contour_Len2;
        static float ratio;
        static float nAngle;
        vector<MATCHLIGHT> matchlights;
        float match_factor_;

        final_armors.clear();

        if (lights.size() < 2)
            return;
        for (unsigned int nI = 0; nI < lights.size() - 1; nI++)
        {
            for (unsigned int nJ = nI + 1; nJ < lights.size(); nJ++)
            {
                //the difference between to angles
                dAngle = fabs(lights[nI].light_angle - lights[nJ].light_angle);
                if(dAngle > param_.max_angle_error)continue;
                //cout<<"dangle:"<<dAngle<<endl;
                //the difference ratio of the two lights' height 
                Contour_Len1 = abs(lights[nI].rect.size.height - lights[nJ].rect.size.height) / max(lights[nI].rect.size.height, lights[nJ].rect.size.height);
                if(Contour_Len1 > param_.max_length_error)continue;
                //cout<<"Contour_Len1:"<<Contour_Len1<<endl;
                //the difference ratio of the two lights' width 
                //Contour_Len2 = abs(lights[nI].rect.size.width - lights[nI].rect.size.width) / max(lights[nI].rect.size.width, lights[nJ].rect.size.width);
                //cout<<"Contour_Len2:"<<Contour_Len2<<endl;
                //the average height of two lights(also the height of the armor defined by these two lights)
                nW = (lights[nI].rect.size.height + lights[nJ].rect.size.height) / 2; //װ�׵ĸ߶�
                //cout<<"nW:"<<nW<<endl;
                //the width between the center points of two lights
                nL = fabs(lights[nI].rect.center.x - lights[nJ].rect.center.x);
                //cout<<"nL:"<<nL<<endl;
                //anyway, the difference of the lights' angle is tiny,so anyone of them can be the angle of the armor 
                nAngle = fabs(lights[nI].light_angle);
                if(nAngle > param_.max_armor_angle)continue;
                //cout<<"nAngle:"<<nAngle<<endl;
                //the ddifference of the y coordinate of the two center points  
                yDiff = abs(lights[nI].rect.center.y - lights[nJ].rect.center.y) / nW;
                if(yDiff > param_.max_yDiff)continue;
                //cout<<"yDiff:"<<yDiff<<endl;
                //the ratio of the width and the height, must larger than 1 
                ratio = nL / nW;
                if(ratio > param_.max_ratio || ratio < param_.min_ratio)continue;
                //the match factor is still rough now, but it can help filter  the most possible target from  the detected armors 
                match_factor_ = dAngle + Contour_Len1 + Contour_Len2 + yDiff + MIN(fabs(ratio - 1.5), fabs(ratio - 3.5));
//			if (dAngle <= param_.max_angle_error && Contour_Len1 < param_.max_length_error  && yDiff < param_.max_yDiff && ratio < param_.max_ratio && ratio > param_.min_ratio && nAngle < param_.max_armor_angle)
//			{
                if (!find_state_)
                    find_state_ = true;
                matchlights.push_back(MATCHLIGHT{false, nI, nJ, match_factor_});
                //                  priority_=dAngle+Contour_Len1+Contour_Len2+yDiff+MIN(abs(ratio-1.5),abs(ratio-3.5));
                //					armor = Armor(lights[nI],lights[nJ]);
                //					final_armors.push_back(armor); 
            }
//		}
        }
        sort(matchlights.begin(), matchlights.end(), compMatchFactor);
        //matching the lights by the priority fractors
        for (int i = 0; i < matchlights.size(); i++)
        {

            if (!matchlights[i].used)
            {
                final_armors.emplace_back(lights[matchlights[i].match_index1], lights[matchlights[i].match_index2]);

                for (int j = i; j < matchlights.size(); j++)
                {
                    if (matchlights[j].used)
                        continue;
                    if (matchlights[j].match_index1 == matchlights[i].match_index1 ||
                        matchlights[j].match_index1 == matchlights[i].match_index2 ||
                        matchlights[j].match_index2 == matchlights[i].match_index1 ||
                        matchlights[j].match_index2 == matchlights[i].match_index2)
                    {
                        matchlights[j].used = true;
                    }
                }
            }
        }
        if (final_armors.empty())
            find_state_ = false;
    }

/**
	 * @brief: preprocession
	 * @param {Mat} [img][roi image]
	 * @return:
	 * @details:if average value in a region of the color_map is larger than 0, then we can inference that in this region the light is more possible to be red
*/
    void ArmorDetector::preprocess(Mat &img)
    {
        //gamma tranform
        Mat bright;
        vector<Mat> channels;
        split(img,channels);
        cvtColor(img,bright,CV_BGR2GRAY);//0,2,1
        //Attention!!!if the caculate result is small than 0, because the mat format is CV_UC3, it will be set as 0.
        b_r = Mat(channels[0] - channels[2]);
        r_b = Mat(channels[2] - channels[0]);
        threshold(bright,threshold_map,180,255,CV_MINMAX);
        color_map = r_b - b_r ;
        //LUT(img, img);
    }

/**
	 * @brief: detect and filter lights in img
	 * @param :img
	 * @return: a vector of possible led object
**/
    vector<LED_Stick> ArmorDetector::light_detection(Mat &img)
    {
        const Mat &roi_image = img;
        Mat hsv_binary;
        float angle_ = 0;
        Scalar_<double> avg;
        float stick_area;
//	if (hsv_mode)
//	{
//
//		if (blue_target)
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
        vector<LED_Stick> LED_Stick_v; 

//	Mat brightness, gray, binary_brightness_img, binary_color_img, process_img;
//	vector<Mat> BGRSplit; //channels subtract
//	split(roi_image, BGRSplit);
//	if (blue_target)
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

        // **��ȡ���ܵĵ���** -���õ������Ҷȣ���Χ����Ӧ��ɫ�Ĺ�Ȧ��Χ
        //    printf("bin_th = %d, color_th = %d\r\n", show_bin_th, show_color_th);
        if (show_bianryimg)
        {
            imshow("binary_brightness_img", threshold_map);
        }
        vector<vector<Point>> contours_light;
        findContours(threshold_map, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);
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
                if((stick_area > param_.max_bar_area) || (stick_area < param_.min_bar_area))continue;
                float rate_height2width = Likely_stick.size.height / Likely_stick.size.width;
                if((rate_height2width < param_.min_bar_height2width) || (rate_height2width > param_.max_bar_height2width))continue;
                angle_ = (Likely_stick.angle > 90.0f) ? (Likely_stick.angle - 180.0f) : (Likely_stick.angle);
                if(fabs(angle_) >= param_.max_bar_angle)continue;
                //height
//            float rate_height2width = Likely_stick.size.height / Likely_stick.size.width;
//            if((rate_height2width > param_.min_bar_height2width) && (rate_height2width < param_.max_bar_height2width))
                //cout<<"stick_area: "<<stick_area<<endl;
                //cout<<"rate_width_height: "<<rate_width_height<<endl;
//            if ((fabs(angle_) <= param_.max_bar_angle) && ((stick_area < param_.max_bar_area) && (stick_area > param_.min_bar_area)) && (rate_height2width > param_.min_bar_height2width) && (rate_height2width < param_.max_bar_height2width))
//            {
                mask = Mat::zeros(img.rows,img.cols,CV_8UC1);
                rectangle(mask,Likely_stick.boundingRect(),Scalar(1),-1);
                avg = mean(color_map,mask);
                if((blue_target&&avg[0]<0)||(!blue_target&&avg[0]>0))
                {
                    LED_Stick Build_stick_information(Likely_stick, angle_);
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
        Rect rect_tmp = roi_rect;//replace this
        if (detect_cnt_==0|| rect_tmp.width == 0 || rect_tmp.height == 0 || lost_cnt_ >= 120)
        {
            roi_rect = Rect(0, 0, img_size.width, img_size.height);
        }
        else if ((detect_cnt_ > 0))
        {
            float scale = 8;
//		if (lost_cnt_ == 30)
//		{
//			scale = 3;
//		}
//		else if(lost_cnt_ == 60)
//		{
//			scale = 4;
//		}
            int w = int(rect_tmp.width * scale);
            int h = int(rect_tmp.height * scale);
            int x = int(rect_tmp.x - (w - rect_tmp.width) * 0.5);
            int y = int(rect_tmp.y - (h - rect_tmp.height) * 0.5);
            //
            roi_rect = Rect(x, y, w, h);
            //makeRectSafe(roi_rect, img.size());
            if (!makeRectSafe(roi_rect, img_size))
            {
                roi_rect = Rect(0, 0, img_size.width, img_size.height);
            }
        }
    }

/**
	 * @brief: make the rect safe
	 * @param {Rect} [rect][a rect may be not safe]
	 *        {Size} [size][the image size, the biggest rect size]
	 * @return:
 */
    inline bool ArmorDetector::makeRectSafe(cv::Rect &rect, cv::Size size)
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
            detect_cnt_ = 0;
            return false;
        }
        if ((pos & cv::Rect2d(0, 0, 640, 480)) != pos)
        {
            detect_cnt_ = 0;
            return false;
        }
        roi_rect = Rect(pos.x - pos.width / 2.0, pos.y - pos.height / 2.0, pos.height * 2, pos.width * 2); //tracker��⵽Ŀ�꣬����roi
        makeRectSafe(roi_rect, Size(640, 480));
        detect_cnt_++;

        if (DetectArmor(src)) 
        {
            last_armor.rect.x += roi_rect.x;
            last_armor.rect.y += roi_rect.y;
            tracker = TrackerToUse::create();
            tracker->init(src, last_armor.rect);
            return true;
        }
        else
        {
            detect_cnt_ = 0;
            return false;
        }
    }


/*for sort*/
    bool compArmorPriority(const Armor &a, const Armor &b)
    {
        return a.priority < b.priority;
    }

/*for sort*/
    bool compMatchFactor(const MATCHLIGHT &a, const MATCHLIGHT &b)
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
					 cv::Mat roi = src(target_box.rect).clone(), roi_gray;  //        bool show_light_blobs = true;
/* ��ʹ��װ�������������ж��Ƿ����
					 cv::cvtColor(roi, roi_gray, CV_RGB2GRAY);
					 cv::threshold(roi_gray, roi_gray, 180, 255, cv::THRESH_BINARY);
					 contour_area = cv::countNonZero(roi_gray);
				 }
				 tracker = TrackerToUse::create();                       // �ɹ���Ѱ��װ�װ壬����tracker����
				 tracker->init(src, target_box.rect);
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
		 angle_seq.clear();
		 sendBoxPosition(0);
	 }

	 if (target_box.rect != cv::Rect2d()) {
		 last_box = target_box;
	 }

	 if (show_armor_box) {                 // ����������ʾ��ǰĿ��װ�װ�
		 showArmorBox("box", src, target_box);
		 cv::waitKey(1);
	 }
 }
 */
