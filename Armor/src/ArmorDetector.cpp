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

    Armor::Armor(const LEDStick &L1, const LEDStick &L2, double priority_)
    {
        errorAngle = fabs(L1.lightAngle - L2.lightAngle);
        armorWidth = fabs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
        armorHeight = fabs(static_cast<int>((L1.rect.size.height + L2.rect.size.height) / 2));
        center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x) / 2);
        center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y) / 2);
        rect = Rect(center - Point2i(armorWidth / 2, armorHeight / 2), Size(armorWidth, armorHeight));
        armorType = (armorWidth / armorHeight > 3.5) ? (BIG_ARMOR) : (SMALL_ARMOR);
        priority = priority_;

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

        targetArmor.init();
        roiRect = Rect(0, 0, FRAMEWIDTH, FRAMEHEIGHT);
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
#if USEROI == 1
        GetRoi(); //get roi
#endif
        return DetectArmor(img);
    }

    bool ArmorDetector::ArmorDetectTaskGPU(Mat &img)
    {
#if USEROI == 1
        GetRoi(); //get roi
#endif
        return DetectArmorGPU(img);
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

        if (showBianryImg)
        {
            imshow("binary_brightness_img", thresholdMap);
        }

        lights = LightDetection(thresholdMap);

        if (showLamps)
        {
            for(auto & light : lights) {
                Point2f rect_point[4]; //
                light.rect.points(rect_point);
                for (int j = 0; j < 4; j++) {
                    line(imgRoi, rect_point[j], rect_point[(j + 1) % 4], Scalar(0, 255, 255), 2);
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
            targetArmor.center +=  Point(roiRect.x, roiRect.y);

            for(int i = 0; i< 4;i++)
            {
                targetArmor.pts[i] = targetArmor.pts[i] + Point2f(roiRect.x, roiRect.y);
            }

            if (showArmorBoxes)
            {
                for (auto & final_armor : possibleArmors)
                {
                    final_armor.rect = final_armor.rect + Point(roiRect.x, roiRect.y);
                    MakeRectSafe(final_armor.rect, img.size());
                    //MakeRectSafe(final_armor.rect, imgRoi.size());
                    rectangle(img, final_armor.rect, Scalar(255, 0, 0), 2);
                    //circle(img,final_armor.center + Point(roiRect.x, roiRect.y),5,Scalar(255,0,0),-1);
                    //putText(img,std::to_string(final_armor.priority),final_armor.center,FONT_HERSHEY_COMPLEX,0.7,Scalar(121,121,255),1);
                }
            }

            if (showArmorBox)
            {
                rectangle(img, targetArmor.rect, Scalar(0, 0, 255), 5);
            }
#if USEROI == 1
            roiRect = targetArmor.rect;
#endif
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

    bool ArmorDetector::DetectArmorGPU(Mat &img)
    {
        findState = false;

        vector<LEDStick> lights;

        gpuImg.upload(img,stream);

        gpuRoiImg = cuda::GpuMat(gpuImg,roiRect);

        PreprocessGPU(gpuRoiImg);

        gpuEdge.download(thresholdMap,stream);

        /*the code below is just using CPU*/
        lights = LightDetection(thresholdMap);

        if (showBianryImg)
        {
            imshow("binary_brightness_img", thresholdMap);
        }

        if (showLamps)
        {
            for(auto & light : lights) {
                Point2f rect_point[4]; //
                light.rect.points(rect_point);
                for (int j = 0; j < 4; j++) {
                    line(imgRoi, rect_point[j], rect_point[(j + 1) % 4], Scalar(0, 255, 255), 2);
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
            targetArmor.center +=  Point(roiRect.x, roiRect.y);

            for(int i = 0; i< 4;i++)
            {
                targetArmor.pts[i] = targetArmor.pts[i] + Point2f(roiRect.x, roiRect.y);
            }

            if (showArmorBoxes)
            {
                for (auto & final_armor : possibleArmors)
                {
                    final_armor.rect = final_armor.rect + Point(roiRect.x, roiRect.y);
                    MakeRectSafe(final_armor.rect, img.size());
                    //MakeRectSafe(final_armor.rect, imgRoi.size());
                    rectangle(img, final_armor.rect, Scalar(255, 0, 0), 2);
                    //circle(img,final_armor.center + Point(roiRect.x, roiRect.y),5,Scalar(255,0,0),-1);
                    putText(img,std::to_string(final_armor.priority),final_armor.center,FONT_HERSHEY_COMPLEX,0.7,Scalar(121,121,255),1);
                }
            }

            if (showArmorBox)
            {
                rectangle(img, targetArmor.rect, Scalar(0, 0, 255), 5);
            }
#if USEROI == 1
            roiRect = targetArmor.rect;
#endif
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
        static float dAvgB;
        static float sizeRatio;

//        double netOutput = -1, maxNetPre = -1;
//        int netSelect = 0;

        vector<MatchLight> matchLights;

//        matchParams.clear();
//        MatchParam matchParam;

        static MatchLight matchLight;
        int matchCount = 0;
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
                contourLen2 = abs(lights[i].rect.size.width - lights[j].rect.size.width) / max(lights[i].rect.size.width, lights[j].rect.size.width);
                //cout<<"contourLen2:"<<contourLen2<<endl;

                /*the average height of two lights(also the height of the armor defined by these two lights)*/
                nW = (lights[i].rect.size.height + lights[j].rect.size.height) / 2;
                //cout<<"nW:"<<nW<<endl;

                /*the width between the center points of two lights*/
                nL = fabs(lights[i].rect.center.x - lights[j].rect.center.x);
                //cout<<"nL:"<<nL<<endl;

                /*anyway, the difference of the lights' angle is tiny,so anyone of them can be the angle of the armor*/
                nAngle = fabs((lights[i].lightAngle + lights[j].lightAngle)/2);
                if(nAngle > param.maxArmorAngle)continue;
                //cout<<"nAngle:"<<nAngle<<endl;

                /*the ddifference of the y coordinate of the two center points*/
                yDiff = abs(lights[i].rect.center.y - lights[j].rect.center.y) / nW;
                if(yDiff > param.maxYDiff)continue;
                //cout<<"yDiff:"<<yDiff<<endl;

                /*the ratio of the width and the height, must larger than 1 */
                ratio = nL / nW;
                if(ratio > param.maxRatio || ratio < param.minRatio)continue;

                /*difference of average brightness*/
                dAvgB = abs(lights[i].avgBrightness - lights[j].avgBrightness);

                /*ratio of the two lamp's area, larger than 1*/
                sizeRatio = lights[i].size/lights[j].size;

                sizeRatio = (sizeRatio > 1)?(sizeRatio):(1/sizeRatio);

                /*the match factor is still rough now, but it can help filter  the most possible target from  the detected armors*/
                /*Of course, we need to find a formula that is more reasonable*/
                match_factor_ = sizeRatio +contourLen1  + dAvgB + exp(dAngle + yDiff + MIN(fabs(ratio - 1.5), fabs(ratio - 3.5)));

                if (!findState)
                    findState = true;

                matchLight = MatchLight (false, i, j, match_factor_);
//                matchParam = MatchParam(dAngle,contourLen1,contourLen2,ratio,nAngle,yDiff,dAvgB,sizeRatio);

                matchLights.push_back(matchLight);
//                matchParams.push_back(matchParam);

                /*clip image and extract armor from it and display window for every armor*/
                //Armor curArmor = Armor(lights[matchLight.matchIndex1],lights[matchLight.matchIndex2],matchLight.matchFactor);
//                netOutput = net.Fit(dAngle,contourLen1,contourLen2,ratio,nAngle,yDiff,dAvgB,sizeRatio);
//                if(netOutput > maxNetPre)
//                {
//                    maxNetPre = netOutput;
//                    netSelect = matchCount;
//                }
//
//                printf("==============================================\n");
//                printf("Match Count %d\n",matchCount++);
//
//                printf("dAngle: %f\n",dAngle);
//                printf("contourlen1: %f\n", contourLen1);
//                printf("contourlen2: %f\n", contourLen2);
//                printf("Ratio: %f\n", ratio);
//                printf("nAngle: %f\n", nAngle);
//                printf("yDiff: %f\n", yDiff);
//                printf("dAvgB: %f\n", dAvgB);
//                printf("Armor Type Error: %f\n", exp(MIN(fabs(ratio - 1.5), fabs(ratio - 3.5))));
//                printf("Match Fractor: %f\n", match_factor_);
//                //printf("Net Output: %lf\n",  net.Fit(dAngle,contourLen1,contourLen2,ratio,nAngle,yDiff,dAvgB,sizeRatio));
//                printf("===============================================\n");
            }

        }

        /*sort these pairs of lamps by match factor*/
        sort(matchLights.begin(), matchLights.end(), compMatchFactor);
//        Armor curArmor = Armor(lights[matchLights[netSelect].matchIndex1],lights[matchLights[netSelect].matchIndex2],matchLights[netSelect].matchFactor);
//        imshow("Net Selected",imgRoi(curArmor.rect));

        /*matching the lights by the priority factors*/
        for (int i = 0; i < matchLights.size(); i++)
        {
            if (!matchLights[i].used)
            {
                possibleArmors.emplace_back(lights[matchLights[i].matchIndex1], lights[matchLights[i].matchIndex2],matchLights[i].matchFactor);

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
    * @param [type] choose to preprocess the current image or the lastest two images, when the type is true, parameter
    * img must be the origin image but not the roi image
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

        //imshow("gray",bright);

        //Attention!!!if the caculate result is small than 0, because the mat format is CV_UC3, it will be set as 0.
        bSubR = Mat(channels[0] - channels[2]);
        rSubB = Mat(channels[2] - channels[0]);

        GaussianBlur(bright,bright,Size(5,5),3);
        threshold(bright, thresholdMap, 180, 255, CV_MINMAX);
        //Canny(thresholdMap,thresholdMap,100,200);
        colorMap = rSubB - bSubR ;

    }

    void ArmorDetector::PreprocessGPU(cuda::GpuMat& img)
    {
        img.convertTo(gpuRoiImgFloat,CV_32F,stream);

        cuda::split(gpuRoiImgFloat,gpuRoiImgVector,stream);

        cuda::cvtColor(gpuRoiImgFloat,gpuGray,CV_BGR2GRAY,0,stream);

        cuda::subtract(gpuRoiImgVector[0],gpuRoiImgVector[2],gpuBSubR,noArray(),-1,stream);

        cuda::subtract(gpuRoiImgVector[2],gpuRoiImgVector[0],gpuRSubB,noArray(),-1,stream);

        gauss->apply(gpuGray,gpuBlur);

        cuda::threshold(gpuBlur,gpuBright,180,255,THRESH_BINARY,stream);

        gpuBright.convertTo(gpuEdge,CV_8UC1,stream);

        canny_edg->detect(gpuEdge, gpuEdge,stream);

    }
    /**
    * @brief detect and filter lights in img
    * @param img
    * @return a vector of possible led object
    **/
    vector<LEDStick> ArmorDetector::LightDetection(Mat& img)
    {
        Mat hsv_binary,lampImage;
        float angle_ = 0;
        Scalar_<double> avg,avgBrghtness;
        float stick_area;
        Rect rectLamp;
        vector<LEDStick> LED_Stick_v;

        vector<vector<Point>> contours_light;

        /*
         * =============================================================================================================
         * There are not any OpenCV cuda function that is similar to findContours(), but someone suggest us to feed
         * the function with the data from GPU Image's canny or Sobel edge. We can try and debug this!
         * =============================================================================================================
         * */
        findContours(img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        for (auto & i : contours_light)
        {
            if (i.size() < 5)
                continue;

            double length = arcLength(i, true);

            if (length > 10 && length < 4000)
            {
                RotatedRect Likely_stick = fitEllipse(i);
                stick_area = Likely_stick.size.width * Likely_stick.size.height;
                if((stick_area > param.maxLightArea) || (stick_area < param.minLightArea))continue;
                float rate_height2width = Likely_stick.size.height / Likely_stick.size.width;
                if((rate_height2width < param.minLightW2H) || (rate_height2width > param.maxLightW2H))continue;

                angle_ = (Likely_stick.angle > 90.0f) ? (Likely_stick.angle - 180.0f) : (Likely_stick.angle);

                if(fabs(angle_) >= param.maxLightAngle)continue;

                rectLamp = Likely_stick.boundingRect();
                MakeRectSafe(rectLamp,img.size());
                mask = Mat::ones(rectLamp.height,rectLamp.width,CV_8UC1);

                /*Add this to make sure numbers on armors will not be recognized as lamps*/
                lampImage = img(rectLamp);
                avgBrghtness = mean(lampImage,mask);

//                lampImage = colorMap(rectLamp);
//                avg = mean(lampImage, mask);

                avg = Scalar_<double>(1);

                if((blueTarget && avg[0] < 0) || (!blueTarget && avg[0] > 0))
                {
                    LEDStick Build_stick_information(Likely_stick, angle_,avgBrghtness[0],stick_area);
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
    void ArmorDetector::GetRoi()
    {
        Rect rect_tmp = roiRect;
        if (lostCnt>3||rect_tmp.width == 0|| rect_tmp.height == 0)
        {
            roiRect = Rect(0, 0,FRAMEWIDTH, FRAMEHEIGHT);
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
            if (!MakeRectSafe(roiRect, Size(FRAMEWIDTH, FRAMEHEIGHT)))
            {
                roiRect = Rect(0, 0, FRAMEHEIGHT, FRAMEWIDTH);
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
        if(rect.x >= size.width || rect.y >= size.height)rect = Rect(0,0,0,0);
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
        if ((pos & cv::Rect2d(0, 0, FRAMEWIDTH, FRAMEHEIGHT)) != pos)
        {
            detectCnt = 0;
            return false;
        }
#if USEROI == 1
        roiRect = Rect(pos.x - pos.width , pos.y - pos.height , 3*pos.width, 3*pos.height); //tracker

        MakeRectSafe(roiRect, Size(FRAMEWIDTH, FRAMEHEIGHT));
#endif
        if (DetectArmor(src))
        {
            detectCnt++;
            return true;
        }
        else
        {
            detectCnt = 0;
            return false;
        }
    }

    /**
     * @brief initialize the ArmorCompare instance
     * @param none
     * @return none
     */
    void ArmorCompare::InitCompare()
    {
        Init();
        Mat lastBright = Mat(FRAMEHEIGHT,FRAMEWIDTH,CV_32F,Scalar(0));
        Mat dBright = Mat(FRAMEHEIGHT,FRAMEWIDTH,CV_32F,Scalar(0));
        dBrightGPU.upload(dBright,stream2);
        lastBrightGPU.upload(lastBright,stream2);
        //alg_farn = cv::cuda::FarnebackOpticalFlow::create();
    }

    /**
     * @brief preprocess, inherit from Preprocess() at 437 and almost the same with it
     * @param img image
     * @return none
     * @details ignore the attention at line 645, because the code was casually added in class Detector at the beginning
     * of the time, and we still keep the attention in our code because it reminds us: never change your code casually,
     * because you may feel very painful when you try to analysis and decoupling your code.
     */
    void ArmorCompare::PreprocessGPU(cuda::GpuMat &img)
    {
        img.convertTo(imgFloatGPU,CV_32F,stream2);
        cuda::cvtColor(imgFloatGPU,dBrightGPU,CV_BGR2GRAY,0,stream2);
        cuda::subtract(dBrightGPU,lastBrightGPU,opfFloatGpu);
        opfFloatGpu.convertTo(opfGpuX,CV_8UC1,stream2);
        cuda::threshold(opfGpuX,opfGpuX,140,255,THRESH_BINARY,stream2);
//
//        alg_farn->calc(lastBrightGPU,dBrightGPU,opfFloatGpu,stream2);
//        cuda::split(opfFloatGpu,opfVector,stream2);
//
//        cuda::normalize(opfVector[0],opfVector[0],0,255,NORM_MINMAX,-1,noArray(),stream2);
//
//        cuda::normalize(opfVector[1],opfVector[1],0,255,NORM_MINMAX,-1,noArray(),stream2);
//
//        gauss->apply(opfVector[0],opfVector[0]);
//
//        opfVector[0].convertTo(opfGpuX,CV_8UC1,stream2);
//        //opfVector[1].convertTo(opfGpuY,CV_8UC1,stream2);
//
//        cuda::threshold(opfGpuX,opfGpuX,180,255,THRESH_BINARY,stream2);
        //cuda::equalizeHist(opfGpuX,opfGpuX,stream2);

//        Mat bright;
//        vector<Mat> channels;
//        split(img,channels);
//        cvtColor(img,bright,CV_BGR2GRAY);//0,2,1
//
//        //Attention!!!if the caculate result is small than 0, because the mat format is CV_UC3, it will be set as 0.
//        bSubR = Mat(channels[0] - channels[2]);
//        rSubB = Mat(channels[2] - channels[0]);
//        colorMap = rSubB - bSubR ;
//
//        dBright = bright - lastBright;
//        normalize(dBright,dBright,0,255,NORM_MINMAX);
//
//        GaussianBlur(dBright,dBright,Size(5,5),3);
//        threshold(dBright, thresholdMap, 180, 255, CV_MINMAX);

        lastBrightGPU = dBrightGPU.clone();
    }

    /**
     * @brief inherit from DetectArmor() at 143 and almost the same with it
     * @param img
     * @return find target armor or not
     */
    bool ArmorCompare::DetectArmor(Mat &img)
    {
        findState = false;

        vector<LEDStick> lights;

        imgGPU.upload(img,stream2);

        PreprocessGPU(imgGPU);

        opfGpuX.download(optImgX,stream2);

        lights = LightDetection(optImgX);

        if (showBianryImg)
        {
            imshow("optical flow X", optImgX);
            //imshow("optical flow Y", optImgY);

        }

        if (showLamps)
        {
            for(auto & light : lights) {
                Point2f rect_point[4]; //
                light.rect.points(rect_point);
                for (int j = 0; j < 4; j++) {
                    line(img, rect_point[j], rect_point[(j + 1) % 4], Scalar(0, 255, 255), 2);
                }
            }
        }

        MaxMatch(lights);

        if(findState)
        {
            targetArmor = possibleArmors[0];

            if (showArmorBoxes)
            {
                for (auto & final_armor : possibleArmors)
                {
                    final_armor.rect = final_armor.rect + Point(roiRect.x, roiRect.y);
                    MakeRectSafe(final_armor.rect, img.size());
                    //MakeRectSafe(final_armor.rect, imgRoi.size());
                    rectangle(img, final_armor.rect, Scalar(255, 0, 0), 2);
                    //circle(img,final_armor.center + Point(roiRect.x, roiRect.y),5,Scalar(255,0,0),-1);
                    //putText(img,std::to_string(final_armor.priority),final_armor.center,FONT_HERSHEY_COMPLEX,0.7,Scalar(121,121,255),1);
                }
            }

            possibleArmors.clear();

            return true;
        }
        else
        {
            possibleArmors.clear();
            return false;
        }
    }

    /**
     * @brief this function shall to serve for building our own database, unfortunately the database built by this way is
     * not good.
     * @param fileP file descriptor
     * @param selectedIndex1 the index of target armor
     * @param selectedIndex2 the index of target armor, because there may be two or more armors in the field of vision at one time
     */
    void ArmorDetector::saveMatchParam(FILE* fileP,int selectedIndex1,int selectedIndex2)
    {
        if(fileP == nullptr)
        {
            perror("file open Error!\n");
        }
        /*this section set for make database*/
    }

    /**
     * @brief compare two matched lamps' priority
     * @param a matched lamp
     * @param b matched lamp
     * @return match up with the sort function in algorithm.h, sort the elements from smallest matchFractor to largest matchFractor
     */
    bool compMatchFactor(const MatchLight a, const MatchLight b)
    {
        return a.matchFactor < b.matchFactor;
    }
}