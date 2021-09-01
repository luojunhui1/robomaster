#include "ArmorDetector.hpp"
#include <sstream>
#include <algorithm>
#include <thread>

namespace rm
{
    /**
    * @brief Armor constructor
    * @param [L1] one lamp of the armor
    * @param [L2] another lamp of the armor
    * @return none
    * @details none
    */

    Armor::Armor(Lamp L1, Lamp L2, double priority_)
    {
        errorAngle = fabs(L1.lightAngle - L2.lightAngle);
        armorWidth = fabs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
        armorHeight = fabs(static_cast<int>((L1.rect.size.height + L2.rect.size.height) / 2));
        center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x) / 2);
        center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y) / 2);
        rect = Rect(center - Point2i(armorWidth / 2, armorHeight / 2), Size(armorWidth, armorHeight));
        armorType = (armorWidth / armorHeight > 2) ? (BIG_ARMOR) : (SMALL_ARMOR);
        priority = priority_;
        avgRSubBVal = (L1.avgRSubBVal + L2.avgRSubBVal)/2;

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

    Armor::Armor(Rect &rect)
    {
        errorAngle = 0;
        center = rect.tl() + Point(rect.width/2, rect.height/2);
        this->rect = rect;

        pts.resize(4);

        pts[0] = rect.tl();
        pts[1] = rect.tl() + Point(rect.width, 0);
        pts[2] = rect.tl() + Point(rect.width, rect.height);
        pts[3] = rect.tl() + Point(0, rect.height);

        armorWidth = rect.width;
        armorHeight = rect.height;

        armorType =  (armorWidth / armorHeight > 2) ? (BIG_ARMOR) : (SMALL_ARMOR);

        priority = 0;

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
        targetArmor.init();
        lastTarget.init();
        roiRect = Rect(0, 0, FRAMEWIDTH, FRAMEHEIGHT);
        findState = false;
        detectCnt = 0;
        lostCnt = 120;
        armorNumber = 0;
        LoadSvmModel(SVM_PARAM_PATH,Size(SVM_IMAGE_SIZE,SVM_IMAGE_SIZE));
        lossCnt = 0;

        cfgPath = "../Armor/resource/conf.cfg";
        weightPath = "../Armor/resource/528.weights";

        net =Net(DetectionModel(cfgPath, weightPath));
        net.setPreferableBackend(DNN_BACKEND_CUDA);
        net.setPreferableTarget(DNN_TARGET_CUDA);
        outNames = net.getUnconnectedOutLayersNames();

        for (int i = 0; i < outNames.size(); i++)
        {
            printf("output layer name : %s\n", outNames[i].c_str());
        }

        find_not_engineer = false;
    }

    /**
    * @brief: top level detection task
    * @param [img] the image from camera or video that to be processed
    * @return: if ever found armors in this image, return true, otherwise return false
    * @details: none
    */
    bool ArmorDetector::ArmorDetectTask(Mat &img_)
    {
        this->img = img_.clone();
#if USEROI == 1
        GetRoi(); //get roi
#endif

        imgRoi = img(roiRect);

        Preprocess(imgRoi);

        /*open two threads to recognition number and finding target armor*/

        DetectArmor();

        img_ = img.clone();

        return findState;
    }

    /**
    * @brief: detect possible armors
    * @param [img]  the image from camera or video that to be processed
    * @return: if ever found armors in this image, return true, otherwise return false
    * @details: none
    */
    bool ArmorDetector::DetectArmor()
    {
        findState = false;
        armorNumber = 0;

        vector<Lamp> lights;

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
                    //imgRoi is not a bug here, because imgRoi share the same memory with img
                    line(img, rect_point[j] + Point2f(roiRect.x, roiRect.y), rect_point[(j + 1) % 4] + Point2f(roiRect.x, roiRect.y), Scalar(0, 255, 255), 2);
                }
            }
        }

        MaxMatch(lights);

        if (findState)
        {
            detectCnt++;
            lostCnt = 0;

            MakeRectSafe(targetArmor.rect, img.size());

            targetArmor.rect  = targetArmor.rect + Point(roiRect.x, roiRect.y);
            targetArmor.center +=  Point(roiRect.x, roiRect.y);

            for(int i = 0; i< 4;i++)
            {
                targetArmor.pts[i] = targetArmor.pts[i] + Point2f(roiRect.x, roiRect.y);
            }

            if (showArmorBox)
            {
                rectangle(img,roiRect,Scalar(255,255,255),2);

                for (int j = 0; j < 4; j++) {
                    line(img, targetArmor.pts[j], targetArmor.pts[(j + 1) % 4], Scalar(255, 0, 255), 2);
                }
                circle(img,targetArmor.center,5,Scalar(0,0,255),-1);
            }

            /**update roi rect, last armor, average of lamps' R channel subtract B channel value**/
#if USEROI == 1
            roiRect = targetArmor.rect;
#endif

            lastTarget = targetArmor;


            //averageRSubBVal = averageRSubBVal*armorFoundCounter/(armorFoundCounter + 1) + targetArmor.avgRSubBVal/(armorFoundCounter + 1);
            //armorFoundCounter++;
            //cout<<"Average Value of R Sub B : "<<averageRSubBVal<<endl;

            return true;
        }
        else
        {
            detectCnt = 0;
            lostCnt++;

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
    void ArmorDetector::MaxMatch(vector<Lamp> &lights)
    {
        static float deviationAngle, xDiff,yDiff;
        static float nL,nW;
        static float dAngle;
        static float contourLen1;
        static float contourLen2;
        static float ratio;
        static float nAngle;
        static float dAvgB;

        vector<MatchLight> matchLights;

        static MatchLight matchLight;
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

                /*the difference ratio of the two lights' height*/
                contourLen1 = abs(lights[i].rect.size.height - lights[j].rect.size.height) / max(lights[i].rect.size.height, lights[j].rect.size.height);
                if(contourLen1 > param.maxLengthError)continue;

                /*the difference ratio of the two lights' width*/
                contourLen2 = abs(lights[i].rect.size.width - lights[j].rect.size.width) / max(lights[i].rect.size.width, lights[j].rect.size.width);

                /*the average height of two lights(also the height of the armor defined by these two lights)*/
                nW = (lights[i].rect.size.height + lights[j].rect.size.height) / 2;

                /*the width between the center points of two lights*/
                nL = fabs(lights[i].rect.center.x - lights[j].rect.center.x);

                /*the ratio of the width and the height, must larger than 1 */
                ratio = nL / nW;
                if(ratio > param.maxRatio || ratio < param.minRatio)continue;

                /*anyway, the difference of the lights' angle is tiny,so anyone of them can be the angle of the armor*/
                nAngle = fabs((lights[i].lightAngle + lights[j].lightAngle)/2);
                if(nAngle > param.maxArmorAngle)continue;

                /*the deviation angle of two lamps*/
                deviationAngle = fabs(atan((lights[i].rect.center.y - lights[j].rect.center.y)
                                           / (lights[i].rect.center.x - lights[j].rect.center.x))) * 180 / CV_PI;
                if(deviationAngle > param.maxDeviationAngle)continue;

                /*the difference of the y coordinate of the two center points*/
                yDiff = abs(lights[i].rect.center.y - lights[j].rect.center.y) / nW;
                if(yDiff > param.maxYDiff)continue;

                /*difference of average brightness*/
                dAvgB = abs(lights[i].avgRSubBVal - lights[j].avgRSubBVal);

                /*the match factor is still rough now, but it can help filter  the most possible target from  the detected armors
                Of course, we need to find a formula that is more reasonable*/
                match_factor_ = contourLen1  + dAvgB + exp(dAngle + deviationAngle + MIN(fabs(ratio - 1.5), fabs(ratio - 3.5)));

                matchLight = MatchLight (i, j, match_factor_, nW);

                matchLights.emplace_back(matchLight);
            }

        }

        /*sort these pairs of lamps by match factor*/
        if (matchLights.empty())
        {
            findState = false;
            return;
        }

        findState = true;

        sort(matchLights.begin(), matchLights.end(), compMatchFactor);

#if NUM_RECOGNIZE == 1
        uint8_t mostPossibleLampsIndex1 = matchLights[0].matchIndex1, mostPossibleLampsIndex2 = matchLights[0].matchIndex2;
        float curSmallestHeightError = 1000;
        int matchPossibleArmorCount = 0;
        int targetMatchIndex = 0;
        Armor curArmor;

        for(int i  = 0; i < matchLights.size(); i++)
        {
            if(matchLights[i].matchIndex1 == mostPossibleLampsIndex1
               || matchLights[i].matchIndex2 == mostPossibleLampsIndex1
               || matchLights[i].matchIndex1 == mostPossibleLampsIndex2
               || matchLights[i].matchIndex2 == mostPossibleLampsIndex2)
            {
                curArmor = Armor(lights[matchLights[i].matchIndex1], lights[matchLights[i].matchIndex2],matchLights[i].matchFactor);
                MakeRectSafe(curArmor.rect,roiRect.size());

                SetSVMRectPoints(curArmor.pts[0],curArmor.pts[1],curArmor.pts[2],curArmor.pts[3]);

                armorNumber = GetArmorNumber();
                if(armorNumber != 0 && (armorNumber == 1) || (armorNumber == 3) || (armorNumber == 4))
                {
                    targetMatchIndex = i;
                    break;
                }
                if(fabs(matchLights[i].lampHeight - lastTarget.armorHeight) < curSmallestHeightError)
                {
                    targetMatchIndex = i;
                    curSmallestHeightError = fabs(matchLights[i].lampHeight - lastTarget.armorHeight);
                }
                matchPossibleArmorCount++;
                if(matchPossibleArmorCount == 5)
                    break;
            }
        }

        targetArmor = Armor(lights[matchLights[targetMatchIndex].matchIndex1], lights[matchLights[targetMatchIndex].matchIndex2]\
                            ,matchLights[targetMatchIndex].matchFactor);
#else
        targetArmor = Armor(lights[matchLights[0].matchIndex1], lights[matchLights[0].matchIndex2]\
                            ,matchLights[0].matchFactor);
#endif
        MakeRectSafe(targetArmor.rect,roiRect.size());
#if NUM_RECOGNIZE == 1
        if(armorNumber == 0)
        {
            SetSVMRectPoints(targetArmor.pts[0],targetArmor.pts[1],targetArmor.pts[2],targetArmor.pts[3]);
            armorNumber = GetArmorNumber();
        }
#endif
    }

    /**
    * @brief pre-procession of an image captured
    * @param [img] the ROI image that clipped by the GetRIO function
    * @param [type] choose to Preprocess the current image or the lastest two images, when the type is true, parameter
    * img must be the origin image but not the roi image
    * @return none
    * @details if average value in a region of the colorMap is larger than 0, then we can inference that in this region
    * the light is more possible to be red; the reason for why we not just subtract the red channel with blue channel is
    * that the center of lamps always be white and the result of the subtraction is always small. And we have tested that
    * if we use canny to detect the edge of lamps for reducing the computing burden of findContours() function which works
    * for GPU, the result shows that edge detection costs more time(using same video, without edge detection, task costs
    * 4.19548ms;with edge detection, task costs 4.27403ms)
    */
    void ArmorDetector::Preprocess(Mat &img)
    {
        Mat bright;
        vector<Mat> channels;
        split(img,channels);
        cvtColor(img,bright,CV_BGR2GRAY);//0,2,1

        //Attention!!!if the calculate result is small than 0, because the mat format is CV_UC3, it will be set as 0.
        cv::subtract(channels[0],channels[2],bSubR);
        cv::subtract(channels[2],channels[1],rSubB);


        threshold(bright, svmBinaryImage, 20, 255, CV_MINMAX);

        GaussianBlur(bright,bright,Size(5,5),3);
        threshold(bright, thresholdMap, 130, 255, CV_MINMAX);

        colorMap = Mat_<int>(rSubB) - Mat_<int>(bSubR);
    }

    /**
    * @brief detect and filter lights in img
    * @param img
    * @return a vector of possible led object
    **/
    vector<Lamp> ArmorDetector::LightDetection(Mat& img)
    {
        Mat hsv_binary;
        Mat_<int> lampImage;
        float angle_ = 0;
        Scalar_<double> avg,avgBrightness;
        float lampArea;

        RotatedRect possibleLamp;
        Rect rectLamp;
        vector<Lamp> lampVector;

        vector<vector<Point>> contoursLight;

        findContours(img, contoursLight, RETR_EXTERNAL, CHAIN_APPROX_NONE);

#pragma omp parallel for
        for (auto & i : contoursLight)
        {
            if (i.size() < 5)
                continue;

            double length = arcLength(i, true);

            if (length > 10 && length < 4000)
            {

                possibleLamp = fitEllipse(i);
                lampArea = possibleLamp.size.width * possibleLamp.size.height;
                //LOGM("lampArea : %f\n",lampArea);
                if((lampArea > param.maxLightArea) || (lampArea < param.minLightArea))continue;
                float rate_height2width = possibleLamp.size.height / possibleLamp.size.width;
                //LOGM("rate_height2width : %f\n",rate_height2width);
                if((rate_height2width < param.minLightW2H) || (rate_height2width > param.maxLightW2H))continue;

                angle_ = (possibleLamp.angle > 90.0f) ? (possibleLamp.angle - 180.0f) : (possibleLamp.angle);

                //LOGM("angle_ : %f\n",angle_);
                if(fabs(angle_) >= param.maxLightAngle)continue;

                rectLamp = possibleLamp.boundingRect();
                MakeRectSafe(rectLamp,colorMap.size());
                mask = Mat::ones(rectLamp.height,rectLamp.width,CV_8UC1);

                /*Add this to make sure numbers on armors will not be recognized as lamps*/
                lampImage = colorMap(rectLamp);
                avgBrightness = mean(lampImage, mask);

                avg = Scalar_<float>(avgBrightness);

                //cout<<avg<<endl;

                if((blueTarget && avg[0] < -30) || (!blueTarget && avg[0] > 30))
                {
                    Lamp buildLampInfo(possibleLamp, angle_, avg[0]);
                    lampVector.emplace_back(buildLampInfo);
                }
            }
        }

        //LOGM("SIZE OF lampVector : %d\n", lampVector.size());

        return lampVector;
    }

    /**
    * @brief get he region of interest
    * @param [img] the image from camera or video that to be processed
    * @return none
    * @details none
    */
    void ArmorDetector::GetRoi()
    {
        Rect rectTemp = roiRect;
        if (lostCnt>3 || rectTemp.width == 0 || rectTemp.height == 0)
        {
            roiRect = Rect(0, 0,FRAMEWIDTH, FRAMEHEIGHT);
        }
        else if(detectCnt>0)
        {
            float scale = 3;
            int w = int(rectTemp.width * scale);
            int h = int(rectTemp.height * scale);
            int x = int(rectTemp.x - (w - rectTemp.width) * 0.5);
            int y = int(rectTemp.y - (h - rectTemp.height) * 0.5);

            roiRect = Rect(x, y, w, h);
            //MakeRectSafe(roiRect, img.size());
            if (!MakeRectSafe(roiRect, Size(FRAMEWIDTH, FRAMEHEIGHT)))
            {
                roiRect = Rect(0, 0, FRAMEWIDTH, FRAMEHEIGHT);
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
    inline bool MakeRectSafe(cv::Rect &rect, const cv::Size& size)
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
        findState = false;

        Preprocess(src);

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
        imgRoi = src(roiRect);

        Preprocess(imgRoi);

        img = src.clone();

        if (DetectArmor())
        {
            detectCnt++;
            src = img.clone();
            return true;
        }

        detectCnt = 0;
        src = img.clone();
        return false;
    }

    /**
     * @brief load SVM parameters
     * @param model_path file path
     * @param armorImgSize size of armor
     * @return none
     */
    void ArmorDetector::LoadSvmModel(const char *model_path, const Size& armorImgSize)
    {
        svm = StatModel::load<SVM>(model_path);
        if(svm.empty())
        {
            cout<<"Svm load error! Please check the path!"<<endl;
            exit(0);
        }
        svmArmorSize = armorImgSize;

        //set dstPoints (the same to armorImgSize, as it can avoid resize armorImg)
        dstPoints[0] = Point2f(0, 0);
        dstPoints[1] = Point2f(armorImgSize.width, 0);
        dstPoints[2] = Point2f(armorImgSize.width, armorImgSize.height);
        dstPoints[3] = Point2f(0, armorImgSize.height);
    }

    void ArmorDetector::SetSVMRectPoints(Point2f& lt, Point2f& rt, Point2f& lb, Point2f& rb)
    {
        srcPoints[0] = lt;
        srcPoints[1] = rt;
        srcPoints[2] = lb;
        srcPoints[3] = rb;
    }
    void ArmorDetector::SetSVMRectPoints(Point2f&& lt, Rect& rectArea)
    {
        srcPoints[0] = lt + Point2f(rectArea.x, rectArea.y);
        srcPoints[1] = srcPoints[0] + Point2f(rectArea.width, 0);
        srcPoints[2] = srcPoints[0] + Point2f(rectArea.width, rectArea.height);
        srcPoints[3] = srcPoints[0] + Point2f(0, rectArea.height);
    }

    /**
     * @brief recognize the number of target armor, only works when USEROI == 1
     * @return if USEROI == 1 and recognizing number successfully, return the number of target armor, or return -1
     */
    int ArmorDetector::GetArmorNumber()
    {
#if USEROI == 1
        warpPerspective_mat = getPerspectiveTransform(srcPoints, dstPoints);
        warpPerspective(svmBinaryImage, warpPerspective_dst, warpPerspective_mat, Size(SVM_IMAGE_SIZE,SVM_IMAGE_SIZE), INTER_NEAREST, BORDER_CONSTANT, Scalar(0)); //warpPerspective to get armorImage

        warpPerspective_dst = warpPerspective_dst.colRange(6,34).clone();
        resize(warpPerspective_dst,warpPerspective_dst,Size(SVM_IMAGE_SIZE,SVM_IMAGE_SIZE));

        pyrDown(warpPerspective_dst,warpPerspective_dst);
       // Canny(warpPerspective_dst,warpPerspective_dst, 0, 200);

       // imshow("warpPerspective_dst",warpPerspective_dst);

        svmParamMatrix = warpPerspective_dst.reshape(1, 1);
        svmParamMatrix.convertTo(svmParamMatrix, CV_32FC1);

        int number = (int)(svm->predict(svmParamMatrix) + 0.5 );

        return number;
#else
        return 0;
#endif
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

    bool ArmorDetector::ModelDetectTask(Mat &frame)
    {
        inputBlob = blobFromImage(frame, 1 / 255.F, Size(320, 320), Scalar(), true, false);//输入图像设置，input为32的整数倍，不同尺寸速度不同精度不同

        net.setInput(inputBlob);

        net.forward(outs, outNames);

        boxes.clear();
        classIds.clear();
        confidences.clear();

        for (auto & out : outs)
        {
            // detected objects and C is a number of classes + 4 where the first 4
            float* data = (float*)out.data;
            for (int j = 0; j < out.rows; ++j, data += out.cols)
            {
                Mat scores = out.row(j).colRange(5, out.cols);
                minMaxLoc(scores, nullptr, &confidence, 0, &classIdPoint);
                if (confidence > 0.5)
                {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(Rect(left, top, width, height));
                }
            }
        }

        //---------------------------非极大抑制---------------------------
        NMSBoxes(boxes, confidences, 0.5, 0.5, indices);

        int index;
        findState = false;

        dis2LastCenter = 1<<30;

        for (index = 0; index < indices.size(); ++index)
        {
            if((classIds[indices[index]] < 7 && blueTarget) || (classIds[indices[index]] > 7 && !blueTarget))
                continue;

            if(!find_not_engineer && indices[index] != 1 && indices[index] != 8)
            {
                targetArmor = Armor(boxes[indices[index]]);
                findState = true;
                break;
            }
            else if(find_not_engineer)
            {
                Point curr = (boxes[indices[index]].tl() + boxes[indices[index]].br())/2 - lastTarget.center;
                if(dis2LastCenter > curr.x*curr.x + curr.y*curr.y)
                {
                    targetArmor = Armor(boxes[indices[index]]);
                    findState = true;
                    find_not_engineer = classIds[indices[index]] == 1 || classIds[indices[index]] == 8;
                }
            }
        }

        if (findState)
        {
            detectCnt++;
            lostCnt = 0;

            MakeRectSafe(targetArmor.rect, img.size());

            if (showArmorBox)
            {
                //rectangle(frame,roiRect,Scalar(255,255,255),2);

                for (int j = 0; j < 4; j++) {
                    line(frame, targetArmor.pts[j], targetArmor.pts[(j + 1) % 4], Scalar(0, 255, 255), 2);
                }

                circle(frame,targetArmor.center,10,Scalar(0,255,255),-1);
            }

            /**update roi rect, last armor, average of lamps' R channel subtract B channel value**/
#if USEROI == 1
            roiRect = targetArmor.rect;
#endif

            lastTarget = targetArmor;

            return true;
        }
        else
        {
            detectCnt = 0;
            lostCnt++;
            return false;
        }
    }
}