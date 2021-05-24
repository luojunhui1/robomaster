/*********************************************************************************
  *Copyright(C),2019-2021,西南交通大学HELIOS战队All Rights Reserved
  *FileName:  EnergyDetector.cpp
  *Author:  黄冬婷
  *Version: 1.2
  *Date:  2021.01.21
  *Description: 能量机关识别及预测
  *Function List:
     1.EnergyTask   能量机关任务执行接口
     2.getTargetPoint   获得目标点
     3.getPredictPoint    寻找攻击点
**********************************************************************************/

#include "EnergyDetector.h"

using namespace std;
extern McuData mcu_data;

/**
 * @brief EnergyDetector::EnergyDetector
 * @param null
 * @return null
 * @remark Energy类构造函数，初始化有关参数
 */
EnergyDetector::EnergyDetector() {
    initEnergy();
    initEnergyPartParam();//对能量机关参数进行初始化
}
/**
 * @brief EnergyDetector::EnergyDetectorDetector
 * @param null
 * @return null
 * @remark Energy类析构函数
 */
EnergyDetector::~EnergyDetector() = default;
/**
 * @brief EnergyDetector::initEnergy
 * @param null
 * @return null
 * @remark 初始化成员变量
 */
void EnergyDetector::initEnergy() {
    show_armors=true;//是否显示图像
    show_target_armor= false;//是否显示调试过程
    show_strip_fan = false;//是否显示报错
    show_center_R = true;//是否显示数据
    show_target_point = true;
    show_predict_point = true;

    last_target_polar_angle_judge_rotation = -1000;//上一帧待击打装甲板的极坐标角度（用于判断旋向）
    clockwise_rotation_init_cnt = 0;//装甲板顺时针旋转次数
    anticlockwise_rotation_init_cnt = 0;//装甲板逆时针旋转s次数
    energy_rotation_init = true;//若仍在判断风车旋转方向，则为true
    predict_rad = 0;//预测提前角
    predict_rad_norm = 25;// 预测提前角的绝对值
    predict_point = Point(0, 0);//预测打击点初始化
    pts.resize(4);
}
/**
 * @brief EnergyDetector::initEnergyPartParam
 * @param null
 * @return null
 * @remark 初始化参数
 */
void EnergyDetector::initEnergyPartParam() {
    _flow.BLUE_GRAY_THRESH = 100;//敌方红色时的阈值
    _flow.RED_GRAY_THRESH = 180;//敌方蓝色时的阈值

    _flow.armor_contour_area_max = 800; //装甲板的相关筛选参数
    _flow.armor_contour_area_min = 180;
    _flow.armor_contour_length_max = 50;
    _flow.armor_contour_length_min = 10;
    _flow.armor_contour_width_max = 30;
    _flow.armor_contour_width_min = 0;
    _flow.armor_contour_hw_ratio_max = 3;
    _flow.armor_contour_hw_ratio_min = 1;

    _flow.flow_strip_fan_contour_area_max = 2000;//流动条所在扇叶的相关筛选参数
    _flow.flow_strip_fan_contour_area_min = 500;
    _flow.flow_strip_fan_contour_length_max = 100;
    _flow.flow_strip_fan_contour_length_min = 60;
    _flow.flow_strip_fan_contour_width_max = 52;
    _flow.flow_strip_fan_contour_width_min = 20;
    _flow.flow_strip_fan_contour_hw_ratio_max = 2.8;
    _flow.flow_strip_fan_contour_hw_ratio_min = 1.2;
    _flow.flow_strip_fan_contour_area_ratio_max = 0.58;
    _flow.flow_strip_fan_contour_area_ratio_min = 0.34;

    _flow.Strip_Fan_Distance_max = 56;//流动条到装甲板距离参数
    _flow.Strip_Fan_Distance_min = 28;

    _flow.flow_strip_contour_area_max = 700;//流动条相关参数筛选
    _flow.flow_strip_contour_area_min = 50;
    _flow.flow_strip_contour_length_max = 55;
    _flow.flow_strip_contour_length_min = 40;//32
    _flow.flow_strip_contour_width_max = 20;
    _flow.flow_strip_contour_width_min = 4;
    _flow.flow_strip_contour_hw_ratio_min = 3;
    _flow.flow_strip_contour_hw_ratio_max = 7;
    _flow.flow_strip_contour_area_ratio_min = 0.6;
    _flow.flow_strip_contour_intersection_area_min = 100;

    _flow.target_intersection_contour_area_min = 40;//重合面积

    _flow.twin_point_max = 20;

    _flow.Center_R_Control_area_max = 80;//中心R标筛选相关参数
    _flow.Center_R_Control_area_min = 30;
    _flow.Center_R_Control_length_max = 20;
    _flow.Center_R_Control_length_min = 6;
    _flow.Center_R_Control_width_max = 20;
    _flow.Center_R_Control_width_min = 6;
    _flow.Center_R_Control_radio_max = 1.2;
    _flow.Center_R_Control_radio_min = 1;
    _flow.Center_R_Control_area_radio_min = 0.6;
    _flow.Center_R_Control_area_intersection_area_min = 10;

    _flow.flow_area_max = 5000;//扇叶筛选相关参数
    _flow.flow_area_min = 1500;
    _flow.flow_length_max = 100;
    _flow.flow_length_min = 45;
    _flow.flow_width_max = 52;
    _flow.flow_width_min = 10;
    _flow.flow_aim_max = 3.5;
    _flow.flow_aim_min = 1.2;
    _flow.flow_area_ratio_min = 0.6;
}
/**
 * @brief EnergyDetector::clearAll
 * @param null
 * @return null
 * @remark 在每帧任务开始前清空容器
 */
void EnergyDetector::clearAll() {
    //fans.clear();
    armors.clear();
    flow_strip_fans.clear();
    target_armors.clear();
    //flow_strips.clear();
    centerRs.clear();
    armor_centers.clear();
}
/**
 * @brief EnergyDetector::EnergyTask
 * @param Mat& src
 * @return null
 * @remark 能量机关任务执行接口
 */
void EnergyDetector::EnergyTask(Mat& src, bool mode) {
    clearAll();

    Mat binary;
    BIG_MODE = mode;

    // cout<<"ENERGY"<<endl;
    Mat img = src.clone();
    binary = preprocess(img);
//    if(showEnergy)
//        imshow("preprocess",binary);

    detectArmor(binary);
    detectFlowStripFan(binary);
    //detectR(binary,src);
    if(target_armor_centers.size()==3){
        calR();
        target_armor_centers.clear();
    }
    getTargetPoint(binary);


    getPredictPoint(src);

    getPts(target_armor);

#if DEBUG_MSG == 1
    LOGM("GET TARGET!\n");
#endif
}
/**
 * @brief EnergyDetector::preprocess
 * @param Mat& src
 * @return Mat& binary
 * @remark 图像预处理，完成二值化 todo 结合颜色分情况二值化
 */

Mat EnergyDetector::preprocess(Mat &src) {
    Mat dst,binary;
    cvtColor(src, dst, COLOR_BGR2GRAY);
//    if (blueTarget)
//    {
//        threshold(dst, binary, _flow.BLUE_GRAY_THRESH, 255, THRESH_BINARY);
//    }
//    else
//    {
//        threshold(dst, binary, _flow.RED_GRAY_THRESH, 255, THRESH_BINARY);
//    }
    Mat single;
    vector<Mat> channels;
    split(src,channels);
    if(blueTarget){
        single = channels.at(0);
    }else{
        single = channels.at(2);
    }

    //cvtColor(single,bright,COLOR_BGR2GRAY);
    threshold(single, binary, 130, 255, 32);

    //imshow("bin",binary);

    return binary;
}
/**
 * @brief EnergyDetector::detectBuff
 * @param Mat& src
 * @return null
 * @remark 检测所有可能的装甲
 */
void EnergyDetector::detectArmor(Mat &src) {
    //armor dilate
    Mat armor_dilate = src.clone();
    Mat element_dilate_1 = Mat(5,5,CV_8UC1,255);
    element_dilate_1 = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(armor_dilate, armor_dilate, element_dilate_1);

    //寻找所有装甲
    std::vector<vector<Point> > armor_contours;
    std::vector<vector<Point> > armor_contours_external;//用总轮廓减去外轮廓，只保留内轮廓，除去流动条的影响。
    findContours(armor_dilate, armor_contours, RETR_LIST, CHAIN_APPROX_NONE);
    findContours(armor_dilate, armor_contours_external, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    for (int i = 0; i < armor_contours_external.size(); i++)//去除外轮廓
    {
        int external_contour_size = armor_contours_external[i].size();
        for (int j = 0; j < armor_contours.size(); j++) {
            int all_size = armor_contours[j].size();
            if (external_contour_size == all_size) {
                swap(armor_contours[j], armor_contours[armor_contours.size() - 1]);
                armor_contours.pop_back();//清除掉流动条
                break;
            }
        }
    }
    //drawContours(show,armor_contours,-1,Scalar(60, 0, 255));
    for (auto &armor_contour : armor_contours) {
        if (!isValidArmorContour(armor_contour)) {
            continue;
        }
        armors.emplace_back(cv::minAreaRect(armor_contour));//回传所有装甲板到armors容器中
        armor_centers.emplace_back(cv::minAreaRect(armor_contour).center);//回传所有装甲板center到armor_center容器中
    }
}

/**
 * @brief EnergyDetector::detectFlowStripFan
 * @param Mat& src
 * @return null
 * @remark 检测所有可能的流动条所在的装甲板
 */
void EnergyDetector::detectFlowStripFan(Mat &src) {
    //flow_strip_fan dilate
    Mat flow_fan_dilate = src.clone();
    //Mat element_dilate_1 = Mat(5,5,CV_8UC1,255);
    Mat element_dilate_1 = getStructuringElement(MORPH_RECT, Size(5, 5));

    //Mat element_erode_1 = Mat(2,2,CV_8UC1,255);
    Mat element_erode_1 = getStructuringElement(MORPH_RECT, Size(2, 2));

    //Mat element_erode_2 = Mat(2,2,CV_8UC1,255);
    Mat element_erode_2 = getStructuringElement(MORPH_RECT, Size(2, 2));

    //Mat element_erode_3 = Mat(1,1,CV_8UC1,255);
    Mat element_erode_3 = getStructuringElement(MORPH_RECT, Size(1, 1));

    dilate(flow_fan_dilate, flow_fan_dilate, element_dilate_1);
    erode(flow_fan_dilate, flow_fan_dilate, element_erode_1);
    erode(flow_fan_dilate, flow_fan_dilate, element_erode_2);
    erode(flow_fan_dilate, flow_fan_dilate, element_erode_3);

    //寻找所有流动条所在扇叶
    vector<vector<Point> > flow_strip_fan_contours;
    findContours(flow_fan_dilate, flow_strip_fan_contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    vector<cv::RotatedRect> candidate_flow_strip_fans;

    for (auto &flow_strip_fan_contour : flow_strip_fan_contours) {
        if (!isValidFlowStripFanContour(flow_fan_dilate, flow_strip_fan_contour)) {
            continue;
        }
        flow_strip_fans.emplace_back(cv::minAreaRect(flow_strip_fan_contour));
    }

#if DEBUG_MSG == 1
    if (flow_strip_fans.empty()) {
        LOGM("flow strip fan false!\n");
    }
    if (!flow_strip_fans.empty()) {
        LOGM("flow strip fan success!\n");
    }
#endif
}
/**
 * @brief EnergyDetector::detectR
 * @param Mat& src
 * @return null
 * @remark 检测所有可能的中心R，并筛选出唯一中心点centerR，确定circle_center_point
 */
void EnergyDetector::detectR(Mat &src, Mat &show) {
    //R dilate
    Mat R_dilate = src.clone();
    Mat gray_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    Mat hsv;
    Mat mask;
    cvtColor(show, hsv, COLOR_RGB2HSV);
    inRange(hsv, Scalar(0, 10, 46), Scalar(180, 60, 255), mask);
    dilate(mask, mask, gray_element);

    R_dilate = R_dilate - mask;
    dilate(R_dilate, R_dilate, gray_element);
    erode(R_dilate, R_dilate, element);
//    imshow("R_dilate",R_dilate);
    //todo find R center
    vector<vector<Point> > center_R_contours;
    findContours(R_dilate, center_R_contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    for (auto &center_R_contour : center_R_contours) {
        if (!isValidCenterRContour(center_R_contour)) {
            continue;
        }
        RotatedRect tmp_R = cv::minAreaRect(center_R_contour);

        for (auto & flow_strip_fan : flow_strip_fans) {
            std::vector<cv::Point2f> intersection;
            if (rotatedRectangleIntersection(flow_strip_fan, tmp_R, intersection) != 0){
                inter_flag = true;
                break;}
        }

        if(!inter_flag) {
            centerRs.emplace_back(cv::minAreaRect(center_R_contour));
        }
    }
    //给出唯一centerR
    if(!centerRs.empty()) {
        if (centerRs.size() == 1) {
            centerR = centerRs[0];
            pre_centerR = centerR;
        } else {
            //计算前后两帧中心点距离
            int *dis = (int *) malloc(centerRs.size() * sizeof(int));
            memset(dis, 0, centerRs.size() * sizeof(int));
            for (int i = 0; i < centerRs.size(); i++) {
                *(dis + i) += abs(pre_centerR.center.x - centerRs[i].center.x);
                *(dis + i) += abs(pre_centerR.center.y - centerRs[i].center.y);
            }
            int min_dis = *dis;
            int min_xuhao = 0;
            for (int t = 1; t < centerRs.size(); t++) {
                if (*(dis + t) < min_dis) {
                    min_dis = *(dis + t);
                    min_xuhao = t;
                }
            }
            centerR = centerRs[min_xuhao];
            float center_ratio = centerRs[min_xuhao].size.height/centerRs[min_xuhao].size.width;
            if((center_ratio>1.2)||(center_ratio<0.75))
                centerR = pre_centerR;
            free(dis);
            pre_centerR = centerR;//update pre
        }
    }
    else
        centerR = pre_centerR;

    circle_center_point = centerR.center;
}

void EnergyDetector::calR() {
    Point2f one = target_armor_centers[0];
    Point2f two = target_armor_centers[1];
    Point2f three = target_armor_centers[2];
    float a1 = 2*(two.x-one.x);
    float b1 = 2*(two.y-one.y);
    float c1 = pow(two.x,2)+pow(two.y,2)-pow(one.x,2)-pow(one.y,2);
    float a2 = 2*(three.x-two.x);
    float b2 = 2*(three.y-two.y);
    float c2 = pow(three.x,2)+pow(three.y,2)-pow(two.x,2)-pow(two.y,2);

    circle_center_point.x = ((c1*b2)-(c2*b1))/((a1*b2)-(a2*b1));
    circle_center_point.y = ((a1*c2)-(a2*c1))/((a1*b2)-(a2*b1));

}



/**
 * @brief EnergyDetector::getTargetPoint
 * @param Mat& src
 * @return null
 * @remark 根据armor与flowStripFan情况判断给出唯一目标装甲板target_armor以及其中心点target_point
 */
void EnergyDetector::getTargetPoint(Mat &src) {
    //todo find armor in best_fan 目标armor
    for (auto &candidate_flow_strip_fan : flow_strip_fans) {
        //为target_armors打分
        for (auto & armor : armors) {
            std::vector<cv::Point2f> intersection;
            if (rotatedRectangleIntersection(armor, candidate_flow_strip_fan, intersection) == 0)
                continue;
            double cur_contour_area = contourArea(intersection);
            if (cur_contour_area > _flow.target_intersection_contour_area_min) {
                //*grade += 10;
                target_armors.emplace_back(armor);
                //cout << "Armor:" << armors.at(i).center << " ";
            }//返回目标装甲板参数
        }
    }
    if(!target_armors.empty()){
        if(target_armors.size()==1){
            target_armor = target_armors[0];
        }
        else{
            //为target_armors打分
            int *grade = (int *) malloc(target_armors.size() * sizeof(int));
            memset(grade, 0, target_armors.size() * sizeof(int));
            for(auto candidate_target:target_armors ){
                *grade += pointDistance(target_armor.center,lst_target_armor.center);      //距离远的为新的待打击装甲
            }
            int max_grade = *grade;
            int max_xuhao = 0;
            for (int t = 1; t < target_armors.size(); t++) {
                if (*(grade + t) < max_grade) {
                    max_grade = *(grade + t);
                    max_xuhao = t;
                }
            }
            free(grade);
            target_armor = target_armors[max_xuhao];
        }
        lst_target_armor = target_armor;        //update
    }
    else{
        target_armor = lst_target_armor;
    }
    target_point = target_armor.center;
    if(target_armor_centers.size()>0 &&target_armor_centers.size()<3&& pointDistance(target_point,target_armor_centers[target_armor_centers.size()-1])>60) {
        target_armor_centers.push_back(target_point);
        //cout<<"add"<<endl;
    }else if(target_armor_centers.size()==0) target_armor_centers.push_back(target_point);

}




/**
 * @brief EnergyDetector::getPredictPoint
 * @param Mat src
 * @return null
 * @remark 根据运动曲线规律给出预测射击点位置predict_point
 */
void EnergyDetector::getPredictPoint(Mat src) {
    Point predict_P;
    target_polar_angle = toPolar(target_point).angle;
    initRotation();

//todo SMALL MODE
    if(!BIG_MODE)
        predict_rad_norm = SHOOT_TIME*small_rate;// 提前角一定

//todo BIG MODE
    else {
        //1s内的target_angle信息
        if (target_angle_1second.size() < FPS) {
            target_angle_1second.push_back(target_polar_angle);
            predict_rad_norm = SHOOT_TIME*small_rate;// default
          //  cout<<"0"<<endl<<"-----------"<<endl;
        } else {
            target_angle_1second.erase(target_angle_1second.begin());
            target_angle_1second.push_back(target_polar_angle);

            //判断是否是新的待打击armor todo 跳一下的情况
            if (((int) abs(target_polar_angle - target_angle_1second[FPS-2] - 360) % 180) >= MAX_ANGLE) {
               // cout<<"1"<<endl<<"-----------"<<endl;
                //new_armor
                //前若干帧还在跳 不能用？
                tiao_armor_f++;
                t1_real = getTickCount();
                t1 = (t1_real - t0_real)/getTickFrequency() + t0;    //ms->s
                predict_rad_norm = calPreAngle(t1, t1 + shoot_Debug);    //用预测速度替代测量速度
            } else {
               // cout<<"2"<<endl<<"-----------"<<endl;
                //第一个
                start_flag = true;
                //稳定的new armor
                tiao_armor_f = 0;
                //计算角速度 rad/s
                float v0_rate_1 = (int) abs(target_angle_1second[(FPS / 2) - 1] - target_angle_1second[0] - 360) % 180;
                float v0_rate_2 = (int) abs(target_angle_1second[FPS] - target_angle_1second[FPS / 2] - 360) % 180;
                v0_rate = (int) abs(target_polar_angle - target_angle_1second[0] - 360) % 180;
                v0_rate = v0_rate * PI / 180;
                if (((v0_rate < 0.52) || (v0_rate > 2.09))) {     //((v0_rate < 0.52) || (v0_rate > 2.09))
                    //cout<<"3"<<endl<<"-----------"<<endl;
                    t1_real = getTickCount();

                    t1 = (t1_real - t0_real)/getTickFrequency() + t0;     //ms->s
                    predict_rad_norm = calPreAngle(t1, t1 + shoot_Debug);    //用预测速度替代测量速度
                }
                else{
                   // cout<<"4"<<endl<<"-----------"<<endl;
                    float wt = asin(((v0_rate) - 1.305) / 0.785);       //wt--rad/s ,[-1,1]
                    if (wt > 0) {
                        if (v0_rate_1 > v0_rate_2)
                            wt += 90 * (PI / 180);      //90~180
                    } else {
                        if (v0_rate_1 > v0_rate_2)
                            wt -= 90 * (PI / 180);      //-90~-180
                    }
                    t0_real = getTickCount();       //获得开始点时间
                    t0 = wt / OMEGA;      //映射到速度三角函数里
                    predict_rad_norm = calPreAngle(t0, t0 + shoot_Debug);
                }
            }
        }
        predict_rad_norm = (predict_rad_norm*180)/PI;   //转回角度制
    }

    //todo 转回直角坐标系
    if (energy_rotation_direction == 1) predict_rad = abs(predict_rad_norm);
    else if (energy_rotation_direction == -1) predict_rad = -predict_rad_norm;
    predict_point = rotate(target_point);

    //cout<<"rate:"<<v0_rate<<endl;
    //cout<<"angel:"<<predict_rad_norm<<endl;
    //cout<<"fq:"<<getTickFrequency()<<endl;
    //predict_P = predict_point;
//    if(showEnergy)
//    {
//        circle(src,predict_point,2,Scalar(139,139,0),2);
//        circle(src,target_point,2,Scalar(0,139,139),2);
//
//        string ra = to_string(v0_rate);//转速---rad/s
//        putText(src,ra,Point(100,100),FONT_HERSHEY_PLAIN,2,Scalar(139,138,8));
//
//        string rad = to_string(predict_rad_norm);//提前角
//        putText(src,rad,Point(100,200),FONT_HERSHEY_PLAIN,2,Scalar(139,138,8));
//    }

}

/**
 * @brief EnergyDetector::initRotation
 * @param null
 * @return null
 * @remark 用于判断能量机关旋转方向
 */
void EnergyDetector::initRotation() {
    if (target_polar_angle >= -180 && last_target_polar_angle_judge_rotation >= -180
        && fabs(target_polar_angle - last_target_polar_angle_judge_rotation) < 30) {
        //target_polar_angle和last_target_polar_angle_judge_rotation的初值均为1000，大于-180表示刚开始几帧不要
        //若两者比较接近，则说明没有切换目标，因此可以用于顺逆时针的判断
        if (target_polar_angle < last_target_polar_angle_judge_rotation) clockwise_rotation_init_cnt++;
        else if (target_polar_angle > last_target_polar_angle_judge_rotation) anticlockwise_rotation_init_cnt++;
    }
    //由于刚开始圆心判断不准，角度变化可能计算有误，因此需要在角度正向或逆向变化足够大时才可确定是否为顺逆时针
    if (clockwise_rotation_init_cnt == 15) {
        energy_rotation_direction = 1;//顺时针变化30次，确定为顺时针
        //cout << "rotation: " << energy_rotation_direction << endl;
        energy_rotation_init = false;
    }
    else if (anticlockwise_rotation_init_cnt == 15) {
        energy_rotation_direction = -1;//逆时针变化30次，确定为顺时针
        //cout << "rotation: " << energy_rotation_direction << endl;
        energy_rotation_init = false;
    }
    last_target_polar_angle_judge_rotation = target_polar_angle;
}


/**
 * @brief EnergyDetector::calPreAngle
 * @param float start_time, float end_time
 * @return float delta_angle
 * @remark 根据正弦速度曲线以及总射击时间给出射击提前角
 */
float EnergyDetector::calPreAngle(float start_time, float end_time) {
    float delta_angle;
    delta_angle = 0.785*(cos(OMEGA*start_time)-cos(OMEGA*end_time)+1.305*(end_time-start_time));
    return delta_angle;
}

/**
 * @brief EnergyDetector::rotate
 * @param Point target_point
 * @return Point trans_point
 * @remark 计算预测的击打点坐标 todo 后续用toCartesian 替代
 */
Point EnergyDetector::rotate(cv::Point target_point) const {
    int x1, x2, y1, y2;
    Point trans_point;
    //    为了减小强制转换的误差
    x1 = circle_center_point.x * 100;
    x2 = target_point.x * 100;
    y1 = circle_center_point.y * 100;
    y2 = target_point.y * 100;

    trans_point.x = static_cast<int>(
            (x1 + (x2 - x1) * cos(-predict_rad * 3.14 / 180.0) - (y1 - y2) * sin(-predict_rad * 3.14 / 180.0)) / 100);
    trans_point.y = static_cast<int>(
            (y1 - (x2 - x1) * sin(-predict_rad * 3.14 / 180.0) - (y1 - y2) * cos(-predict_rad * 3.14 / 180.0)) / 100);

    return trans_point;
}


/**
 * @brief EnergyDetector::isValidCenterRContour
 * @param vector<Point>& center_R_contour
 * @return bool
 * @remark 判断找到的中心点R尺寸是否合格
 */
bool EnergyDetector::isValidCenterRContour(const vector<cv::Point>& center_R_contour) {
    double cur_contour_area = contourArea(center_R_contour);
    if (cur_contour_area > _flow.Center_R_Control_area_max ||
        cur_contour_area < _flow.Center_R_Control_area_min) {
        return false;
    }
    return true;
}

/**
 * @brief EnergyDetector::isValidArmorContour
 * @param vector<Point>& armor_contour
 * @return bool
 * @remark 判断找到的装甲Armor尺寸是否合格
 */
bool EnergyDetector::isValidArmorContour(const vector<cv::Point>& armor_contour) const {
    double cur_contour_area = contourArea(armor_contour);
    if (cur_contour_area > _flow.armor_contour_area_max ||
        cur_contour_area < _flow.armor_contour_area_min) {
        return false;
    }
    RotatedRect cur_rect = minAreaRect(armor_contour);
    Size2f cur_size = cur_rect.size;
    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    if (length < _flow.armor_contour_length_min || width < _flow.armor_contour_width_min ||
        length >  _flow.armor_contour_length_max || width > _flow.armor_contour_width_max) {
        return false;
    }
    float length_width_ratio = length / width;
    if (length_width_ratio > _flow.armor_contour_hw_ratio_max ||
        length_width_ratio < _flow.armor_contour_hw_ratio_min) {
        return false;
    }
    return true;
}


/**
 * @brief EnergyDetector::isValidFlowStripFanContour
 * @param vector<Point>& flow_strip_fan_contour
 * @return bool
 * @remark 判断找到的含有流动条的扇叶尺寸是否合格
 */
bool EnergyDetector::isValidFlowStripFanContour(cv::Mat& src, const vector<cv::Point>& flow_strip_fan_contour) const {
    double cur_contour_area = contourArea(flow_strip_fan_contour);
    if (cur_contour_area > _flow.flow_strip_fan_contour_area_max ||
        cur_contour_area < _flow.flow_strip_fan_contour_area_min) {
        return false;
    }
    RotatedRect cur_rect = minAreaRect(flow_strip_fan_contour);
    Size2f cur_size = cur_rect.size;
    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    if (length < _flow.flow_strip_fan_contour_length_min
        || width < _flow.flow_strip_fan_contour_width_min
        || length > _flow.flow_strip_fan_contour_length_max
        || width > _flow.flow_strip_fan_contour_width_max) {
        return false;
    }
    float length_width_ratio = length / width;
    if (length_width_ratio > _flow.flow_strip_fan_contour_hw_ratio_max ||
        length_width_ratio < _flow.flow_strip_fan_contour_hw_ratio_min) {
        return false;
    }
    if (cur_contour_area / cur_size.area() < _flow.flow_strip_fan_contour_area_ratio_min
        || cur_contour_area / cur_size.area() > _flow.flow_strip_fan_contour_area_ratio_max) {
        return false;
    }
    return true;
}


//todo 工具类函数
/**
 * @brief EnergyDetector::pointDistance
 * @param Point point_1,Point point_2
 * @return double distance
 * @remark 输入两点，返回两点间距离
 */
double EnergyDetector::pointDistance(cv::Point point_1, cv::Point point_2) {
    double distance = 0;
    distance = sqrt(pow(static_cast<double>(point_1.x - point_2.x), 2) + pow(static_cast<double>(point_1.y - point_2.y), 2));
    return distance;
}
/**
 * @brief EnergyDetector::toPolar
 * @param Point cart
 * @return polar trans
 * @remark 输入世界坐标系坐标，输出极坐标系坐标
 */
polarLocal EnergyDetector::toPolar(Point cart){
    polarLocal trans;
    trans.angle = static_cast<float>(180 / 3.14 * atan2((-1 * (cart.y - circle_center_point.y)), (cart.x - circle_center_point.x)));
    trans.radius = pointDistance(cart,circle_center_point);
    return trans;
}
/**
 * @brief EnergyDetector::toCartesian
 * @param polarLocal pol
 * @return Point trans
 * @remark 输入极坐标系坐标，输出世界坐标系坐标
 */
Point EnergyDetector::toCartesian(polarLocal pol){
    int delta_x = cos(pol.angle)*pol.radius;    //R*cos = x2-x1 todo cos里是弧度而非角度
    int delta_y;
    delta_y = sqrt(pow(pol.radius, 2) - pow( delta_x,2));   //(y2-y1)^2 = sqrt(radius^2-(x2-x1)^2)
    Point trans;
    trans.x = circle_center_point.x + delta_x;
    trans.y = circle_center_point.y + delta_y;
    return trans;
}

void EnergyDetector::getPts(RotatedRect armor){
    Point2f rectPoints[4];//定义矩形的4个顶点
    armor.points(rectPoints); //计算矩形的4个顶点
    //judge long side
    if(sqrt(pow((rectPoints[0].x-rectPoints[1].x),2)+pow((rectPoints[0].y-rectPoints[1].y),2))
       >sqrt(pow((rectPoints[2].x-rectPoints[1].x),2)+pow((rectPoints[2].y-rectPoints[1].y),2))){
        //pts[0]-pts[1] is long side
        pts[0] = rectPoints[0];
        pts[1] = rectPoints[1];
        pts[2] = rectPoints[2];
        pts[3] = rectPoints[3];
    }else{
        //pts[1]-pts[2] is long side
        pts[0] = rectPoints[1];
        pts[1] = rectPoints[2];
        pts[2] = rectPoints[3];
        pts[3] = rectPoints[0];
    }
}

Point EnergyDetector::getPredict(){
    return predict_point;
}
Point EnergyDetector::getOffset(){
    return predict_point-target_point;
}