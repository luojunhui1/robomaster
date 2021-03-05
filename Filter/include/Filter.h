/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  Filter.h
  *Author:  解佳朋
  *Version: 1.3.1.200312_RC
  *Date:  2020.03.12
  *Description: 卡尔曼滤波工具类
  *Function List:
     1.KF_two   构造函数包含有参构造和无参构造
     2.Prediction   传入状态矩阵,进行预测部分计算
     3.GetPrediction    包含有参和无参重载,无参代表直接使用类内状态向量和状态矩阵相乘,有参代表与传入状态矩阵相乘
     4.set_x    状态向量初始化
     5.update   状态更新
**********************************************************************************/
#ifndef FILTER_H
#define FILTER_H

#include<iostream>
#include<Eigen/Dense>
#include<stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

//第二版一阶卡尔曼预测,角度版
namespace rm {
    class KF_two {
    private:
        double pitch;                                       //测量pitch角度
        double yaw;                                       //测量yaw角度

    public:

        Eigen::VectorXd x_;                         //状态向量[锁定目标绝对pitch,锁定目标绝对yaw,v_pitch,v_yaw]

        KF_two(Eigen::MatrixXd P_in, Eigen::MatrixXd Q_in, Eigen::MatrixXd H_in, Eigen::MatrixXd R_in);

        void Prediction(Eigen::MatrixXd _F);

        Eigen::VectorXd GetPrediction();

        Eigen::VectorXd GetPrediction(Eigen::MatrixXd _F);

        Eigen::MatrixXd F;                           //状态转移矩阵

        Eigen::MatrixXd P;                          //状态协方差矩阵
        Eigen::MatrixXd Q;                          //过程噪声

        Eigen::MatrixXd H;                          //测量矩阵

        Eigen::MatrixXd R;                          //测量噪声矩阵
        Eigen::MatrixXd K;                          //卡尔曼增益

        bool is_set_x = false;                     //判断是否赋初值

        KF_two();                                          //创建
        void set_x(Eigen::VectorXd x, Eigen::MatrixXd _F);                                   //赋初值
        void set_x(Eigen::VectorXd x);                                   //赋初值
        void update(Eigen::VectorXd z, Eigen::MatrixXd _F);             //更新
        Eigen::VectorXd get_x();                                  //返回状态向量
        //    void initialize();                                    //初始化

        //一阶卡尔曼预测
        int q = 120;
        int r = 1200;//5

    };

//射击模式
    enum pattern {
        FirstFind,        //首次识别
        Shoot,              //连续射击
        stop,               //非连续
        buffering        //缓冲
    };


    class Kalman {
    public:
        Kalman();

        Point2f SetKF(const Point2f& BestArmor);

        void FirstFind(const Point2f& BestArmor);                                                                                                                             //首次识别
        void FirstSetFilter(Point2f BestArmor);                            //首次击打相同目标
        void ContinueSetFilter(const Point2f& BestArmor);                  //连续击打同一目标
        bool JudgeArmor(int &flag);

        bool JudgeArmor_v(int &flag);

        float sum_x;
        float sum_y;

        KF_two KF_forecast;             //预测一阶滤波
        KF_two KF_tz;
        KF_two KF_tuoluo;
        KF_two KF_SecOrder;        //预测二阶滤波
        bool isSetKF_tz = false;
        float v_tx_old;                     //线速度保留量
        float v_ty_old;
        float v_tz_old;
        float p_tx_old;                  //位置保留量
        float p_ty_old;
        float p_tx_now;
        float p_ty_now;
        float p_tz_old;
        float p_predictx;       //上一帧预测值
        float p_predicty;
        float delta_x = 0;
        float delta_y = 0;

        //todo prameter in judge
        float px[20];
        bool set_px;
        int px_flag;
        float Vmean;
        float dx;
        int w0 = 0.4;
        int w1 = 0.6;

        float vx[20];
        bool set_vx;
        int vx_flag;
        float Amean;
        float ax;
        float v_tx_now;

        int fx;

        //CarData carDatasOld;                //上次收数
        Point2f old_Poisition;              //上级的二阶预测的位置
        Point2f old_objectP;

        double t = 20;      //time between 2 img;

        bool flag = true;
    };
}
#endif // FILTER_H
