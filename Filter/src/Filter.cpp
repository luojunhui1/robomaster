/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  Filter.cpp
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
//#include "Filter.h"
//#include "Filter"
#include "Filter.h"

namespace rm {
    KF_two::KF_two() {
        //状态协方差矩阵附初值,搭配绝对位置的移动预测

        Eigen::MatrixXd P_in = Eigen::MatrixXd(4, 4);
        P_in << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1, 0.0,
                0.0, 0.0, 0.0, 1;
        P = P_in;

        //过程噪声矩阵附初值
        Eigen::MatrixXd Q_in(4, 4);
        Q_in << q, 0.0, 0.0, 0.0,
                0.0, q, 0.0, 0.0,
                0.0, 0.0, q, 0.0,
                0.0, 0.0, 0.0, q;
        Q = Q_in;

        //测量矩阵附初值
        Eigen::MatrixXd H_in(4, 4);
        H_in << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
        H = H_in;

        //测量噪声矩阵附初值
        Eigen::MatrixXd R_in(4, 4);
//    R_in<<1000, 0.0, 0.0, 0.0,
//            0.0, 1000, 0.0, 0.0,
//            0.0, 0.0, 1000, 0.0,
//            0.0, 0.0, 0.0, 1000;
        R_in << r, 0.0, 0.0, 0.0,
                0.0, r, 0.0, 0.0,
                0.0, 0.0, r, 0.0,
                0.0, 0.0, 0.0, r;
        R = R_in;

    }

/**
 * @brief KF_two类初始化重载
 * @param P_in状态协方差矩阵   Q_in过程噪声矩阵  H_in测量矩阵    R_in测量噪声矩阵
 */
    KF_two::KF_two(Eigen::MatrixXd P_in, Eigen::MatrixXd Q_in, Eigen::MatrixXd H_in, Eigen::MatrixXd R_in) {
        P = P_in;
        Q = Q_in;
        H = H_in;
        R = R_in;
    }


/**
 * @brief 给状态向量附初值
 * @param x 状态向量初值      _F状态转移矩阵
 */
    void KF_two::set_x(Eigen::VectorXd x, Eigen::MatrixXd _F) {
        F = _F;
        x_ = x;


        is_set_x = true;
    }

    void KF_two::set_x(Eigen::VectorXd x) {
        x_ = x;
        is_set_x = true;
    }

/**
 * @brief KF_two类初始化重载
 * @param _F对应当前状态的状态转移矩阵
 */
    void KF_two::Prediction(Eigen::MatrixXd _F) {
        F = _F;
        //得到预测值
        x_ = F * x_;
        P = F * P * F.transpose() + Q;
    }

/**
 * @brief KF_two 返回相乘量
 * @param _F对应当前状态的状态转移矩阵
 * @return 状态向量
 */
    Eigen::VectorXd KF_two::GetPrediction(Eigen::MatrixXd _F) {
        return _F * x_;
    }

/**
 * @brief KF_two::GetPrediction
 */
    Eigen::VectorXd KF_two::GetPrediction() {
        return F * x_;
    }

/**
 * @brief 返回状态向量，获得预测值
 * @return 状态向量
 */
    Eigen::VectorXd KF_two::get_x() {
        return x_;
    }

//更新状态
    void KF_two::update(Eigen::VectorXd z, Eigen::MatrixXd _F) {
        F = _F;
        Eigen::MatrixXd y = z - H * x_;
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();
        x_ = x_ + (K * y);
        int size = x_.size();

//    Eigen::MatrixXd I = Eigen::MatrixXd(size,size);
        if (size == 4) {
            Eigen::MatrixXd I(4, 4);
            I << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
            P = (I - K * H) * P;
        }

        if (size == 2) {
            Eigen::MatrixXd I(2, 2);
            I << 1, 0,
                    0, 1;
            P = (I - K * H) * P;
        }

        if (size == 6) {
            Eigen::MatrixXd I(6, 6);
            I << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            P = (I - K * H) * P;
        }

        if (size == 4) {
            Eigen::MatrixXd I(4, 4);
            I << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;
            P = (I - K * H) * P;
        }
    }

    Kalman::Kalman() {
        //测距滤波初始化,仅需执行一次
        if (!isSetKF_tz) {
            isSetKF_tz = true;
            //状态协方差矩阵附初值

            Eigen::MatrixXd P_in = Eigen::MatrixXd(2, 2);
            P_in << 1.0, 0.0,
                    0.0, 1.0;
            KF_tz.P = P_in;

            //过程噪声矩阵附初值
            Eigen::MatrixXd Q_in(2, 2);
            Q_in << 1.0, 0.0,
                    0.0, 1e-1;//1E-1
            KF_tz.Q = Q_in;

            //测量矩阵附初值
            Eigen::MatrixXd H_in(1, 2);
            H_in << 1.0, 0.0;
            KF_tz.H = H_in;

            //测量噪声矩阵附初值
            Eigen::MatrixXd R_in(1, 1);
            R_in << 10;
            KF_tz.R = R_in;

            Eigen::MatrixXd F_in(2, 2);
            F_in << 1.0, 1.0,
                    0.0, 1.0;
            KF_tz.F = F_in;
        }
    }

    void Kalman::FirstFind(const Point2f& targetPoint) {
        //cout << "FIRST FOUND" << endl;
        //状态保留
        p_tx_old = targetPoint.x;
        p_ty_old = targetPoint.y;

        v_tx_old = 0;
        v_ty_old = 0;
    }

    bool Kalman::JudgeArmor(int &flag) {
        dx = (p_tx_now - px[(flag + 19) % 20]) / 20;
        if (abs(dx) < abs(8 * Vmean)) {
            if (flag == 20)
                flag = 0;
            px[flag] = p_tx_now;
        } else {
            if (flag == 20) {
                flag = 0;
            }
            px[flag] = px[(flag + 19) % 20];
        }
        //ax = (v_tx_now - px[flag])/20;
        Vmean = Vmean * 0.8 + dx * 0.2;//todo update v
    }

    bool Kalman::JudgeArmor_v(int &flag) {
        ax = (v_tx_now - vx[(flag + 19) % 20]) / 20;
        if (abs(ax) < abs(2.9 * Amean)) {
            if (flag == 20)
                flag = 0;
            vx[flag] = v_tx_now;
        } else {
            if (flag == 20) {
                flag = 0;
            }
            vx[flag] = vx[(flag + 19) % 20];
        }
        ax = (v_tx_now - vx[flag]) / 20;
        Amean = Amean * 0.85 + ax * 0.15;//todo update a
    }

    void Kalman::FirstSetFilter(Point2f  targetPoint) {
        //cout << "第一次滤波" << endl;
        //目标移动速度,单位为cm/ms
        v_tx_now = (targetPoint.x - p_tx_old) / t;
        float v_ty_now = (targetPoint.y - p_ty_old) / t;

        p_tx_now = targetPoint.x;// px,py in this pf
        p_ty_now = targetPoint.y;
        //todo record vx begin
        px_flag = 0;
        vx_flag = 0;
        px[px_flag] = p_tx_now;

        vx[px_flag] = v_tx_now;

        set_px = false;
        set_vx = false;
        Vmean = 0.08;//how to init Vmean?
        Amean = 0.08;

        fx = 0;
        //传参
        Eigen::VectorXd x(4, 1);
        x << targetPoint.x, targetPoint.y, v_tx_now * 1000, v_ty_now * 1000;
        KF_forecast.set_x(x);
        //update p & v
        p_tx_old = targetPoint.x;
        p_ty_old = targetPoint.y;

        old_objectP.x = p_tx_old;
        old_objectP.y = p_ty_old;

//        if((abs(v_ty_now)<=0.5)&&(abs(v_tx_now)>0.2)&&(abs(v_tx_now)<0.6)) {
//            v_tx_old = (v_tx_now)*50;
//            v_ty_old = v_ty_now;
//        }
//        else if((abs(v_ty_now)<=0.5)&&abs(v_tx_now)>=0.6)
//        {
//            v_tx_old = (v_tx_now)*10;
//            v_ty_old = v_ty_now;
//        }

    }


    void Kalman::ContinueSetFilter(const Point2f&  targetPoint) {
        //cout << "连续滤波" << endl;

        p_tx_now = targetPoint.x;// px,py in this pf
        p_ty_now = targetPoint.y;

        //todo judge if it is reasonable
        px_flag++;
        //vx_flag++;
        if (!set_px) {
            if (px_flag == 19) {
                set_px = true;
            }
            px[px_flag] = p_tx_now;
        }
        JudgeArmor(px_flag);

        v_tx_now = (px[(px_flag + 20) % 20] - px[(px_flag + 19) % 20]) / t;

        vx_flag++;
        if (!set_vx) {
            if (vx_flag == 19) {
                set_vx = true;
            }
            vx[vx_flag] = v_tx_now;
        }
        JudgeArmor_v(vx_flag);


//        float v_tx_now = (BestArmor.center.x - p_tx_old)/t;
//        float v_ty_now =( BestArmor.center.y - p_ty_old)/t;
        v_tx_now = vx[vx_flag];
        //float v_tx_now = (px[(px_flag+20)%20] - px[(px_flag+19)%20])/t;
        float v_ty_now = (targetPoint.y - p_ty_old) / t;

        float v_sum = 0;
        for (int t = 0; t < 19; t++) {
            v_sum = v_sum + vx[t];
        }
//        if(abs(v_sum)>1.7){
//            v_tx_old = v_tx_now;
//            v_ty_old = v_ty_now;
//        }

        if (abs(v_tx_now) > 0.15 && (abs(v_sum) > 1.6)) {
            v_tx_old = v_tx_now;
            v_ty_old = v_ty_now;
            if (v_sum >= 0)
                fx = 1;
            else
                fx = -1;
        }
        //todo calculate errors
        p_tx_old = targetPoint.x;
        p_ty_old = targetPoint.y;



//        if((abs(v_tx_now)>0.3)&&(abs(v_tx_now)<0.6)) {
//            v_tx_old = (v_tx_now)*50;
//            v_ty_old = v_ty_now;
//        }
//        else if(abs(v_tx_now)>=0.6)
//        {
//            v_tx_old = (v_tx_now)*10;
//            v_ty_old = v_ty_now;
//        }

        //todo record vx
//        px_flag++;
//        if(px_flag<20){
//            //px[px_flag] = (p_tx_old-px[0])/px_flag;
//            px[px_flag] = p_tx_old;
//            Vmean = 0.4*Vmean + 0.6*(p_tx_old-px[0])/px_flag;
//        }//not enought for 20 fp
//        else if(px_flag == 20){
//            px_flag = px_flag - 20;
//            if(abs(px[px_flag]-px[px_flag-19])<=abs(20*((Vmean*(1.1-w0))/w1)))   //judge
//                //px[px_flag] = (p_tx_old-px[px_flag-19])/20;
//                px[px_flag] = p_tx_old;
//            else
//                px[px_flag] = px[px_flag-1];
//            Vmean = Vmean*w1+((px[px_flag]-px[px_flag-19])/20)*w0;
//        }


//        delta_x =+ abs(p_tx_old - p_predictx);
//        delta_y =+ abs(p_ty_old - p_predicty);

        //cout<<'x'<<delta_x<<'y'<<delta_y<<endl;

        Eigen::VectorXd z(4, 1);
        //z<<BestArmor.center.x,BestArmor.center.y,v_tx_now*1000,v_ty_now*1000;
        //z<<BestArmor.center.x,BestArmor.center.y,v_tx_now*1000,v_ty_now*1000;
        z << px[px_flag], targetPoint.y, v_tx_now * 1000, v_ty_now * 1000;
        //得到状态转移矩阵
        Eigen::MatrixXd F_in(4, 4);
        F_in << 1.0, 0.0, t / 1000, 0.0,
                0.0, 1.0, 0.0, t / 1000,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

//预测上一最佳状态值
        KF_forecast.Prediction(F_in);

//更新状态量
        KF_forecast.update(z, F_in);

        //todo 是否需要这一步 传参
//        p_tx_old = KF_forecast.x_(0);
//        p_ty_old = KF_forecast.x_(1);
//        if((abs(v_tx_now)>0.25)&&(abs(v_tx_now)<0.5)) {
//            v_tx_old = (v_tx_now)*50;
//            v_ty_old = v_ty_now;
//        }
//        else if(abs(v_tx_now)>=0.5)
//        {
//            v_tx_old = (v_tx_now)*15;
//            v_ty_old = v_ty_now;
//        }
//        if((p_ty_old<70)&&(abs(v_tx_now)>0.15)){
//            v_tx_old = (v_tx_now)*50;
//            v_ty_old = v_ty_now;
//        }

//        if((v_ty_now>-0.1)&&(abs(v_tx_now)>0.3)&&(abs(v_tx_now)<0.6)) {
//            v_tx_old = (v_tx_now)*50;
//            v_ty_old = v_ty_now;
//        }
//        else if((abs(v_ty_now)<=0.5)&&abs(v_tx_now)>=0.6)
//        {
//            v_tx_old = (v_tx_now)*10;
//            v_ty_old = v_ty_now;
//        }

        //todo
//        v_tx_old = v_tx_now*50;
//        v_ty_old = v_ty_now;

    }

//todo 测量误差
    Point2f Kalman::SetKF(const Point2f& targetPoint) {
        Point2f position;
        float delta_x = 0;
        float delta_y = 0;

        if (flag) {
            FirstFind(targetPoint);
            KF_forecast.is_set_x = false;
            //状态协方差矩阵重新复位
            Eigen::MatrixXd P_in = Eigen::MatrixXd(4, 4);
            P_in << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;
            KF_forecast.P = P_in;//init
            //cout << "x:" << BestArmor.center.x << "y:" << BestArmor.center.y << endl;
            flag = false;
            return Point2f(0,0);
        }

        if (!KF_forecast.is_set_x) {
            //第一次连续射击
            cout << "First set Filter" << endl;
            FirstSetFilter(targetPoint);
        } else {
            cout << "Continue set Filter" << endl;
            ContinueSetFilter(targetPoint);
        }
//        v_tx_old = (BestArmor.center.x - p_tx_old)/t;
//        v_ty_old = (BestArmor.center.y - p_ty_old)/t;
        //double ShootTime = t + SHOOT_DELAY_TIME+Recive.getClock() - CarDatas.BeginToNowTime;
        double ShootTime = t;


        double raw_x = KF_forecast.x_(0);
        double raw_y = KF_forecast.x_(1);
        //todo predict
        //v_tx_now*shootTime
        position.x = KF_forecast.x_(0) + v_tx_old * 100;//fx
        //position.x = KF_forecast.x_(0)+v_tx_old*70;//+Vmean*50;//+KF_forecast.x_(2)/1000*ShootTime;
        position.y = KF_forecast.x_(1) + v_ty_old * 50;//*ShootTime;
        p_predictx = position.x;
        p_predicty = position.y;
//        position.x += (0.5*(KF_forecast.x_(2)/1000 - v_tx_old)*10*pow(ShootTime/1000.0,2))*100;
//        position.y += (0.5*(KF_forecast.x_(3)/1000 - v_ty_old)*10*pow(ShootTime/1000.0,2))*100;

        //update
//        v_tx_old = KF_forecast.x_(2)/1000;
//        v_ty_old = KF_forecast.x_(3)/1000;
//        cout << "x:" << position.x << "y:" << position.y << endl;

        return Point2f(position - targetPoint);
    }
}



