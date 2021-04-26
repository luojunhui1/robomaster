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
    }

    void KF_two::set_x(Eigen::VectorXd x) {
        x_ = x;
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

    /**
     * @brief Judge current input point is an armor or not through the compare the current velocity with local velocity
     * parameter, update px to make sure px[flag] stores the most possible x coordinate index of the current armor.
     * @param flag actually an index to operate px circularly
     * @return none
     * @details none
     */
    bool Kalman::JudgeArmor(int &flag) {
        dx = (p_tx_now - px[(flag + (FLAME-1)) % FLAME]) / FLAME;
        if (abs(dx) < abs(8 * Vmean)) {
            if (flag == FLAME)
                flag = 0;
            px[flag] = p_tx_now;
        } else {
            if (flag == FLAME) {
                flag = 0;
            }
            px[flag] = px[(flag + (FLAME-1)) % FLAME];
        }
        //ax = (v_tx_now - px[flag])/20;
        Vmean = Vmean * 0.8 + dx * 0.2;//todo update v
    }

    bool Kalman::JudgeArmor_v(int &flag) {
        ax = (v_tx_now - vx[(flag + (FLAME-1)) % FLAME]) / FLAME;
        if (abs(ax) < abs(2.9 * Amean)) {
            if (flag == FLAME)
                flag = 0;
            vx[flag] = v_tx_now;
        } else {
            if (flag == FLAME) {
                flag = 0;
            }
            vx[flag] = vx[(flag + (FLAME-1)) % FLAME];
        }
        ax = (v_tx_now - vx[flag]) / FLAME;
        Amean = Amean * 0.85 + ax * 0.15;//todo update a
    }

    void Kalman::FirstSetFilter(Point2f  targetPoint) {
        //cout << "第一次滤波" << endl;
        //目标移动速度,单位为cm/ms
        v_tx_now = (targetPoint.x - p_tx_old) / t;

        /*local variable v_ty_now ? */
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

        /*velocity multiplied with 1000?*/
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


    bool Kalman::UpdateFilter(const Point2f&  targetPoint) {

        p_tx_now = targetPoint.x;
        p_ty_now = targetPoint.y;

        //first update x coordination and y coordination
        if(p_tx_old == 0 && p_ty_old == 0)
        {
            p_tx_old = p_tx_now;
            p_ty_old = p_ty_now;

            return false;
        }

        //first time detected target armor continuously, update velocity parameters
        if(v_tx_old == 0 && v_ty_old == 0)
        {
            v_tx_now = (targetPoint.x - p_tx_old) / t;

            v_ty_now = (targetPoint.y - p_ty_old) / t;

            p_tx_now = targetPoint.x;
            p_ty_now = targetPoint.y;

            px_flag = 0;
            vx_flag = 0;

            px[px_flag] = p_tx_now;

            vx[px_flag] = v_tx_now;

            set_px = false;
            set_vx = false;

            Vmean = 0.02;//how to init Vmean?
            Amean = 0.04;

            fx = 0;

            //传参
            Eigen::VectorXd x(4, 1);

            /*velocity multiplied with 1000?*/
            x << targetPoint.x, targetPoint.y, v_tx_now * 1000, v_ty_now * 1000;
            KF_forecast.set_x(x);

            //update p & v
            p_tx_old = targetPoint.x;
            p_ty_old = targetPoint.y;

            v_tx_old = v_tx_now;
            v_ty_old = v_ty_now;

            return true;
        }

        //todo judge if it is reasonable
        /**predict coordinate**/
        px_flag++;

        /**judge if px have been updated 20 times or not**/
        if (!set_px) {
            if (px_flag == (FLAME-1)) {
                set_px = true;
            }
            px[px_flag] = p_tx_now;
        }
        JudgeArmor(px_flag);

        /**predict velocity**/
        v_tx_now = (px[(px_flag + FLAME) % FLAME] - px[(px_flag + (FLAME-1)) % FLAME]) / t;
        vx_flag++;

        if (!set_vx) {
            if (vx_flag == (FLAME-1)) {
                set_vx = true;
            }
            vx[vx_flag] = v_tx_now;
        }
        JudgeArmor_v(vx_flag);

        v_tx_now = vx[vx_flag];
        v_ty_now = (targetPoint.y - p_ty_old) / t;

        float v_sum = 0;
        for (int i = 0; i < (FLAME-1); i++) {
            v_sum = v_sum + vx[i];
        }

        if (abs(v_tx_now) > 0.03 && (abs(v_sum) > 0.4)) {
            v_tx_old = v_tx_now;
            v_ty_old = v_ty_now;
            if (v_sum >= 0)
                fx = 1;
            else
                fx = -1;
        }

        Eigen::VectorXd z(4, 1);
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

        p_tx_old = p_tx_now;
        p_ty_old = p_ty_now;

        v_tx_old = v_tx_now;
        v_ty_old = v_ty_now;

        return true;
    }

    Point2f Kalman::GetShoot(Point2f forecast_position,float target_speed_x,float target_speed_y,float shoot_time){
        Point2f shoot_point;
        shoot_point.x = forecast_position.x + (target_speed_x)*(shoot_time);
        shoot_point.y = forecast_position.y + (target_speed_y)*(shoot_time);
        return shoot_point;
    }

//todo 测量误差
    Point2f Kalman::SetKF(const Point2f& targetPoint, int clear) {
//        cout<<"Come in SetKF"<<endl;

        Point2f position;
        float delta_x = 0;
        float delta_y = 0;

        //clear filter parameters
        if (clear > 0) {
            p_tx_old = 0;
            p_ty_old = 0;

            v_tx_old = 0;
            v_ty_old = 0;

            //状态协方差矩阵重新复位
            Eigen::MatrixXd P_in = Eigen::MatrixXd(4, 4);
            P_in << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;
            KF_forecast.P = P_in;//init
            return Point2f(0,0);
        }

        cout<<"Come in Update Filter"<<endl;
        if(!UpdateFilter(targetPoint))
        {
            cout<<"Come in Update Filter"<<endl;
            return Point2f(0,0);
        }

        // position.x = KF_forecast.x_(0) + v_tx_old * 100;//fx
        // position.y = KF_forecast.x_(1) + v_ty_old * 50;//*ShootTime;

        cout<<"Come in KF Forecast"<<endl;
        position.x = KF_forecast.x_(0);
        position.y = KF_forecast.x_(1);

        cout<<"Old Update"<<endl;
        v_tx_old = KF_forecast.x_(2)/1000;
        v_ty_old = KF_forecast.x_(3)/1000;

//        cout<<"Come in GetShoot"<<endl;
        position = GetShoot(position,v_tx_old,v_ty_old,100);//shoot_time as 100

        p_predictx = position.x;
        p_predicty = position.y;

        return Point2f(position - targetPoint);
    }
}



