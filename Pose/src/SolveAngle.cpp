#include "SolveAngle.hpp"
#include "Kalman.hpp"
#include "preoptions.h"

#define SHOOTFORMULAX(X) (60 - (abs(X - FRAMEWIDTH/2)))
#define SHOOTFORMULAY(Y) (40 - (abs(Y - FRAMEHEIGHT/2)))
#define SHOOTFORMULA(X,Y) (3600 - ((Y - FRAMEHEIGHT/2)*(Y - FRAMEHEIGHT/2) + (X - FRAMEWIDTH/2)*(X - FRAMEWIDTH/2)))

using namespace cv;

SolveAngle::SolveAngle()
{
	string filename ="../Pose/camera.xml";
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened()) {
		cout << "no such file" << endl;
		return;
	}
	switch(carName)
    {
        case HERO:
            fs["Distortion_Coefficients4_Realsense"] >> distortionCoefficients;
            fs["Camera_Matrix_Realsense"] >> cameraMatrix;
            break;
        case INFANTRY_MELEE:
            fs["Distortion_Coefficients_INFANTRY_1"] >> distortionCoefficients;
            fs["Camera_Matrix_INFANTRY_1"] >> cameraMatrix;
            break;
        case INFANTRY_TRACK:
            fs["Distortion_Coefficients_INFANTRY_NONE_1"] >> distortionCoefficients;
            fs["Camera_Matrix_INFANTRY_NONE_1"] >> cameraMatrix;
            break;
        case VIDEO:
        case SENTRY:
            fs["Distortion_Coefficients_DAHUA_1"] >> distortionCoefficients;
            fs["Camera_Matrix_DAHUA_1"] >> cameraMatrix;
            break;
        case UAV:
            break;
        case NOTDEFINED:
            break;
    }
	fs.release();
}

void SolveAngle::Generate2DPoints(Rect rect)
{
	Point2f pt;
	pt.x = rect.x;
	pt.y = rect.y;
    rectPoint2D.push_back(pt);

	pt.x = rect.x + rect.width;
	pt.y = rect.y;
    rectPoint2D.push_back(pt);

	pt.x = rect.x + rect.width;
	pt.y = rect.y + rect.height;
    rectPoint2D.push_back(pt);

	pt.x = rect.x;
	pt.y = rect.y + rect.height;
    rectPoint2D.push_back(pt);
}

void SolveAngle::GetPose(const Rect& rect, float ballet_speed, bool small)
{
	Kalman1  kalman;
    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
	Generate3DPoints(small);
	rvecs = Mat::zeros(3, 1, CV_64FC1);
	tvecs = Mat::zeros(3, 1, CV_64FC1);
	Generate2DPoints(rect);
	solvePnP(targetPoints3D, rectPoint2D, cameraMatrix, distortionCoefficients, rvecs, tvecs,false, SOLVEPNP_P3P);
    rvecs.convertTo(Rvec, CV_32F);    //旋转向量
    tvecs.convertTo(Tvec, CV_32F);   //平移向量

    cv::Mat_<float> rotMat(3, 3);
    cv::Mat_<float> R_n(3, 3);
    cv::Mat_<float> P_oc;
    cv::Rodrigues(Rvec, rotMat);  //由于solvePnP返回的是旋转向量，故用罗德里格斯变换变成旋转矩阵
    float cos_beta = sqrt(rotMat.at<float>(0, 0) * rotMat.at<float>(0, 0) + \
                            rotMat.at<float>(1, 0) * rotMat.at<float>(1, 0));  //矩阵元素下标都从0开始（对应公式中是sqrt(r11*r11+r21*r21)），sy=sqrt(cosβ*cosβ)
    float alpha,beta,gamma;
    if(cos_beta>1e-6)
    {
        alpha = atan2(rotMat.at<float>(2, 1),rotMat.at<float>(2, 2))*180/3.14 - 25;
        beta  = atan2(-rotMat.at<float>(2, 0),cos_beta)*180/3.14 - 7;
        gamma = atan2(rotMat.at<float>(1, 0),rotMat.at<float>(0, 0))*180/3.14;
    }
    else
    {
        alpha = atan2(-rotMat.at<float>(1, 2),rotMat.at<float>(1, 1))*180/3.14 - 25;
        beta  = atan2(-rotMat.at<float>(2, 0),cos_beta)*180/3.14 - 7;
        gamma = 0;
    }

    yaw = alpha,pitch = beta;
    //格式转换
//    Eigen::MatrixX2f R_n(3,3);
//    Eigen::MatrixX2f T_n(Tvec.rows,Tvec.cols);
    cv::invert(rotMat,R_n);
//    cv2eigen(rotMat, R_n);
//    cv2eigen(Tvec, T_n);
//
//    Eigen::Vector3f P_oc;
    P_oc = -R_n*Tvec;
    dist =  sqrt(P_oc.at<float>(0,0)*P_oc.at<float>(0,0) + P_oc.at<float>(1,0)*P_oc.at<float>(1,0)\
            + P_oc.at<float>(2,0)*P_oc.at<float>(2,0));

//    historyPitch[curAngleCount] = pitch;
//    historyYaw[curAngleCount] = yaw;
//    curAngleCount = (++curAngleCount)%4;

//    averageP = averagePitch();
//    averageY = averageYaw();

    rectPoint2D.clear();
    targetPoints3D.clear();
}

void SolveAngle::GetPoseV(Point2f predictOffset, const vector<Point2f>& pts, float ballet_speed, bool small)
{
    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    Generate3DPoints(small);
    rvecs = Mat::zeros(3, 1, CV_64FC1);
    tvecs = Mat::zeros(3, 1, CV_64FC1);

    if(pts.size() != 4)return;

    for(auto p:pts)
    {
        p += predictOffset;
    }

    solvePnP(targetPoints3D, pts, cameraMatrix, distortionCoefficients, rvecs, tvecs,false, SOLVEPNP_ITERATIVE);
    rvecs.convertTo(Rvec, CV_32F);    //旋转向量
    tvecs.convertTo(Tvec, CV_32F);   //平移向量

    yaw = atan(tvecs.at<double>(0, 0) / tvecs.at<double>(2, 0)) / 2 / CV_PI * 360;
    pitch = -1.0*atan(tvecs.at<double>(1, 0) / tvecs.at<double>(2, 0)) / 2 / CV_PI * 360;
    dist = sqrt(tvecs.at<double>(0, 0)*tvecs.at<double>(0, 0) + tvecs.at<double>(1, 0)*tvecs.at<double>(1, 0) + tvecs.at<double>(2, 0)* tvecs.at<double>(2, 0));

    averageX = 0;
    averageY = 0;

    for(auto &p:pts)
    {
        averageX += 1.0/4*p.x;
        averageY += 1.0/4*p.y;
    }

/**********************************************************************************************************************
 *                                            Shoot Logic                                                             *
 *********************************************************************************************************************/
    shootPriority += SHOOTFORMULA(averageX,averageY);
    shootPriority = (shootPriority<0)?(0):((shootPriority > 1000)?(1000):(shootPriority));
    shoot = (shootPriority > 0);

    rectPoint2D.clear();
    targetPoints3D.clear();
}

void SolveAngle::Generate3DPoints(bool mode)
{
    //because the armor is  incline,so the height of the armor should be smaller than reality.
	if (mode)
	{
        targetHeight3D = 125;
        targetWidth3D = 135;

	}
	else
	{
        targetHeight3D = 125;
        targetWidth3D = 230;

	}
    targetPoints3D.emplace_back(-targetWidth3D / 2, -targetHeight3D / 2, 0);
    targetPoints3D.emplace_back(targetWidth3D / 2, -targetHeight3D / 2, 0);
    targetPoints3D.emplace_back(targetWidth3D / 2, targetHeight3D / 2, 0);
    targetPoints3D.emplace_back(-targetWidth3D / 2, targetHeight3D / 2, 0);
}

