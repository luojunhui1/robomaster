#include <opencv2/core/eigen.hpp>
#include "SolveAngle.hpp"
#include "Kalman.hpp"

using namespace cv;

SolveAngle::SolveAngle()
{
	string filename ="../camera.xml";
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened()) {
		cout << "no such file" << endl;
		return;
	}
	fs["Distortion_Coefficients"] >> distortionCoefficients;
	fs["Camera_Matrix"] >> cameraMatrix;
	fs.release();
}

void SolveAngle::Generate2DPoints(Rect& rect)
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

void SolveAngle::getAngle(Rect rect, float ballet_speed, float& yaw, float& pitch, float& dist,bool small)
{
	Kalman1  kalman;
    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
	Generate3DPoints(small);
	rvecs = Mat::zeros(3, 1, CV_64FC1);
	tvecs = Mat::zeros(3, 1, CV_64FC1);
	Generate2DPoints(rect);
	solvePnP(targetPoints3D, rectPoint2D, cameraMatrix, distortionCoefficients, rvecs, tvecs,false, CV_P3P);
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
        alpha = atan2(rotMat.at<float>(2, 1),rotMat.at<float>(2, 2))*180/3.14;
        beta  = atan2(-rotMat.at<float>(2, 0),cos_beta)*180/3.14;
        gamma = atan2(rotMat.at<float>(1, 0),rotMat.at<float>(0, 0))*180/3.14;
    }
    else
    {
        alpha = atan2(-rotMat.at<float>(1, 2),rotMat.at<float>(1, 1))*180/3.14;
        beta  = atan2(-rotMat.at<float>(2, 0),cos_beta)*180/3.14;
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
    dist =  sqrt(P_oc.at<float>(0,0)*P_oc.at<float>(0,0) + P_oc.at<float>(0,1)*P_oc.at<float>(0,1)\
            + P_oc.at<float>(0,2)*P_oc.at<float>(0,2));
}

void SolveAngle::Generate3DPoints(bool mode)
{
	if (mode)
	{
        targetHeight3D = 57;
        targetWidth3D = 135;

	}
	else
	{
        targetHeight3D = 57;
        targetWidth3D = 230;

	}
    targetPoints3D.emplace_back(-targetWidth3D / 2, -targetHeight3D / 2, 0);
    targetPoints3D.emplace_back(targetWidth3D / 2, -targetHeight3D / 2, 0);
    targetPoints3D.emplace_back(targetWidth3D / 2, targetHeight3D / 2, 0);
    targetPoints3D.emplace_back(-targetWidth3D / 2, targetHeight3D / 2, 0);
}

