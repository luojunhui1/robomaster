#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/LU"
using namespace Eigen;
using namespace std;
using namespace cv;

class SolveAngle
{
public:
	SolveAngle();

	vector<Point2f> rectPoint2D;
	void Generate2DPoints(Rect& rect);
	void GetAngle(Rect rect, float balletSpeed,bool small);
	void Generate3DPoints(bool mode);
private:
	Mat cameraMatrix;
	Mat distortionCoefficients;
	Mat rvecs;
	Mat tvecs;
	vector<Point3f> targetPoints3D;
	float targetWidth3D;
	float targetHeight3D;
public:
    float distance;
    float yaw;
    float pitch;

    float lastDistance;
    float lastYaw;
    float lastPitch;
};
