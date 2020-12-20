#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
using namespace std;
using namespace cv;

class SolveAngle
{
public:
	SolveAngle();
	float height_world = 60.0;
	float overlap_dist = 100000.0;
	float barrel_ptz_offset_x = -0;
	float barrel_ptz_offset_y = -0;

	float ptz_camera_x = 0;
	float ptz_camera_y = 52.5;
	float ptz_camera_z = -135;
	float scale = 0.99f;
	float f_ = 1500;

	float averageP;
	float averageY;
	vector<Point2f> rectPoint2D;
	void Generate2DPoints(Rect rect);
	void GetPose(const Rect& rect, float ballet_speed, float& yaw, float& pitch, float& dist, bool small);
    void GetPoseV(const vector<Point2f> pts, float ballet_speed, float& yaw, float& pitch, float& dist,bool small);
	void Generate3DPoints(bool mode);
private:
	Mat_<double> cameraMatrix;
	Mat_<double> distortionCoefficients;
	Mat rvecs;
	Mat tvecs;
	vector<Point3f> targetPoints3D;
	float targetWidth3D{};
	float targetHeight3D{};

	float historyPitch[4];
    float historyYaw[4];
    inline float averagePitch();
    inline float averageYaw();
    int curAngleCount;
private:
	float fx_{};
	float fy_{};
	float cx_{};
	float cy_{};


	
};
