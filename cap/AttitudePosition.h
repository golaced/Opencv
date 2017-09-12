#ifndef __AttitudePosition_H__
#define __AttitudePosition_H__

#include <iostream>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include <sstream>
#include <opencv2/opencv.hpp>


class Camera
{
public:


        float x,y,z;
	float Pit;
	float Yaw;
 	float Rol;
        int check;
        int cx,cy,cr;

private:

};

std::vector<cv::Mat> getR(float alpha_X, float alpha_Y, float alpha_Z);
extern float mark_map[6][5];
void getCameraPos(cv::Mat Rvec, cv::Mat Tvec, cv::Point3f &pos);
void getCameraPosWithMarkers(std::vector< aruco::Marker > Markers, Camera &atti_camera);
void getAttitude(aruco::Marker marker, Camera &attitude);
extern 	Camera attitude_camera;
#endif // __AttitudePosition_H__
