/*
 * imageTrans.cpp
 *
 *  Created on: Sep 14, 2016
 *      Author: odroid
 */

#include "imageTrans.h"
#include <iomanip>
using namespace std;
using namespace cv;

//void warpFfine(cv::Mat &inputIm, cv::Mat &tempImg, float angle)
//{
//	CV_Assert(!inputIm.empty());
//	Mat inputImg;
//	inputIm.copyTo(inputImg);
//	float radian = (float) (angle / 180.0 * CV_PI);
//	int uniSize = (int) (max(inputImg.cols, inputImg.rows) * 1.414);
//	int dx = (int) (uniSize - inputImg.cols) / 2;
//	int dy = (int) (uniSize - inputImg.rows) / 2;
//	copyMakeBorder(inputImg, tempImg, dy, dy, dx, dx, BORDER_CONSTANT);
//	Point2f center((float) (tempImg.cols / 2), (float) (tempImg.rows / 2));
//	Mat affine_matrix = getRotationMatrix2D(center, angle, 1.0);
//	warpAffine(tempImg, tempImg, affine_matrix, tempImg.size());
//	float sinVal = fabs(sin(radian));
//	float cosVal = fabs(cos(radian));
//	Size targetSize((int) (inputImg.cols * cosVal + inputImg.rows * sinVal),
//			(int) (inputImg.cols * sinVal + inputImg.rows * cosVal));
//	int x = (tempImg.cols - targetSize.width) / 2;
//	int y = (tempImg.rows - targetSize.height) / 2;
//	Rect rect(x, y, targetSize.width, targetSize.height);
//	tempImg = Mat(tempImg, rect);
//}

std::vector<std::string> state_str;

void init()
{
	state_str.resize(30, "none");
    state_str[SG_LOW_CHECK] 		= 	"G_LC";
    state_str[SG_MID_CHECK]			=	"G_MC";
	state_str[SU_UP1] 				= 	"U_UP1";
	state_str[SU_HOLD] 				= 	"U_HOLD";
	state_str[SD_RETRY_UP] 			= 	"R_UP";
	state_str[SD_RETRY_UP_HOLD] 	= 	"R_HOLD";
	state_str[SD_CHECK_TARGET] 		= 	"CHK_TARGET";
	state_str[SD_FLY_TARGET] 		= 	"FLY_TARGET";
	state_str[TO_START] 			= 	"TO_START";

	state_str[SD_HOLD] 				= 	"D_HOLD";
	state_str[SD_MISS_SEARCH] 		= 	"D_MISS";
	state_str[SD_HOLD2] 			= 	"D_HOLD2";
	state_str[SD_HIGH_FAST_DOWN] 	= 	"D_FastD";
	state_str[SD_CIRCLE_SLOW_DOWN] 	= 	"D_CSD";
	state_str[SD_CIRCLE_HOLD] 		= 	"D_CHOLD";
	state_str[SD_CIRCLE_MID_DOWN] 	= 	"D_CMD";
	state_str[SD_CHECK_G] 			= 	"D_GC";
	state_str[SD_SHUT_DOWN] 		= 	"SHUT_DOWN";
	state_str[SD_SAFE] 				= 	"D_SAFE";
	state_str[HOLD_BACK] 			= 	"HOLD_BACK";
	state_str[BREAK] 				= 	"BREAK";
	state_str[SHUT] 				= 	"SHUT";
	state_str[GO_HOME] 				= 	"GO_HOME";
}

int video_num = 0;
//std::ifstream video_num_read;
std::ofstream video_num_write;
#ifdef WIN32
std::string video_num_path(
	"D:/IMAV/VideoLocation/video_num.txt");
std::string writer_path("D:/IMAV/VideoLocation/");
#else
std::string video_num_path(
    "/home/linaro/QT/cap/video/video_num.txt");
std::string writer_path("/home/linaro/QT/cap/video/");

#endif // WIN32



void startWriteVideo(std::ifstream &video_num_read,
		cv::VideoWriter &video_writer)
{
	video_num_read.open(video_num_path.c_str());
	video_num_read >> video_num;
	video_num_read.close();

	cout << video_num << endl;

	video_num_write.open(video_num_path.c_str());
	video_num_write << (video_num + 1);
	video_num_write.close();

	if (video_writer.isOpened())
	{
		video_writer.release();
	}

	std::stringstream ss;
	string video_name;

	ss << video_num;
	ss >> video_name;
	video_name += ".avi";

	video_writer.open(
			writer_path + video_name,
            CV_FOURCC('D', 'I', 'V', 'X'), 15, Size(320, 240));

}

void startWriteFile(std::ofstream &gps_writer)
{
	std::stringstream ss;
	string file_name;

	ss << video_num;
	ss >> file_name;
	file_name += ".txt";
	gps_writer.open(writer_path + file_name);
        gps_writer<<setprecision(15);
	//gps_writer << "latitude    longitude    height" << endl;
}
