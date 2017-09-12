/*
 * select.h
 *
 *  Created on: May 26, 2016
 *      Author: odroid
 */

#ifndef SELECT_H_
#define SELECT_H_

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv.h"
#include <cstdio>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cxcore.h>

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
cv::Rect auto_select(cv::Mat img0 , cv::Point pt);


#endif /* SELECT_H_ */
