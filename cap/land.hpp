#ifndef LAND_HPP
#define LAND_HPP

#include <iostream>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include <sstream>
#include <opencv2/opencv.hpp>


typedef struct
{
  float x;
  float y;
  float size;
  int  id;
  int check;
}MARK;

extern MARK mark[4];

#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif // LAND_HPP
