#include "select.h"
using namespace cv;
using namespace std;


Mat markerMask, img;
Point prevPt(-1, -1);

cv::Rect auto_select(Mat img0 , Point pt)
{
    Mat imgGray;
    resize(img0,img0,Size(320,240),0,0,CV_INTER_LINEAR);
    cv::Mat binary;// = cv::imread("binary.bmp",0);  //Ã»¼Ó0º¦µÃÎÒÕÒºÃ¾Ã´íÎóµÄÔ­Òò£¬¼´Ê¹"binary.bmp"Îª¶þÖµÍ¼Ïñ²»¼Ó0¶Á½øÀ´»¹ÊÇ3Í¨µÀ
     cv::threshold(img0,binary,60,255,CV_THRESH_BINARY);  //ãÐÖµ»¯²Ù×÷
    // binary = 255 - binary; //ÈÃÇ°¾°±äÎª°×É«ÇøÓò
    // cv::namedWindow("binary",CV_WINDOW_AUTOSIZE);
     //cv::imshow("binary",binary);

     cv::Mat fImage;
     cv::erode(binary,fImage,cv::Mat(),cv::Point(-1,-1),6); //binary = 255 - binary; //ÈÃÇ°¾°±äÎª°×É«ÇøÓò//¸¯Ê´È¥µôÐ¡µÄ¸ÉÈÅÎïÌåµÃµ½Ç°¾°Í¼Ïñ
    // cv::namedWindow("eroded",CV_WINDOW_AUTOSIZE);
    // cv::imshow("eroded",fImage);

     cv::Mat bImage;
     cv::dilate(binary,bImage,cv::Mat(),cv::Point(-1,-1),6);
     cv::threshold(bImage,bImage,1,128,cv::THRESH_BINARY_INV);//¶ÔÔ­Ê¼¶þÖµÍ¼ÏñãÐÖµ»¯²¢È¡·´£¬µÃµ½±³¾°Í¼Ïñ
     //cv::namedWindow("bImage",CV_WINDOW_AUTOSIZE);
    // cv::imshow("bImage",bImage);

     Mat markers2(markerMask.size(), CV_32S);
     markers2 = fImage + bImage;  //´´½¨±ê¼ÇÍ¼Ïñ
     //cv::namedWindow("marker",CV_WINDOW_AUTOSIZE);
   // cv::imshow("marker",markers2);

    img0.copyTo(img);
    cvtColor(img, markerMask, COLOR_BGR2GRAY);
    cvtColor(markerMask, imgGray, COLOR_GRAY2BGR);
    markerMask = Scalar::all(0);
    //imshow( "image", img );

//mask

         CvPoint a = cvPoint(1,1);
         CvPoint b = cvPoint( img.cols -1, img.rows-1);
#define SIZE_MASK 2
#define SIZE2 1
#define DIS 6
     rectangle(markerMask, Point(pt.x-SIZE_MASK-DIS,pt.y-SIZE_MASK-DIS), Point(pt.x+SIZE_MASK-DIS,pt.y+SIZE_MASK-DIS), Scalar::all(255), SIZE2);
     rectangle(img, Point(pt.x-SIZE_MASK-DIS,pt.y-SIZE_MASK-DIS), Point(pt.x+SIZE_MASK-DIS,pt.y+SIZE_MASK-DIS), Scalar::all(255), SIZE2);
	rectangle(markerMask, Point(pt.x-SIZE_MASK+DIS,pt.y-SIZE_MASK-DIS), Point(pt.x+SIZE_MASK+DIS,pt.y+SIZE_MASK-DIS), Scalar::all(255), SIZE2);
	rectangle(img, Point(pt.x-SIZE_MASK+DIS,pt.y-SIZE_MASK-DIS), Point(pt.x+SIZE_MASK+DIS,pt.y+SIZE_MASK-DIS), Scalar::all(255), SIZE2);
	rectangle(markerMask, Point(pt.x-SIZE_MASK+DIS,pt.y-SIZE_MASK+DIS), Point(pt.x+SIZE_MASK+DIS,pt.y+SIZE_MASK+DIS), Scalar::all(255), SIZE2);
	rectangle(img, Point(pt.x-SIZE_MASK+DIS,pt.y-SIZE_MASK+DIS), Point(pt.x+SIZE_MASK+DIS,pt.y+SIZE_MASK+DIS), Scalar::all(255), SIZE2);
	rectangle(markerMask, Point(pt.x-SIZE_MASK-DIS,pt.y-SIZE_MASK+DIS), Point(pt.x+SIZE_MASK-DIS,pt.y+SIZE_MASK+DIS), Scalar::all(255), SIZE2);
	 rectangle(img, Point(pt.x-SIZE_MASK-DIS,pt.y-SIZE_MASK+DIS), Point(pt.x+SIZE_MASK-DIS,pt.y+SIZE_MASK+DIS), Scalar::all(255), SIZE2);

         rectangle(markerMask, a, b, Scalar::all(255), 12);
         rectangle(img, a, b, Scalar::all(255), 12);





            int i, j,k, compCount = 0;
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;

            findContours(markerMask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

           // if( contours.empty() )
            //    break;
            Mat markers(markerMask.size(), CV_32S);
            markers = Scalar::all(0);
            int idx = 0;
            for( ; idx >= 0; idx = hierarchy[idx][0], compCount++ )
                drawContours(markers, contours, idx, Scalar::all(compCount+1), -1, 8, hierarchy, INT_MAX);

           // if( compCount == 0 )
            //    continue;

            vector<Vec3b> colorTab;
            for( i = 0; i < compCount; i++ )
            {
                int b = theRNG().uniform(0, 255);
                int g = theRNG().uniform(0, 255);
                int r = theRNG().uniform(0, 255);

                colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
            }

            double t = (double)getTickCount();
            //imshow( "markers", markers );

            watershed( img0, markers );
            t = (double)getTickCount() - t;
            //printf( "execution time = %gms\n", t*1000./getTickFrequency() );

            Mat wshed(markers.size(), CV_8UC3);
            Mat wshed2(markers.size(), CV_8UC3);
            for( i = 0; i < markers.rows; i++ )
                         for( j = 0; j < markers.cols; j++ )
                         {
                        	 wshed2.at<Vec3b>(i,j) = Vec3b(0,0,0);
                        	// wshed.at<Vec3b>(i,j) = Vec3b(0,0,0);
                         }
            // paint the watershed image
            for( i = 0; i < markers.rows; i++ )
                for( j = 0; j < markers.cols; j++ )
                {
                    int index = markers.at<int>(i,j);
                    if( index == -1 )
                        {wshed.at<Vec3b>(i,j) = Vec3b(255,255,255);
                        wshed2.at<Vec3b>(i,j) = Vec3b(255,255,255);
                        }
                    else if( index <= 0 || index > compCount )
                    { wshed.at<Vec3b>(i,j) = Vec3b(0,0,0);
                    wshed2.at<Vec3b>(i,j) = Vec3b(0,0,0);
                    }
                    else
                    { wshed.at<Vec3b>(i,j) = colorTab[index - 1];}
                }

            wshed = wshed*0.5 + imgGray*0.5;
           //imshow( "watershed transform", wshed );
          // imshow( "watershed transform2", wshed2 );
            Mat wshed2_fill;
            wshed2.copyTo(wshed2_fill);
               for( i = 5; i < wshed2.cols-5; i++ )  //col y  row x
               { int xr,yr;
                  xr=0;yr=0;
                               for( j = 5; j < wshed2.rows-5; j++ )
                               {
                            	   Vec3b &index = wshed2.at<Vec3b>(j,i);
                            	    if( index[0] == 255&&xr>0){
                            	                                	   for( k = xr; k < j; k++ )
                            	                                		   wshed2_fill.at<Vec3b>(k,i) = Vec3b(255,255,255);
                            	                                        xr=0;}
                                       if( index[0] == 255&&xr==0){
                                       xr=j;
                                       }
                               }
               }

             //  imshow( "watershed transform3", wshed2_fill );
#define ttt  10
               Mat element = getStructuringElement(MORPH_RECT, Size(ttt, ttt));
               morphologyEx(wshed2_fill, wshed2_fill, MORPH_OPEN, element);
                morphologyEx(wshed2_fill, wshed2_fill, MORPH_CLOSE, element);
               //imshow( "watershed transform4", wshed2_fill );

               Mat thr_out;
               vector<vector<Point> >  contours1;
               vector<Vec4i> hierarchy1;
               Mat fill_gray;
               cvtColor(wshed2_fill,fill_gray,COLOR_BGR2GRAY);
               threshold(fill_gray,thr_out,60,255,CV_THRESH_BINARY);  //ãÐÖµ»¯²Ù×÷
               findContours(thr_out,contours1,hierarchy1,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));
               vector<vector<Point> > contours_poly(contours1.size());
               vector<Rect> boundRect(contours1.size());
               vector<Point2f>center(contours1.size());
               vector<float>radius(contours1.size());
               for(int i=0;i<contours1.size();i++)
               {
            	   approxPolyDP(Mat(contours1[i]),contours_poly[i],3,1);
            	   boundRect[i]=boundingRect(Mat(contours_poly[i]));
            	   minEnclosingCircle(contours_poly[i],center[i],radius[i]);
               }
            	   Mat drawing=Mat::zeros(thr_out.size(),CV_8UC3);
            	  // boundRect[0].height=LIMIT(boundRect[0].height,80,120);
            	  // boundRect[0].width=LIMIT(boundRect[0].width,80,120);
            	   for(int i=0;i<contours1.size();i++)
            	                  {
            	               	   rectangle(img,boundRect[i].tl(),boundRect[i].br(),CV_RGB(255,0,0),2,8,0);
            	                  }
            	   //imshow( "image", img );



    return boundRect[0];
}
