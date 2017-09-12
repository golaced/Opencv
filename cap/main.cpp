
#include "iostream"
#include "imageTrans.h"
#include <sys/time.h>
#include "select.h"
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <dlib/opencv.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <serial/serial.h>
#include "inifile.h"
#include "land.hpp"
#include "AttitudePosition.h"
using namespace std;
using namespace cv;
using namespace dlib;
using namespace aruco;
using namespace inifile;

extern int write_flag;
extern int newfile_flag;
std::ifstream video_num_read;

void uartSent(void);
void uartSent2(void);
serial::Serial serial_port("/dev/ttyS1",115200 , serial::Timeout::simpleTimeout(1000));

std::vector< aruco::Marker > Markers;
correlation_tracker tracker;
dlib::rectangle getInitPosition(cv::Rect r) {
    return dlib::rectangle(r.x, r.y, r.x + r.width, r.y + r.height);
}

cv::Rect dlibRect2CVRect(dlib::rectangle r) {
    return cv::Rect(r.left(), r.top(), r.width(), r.height());
}

#define MARKER_CONFIG_FILENAME std::string("/home/linaro/QT/cap/markerconfig.ini")
float Scale_pix=1;
float scale_markers=1;
int marker_row_num;
int marker_col_num;
double marker_size;
double markers_row_distance;
double markers_col_distance;
int show_traj,save_video,use_kcf;
int camera_sel,en_detect_switch,en_auto_selsect;
int cnt_dt,cnt_td;
string cal_file;
MARK mark[4];
void loadMarkerConfig(const std::string &configfilename)
{
    inifile::IniFile m_inifile;
    m_inifile.load(configfilename);
    std::string section("MarkerConfig");
    std::string section1("Tracking");
    int ret = 0;
    scale_markers= m_inifile.getDoubleValue(section, "scale_markers", ret) ;
    marker_size=mark[0].size = m_inifile.getDoubleValue(section, "mark0_size", ret)/100.*scale_markers;
    mark[0].x = m_inifile.getDoubleValue(section, "mark0_x", ret)*scale_markers;
    mark[0].y = m_inifile.getDoubleValue(section, "mark0_y", ret)*scale_markers;
    mark[0].id = m_inifile.getIntValue(section, "mark0_id", ret);

    mark[1].size = m_inifile.getDoubleValue(section, "mark1_size", ret)/100.*scale_markers;
    mark[1].x = m_inifile.getDoubleValue(section, "mark1_x", ret)*scale_markers;
    mark[1].y = m_inifile.getDoubleValue(section, "mark1_y", ret)*scale_markers;
    mark[1].id = m_inifile.getIntValue(section, "mark1_id", ret);

    mark[2].size = m_inifile.getDoubleValue(section, "mark2_size", ret)/100.*scale_markers;
    mark[2].x = m_inifile.getDoubleValue(section, "mark2_x", ret)*scale_markers;
    mark[2].y = m_inifile.getDoubleValue(section, "mark2_y", ret)*scale_markers;
    mark[2].id = m_inifile.getIntValue(section, "mark2_id", ret);

    mark[3].size = m_inifile.getDoubleValue(section, "mark3_size", ret)/100.*scale_markers;
    mark[3].x = m_inifile.getDoubleValue(section, "mark3_x", ret)*scale_markers;
    mark[3].y = m_inifile.getDoubleValue(section, "mark3_y", ret)*scale_markers;
    mark[3].id = m_inifile.getIntValue(section, "mark3_id", ret);


    camera_sel=m_inifile.getIntValue(section, "camera_sel", ret);
    show_traj=m_inifile.getIntValue(section, "show_traj", ret);
    cal_file=m_inifile.getStringValue(section, "cal_file", ret);
    save_video=m_inifile.getIntValue(section, "save_video", ret);
    use_kcf=m_inifile.getIntValue(section1,"use_kcf",ret);
    en_detect_switch=m_inifile.getIntValue(section1, "en_detect_switch", ret);
    cnt_td=m_inifile.getIntValue(section1,"detect_track_cnt",ret);
    cnt_dt=m_inifile.getIntValue(section1,"detect_loss_cnt",ret);
    en_auto_selsect=m_inifile.getIntValue(section1,"en_auto_selsect",ret);
}

VideoCapture cap;
Mat capture;
bool select_flag = false;
Point pt_origin;
Point pt_end;
int select_r[4];
int track_out[3],track_out_mine[3];
float tracker_psr;
int pause_flow = 0;
int force_reset = 0;
bool start_track=false;
bool needToInit = false;
void onMouse(int event, int x, int y, int, void* param) {
 //  cout<<"mouse_event"<<endl;
     if (select_flag) {
        pause_flow = 1;
        select_r[0] = MIN(pt_origin.x, x);
        select_r[1] = MIN(pt_origin.y, y);
        select_r[2] = abs(x - pt_origin.x);
        select_r[3] = abs(y - pt_origin.y);
        ((Rect*) (param))->x = MIN(pt_origin.x, x);
        ((Rect*) (param))->y = MIN(pt_origin.y, y);
        ((Rect*) (param))->width = abs(x - pt_origin.x);
        ((Rect*) (param))->height = abs(y - pt_origin.y);

    }
    if (event == CV_EVENT_LBUTTONDOWN) {
        pause_flow = 1;
        select_flag = true;
        pt_origin = Point(x, y);
        *(Rect*) (param) = Rect(x, y, 0, 0);
    } else if (event == CV_EVENT_LBUTTONUP) {
        pt_end = Point(x, y);
        select_flag = false;
        needToInit = true;
        pause_flow = 0;
    } else if (event == CV_EVENT_RBUTTONDOWN) {
        pause_flow = 1;
        select_flag = true;
        pt_origin = Point(x, y);
        *(Rect*) (param) = Rect(x, y, 0, 0);
    } else if (event == CV_EVENT_RBUTTONUP) {
        force_reset = 1;
        pt_end = Point(x, y);
        select_flag = false;
        needToInit = true;
        pause_flow = 0;
        Rect r = auto_select(capture, pt_end);
        //
        int S1 = 0;
        if (r.width * r.height < 45 * 45) {
            S1 = 20;
        }
        ((Rect*) (param))->x = r.x - S1 * 1.5;
        ((Rect*) (param))->y = r.y - S1;
        ((Rect*) (param))->width = r.width + S1 * 1.5;
        ((Rect*) (param))->height = r.height + S1;
        select_r[0] = ((Rect*) (param))->x;
        select_r[1] = ((Rect*) (param))->y;
        select_r[2] = ((Rect*) (param))->width;
        select_r[3] = ((Rect*) (param))->height;
    }
}

void warpFfine(Mat &inputIm, Mat &tempImg, float angle) {
    CV_Assert(!inputIm.empty());
    Mat inputImg;
    inputIm.copyTo(inputImg);
    float radian = (float) (angle / 180.0 * CV_PI);
    int uniSize = (int) (max(inputImg.cols, inputImg.rows) * 1.414);
    int dx = (int) (uniSize - inputImg.cols) / 2;
    int dy = (int) (uniSize - inputImg.rows) / 2;
    copyMakeBorder(inputImg, tempImg, dy, dy, dx, dx, BORDER_CONSTANT);
    Point2f center((float) (tempImg.cols / 2), (float) (tempImg.rows / 2));
    Mat affine_matrix = getRotationMatrix2D(center, angle, 1.0);
    warpAffine(tempImg, tempImg, affine_matrix, tempImg.size());
    float sinVal = fabs(sin(radian));
    float cosVal = fabs(cos(radian));
    Size targetSize((int) (inputImg.cols * cosVal + inputImg.rows * sinVal),
            (int) (inputImg.cols * sinVal + inputImg.rows * cosVal));
    int x = (tempImg.cols - targetSize.width) / 2;
    int y = (tempImg.rows - targetSize.height) / 2;
    Rect rect(x, y, targetSize.width, targetSize.height);
    tempImg = Mat(tempImg, rect);
}

int main()
{    aruco::CameraParameters CamParam;
     MarkerDetector MDetector;
     Mat img_in,imgSrc,imgSrcCopy,imgGray;
     Rect *pselect = new Rect;
     namedWindow("BAR", 1);
     setMouseCallback("BAR", onMouse, pselect); //mouse interface
     cv::TickMeter tm;
     cap.open(0);
      cap.set(CV_CAP_PROP_FRAME_WIDTH , 320* Scale_pix);
      cap.set(CV_CAP_PROP_FRAME_HEIGHT , 240* Scale_pix);

       cv::VideoWriter video_writer;
        std::ofstream file_writer;

     //while(1){cout<<"ruan"<<endl;uartSent();}
        if ( !cap.isOpened() )  // if not success, exit program
        {
             cout << "Cannot open the web cam" << endl;
             return -1;
        }
        loadMarkerConfig(MARKER_CONFIG_FILENAME);
        //read camera parameters if specifed
        CamParam.readFromXMLFile(cal_file);
        // resizes the parameters to fit the size of the input image
        cv::Size InImage_size(320*Scale_pix,240*Scale_pix);
        CamParam.resize(InImage_size);
        cout << CamParam.CameraMatrix << endl;
        cout << CamParam.Distorsion << endl;
        int p1 = 7;
        int p2 = 7;
        int t_p_range = 0;
        int subpix=1;
        p1 = p1 / 2 * 2 + 1;
        p2 = p2 / 2 * 2 + 1;
        MDetector.setThresholdParamRange(t_p_range);
        MDetector.setThresholdParams(p1, p2);
        MDetector.setCornerRefinementMethod(MDetector.SUBPIX ,subpix);//=1;//.minCornerDistance()
        MDetector.setThresholdMethod(MDetector.ADPT_THRES);
        cap >> img_in; // read a new frame from video
//        while(true)
//        {
//            cap>>img_in;

//             MDetector.detect(img_in, Markers, CamParam, marker_size);\
//             for (unsigned int i = 0; i < Markers.size(); i++)
//                 Markers[i].draw(img_in, Scalar(0, 0, 255), 2);
//                 imshow("1",img_in);
//            if(waitKey(10)==27)
//            {
//                return 0;
//            }
//         }
        resize(img_in, imgSrc, Size(320 * Scale_pix, 240 * Scale_pix), 0, 0, INTER_CUBIC);
        imgSrc.copyTo(imgSrcCopy);imgSrc.copyTo(capture);
        cvtColor(imgSrc, imgGray, CV_BGR2GRAY);
        cv_image<unsigned char> img(imgGray);
        tracker.start_track(img, getInitPosition(*pselect));


               namedWindow("Thresholded Image", CV_WINDOW_AUTOSIZE);
               int iLowH = 35;
               int iHighH = 90;
               int iLowS = 68;
               int iHighS = 255;
               int iLowV = 8;
               int iHighV = 255;
                cvCreateTrackbar("LowH", "Thresholded Image", &iLowH, 179); //Hue (0 - 179)
                cvCreateTrackbar("HighH", "Thresholded Image", &iHighH, 179);
                cvCreateTrackbar("LowS", "Thresholded Image", &iLowS, 255); //Saturation (0 - 255)
                cvCreateTrackbar("HighS", "Thresholded Image", &iHighS, 255);
                cvCreateTrackbar("LowV", "Thresholded Image", &iLowV, 255); //Value (0 - 255)
                cvCreateTrackbar("HighV", "Thresholded Image", &iHighV, 255);

        while (true)
        {
            tm.reset();
            tm.start();
            cap >>img_in; // read a new frame from video
            resize(img_in, imgSrc, Size(320 * Scale_pix, 240 * Scale_pix), 0, 0, INTER_CUBIC);
           // warpFfine(imgSrc,imgSrc,180);

            Mat imgHSV;
            cv::vector<Mat> hsvSplit;
            cvtColor(imgSrc, imgHSV, COLOR_BGR2HSV);
            split(imgHSV, hsvSplit);
            equalizeHist(hsvSplit[2],hsvSplit[2]);
            merge(hsvSplit,imgHSV);
            Mat imgThresholded;
            inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
            Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
            morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
            Mat element2= getStructuringElement(MORPH_RECT, Size(3,3));
            morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element2);
            imshow("Thresholded Image", imgThresholded); //show the thresholded image

            Mat image;
            threshold(imgThresholded,image,0,255,CV_THRESH_OTSU);
            std::vector<std::vector<cv::Point>> contours;
               cv::findContours(image,contours, // a vector of contours
                   CV_RETR_EXTERNAL, // retrieve the external contours
                   CV_CHAIN_APPROX_NONE); // retrieve all pixels of each contours
//              Mat imageContours=Mat::zeros(image.size(),CV_8UC1);
               Mat imageContours1=Mat::zeros(image.size(),CV_8UC1);
               std::vector<float> track_temp[3];
               int circle_cnt=0;
                   for(int k=0;k<contours.size();k++)
                   {
//                       RotatedRect rect=minAreaRect(contours[i]);
//                       Point2f P[4];
//                       rect.points(P);
//                       for(int j=0;j<=3;j++)
//                       {
//                           line(imageContours,P[j],P[(j+1)%4],Scalar(255),2);
//                       }
                       Point2f center; float radius;
                       minEnclosingCircle(contours[k],center,radius);
                       if(radius>20){
                       circle(imageContours1,center,radius,Scalar(255),2);
                       circle_cnt++;
                       track_temp[0].push_back(center.x);
                       track_temp[1].push_back(center.y);
                       track_temp[2].push_back(radius);
                       }
                   }

           if(circle_cnt>0){
            float temp[10]={0};
            for(int l=0;l<circle_cnt;l++){
            temp[0]+=track_temp[0][l];
            temp[1]+=track_temp[1][l];
            temp[2]+=track_temp[2][l];
              }
              track_out_mine[0]=temp[0]/circle_cnt;
              track_out_mine[1]=temp[1]/circle_cnt;
              track_out_mine[2]=temp[2]/circle_cnt;
           }
           else
            track_out_mine[2]=0;


                   //imshow("MinAreaRect",imageContours);
                   //imshow("MinAreaCircle",imageContours1);


              //
            imgSrc.copyTo(imgSrcCopy);


             if (select_flag) {
             start_track = false;
             cv::rectangle(imgSrc, *pselect, CV_RGB(0, 255, 0), 2, 8, 0);
             }

             else if (start_track&&1) {
                 static int cnt_loss_track = 0, cnt_loss_track5 = 0;
                 cvtColor(imgSrcCopy, imgGray, CV_BGR2GRAY);
                 if(use_kcf)
                 tracker_psr = tracker.update(img) * 2;
                 else
                 tracker_psr = track_out_mine[2]*2;
                 if (tracker_psr < 12)
                     cnt_loss_track++;
                 if (tracker_psr < 7)
                     cnt_loss_track5++;
                 if (cnt_loss_track > 20 || cnt_loss_track5 > 2) {
                     cnt_loss_track = 0;
                     cnt_loss_track5 = 0;
                     start_track = false;
                     continue;
                 }
                 if(use_kcf==0&&track_out_mine[2]>0)
                 cv::circle(imgSrc, Point(track_out_mine[0], track_out_mine[1]),track_out_mine[2], CV_RGB(255, 0, 255), 3);

                 MDetector.detect(imgSrcCopy, Markers, CamParam, marker_size);
                 if(Markers.size()>0){
                 getCameraPosWithMarkers(Markers, attitude_camera);
                 cv::circle(imgSrc, Point(attitude_camera.cx,attitude_camera.cy), 2, CV_RGB(255, 255, 0), 2, 8, 0);

                 for (unsigned int i = 0; i < Markers.size(); i++)
                        {
                        for (unsigned int j = 0; j < 4; j++)
                            {
                            if(Markers[i].id==mark[j].id)
                            Markers[i].draw(imgSrc, Scalar(0, 0, 255), 2);
                            }
                        }
                 }

                 static int detect_cnt;
                 if(detect_cnt++>cnt_td&&en_detect_switch&&use_kcf){
                    detect_cnt=0;
                     //MDetector.detect(imgSrc, Markers, CamParam, marker_size);
                     if(attitude_camera.check>0)
                     {
                         int r1;
                         Point2f temp=Markers[0].getCenter();
                         pt_end = Point(attitude_camera.cx, attitude_camera.cy);
                         r1=123;
                         if(en_auto_selsect){
                         Rect r = auto_select(imgSrc, pt_end);
                         int S1 = 0;
                         if (r.width * r.height < 45 * 45) {
                             S1 = 20;
                         }
                        pselect->x =  r.x - S1 * 1.5;
                        pselect->y =  r.y - S1 * 1.5;
                        pselect->width = r.width + S1 * 1.5;
                        pselect->height = r.height + S1 *1.5;
                         }else{
                             pselect->x = attitude_camera.cx-r1/2;// r.x - S1 * 1.5;
                             pselect->y = attitude_camera.cy-r1/2;//r.y - S1;
                             pselect->width = r1;//r.width + S1 * 1.5;
                             pselect->height = r1;//r.height + S1;
                         }
                        start_track = true;
                        //start_track = false;
                        if(use_kcf)
                        tracker.start_track(img, getInitPosition(*pselect));
                        continue;
                     }
                 }
                 dlib::rectangle rect = tracker.get_position();
                 cv::rectangle(imgSrc, dlibRect2CVRect(tracker.get_position()), CV_RGB(255, 0, 0), 2, 8, 0);
                 cv::circle(imgSrc, Point(rect.bl_corner().x() + rect.width() / 2,rect.bl_corner().y() - rect.height() / 2), 2, CV_RGB(255, 0, 0), 2, 8, 0);
                 line(imgSrc, Point(0, 120 * Scale_pix), Point(320 * Scale_pix, 120 * Scale_pix),Scalar(255, 0, 0), 1);
                 line(imgSrc, Point(160 * Scale_pix, 0),Point(160 * Scale_pix, 280 * Scale_pix),Scalar(255, 0, 0), 1);
               if(use_kcf){
                 track_out[0] = rect.right() - rect.width() / 2;
                 track_out[1] = rect.bottom() - rect.height() / 2;
                 track_out[2] = abs(rect.width() / 2 + rect.height() / 2);
                }else
                {
                   track_out[0]=track_out_mine[0];
                   track_out[1]=track_out_mine[1];
                   track_out[2]=track_out_mine[2];
                }

             } else {
                 //-------------------------mark deteced
                  if(use_kcf)
                      track_out[2] =0;
                  else
                  {
                      track_out[0]=track_out_mine[0];
                      track_out[1]=track_out_mine[1];
                      track_out[2]=track_out_mine[2];
                      if(track_out_mine[2]>0)
                      cv::circle(imgSrc, Point(track_out_mine[0], track_out_mine[1]),track_out_mine[2], CV_RGB(255, 0, 255), 3);

                  }

                  MDetector.detect(imgSrcCopy, Markers, CamParam, marker_size);
                  getCameraPosWithMarkers(Markers, attitude_camera);

                  for (unsigned int i = 0; i < Markers.size(); i++)
                         {
                         for (unsigned int j = 0; j < 4; j++)
                             {
                             if(Markers[i].id==mark[j].id)
                             Markers[i].draw(imgSrc, Scalar(0, 0, 255), 2);
                             }
                         }

                  static int detect_cnt1;
                  if(Markers.size()>0){
                       detect_cnt1++;
                       cv::circle(imgSrc, Point(attitude_camera.cx,attitude_camera.cy), 2, CV_RGB(255, 255, 0), 2, 8, 0);
                       circle(imgSrc, Point(attitude_camera.cx, attitude_camera.cy),3, CV_RGB(255, 0, 0), 3);
                   }


                  if(detect_cnt1>cnt_dt&&en_detect_switch&&1&&use_kcf){
                     detect_cnt1=0;
                      if(attitude_camera.check>0)
                      {

                          pt_end = Point(attitude_camera.cx, attitude_camera.cy);
                          int r1=123;
                          if(en_auto_selsect){
                          Rect r = auto_select(capture, pt_end);
                          int S1 = 0;
                          if (r.width * r.height < 45 * 45) {
                              S1 = 20;
                          }
                         pselect->x =  r.x - S1 * 1.5;
                         pselect->y =  r.y - S1 *1.5;
                         pselect->width = r.width + S1 * 1.5;
                         pselect->height = r.height + S1*1.5;
                          }else{
                              pselect->x = attitude_camera.cx-r1/2;// r.x - S1 * 1.5;
                              pselect->y = attitude_camera.cy-r1/2;//r.y - S1;
                              pselect->width = r1;//r.width + S1 * 1.5;
                              pselect->height = r1;//r.height + S1;
                          }

                         start_track = true;
                         if(use_kcf)
                         tracker.start_track(img, getInitPosition(*pselect));
                         continue;
                      }
                  }
                  line(imgSrc, Point(0, 120 * Scale_pix), Point(320 * Scale_pix, 120 * Scale_pix),Scalar(255, 0, 0), 1);
                  line(imgSrc, Point(160 * Scale_pix, 0),Point(160 * Scale_pix, 280 * Scale_pix),Scalar(255, 0, 0), 1);

                 cv::rectangle(imgSrc, *pselect, CV_RGB(0, 255, 0), 2, 8, 0);
                 char c = waitKey(10);
                 if (c == 32 || force_reset) {
                     force_reset = 0;
                     start_track = true;

                     tracker.start_track(img, getInitPosition(*pselect));
                 }
             }

       imshow("BAR", imgSrc); //show the original image
       if(1){
       uartSent();
      // uartSent2();
       }
       tm.stop();

       video_writer << imgSrc;
       static int flag=0;
       if (save_video&&!flag&&1)
               {
                   video_writer.release();
                   cout << "video_writer.release();" << endl;
                   file_writer.close();
                   cout << "file_writer.close();" << endl;

                   startWriteVideo(video_num_read, video_writer);
                   startWriteFile(file_writer);

                   flag = 1;
               }

       cout<<attitude_camera.check<<" xm="<<attitude_camera.x<<" ym="<<attitude_camera.y
          <<" zm="<<attitude_camera.z<<" Yaw="<<attitude_camera.Yaw<<
            " x="<<track_out[0]<<"  y="<<track_out[1]<<"  r="<<track_out[2]<<
            " t="<<tm.getTimeMilli()<<"ms"<<endl;


       attitude_camera.check=0;
       char key = (char) waitKey(5);
       if(key == 27)
             break;
        }//----end while

         video_writer.release();
         cout << "video_writer.release();" << endl;
         file_writer.close();
         cout << "file_writer.close();" << endl;

    return 0;
}


void uartSent(void)
{  unsigned char data_to_send[50];
    int _cnt = 0, i = 0, sum = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAF;
    data_to_send[_cnt++] = 0x61;
    data_to_send[_cnt++] = 0;


   data_to_send[_cnt++] = int(track_out[0])>>8;
   data_to_send[_cnt++] = int(track_out[0])%256;
   data_to_send[_cnt++] = int(track_out[1])>>8;
   data_to_send[_cnt++] = int(track_out[1])%256;
   data_to_send[_cnt++] = int(track_out[2])>>8;
   data_to_send[_cnt++] = int(track_out[2])%256;

   data_to_send[_cnt++] = 0;//int(attitude_camera.check)%256;
   data_to_send[_cnt++] = attitude_camera.check;//int(attitude_camera.check)>>8;
   data_to_send[_cnt++] = int(attitude_camera.x)>>8;
   data_to_send[_cnt++] = int(attitude_camera.x)%256;
   data_to_send[_cnt++] = int(attitude_camera.y)>>8;
   data_to_send[_cnt++] = int(attitude_camera.y)%256;
   data_to_send[_cnt++] = int(attitude_camera.z)>>8;
   data_to_send[_cnt++] = int(attitude_camera.z)%256;
   data_to_send[_cnt++] = int(attitude_camera.Pit)>>8;
   data_to_send[_cnt++] = int(attitude_camera.Pit)%256;
   data_to_send[_cnt++] = int(attitude_camera.Rol)>>8;
   data_to_send[_cnt++] = int(attitude_camera.Rol)%256;
   data_to_send[_cnt++] = int(attitude_camera.Yaw)>>8;
   data_to_send[_cnt++] = int(attitude_camera.Yaw)%256;
   data_to_send[_cnt++] = int(attitude_camera.cx)>>8;
   data_to_send[_cnt++] = int(attitude_camera.cx)%256;
   data_to_send[_cnt++] = int(attitude_camera.cy)>>8;
   data_to_send[_cnt++] = int(attitude_camera.cy)%256;
   data_to_send[_cnt++] = int(attitude_camera.cr)>>8;
   data_to_send[_cnt++] = int(attitude_camera.cr)%256;
    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
   int Length = _cnt;

    serial_port.write(data_to_send, Length);
}


void uartSent2()
{  unsigned char data_to_send[50];
    int _cnt = 0, i = 0, sum = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAF;
    data_to_send[_cnt++] = 0x62;
    data_to_send[_cnt++] = 0;
//map
    data_to_send[_cnt++] = int(mark_map[0][0])>>8;
    data_to_send[_cnt++] = int(mark_map[0][0])%256;
    data_to_send[_cnt++] = int(mark_map[0][1])>>8;
    data_to_send[_cnt++] = int(mark_map[0][1])%256;
    data_to_send[_cnt++] = int(mark_map[0][2])>>8;
    data_to_send[_cnt++] = int(mark_map[0][2])%256;
    data_to_send[_cnt++] = int(mark_map[0][3])>>8;
    data_to_send[_cnt++] = int(mark_map[0][3])%256;
    data_to_send[_cnt++] = int(mark_map[0][4]);//id

    data_to_send[_cnt++] = int(mark_map[1][0])>>8;
    data_to_send[_cnt++] = int(mark_map[1][0])%256;
    data_to_send[_cnt++] = int(mark_map[1][1])>>8;
    data_to_send[_cnt++] = int(mark_map[1][1])%256;
    data_to_send[_cnt++] = int(mark_map[1][2])>>8;
    data_to_send[_cnt++] = int(mark_map[1][2])%256;
    data_to_send[_cnt++] = int(mark_map[1][3])>>8;
    data_to_send[_cnt++] = int(mark_map[1][3])%256;
    data_to_send[_cnt++] = int(mark_map[1][4]);//id

    data_to_send[_cnt++] = int(mark_map[2][0])>>8;
    data_to_send[_cnt++] = int(mark_map[2][0])%256;
    data_to_send[_cnt++] = int(mark_map[2][1])>>8;
    data_to_send[_cnt++] = int(mark_map[2][1])%256;
    data_to_send[_cnt++] = int(mark_map[2][2])>>8;
    data_to_send[_cnt++] = int(mark_map[2][2])%256;
    data_to_send[_cnt++] = int(mark_map[2][3])>>8;
    data_to_send[_cnt++] = int(mark_map[2][3])%256;
    data_to_send[_cnt++] = int(mark_map[2][4]);//id

    data_to_send[_cnt++] = int(mark_map[3][0])>>8;
    data_to_send[_cnt++] = int(mark_map[3][0])%256;
    data_to_send[_cnt++] = int(mark_map[3][1])>>8;
    data_to_send[_cnt++] = int(mark_map[3][1])%256;
    data_to_send[_cnt++] = int(mark_map[3][2])>>8;
    data_to_send[_cnt++] = int(mark_map[3][2])%256;
    data_to_send[_cnt++] = int(mark_map[3][3])>>8;
    data_to_send[_cnt++] = int(mark_map[3][3])%256;
    data_to_send[_cnt++] = int(mark_map[3][4]);//id
//

    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
   int Length = _cnt;

    serial_port.write(data_to_send, Length);
}
