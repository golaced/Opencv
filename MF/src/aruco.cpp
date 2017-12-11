#include <serial/serial.h>
#include <iostream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <thread>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include "AttitudePosition.h"
#include "MarkerWorldCoornidate.h"
#include "flow_opencv.hpp"
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "my_serial.h"
#include "AM/AM.h"
#include <sstream>
#include <iomanip>
#include <fstream>
#include <wiringSerial.h>
using namespace std;
using namespace cv;
using namespace aruco;
#define TEST_QR 1
#define TEST_FLOW 0
float flow_out[2]={0};
int fd;
pthread_t thread_m[10];
pthread_mutex_t mut,mut1;
int number=0, i_t;Mat frame_t1;
Mat frame_golbal[10];
extern std::vector<MarkerWorld> CoordinateTable;
serial::Serial serial_port("/dev/ttyAMA0",115200 , serial::Timeout::simpleTimeout(1000));
cv::Point3f coordinate_camera;
Attitude attitude_camera;
std::vector< aruco::Marker > Markers;
Mat traj(720, 720, CV_8UC3, Scalar::all(255));
Mat traj_show(720, 720, CV_8UC3, Scalar::all(255));
Mat map_show(720, 720, CV_8UC3, Scalar::all(255));
int video_num = 0;
int write_flag;
int newfile_flag;
std::ifstream video_num_read;
std::ofstream video_num_write;
std::string video_num_path("/home/pi/QT/MF/video/video_num.txt");
std::string writer_path("/home/pi/QT/MF/video/");

string intToString(int v)
{
    char buf[32] = {0};
    snprintf(buf, sizeof(buf), "%u", v);

    string str = buf;
    return str;
}

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

#define MED_WIDTH_NUM 50
#define MED_FIL_ITEM  20

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM];
float med_filter_out[MED_FIL_ITEM];

int med_fil_cnt[MED_FIL_ITEM];
// 1  2  3                                9
float Moving_Median(int item, int width_num, float in)
{
    int i, j;
    float t;
    float tmp[MED_WIDTH_NUM];

    if (item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM)
    {
        return 0;
    }
    else
    {
        if (++med_fil_cnt[item] >= width_num)
        {
            med_fil_cnt[item] = 0;
        }

        med_filter_tmp[item][med_fil_cnt[item]] = in;
        for (i = 0; i < width_num; i++)
        {
            tmp[i] = med_filter_tmp[item][i];
        }

        for (i = 0; i < width_num - 1; i++)
        {
            for (j = 0; j<(width_num - 1 - i); j++)
            {
                if (tmp[j] > tmp[j + 1])
                {
                    t = tmp[j];
                    tmp[j] = tmp[j + 1];
                    tmp[j + 1] = t;
                }
            }
        }
        return (tmp[(int)width_num / 2]);
    }
}



void uartSent()
{  unsigned char data_to_send[50];
    int _cnt = 0, i = 0, sum = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAF;
    data_to_send[_cnt++] = 0x21;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = Markers.size();
    data_to_send[_cnt++] = int(coordinate_camera.x)>>8;
    data_to_send[_cnt++] = int(coordinate_camera.x)%256;
    data_to_send[_cnt++] = int(coordinate_camera.y)>>8;
    data_to_send[_cnt++] = int(coordinate_camera.y)%256;
    data_to_send[_cnt++] = int(coordinate_camera.z)>>8;
    data_to_send[_cnt++] = int(coordinate_camera.z)%256;
    data_to_send[_cnt++] = int(attitude_camera.Pit)>>8;
    data_to_send[_cnt++] = int(attitude_camera.Pit)%256;
    data_to_send[_cnt++] = int(attitude_camera.Rol)>>8;
    data_to_send[_cnt++] = int(attitude_camera.Rol)%256;
    data_to_send[_cnt++] = int(attitude_camera.Yaw)>>8;
    data_to_send[_cnt++] = int(attitude_camera.Yaw)%256;
    data_to_send[_cnt++] = int(flow_out[0])>>8;
    data_to_send[_cnt++] = int(flow_out[0])%256;
    data_to_send[_cnt++] = int(flow_out[1])>>8;
    data_to_send[_cnt++] = int(flow_out[1])%256;
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
//

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
    data_to_send[_cnt++] = 0x22;
    data_to_send[_cnt++] = 0;
//map
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

    data_to_send[_cnt++] = int(mark_map[4][0])>>8;
    data_to_send[_cnt++] = int(mark_map[4][0])%256;
    data_to_send[_cnt++] = int(mark_map[4][1])>>8;
    data_to_send[_cnt++] = int(mark_map[4][1])%256;
    data_to_send[_cnt++] = int(mark_map[4][2])>>8;
    data_to_send[_cnt++] = int(mark_map[4][2])%256;
    data_to_send[_cnt++] = int(mark_map[4][3])>>8;
    data_to_send[_cnt++] = int(mark_map[4][3])%256;
    data_to_send[_cnt++] = int(mark_map[4][4]);//id

    data_to_send[_cnt++] = int(mark_map[5][0])>>8;
    data_to_send[_cnt++] = int(mark_map[5][0])%256;
    data_to_send[_cnt++] = int(mark_map[5][1])>>8;
    data_to_send[_cnt++] = int(mark_map[5][1])%256;
    data_to_send[_cnt++] = int(mark_map[5][2])>>8;
    data_to_send[_cnt++] = int(mark_map[5][2])%256;
    data_to_send[_cnt++] = int(mark_map[5][3])>>8;
    data_to_send[_cnt++] = int(mark_map[5][3])%256;
    data_to_send[_cnt++] = int(mark_map[5][4]);//id
//

    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
   int Length = _cnt;

    serial_port.write(data_to_send, Length);
}

void initTraj(cv::Mat &traj, float rectsize, float offset)
{
    Point origin(offset, offset);
    for (size_t i = 0; i < marker_col_num*marker_row_num; i++)
    {
        Rect rect((CoordinateTable[i].coordinate.x - rectsize / 2) * 2 + origin.x, (CoordinateTable[i].coordinate.y - rectsize / 2) * 2 + origin.y, rectsize * 2, rectsize * 2);

        rectangle(traj, rect, Scalar(0, 0, 0), 2);
    }

    cv::line(traj, origin, Point(origin.x, 690), Scalar(72, 61, 139), 2);
    cv::line(traj, origin, Point(690, origin.y), Scalar(72, 61, 139), 2);

    cv::line(traj, Point(690, origin.y), Point(690 - 20, origin.y - 10), Scalar(72, 61, 139), 2);
    cv::line(traj, Point(690, origin.y), Point(690 - 20, origin.y + 10), Scalar(72, 61, 139), 2);

    cv::line(traj, Point(origin.x, 690), Point(origin.x - 10, 690 - 10), Scalar(72, 61, 139), 2);
    cv::line(traj, Point(origin.x, 690), Point(origin.x + 10, 690 - 10), Scalar(72, 61, 139), 2);
}
bool isEnd = false;
VideoCapture cap;

Mat Filter_aruco(Mat in,int user_threshold)
{
  Mat I_filtered,I;
  if(user_threshold > 0)
  {


  cv::cvtColor(in, I, COLOR_BGR2GRAY);
  //cv::GaussianBlur(I, I_filtered, cv::Size(0,0),2);

  // Weights
  //cv::addWeighted(I, 2.5, I_filtered, -1.5, 0, I_filtered);

  // Equalize histogram
  cv::equalizeHist(I,I_filtered);

  // Treshold
  cv::threshold(I_filtered,I_filtered,user_threshold,0,3);
 }else
      in.copyTo(I_filtered);
  return I_filtered;
}

float marker_map[500], camera_map[4],mark_pos_ero_check,center_fix_flt;
short cloud_size, lock_cnt,first_mark;
#define MARKER_USE_320 1
#if TEST_QR
void thread_marker(void)
#else
void* thread_marker(void*)
#endif
{
    Mat InImage;
    Mat pic_t1;
    initCoordinateTable(CoordinateTable);
#if MARKER_USE_320

    //string cameraParamFileName_low("/home/pi/QT/MF/rasp.yml");
    //string cameraParamFileName_high("/home/pi/QT/MF/PS3_320.yml");
    //string camera_cal_file(cal_file);
    cout<<cal_file<<endl;
    cv::Size InImage_size(320,240);
#else
    string cameraParamFileName("/home/odroid/workspace/QT/MarkerFlow/PS3_640.yml");
    cv::Size InImage_size(640,480);
#endif


    aruco::CameraParameters CamParam;
    MarkerDetector MDetector;
    float MarkerSize = MARKER_SIZE;

    int p1 = 7;
    int p2 = 7;
    int t_p_range = 0;
    int f_thr=0;
    int subpix=1;
    #if  TEST_QR
    cv::namedWindow("thes", 1);
    createTrackbar("p1", "thes", &p1, 101);
    createTrackbar("p2", "thes", &p2, 50);
    createTrackbar("range", "thes", &t_p_range, 31);
    createTrackbar("filter", "thes", &f_thr, 255);
    createTrackbar("subpix", "thes", &subpix, 20);

    //cv::namedWindow("map", 2);
    int cloud_sizep=49;//30;
    int mark_pos_ero_checkp=6*10;
    int lock_cntp=3;
    int center_fix_fltp=66;//0.36*100;
    createTrackbar("cloud_size", "thes", &cloud_sizep, 49);
    createTrackbar("ero_fix", "thes", &mark_pos_ero_checkp, 100);
    createTrackbar("fix_cnt", "thes", &lock_cntp, 20);
    createTrackbar("fix_step", "thes", &center_fix_fltp, 100);
    #endif
    ostringstream ostr_pos;
    ostringstream ostr_angle;
    Point3f pos_camera(0, 0, 0);

    float offset = 50;
    initTraj(traj, marker_size*100, offset);
    Mat traj_empty = traj.clone();
    int drawPointSize = 25;
    vector<Point2f> drawPointKF;
    vector<Point2f> drawPointSrc;
    drawPointKF.resize(drawPointSize,Point2f(0,0));
    drawPointSrc.resize(drawPointSize, Point2f(0, 0));
    cout<<"1_str"<<endl;

    cv::TickMeter tm;
    VideoWriter video_writer;
    std::ofstream file_writer;
    string video_read_s("/home/pi/QT/MF/video_r/"+intToString(read_video_num)+".avi");
    VideoCapture cap1(video_read_s);
    for(;;)
    { // cout<<"1"<<endl;
        tm.reset();
        tm.start();
#if !TEST_QR
        pthread_mutex_lock(&mut);
#endif
#if TEST_QR
        cap >> pic_t1;
        cv::resize(Filter_aruco(pic_t1,f_thr), InImage, InImage_size);
#else
        frame_golbal[1].copyTo(pic_t1);
        cv::resize(pic_t1, InImage, InImage_size);
        pthread_mutex_unlock(&mut);
#endif


        static int init;
        if(!init){init=1;
            //read camera parameters if specifed
            CamParam.readFromXMLFile(cal_file);

            // resizes the parameters to fit the size of the input image
            CamParam.resize(InImage_size);

            cout << CamParam.CameraMatrix << endl;
            cout << CamParam.Distorsion << endl;
        }
        if(read_video_num!=0)
        cap1>>InImage;
        p1 = p1 / 2 * 2 + 1;
        p2 = p2 / 2 * 2 + 1;
        MDetector.setThresholdParamRange(t_p_range);
        MDetector.setThresholdParams(p1, p2);
        MDetector.setCornerRefinementMethod(MDetector.SUBPIX ,subpix);//=1;//.minCornerDistance()
        MDetector.setThresholdMethod(MDetector.ADPT_THRES);
        //MDetector.setMinMaxSize(0.03,0.5);
        // Ok, let's detect
        static int Markers_getArea=1500;
       // if(Markers_getArea<2000||1)
       // CamParam.readFromXMLFile(cameraParamFileName_high);
       // else
       // CamParam.readFromXMLFile(cal_file);
        MDetector.detect(InImage, Markers, CamParam, MarkerSize);
        video_writer << InImage;

        static int flag=0;
       if (save_video&&!flag)
               {
                   video_writer.release();
                   cout << "video_writer.release();" << endl;
                   file_writer.close();
                   cout << "file_writer.close();" << endl;

                   startWriteVideo(video_num_read, video_writer);
                   startWriteFile(file_writer);

                   flag = 1;
               }
        float check_in[6][5]={0};
        if(Markers.size()>0 )
        Markers_getArea=Markers[0].getArea();

        // for each marker, draw info and its boundaries in the image

        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            Markers[i].draw(InImage, Scalar(0, 0, 255), 2);
        }

        if (CamParam.isValid() && MarkerSize != -1)
        {
            for (unsigned int i = 0; i < Markers.size(); i++)
            {
                //CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam);//draw axis
                getAttitude(Markers[i], attitude_camera);

                ostr_angle.clear();
                ostr_angle.str("");
                //ostr_angle << "          Pit=" << (int)attitude_camera.Pit << " " << "Yaw=" << (int)attitude_camera.Yaw << " " << "Rol=" << (int)attitude_camera.Rol;
                ostr_angle << "          Y=" << (int)attitude_camera.Yaw ;//<< " " << "Rol=" << (int)attitude_camera.Rol;


                getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_camera);

                ostr_pos.clear();
                ostr_pos.str("");
                ostr_pos << "          x=" << (int)pos_camera.x;// << " " << "y=" << (int)pos_camera.y ;//<< " " << "z=" << (int)pos_camera.z;
                putText(InImage, ostr_pos.str(), Markers[i].getCenter() + Point2f(-100, -20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);
                ostr_pos.clear();
                ostr_pos.str("");
                ostr_pos << "          y=" << (int)pos_camera.y ;//<< " " << "y=" << (int)pos_camera.y ;//<< " " << "z=" << (int)pos_camera.z;
                putText(InImage, ostr_pos.str(), Markers[i].getCenter() + Point2f(-100, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);


               // putText(InImage, ostr_angle.str(), Markers[i].getCenter() + Point2f(-100, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

            }
        }
        coordinate_camera = Point3f(0,0,0);
        getCameraPosWithMarkers(Markers, coordinate_camera,attitude_camera, 3);//
        int j=0;
        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            Point3f pos_world(0, 0, 0);
            Attitude atti_t;
            if(j<6){
            getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);//camera to local marker o dis
            getAttitude(Markers[i], atti_t);

            check_in[j][0]=pos_world.x;
            check_in[j][1]=pos_world.y;
            check_in[j][2]=pos_world.z;
            check_in[j][3]=atti_t.Yaw;
            check_in[j++][4]=Markers[i].id;
             }
        }

        cloud_size=cloud_sizep;
        mark_pos_ero_check=(float)mark_pos_ero_checkp/10.;
        lock_cnt=lock_cntp;
        center_fix_flt=(float)center_fix_fltp/100.;
        static short init_map=1;
        AM( marker_map,  camera_map, &first_mark,check_in[0],check_in[1],check_in[2],check_in[3],check_in[4],check_in[5],
                cloud_size,  mark_pos_ero_check,  lock_cnt, center_fix_flt,&init_map,0);
        static float map_draw[100][6];
        static int cnt_draw_jra=0;
         j=0;
        float k=1.23;
        int offd=600;
        for(int i=1;i<100;i++){
         if(marker_map[i + 399]==2){
         map_draw[i][0]= marker_map[i - 1]*k;
         map_draw[i][1]= marker_map[i + 99]*k;
         map_draw[i][2]= marker_map[i + 199] ;
         map_draw[i][3]= marker_map[i + 299] ;

         map_draw[i][4]=i;
         if(map_draw[i][5]==0){
         cout<<i<<" "<<map_draw[i][0]<<" "<<map_draw[i][1]<<endl;
         map_draw[i][5]=1;
         circle(map_show, Point(map_draw[i][0]+offd, map_draw[i][1]+offd),8, CV_RGB(0, 0, 255),2);
         }
         }
        }
        if(cnt_draw_jra++>0){cnt_draw_jra=0;
        circle(map_show, Point(camera_map[0]*k+offd, camera_map[1]*k+offd),1, CV_RGB(255, 0, 0),2);
        }
        cv::imshow("map", map_show);
        //cout<<marker_map[sel*4+0]<<" "<<marker_map[sel*4+1]<<" "<<marker_map[sel*4+2]<<" "<<marker_map[sel*4+3]<<endl;
       if(show_traj){
        ostr_pos.clear();
        ostr_pos.str("");
        ostr_pos << "          x=" << (int)coordinate_camera.x << " " << "y=" << (int)coordinate_camera.y<< " z=" << (int)coordinate_camera.z
                 << " " << "Y=" << (int)attitude_camera.Yaw<<" P="<<(int)attitude_camera.Pit<<" R="<<(int)attitude_camera.Rol;

        putText(InImage, ostr_pos.str(), Point(-60, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

        static float x_show = 0, y_show = 0;
        static float x = 0;
        static float y = 0;
        if ((int(coordinate_camera.x) != 0) && (int(coordinate_camera.y) != 0))
        {
            x = (coordinate_camera.x) * 2;
            y = (coordinate_camera.y) * 2;
        }

        x_show = x;
        y_show = y;

        drawPointSrc.insert(drawPointSrc.begin(), Point(x_show, y_show) + Point(offset, offset));
        drawPointSrc.pop_back();
        int draw_size=drawPointSrc.size()*0.2;//draw length
        for (size_t i = 0; i < draw_size; i++)
        {
            circle(traj, drawPointSrc[i], 3, CV_RGB(255, 0, 0), 1);
        }

        circle(traj, Point(x_show, y_show)+Point(offset,offset),3, CV_RGB(255, 0, 0), 1);
        traj.copyTo(traj_show);}
        //show input with augmented information

    #if  TEST_QR
        if(show_camera)
        cv::imshow("thes", InImage);
        // show also the internal image resulting from the threshold operation
        //cv::imshow("thes", MDetector.getThresholdedImage());

        uartSent();uartSent2();

        if(show_traj){traj = traj_empty.clone();
        cv::imshow("in", traj_show);}
   #else
         if(show_traj)
         cv::imshow("thes", InImage);
   #endif
        tm.stop();
        static int cnt_time_show;
        if(cnt_time_show++>10){cnt_time_show=0;
        cout<<"marker:"<<tm.getTimeMilli()<<"ms"<<endl;
        }
       // cout<<"x=: "<<coordinate_camera.x<<"   y=: "<<coordinate_camera.y<<"  marker:"<<tm.getTimeMilli()<<"ms"<<endl;

        char c = (char)waitKey(10);
        if( c == 27 )
            break;
    }
    video_writer.release();
    cout << "video_writer.release();" << endl;
    //file_writer.close();
    // cout << "file_writer.close();" << endl;
    #if !TEST_QR
    pthread_exit(NULL);
    #endif
}

cv::Mat_<float> cam_matrix_flow(3, 3);
cv::Mat_<float> distortion_flow(1, 4);
float scale_flow=1;
float focus=64.0/320.0;
int en_cal_flow=1;
//int en_cal_flow=0;
#if TEST_FLOW
void thread_flow(void)
#else
void* thread_flow(void*)
#endif
{
    Mat InImage;
    Mat pic_t1;
    cv::Size InImage_size_in(320,240);
    cv::Size InImage_size(64*scale_flow,64*scale_flow);
    cout<<"flow_str"<<endl;
    float length_camera = (16) / ((float)4 * 0.006f);	// pixel-size: 6um;
    int num_feat_set = 10;
    float conf_multi_set = 1.645f;
    int conf_multi_set_temp = conf_multi_set*1000;
    int flow_k=140/4.4;
#if TEST_FLOW==1
    cv::namedWindow("flow", 1);
    cv::namedWindow("flow1", 1);
    createTrackbar("feat", "flow1", &num_feat_set, 60);
    createTrackbar("confid", "flow1", &conf_multi_set_temp, conf_multi_set * 2000);
    createTrackbar("k", "flow1", &flow_k,  1000);
#endif
    OpticalFlowOpenCV flow(length_camera, length_camera, num_feat_set, conf_multi_set);
    cv::TickMeter tm;
    float flow_x,flow_y;
    Mat  currImage_c_g,currImage_c_g_hist;
    aruco::CameraParameters cameraParams;
    string cameraParamFileName("/home/pi/QT/MF/rasp.yml");
    cameraParams.readFromXMLFile(cameraParamFileName);
    Mat map1, map2;
    initUndistortRectifyMap(cameraParams.CameraMatrix, cameraParams.Distorsion, Mat(),
        getOptimalNewCameraMatrix(cameraParams.CameraMatrix, cameraParams.Distorsion, InImage_size_in, 1, InImage_size_in, 0),
        InImage_size_in, CV_16SC2, map1, map2);

    for(;;)
    { // cout<<"1"<<endl;
        Mat roiImage;
        tm.reset();
        tm.start();
        
#if TEST_FLOW
        cap >> pic_t1;
        Mat cal;
        cv::resize(pic_t1, cal, InImage_size_in);
        if(en_cal_flow)
        remap(cal, cal, map1, map2, INTER_LINEAR);

        cv::Rect rect((1-focus)*cal.cols / 2, (1-focus)*cal.rows / 2,
                      cal.cols *focus, cal.rows* focus);
        cal(rect).copyTo(roiImage);
        cv::resize(roiImage, InImage, InImage_size);

#else
        pthread_mutex_lock(&mut);
        frame_golbal[2].copyTo(pic_t1);
        Mat cal;
        cv::resize(pic_t1, cal, InImage_size_in);
        if(en_cal_flow)
        remap(cal, cal, map1, map2, INTER_LINEAR);

        cv::Rect rect((1-focus)*cal.cols / 2, (1-focus)*cal.rows / 2,
                      cal.cols *focus, cal.rows* focus);
        cal(rect).copyTo(roiImage);
        cv::resize(roiImage, InImage, InImage_size);
        pthread_mutex_unlock(&mut);
#endif

  
        cvtColor(InImage, currImage_c_g, COLOR_BGR2GRAY);
        //currImage_c_g.copyTo(currImage_c_g_hist);
        equalizeHist(currImage_c_g, currImage_c_g_hist);
        flow.setImageHeight(currImage_c_g_hist.cols);
        flow.setImageWidth(currImage_c_g_hist.rows);
        flow.setConfMultiplier((float)conf_multi_set_temp/1000.);
        if (!currImage_c_g_hist.empty())
        {

        flow.calcFlow(currImage_c_g_hist, flow_x, flow_y);
        flow_out[0]=flow_x*100000*flow_k/100;
        flow_out[1]=flow_y*100000*flow_k/100;
        Point start(InImage.rows / 2, InImage.rows / 2);
        line(InImage, start, Point(InImage.rows / 2 + (flow_x*3000.0),
            InImage.rows / 2 + (flow_y*3000.0)), Scalar(0, 255, 0), 3);
        }
        #if TEST_FLOW==1
        imshow("flow", InImage);
        uartSent();
        #endif
        tm.stop();
        cout<<"x=: "<<(int)(flow_out[0])<<"   y=: "<<(int)(flow_out[1])<<"  flow:"<<tm.getTimeMilli()<<"ms"<<endl;

        char c = (char)waitKey(10);
        if( c == 27 )
            break;
    }
    pthread_exit(NULL);
}

void* thread_uart(void*)
{
    for(;;)
    {
        pthread_mutex_lock(&mut);
        //cout<<"Uart good!"<<endl;
        uartSent();
        uartSent2();
        pthread_mutex_unlock(&mut);
        char c = (char)waitKey(10);
        if( c == 27 )
            break;
    }
    pthread_exit(NULL);
}

void* thread_readcamera(void*)
{
    for(;;)
    {
        pthread_mutex_lock(&mut);
        cap >> frame_golbal[2];
        frame_golbal[2].copyTo(frame_golbal[1]);
        pthread_mutex_unlock(&mut);

        char c = (char)waitKey(10);
        if( c == 27 )
            break;
    }
    pthread_exit(NULL);
}

void thread_create(void)
{
    memset(&thread_m, 0, sizeof(thread_m)); //comment1
#if !(TEST_QR||TEST_FLOW)
    if(use_flow)
    pthread_create(&thread_m[0], NULL, thread_flow, NULL); //comment2
    pthread_create(&thread_m[1], NULL, thread_readcamera, NULL); //comment2
    pthread_create(&thread_m[2], NULL, thread_uart, NULL); //comment2
    pthread_create(&thread_m[3], NULL, thread_marker, NULL); //comment2
 #endif
}
void thread_wait(void)
{
    if(thread_m[0] !=0) { //comment4
        pthread_join(thread_m[0],NULL);
    }
    if(thread_m[1] !=0) { //comment5
        pthread_join(thread_m[1],NULL);
    }
    if(thread_m[2] !=0) { //comment5
        pthread_join(thread_m[2],NULL);
    }
    if(thread_m[3] !=0) { //comment5
        pthread_join(thread_m[3],NULL);
    }
}

int main(void)
{
    try
    {
        while((fd = serialOpen ("/dev/ttyAMA0",115200))<0)
        {
            cout<<"serial err"<<endl;
            usleep(2000*1000);
        }
        init_sys_para();
        cap.open(camera_sel);
        usleep(2000*1000);
        //while(1)
          //      {
            //uartSent();
            //serialPutchar(fd,'a');
            //usleep(20*1000);
       // }
#if     TEST_FLOW
     thread_flow();
#elif   TEST_QR
        thread_marker();
#else
        thread_create();
        thread_wait();
#endif
        return 0;
    }
    catch (std::exception &ex)
    {
        cout << "Exception :" << ex.what() << endl;
    }
}
