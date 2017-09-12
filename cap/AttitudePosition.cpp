#include "AttitudePosition.h"
#include "land.hpp"
#include <vector>

using namespace std;
using namespace cv;
using namespace aruco;

bool SortByDown(const float &p1, const float &p2)//��������������������������������vector����������������
{
	return p1 > p2;//��������
}

std::vector<cv::Mat> getR(float alpha_X, float alpha_Y, float alpha_Z)
{
	Mat R_X = Mat::eye(3, 3, CV_32FC1);
	Mat R_Y = Mat::eye(3, 3, CV_32FC1);
	Mat R_Z = Mat::eye(3, 3, CV_32FC1);

	alpha_X /= 57.3;
	alpha_Y /= 57.3;
	alpha_Z /= 57.3;

	R_X.at<float>(1, 1) = cos(alpha_X);
	R_X.at<float>(1, 2) = sin(alpha_X);
	R_X.at<float>(2, 1) = -sin(alpha_X);
	R_X.at<float>(2, 2) = cos(alpha_X);

	R_Y.at<float>(0, 0) = cos(alpha_Y);
	R_Y.at<float>(0, 2) = -sin(alpha_Y);
	R_Y.at<float>(2, 0) = sin(alpha_Y);
	R_Y.at<float>(2, 2) = cos(alpha_Y);

	R_Z.at<float>(0, 0) = cos(alpha_Z);
	R_Z.at<float>(0, 1) = sin(alpha_Z);
	R_Z.at<float>(1, 0) = -sin(alpha_Z);
	R_Z.at<float>(1, 1) = cos(alpha_Z);

	vector<Mat> dst;
	dst.push_back(R_X);
	dst.push_back(R_Y);
	dst.push_back(R_Z);

	return dst;

}

float mark_map[6][5];
void mark_map_publish(std::vector< aruco::Marker > Markers)
{    int i,j=0,k ;
    int mark_num=Markers.size();
     if(mark_num>4)
         mark_num=4;
     for (i = 0; i < mark_num; i++)
       for(k=0;k<4;k++)
       if(Markers[i].id==mark[k].id){
        Point3f pos_world(0, 0, 0);
        Camera atti_t;
        getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);//camera to local marker o dis
        getAttitude(Markers[i], atti_t);
        mark_map[j][0]=pos_world.x;
        mark_map[j][1]=pos_world.y;
        mark_map[j][2]=pos_world.z;
        mark_map[j][3]=atti_t.Yaw;
        mark_map[j][4]=Markers[k].id;
        j++;
       }

}

cv::Point3f getCameraLocation1(cv::Mat Rvec,cv::Mat Tvec) {

cv::Mat m33(3,3,CV_32FC1);
cv::Rodrigues(Rvec, m33)  ;

cv::Mat m44=cv::Mat::eye(4,4,CV_32FC1);
for (int i=0;i<3;i++)
    for (int j=0;j<3;j++)
        m44.at<float>(i,j)=m33.at<float>(i,j);

//now, add translation information
for (int i=0;i<3;i++)
    m44.at<float>(i,3)=Tvec.at<float>(0,i);
//invert the matrix
m44.inv();
return  cv::Point3f( m44.at<float>(0,0),m44.at<float>(0,1),m44.at<float>(0,2));
}

void getCameraPos(cv::Mat Rvec, cv::Mat Tvec, cv::Point3f &pos)
{
	Mat Rot(3, 3, CV_32FC1);
	Rodrigues(Rvec, Rot);
    Rot = Rot.t();  // rotation of inverse
    Mat pos_camera = -Rot * Tvec*100; // translation of inverse
    pos.x = pos_camera.at<float>(0, 0);
    pos.y = -pos_camera.at<float>(1, 0);
    pos.z = pos_camera.at<float>(2, 0);
}

Camera attitude_camera;
void getCameraPosWithMarkers(std::vector< aruco::Marker > Markers, Camera &atti_camera)
{   int cnt=0;
    Point3f pos_world(0, 0, 0),temp,temp_c;
    Camera atti_t;
    mark_map_publish(Markers);

    std::vector<float> vec_x, vec_y, vec_z,vec_pit,vec_rol,vec_yaw,vec_cx,vec_cy,vec_size;
    for ( int i = 0; i < Markers.size(); i++)
     for( int j=0;j<4;j++)
       if(Markers[i].id==mark[j].id&&mark[i].size>0&&Markers[i].getArea()>333){
        cnt++;
        getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);//camera to local marker o dis
        getAttitude(Markers[i], atti_t);
        temp.x=pos_world.x*mark[j].size/mark[0].size+mark[j].x;
        temp.y=-pos_world.y*mark[j].size/mark[0].size+mark[j].y;
        int en_test=0;
//        if(Markers[i].id==97&&en_test)
//            cout<<"97:-----x="<<temp.x<<"  y="<<temp.y<<endl;
//        if(Markers[i].id==98&&en_test)
//            cout<<"98:-----x="<<temp.x<<"  y="<<temp.y<<endl;
//        if(Markers[i].id==99&&en_test)
//            cout<<"99:-----x="<<temp.x<<"  y="<<temp.y<<endl;
        if(Markers[i].id==100&&en_test)
            cout<<"100:-----x="<<temp.x<<"  y="<<temp.y<<endl;
        temp.z=pos_world.z*mark[j].size/mark[0].size;
        temp_c.x=Markers[i].getCenter().x;
        temp_c.y=Markers[i].getCenter().y;
        vec_x.push_back(temp.x);
        vec_y.push_back(temp.y);
        vec_z.push_back(temp.z);
        vec_pit.push_back(atti_t.Pit);
        vec_rol.push_back(atti_t.Rol);
        vec_yaw.push_back(atti_t.Yaw);
        vec_cx.push_back(temp_c.x);
        vec_cy.push_back(temp_c.y);
        vec_size.push_back(Markers[i].getArea());
       }

       atti_camera.check=cnt;
    if(vec_x.size()>0){
        float temp[10]={0};
        for(unsigned int i=0;i<vec_x.size();i++){
            temp[0]+=vec_x[i];
            temp[1]+=vec_y[i];
            temp[2]+=vec_z[i];
            temp[3]+=vec_pit[i];
            temp[4]+=vec_rol[i];
            temp[5]+=vec_yaw[i];
            temp[6]+=vec_cx[i];
            temp[7]+=vec_cy[i];
            temp[8]+=vec_size[i];
        }

        atti_camera.x=temp[0]/vec_x.size();
        atti_camera.y=temp[1]/vec_y.size();
        atti_camera.z=temp[2]/vec_z.size();
        atti_camera.Pit=temp[3]/vec_pit.size();
        atti_camera.Rol=temp[4]/vec_rol.size();
        atti_camera.Yaw=temp[5]/vec_yaw.size();
        attitude_camera.cx=temp[6]/vec_cx.size();
        attitude_camera.cy=temp[7]/vec_cy.size();
        attitude_camera.cr=temp[8]/vec_size.size();
    }
}

void getAttitude(aruco::Marker marker, Camera &attitude)
{
	double pos[3] = { 0 };
	double ori[4] = { 0 };

	double q0, q1, q2, q3;

	marker.OgreGetPoseParameters(pos, ori);
	pos[0] = -pos[0];
	pos[1] = -pos[1];

	q0 = ori[0]; q1 = ori[1]; q2 = ori[2]; q3 = ori[3];

	attitude.Pit = atan2(2 * (q0 * q1 + q2 * q3), -1 + 2 * (q1 * q1 + q2 * q2)) *57.3f;
	attitude.Rol = asin(2 * (q1 * q3 - q0 * q2)) *57.3f;
	attitude.Yaw = -atan2(2 * (-q1 * q2 - q0 * q3), 1 - 2 * (q0 * q0 + q1 * q1)) *57.3f;
}
