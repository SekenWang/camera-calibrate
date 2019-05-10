#ifndef ARUCO_DEC_H
#define ARUCO_DEC_H

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <stdexcept>
#include <math.h>

#include <ros/ros.h>

#include "opencv2/imgproc/imgproc.hpp"			//OpenCV
#include "opencv2/highgui/highgui.hpp"			//OpenCV
#include "opencv2/opencv.hpp"					//OpenCV
#include "opencv2/aruco.hpp"					//OpenCV For Aruco
#include "opencv2/calib3d/calib3d.hpp"			//OpenCV
#include "opencv2/core/core.hpp"				//OpenCV
#include "Eigen/Dense"							//Eigen
#include "Eigen/Core"							//Eigen
#include "opencv2/core/eigen.hpp"				//OpenCV-Eigen
#include "Eigen/Geometry"						//Eigen

/********************ROS Standard********************/
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>				//Bridge between opencv and ros...
/********************ROS Standard********************/

#include <aruco_pos/ArUco_Pos_Att.h>			//user defined


#include <mavros_msgs/CommandBool.h>			//mavros
#include <mavros_msgs/State.h>

#define GET_MIN(a, b)	(a < b ? a : b)

#define ERROR(str)		cout << "ERROR OCCURRED! In function : " << #str << endl
#define WARNING(str)	cout << "WARNING! In function : " << #str << endl

#define IMAGE_W			640
#define IMAGE_H			480
#define MAX_ARUCO		4

#define IMAGE_SIZE		(IMAGE_W*IMAGE_H)





using namespace cv;
using namespace std;
using namespace Eigen;

/*****************Camera0 Parameters*****************/
const double fx = 586.605396;
const double dx = 302.830434;
const double fy = 588.461572;
const double dy = 210.371504;
const double k1 = -0.469717;
const double k2 = 0.619264;
const double p1 = 0.00120925;
const double p2 = -0.00457535;
const double p3 = -0.643091;
/*****************Camera0 Parameters*****************/

const string dicname = "./src/aruco_pos/location_of_markers.csv";
const string dicname4launch1 = "/home/yyf/UAV206/src/aruco_pos/location_of_markers.csv";
const string dicname4launch2 = "/home/nvidia/UAV206/src/aruco_pos/location_of_markers.csv";

typedef struct _ArUco_Dic	//保存字典，csv文件必须按字典序号升序储存
{
	int _id;
	Vector3d _pos;
}ArUco_Dic;

extern Mat cameraMatrix;	//Camera Matrix
extern Mat distCoeffs;		//Distortion Matrix

extern vector<ArUco_Dic> aruco_dic;			//默认id和向量元素的位置重合
extern Ptr<aruco::Dictionary> Dictionary;	//字典
extern vector<vector<Point2f> > Corners;
extern vector<int> Ids;

extern Vector3d v_position_bf;		//position before filter
extern Vector3d v_position;		//position
extern Vector3d v_eular;		//eular
extern Vector4d v_quat;			//quat


void ArUco_Create_Dictionary(void);
//Time: 2018_04_21
//Func: Create a dictionary
//Para: None
//Tips: None
void ArUco_Load_Dictionary(vector<string> _dicname, vector<ArUco_Dic>& _ArUco_Dic, Ptr<aruco::Dictionary>& _dictionary);
//Time: 2018_04_21
//Func: 加载一个字典
//Para: 字典名称字符串，存放字典的结果结构体，字典
//Tips: None
void ArUco_Camera_Init(Mat& _cameraMatrix, Mat& _distCoeffs);
//Time: 2018_04_21
//Func: 初始化相机矩阵
//Para: 相机矩阵、畸变矩阵
//Tips: None
void ArUco_Detect(Mat _image, Ptr<aruco::Dictionary> _dictionary, vector<vector<Point2f> >& _corners, vector<int>& _ids);
//Time: 2018_04_21
//Func: 対一幅图进行检测
//Para: 图像，字典，检测的角点结果向量，结果向量ID号
//Tips: None
void ArUco_Uav_Pos_Cal(	Mat& img,
						vector<vector<Point2f> > _corners,
						vector<int>& _ids,
						Mat _cameraMatrix,
						Mat _distCoeffs,
						vector<ArUco_Dic> _ArUco_Dic,
						Vector3d& _P_ocf,
						Vector3d& _eular,
						Vector4d& _quat);
//Time: 2018_04_21
//Func: 对无人机位置进行位置解算
//Para: 原图像，角点信息，二维码ID，相机矩阵，畸变矩阵，字典，位置向量，欧拉角
//Tips: 暂无
void Msg_Update(aruco_pos::ArUco_Pos_Att& aruco_pos_att);
//Time: 2018_05_07
//Func: message update
//Para: ArUco_Pos::ArUco_Pos_Att&
//Tips: none

void bandstop_filter(Vector3d& inputdata, Vector3d& outputdata);
//time: 2008_05_18
//Func: bandstop_filter
//para: Vector3d : inputdata outputdata
//tips: none

void lowpass_filter(Vector3d& inputdata, Vector3d& outputdata);
//time: 2008_05_18
//Func: lowpass_filter
//para: Vector3d : inputdata outputdata
//tips: none

void Marker_Detect(Mat _image, Ptr<aruco::Dictionary> _dictionary, vector<vector<Point2f> >& _corners, vector<int>& _ids)
//Func: 目标二维码检测

void marker_find(Mat& img,
						vector<vector<Point2f> > _corners,
						vector<int>& _ids,
						Mat _cameraMatrix,
						Mat _distCoeffs,
						vector3d& _position_catcher,  //动作捕捉输出的位置
						Quaterniond& _quat_catcher	//动作捕捉输出的四元数
						vector3d& _position_marker	//检测二维码的位置
						Quaterniond& _quat)	//检测二维码的四元数
//Func: 根据动作捕捉输入的位置和四元数解算二维码的位置和姿态
#endif // ARUCO_DEC_H
