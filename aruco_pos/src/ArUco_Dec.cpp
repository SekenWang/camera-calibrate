#include "aruco_dec.h"

Mat cameraMatrix;						//Camera Matrix
Mat distCoeffs;							//Distortion Matrix
vector<ArUco_Dic> aruco_dic;			//
Ptr<aruco::Dictionary> Dictionary;		//
vector<vector<Point2f> > Corners;		//
vector<int> Ids;						//
Vector3d v_position;				//Camera Position
Vector3d v_position_bf;				
Vector3d v_eular;						//Camera Eular
Vector4d v_quat;
//static float xv[11][2], yv[11][2];
static float xv_lowpass[5+1][2], yv_lowpass[5+1][2];


void ArUco_Create_Dictionary(void)
{
	cout << "Default" << endl;
	return;
}
void ArUco_Load_Dictionary(vector<string> _dicname, vector<ArUco_Dic>& _ArUco_Dic, Ptr<aruco::Dictionary>& _dictionary)
{
	_ArUco_Dic.clear();

	int id = 1;
	float x = 0., y = 0., z = 0.;
	ArUco_Dic temp = { 0 };

	FILE *fp = NULL;
	for(vector<string>::iterator it=_dicname.begin();it!=_dicname.end();it++)
	{
		fp = fopen((char*)(it->c_str()), "r");
		if (!fp)
		{
			WARNING(ArUco_Load_Dictionary);
			cout << "Can not open this file! Next." << endl;
			//exit(0);
		}
		else
			break;
	}
	if(!fp)
	{
		ERROR(ArUco_Load_Dictionary);
		cout << "Can not find csv" << endl;
		exit(0);
	}

	while (!feof(fp))
	{
		fscanf(fp, "%f,%f,%f/n", &x, &y, &z);
		temp._id = id;
		temp._pos[0] = (double)x;
		temp._pos[1] = (double)y;
		temp._pos[2] = (double)z;
		_ArUco_Dic.push_back(temp);
		id++;
	}
	fclose(fp);
	_dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
}
void ArUco_Camera_Init(Mat& _cameraMatrix, Mat& _distCoeffs)
{
	_cameraMatrix = (Mat_<double>(3, 3) << fx, 0, dx, 0, fy, dy, 0, 0, 1);
	_distCoeffs = (Mat_<double>(5, 1) << k1, k2, p1, p2, p3);
	//TODO: 还包括相机和无人机之间的坐标变换等等
}
void ArUco_Detect(Mat _image, Ptr<aruco::Dictionary> _dictionary, vector<vector<Point2f> >& _corners, vector<int>& _ids)
{
	_corners.clear();
	_ids.clear();
	aruco::detectMarkers(_image, _dictionary, _corners, _ids);
}
void ArUco_Uav_Pos_Cal(	Mat& img,
						vector<vector<Point2f> > _corners,
						vector<int>& _ids,
						Mat _cameraMatrix,
						Mat _distCoeffs,
						vector<ArUco_Dic> _ArUco_Dic,
						Vector3d& _P_ocf,
						Vector3d& _eular,
						Vector4d& _quat)
{
	_P_ocf[0] = 0.;	_P_ocf[1] = 0.;	_P_ocf[2] = 0.;
	_eular[0] = 0.;	_eular[1] = 0.;	_eular[2] = 0.;
	//Vector3d P_ocf(0, 0, 0);
	//Vector3d sum(0, 0, 0);
	vector<Vec3d> rvecs, tvecs;
	aruco::estimatePoseSingleMarkers(_corners, 0.08, _cameraMatrix, _distCoeffs, rvecs, tvecs);
	//得到旋转、平移向量
	for (int i = 0; i < _ids.size(); i++)
	{
		aruco::drawAxis(img, _cameraMatrix, _distCoeffs, rvecs[i], tvecs[i], 0.05);

		Mat_<double> rotMat(3, 3, CV_32FC2);
		rotMat = Mat::zeros(3, 3, CV_32FC2);
		Mat_<double> Rvec(3, 1);
		Mat Tvec;
		Tvec = (Mat_<double>(3, 1) << tvecs[i][0], tvecs[i][1], tvecs[i][2]);
		Rvec = (Mat_<double>(3, 1) << rvecs[i][0], rvecs[i][1], rvecs[i][2]);
		Rodrigues(Rvec, rotMat);  //用罗德里格斯变换变成旋转矩阵
		/*******siyuanshu******

			double trace = rotMat[0][0] + rotMat[1][1] + rotMat[2][2];
			double transtemp = 0.0;
			if (trace > 0)
			{
				transtemp = sqrt(trace + 1.0)*0.5;
				_quat[0] = transtemp;
				transtemp = transtemp*0.5;
				_quat[1] = (rotMat[1][2] - rotMat[2][1])*transtemp;
				_quat[2] = (rotMat[2][0] - rotMat[0][2])*transtemp;
				_quat[3] = (rotMat[0][1] - rotMat[1][0])*transtemp;
			}
			else
			{
				if (rotMat[0][0] > rotMat[1][1] && rotMat[0][0] > rotMat[2][2])
				{
					transtemp = 2.0 * sqrt(1.0 + rotMat[0][0] - rotMat[1][1] - rotMat[2][2]);
					_quat[0] = (rotMat[2][1] - rotMat[1][2]) / transtemp;
					_quat[1] = 0.25* transtemp;
					_quat[2] = (rotMat[0][1] + rotMat[1][0]) / transtemp;
					_quat[3] = (rotMat[0][2] + rotMat[2][0]) / transtemp;
				}
				else if (rotMat[1][1] > rotMat[2][2])
				{
					transtemp = 2.0 * sqrt(1.0 + rotMat[1][1] - rotMat[0][0] - rotMat[2][2]);
					_quat[0] = (rotMat[0][2] - rotMat[2][0]) / transtemp;
					_quat[1] = (rotMat[0][1] + rotMat[1][0]) / transtemp;
					_quat[2] = 0.25 * transtemp;
					_quat[3] = (rotMat[1][2] + rotMat[2][1]) / transtemp;
				}
				else
				{
					transtemp = 2.0 * sqrt(1.0 + rotMat[2][2] - rotMat[0][0] - rotMat[1][1]);
					_quat[0] = (rotMat[1][0] - rotMat[0][1]) / transtemp;
					_quat[1] = (rotMat[0][2] + rotMat[2][0]) / transtemp;
					_quat[2] = (rotMat[1][2] + rotMat[2][1]) / transtemp;
					_quat[3] = 0.25 * transtemp;
				}
			}
		/*******siyuanshu******/

		int n = _ids[i];
		//格式转换
		Matrix<double, 3, 3> R_n;
		Matrix<double, Eigen::Dynamic, Eigen::Dynamic> T_n;
		Vector3d Rvec_n;

		cout << "id = " << _ids[i] << endl;
		cv2eigen(rotMat, R_n);
		cv2eigen(Tvec, T_n);

		//cout << "R_n = " << R_n << endl;
		cout << "t_n = " << T_n << endl;
		Vector3d P_c(0, 0, 0), P_oc(0, 0, 0);
		P_c = _ArUco_Dic[_ids[i]]._pos;			//点对应世界坐标
		P_oc = R_n.transpose()*(-T_n)+P_c;		//求解世界坐标系下的位置
		_P_ocf = _P_ocf + P_oc / _ids.size();

		Vector3d temp(0, 0, 0);
		temp[0] = atan2(R_n(1, 0), R_n(0, 0)) / CV_PI * 180;
		temp[1] = atan2(-1 * R_n(2, 0), sqrt(R_n(2, 1)*R_n(2, 1) + R_n(2, 2)*R_n(2, 2))) / CV_PI * 180;
		temp[2] = atan2(R_n(2, 1), R_n(2, 2)) / CV_PI * 180;
		_eular = _eular + temp / _ids.size();
	}
	cout << "yaw pitch roll = " << _eular << endl;	//返回姿态角平均值 类型 eigen::vector3d
	cout << "P_ocf = " << _P_ocf << endl;			//单位：m    返回位置平均值类型 eigen::vector3d
	cout << "si yuan shu = "<<_quat<<endl;
}
void Msg_Update(aruco_pos::ArUco_Pos_Att& aruco_pos_att)
{
	aruco_pos_att.header_pre2 = aruco_pos_att.header_pre;
	aruco_pos_att.header_pre = aruco_pos_att.header_cur;

	aruco_pos_att.Max_Limit_Exceeded_pre2 = aruco_pos_att.Max_Limit_Exceeded_pre;
	aruco_pos_att.Max_Limit_Exceeded_pre = aruco_pos_att.Max_Limit_Exceeded_cur;

	aruco_pos_att.aruco_is_detect_pre2 = aruco_pos_att.aruco_is_detect_pre;
	aruco_pos_att.aruco_is_detect_pre = aruco_pos_att.aruco_is_detect_cur;

	aruco_pos_att.aruco_local_pre2 = aruco_pos_att.aruco_local_pre;
	aruco_pos_att.aruco_local_pre = aruco_pos_att.aruco_local_cur;

	aruco_pos_att.aruco_total_num_pre2 = aruco_pos_att.aruco_total_num_pre;
	aruco_pos_att.aruco_total_num_pre = aruco_pos_att.aruco_total_num_cur;
}
/*
void bandstop_filter(Vector3d& inputdata, Vector3d& outputdata)
{
	
	for(int i = 0;i<2;i++)
	{
			xv[0][i] = xv[1][i]; xv[1][i] = xv[2][i]; xv[2][i] = xv[3][i]; xv[3][i] = xv[4][i]; xv[4][i] = xv[5][i]; xv[5][i] = xv[6][i]; xv[6][i] = xv[7][i]; xv[7][i] = xv[8][i]; xv[8][i] = xv[9][i]; xv[9][i] = xv[10][i]; 
       			xv[10][i] = inputdata[i] / 2.588682783e+05;
        		yv[0][i] = yv[1][i]; yv[1][i] = yv[2][i]; yv[2][i] = yv[3][i]; yv[3][i] = yv[4][i]; yv[4][i] = yv[5][i]; yv[5][i] = yv[6][i]; yv[6][i] = yv[7][i]; yv[7][i] = yv[8][i]; yv[8][i] = yv[9][i]; yv[9][i] = yv[10][i]; 
        		yv[10][i] =   (xv[0][i] + xv[10][i]) -   0.0000000004 * (xv[1][i] + xv[9][i]) + 5 * (xv[2][i] + xv[8][i])
                     -   0.0000000014 * (xv[3][i] + xv[7][i]) + 10 * (xv[4][i] + xv[6][i]) -   0.0000000022 * xv[5][i]
                     + (  0.5679688348 * yv[0][i]) + (  0.0000000000 * yv[1][i])
                     + ( -3.1605155086 * yv[2][i]) + ( -0.0000000001 * yv[3][i])
                     + (  7.0547556806 * yv[4][i]) + (  0.0000000001 * yv[5][i])
                     + ( -7.8977394350 * yv[6][i]) + ( -0.0000000001 * yv[7][i])
                     + (  4.4354068132 * yv[8][i]) + (  0.0000000000 * yv[9][i]);
        outputdata[i] = yv[10][i];
	outputdata[2] = inputdata[2];
	}
}
*/

/*void lowpass_filter(Vector3d& inputdata, Vector3d& outputdata)
{
	for(int i = 0;i<3;i++)
	{
	xv_lowpass[0][i] = xv_lowpass[1][i]; xv_lowpass[1][i] = xv_lowpass[2][i]; xv_lowpass[2][i] = xv_lowpass[3][i]; xv_lowpass[3][i] = xv_lowpass[4][i]; xv_lowpass[4][i] = xv_lowpass[5][i]; 
        xv_lowpass[5][i] = inputdata[i] / 2.996744537e+06;
        yv_lowpass[0][i] = yv_lowpass[1][i]; yv_lowpass[1][i] = yv_lowpass[2][i]; yv_lowpass[2][i] = yv_lowpass[3][i]; yv_lowpass[3][i] = yv_lowpass[4][i]; yv_lowpass[4][i] = yv_lowpass[5][i]; 
        yv_lowpass[5][i] =   (xv_lowpass[0][i] + xv_lowpass[5][i]) + 5 * (xv_lowpass[1][i] + xv_lowpass[4][i]) + 10 * (xv_lowpass[2][i] + xv_lowpass[3][i])
                     + (  0.7124312835 * yv_lowpass[0][i]) + ( -3.8035532257 * yv_lowpass[1][i])
                     + (  8.1313017248 * yv_lowpass[2][i]) + ( -8.7013552442 * yv_lowpass[3][i])
                     + (  4.6611647833 * yv_lowpass[4][i]);
        outputdata[i] = yv_lowpass[5][i];
	}
}*/
