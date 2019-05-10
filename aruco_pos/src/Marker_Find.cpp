#include "aruco_dec.h"


void Marker_Detect(Mat _image, Ptr<aruco::Dictionary> _dictionary, vector<vector<Point2f> >& _corners, vector<int>& _ids)
{
	_dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
	_corners.clear();
	_ids.clear();
	aruco::detectMarkers(_image, _dictionary, _corners, _ids);
}

void marker_find(Mat& img,
						vector<vector<Point2f> > _corners,
						vector<int>& _ids,
						Mat _cameraMatrix,
						Mat _distCoeffs,
						vector3d& _position_catcher,  //动作捕捉输出的位置
						Quaterniond& _quat_catcher	//动作捕捉输出的四元数
						vector3d& _position_marker	//检测二维码的位置
						Quaterniond& _quat)	//检测二维码的四元数
{
	_position_marker[0] = 0.;	_position_marker[1] = 0.;	_position_marker[2] = 0.;
	vector<Vec3d> rvecs, tvecs;
	aruco::estimatePoseSingleMarkers(_corners, 0.08, _cameraMatrix, _distCoeffs, rvecs, tvecs);
	for (int i = 0; i < _ids.size(); i++)
		aruco::drawAxis(img, _cameraMatrix, _distCoeffs, rvecs[i], tvecs[i], 0.05);

		Mat_<double> rotMat(3, 3, CV_32FC2);
		rotMat = Mat::zeros(3, 3, CV_32FC2);
		Mat_<double> Rvec(3, 1);
		Mat Tvec;
		Tvec = (Mat_<double>(3, 1) << tvecs[i][0], tvecs[i][1], tvecs[i][2]);
		Rvec = (Mat_<double>(3, 1) << rvecs[i][0], rvecs[i][1], rvecs[i][2]);
		Rodrigues(Rvec, rotMat);  //用罗德里格斯变换变成旋转矩阵


		int n = _ids[i];
		//格式转换
		Matrix<double, 3, 3> R_n;
		Matrix<double, Eigen::Dynamic, Eigen::Dynamic> T_n;
		Vector3d Rvec_n;

		cout << "id = " << _ids[i] << endl;
		cv2eigen(rotMat, R_n);
		cv2eigen(Tvec, T_n);
		
		//cout << "R_n = " << R_n << endl;
		//cout << "t_n = " << T_n << endl;
		Vector3d R_cw;
		R_cw = _quat_catcher.toRotationMatrix();
		_position_marker = R_cw.transpose()*( T_n  - _catcher_position);
		Vector3d R_mw;
		R_mw = R_n.transpose()*R_cw;
		Vector3d temp(0, 0, 0);
		temp[0] = atan2(R_n(1, 0), R_n(0, 0)) / CV_PI * 180;
		temp[1] = atan2(-1 * R_n(2, 0), sqrt(R_n(2, 1)*R_n(2, 1) + R_n(2, 2)*R_n(2, 2))) / CV_PI * 180;
		temp[2] = atan2(R_n(2, 1), R_n(2, 2)) / CV_PI * 180;
		cout<<"position = "<< _position_marker << endl; //检测二维码的位置
		cout<<"eular = "<< temp << endl;  //检测二维码的欧拉角
		_quat = R_mw;

}
