#include "opencv2/core/core.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/calib3d/calib3d.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include <iostream>  
#include <fstream>  
#include <string>
#include <vector>

using namespace std;
using namespace cv;

int main(void)
{
	vector<vector<Point2f> > Corner_Seq;		//用于储存所有角点序列
	vector<Point2f> corners;					//用于储存单幅图像角点序列
	vector<vector<Point3f> > object_Points;		//保存标定板上角点的三维坐标
	int cali_sum = 15;							//标定需要有效照片数量
	int cali_temp = 0;							//已经提取的有效照片的数量
	int corner_num = 0;							//储存角点数量
	Mat src, gray;
	Size board_size = Size(6, 7);				//棋盘格角点数量
	Size square_size = Size(20, 20);			//实际测量得到的标定板上每个棋盘格的大小
	Size image_size = Size(640, 480);			//图像大小
	stringstream  tempname;
	string filename;

	VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	ofstream fout("caliberation_result.txt");

	if (!cap.isOpened())
	{
		cout << "Open camera error!" << endl;
		return -1;
	}
	//namedWindow("Calibration");
	while (cap.isOpened())
	{
		if (cali_temp >= cali_sum)
		{
			cout << "Corner finished!" << endl;
			break;
		}
		cap >> src;
		if (waitKey(20) == 'g')
		{
			cout << "Get image!" << endl;
			cvtColor(src, gray, CV_BGR2GRAY);
			//corners.clear();
			int fflag = CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK;
			bool flag = findChessboardCorners(src, board_size, corners, fflag);
			//bool flag = findChessboardCorners(src, board_size, corners);
			drawChessboardCorners(src, board_size, corners, true);
			cout << "corner->size = " << corners.size() << endl;
			imshow("debug", src);
			//waitKey(0);
			if (flag)
			{
				cout << "Get corners!" << endl;
				cali_temp++;
				cout << "cali_temp = " << cali_temp << endl;
				tempname << cali_temp;
				tempname >> filename;
				filename += ".jpg";
				//cout << "111" << endl;
				find4QuadCornerSubpix(gray, corners, Size(11, 11));
				//cout << "222" << endl;
				corner_num += corners.size();
				Corner_Seq.push_back(corners);
				drawChessboardCorners(src, board_size, corners, true);
				imwrite(filename, src);
				tempname.clear();
				filename.clear();
				//cout << "333" << endl;
			}
			else
				cout << "Detect Failed." << endl;
		}
		imshow("Calibration", src);
	}
	Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));		//内参矩阵
	vector<int> point_counts;										//每幅图像中角点的数量  
	Mat distortion_coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));	//摄像机的5个畸变系数：k1,k2,p1,p2,k3 
	vector<Mat> tvecsMat;											//每幅图像的平移向量
	vector<Mat> rvecsMat;											//每幅图像的旋转向量

	for (int t = 0; t < cali_sum; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i < board_size.height; i++)
		{
			for (int j = 0; j < board_size.width; j++)
			{
				Point3f tempPoint;
				tempPoint.x = i*square_size.width;
				tempPoint.y = j*square_size.height;
				tempPoint.z = 0.0;
				tempPointSet.push_back(tempPoint);
			}
		}
		object_Points.push_back(tempPointSet);
	}
	
	cout << "开始标定" << endl;
	calibrateCamera(object_Points, Corner_Seq, image_size, intrinsic_matrix, distortion_coeffs, rvecsMat, tvecsMat);
	cout << "标定结束" << endl;

	std::cout << "开始保存定标结果………………" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
	fout << "相机内参数矩阵：" << endl;
	fout << intrinsic_matrix << endl << endl;
	fout << "畸变系数：\n";
	fout << distortion_coeffs << endl << endl << endl;
	for (int i = 0; i < cali_sum; i++)
	{
		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		fout << rvecsMat[i] << endl;
		/* 将旋转向量转换为相对应的旋转矩阵 */
		Rodrigues(rvecsMat[i], rotation_matrix);
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		fout << rotation_matrix << endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		fout << tvecsMat[i] << endl << endl;
	}
	std::cout << "完成保存" << endl;
	fout << endl;

	destroyAllWindows();

	waitKey(0);

	cout << "开始畸变校正" << endl;
	VideoCapture capp(0);
	capp.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	capp.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	if (!capp.isOpened())
	{
		cout << "Open camera error!" << endl;
		return -1;
	}
	while (capp.isOpened())
	{
		capp >> src;
		Mat dst;
		undistort(src, dst, intrinsic_matrix, distortion_coeffs);
		imshow("src", src);
		imshow("dst", dst);
		if (waitKey(30) == 's')
		{
			imwrite("before.jpg", src);
			imwrite("after.jpg", dst);
		}
		if (waitKey(30) == 27)
			break;
	}
	
	cout << "畸变校正结束" << endl;
	return 0;
}