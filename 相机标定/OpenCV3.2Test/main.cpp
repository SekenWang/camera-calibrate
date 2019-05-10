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
	vector<vector<Point2f> > Corner_Seq;		//���ڴ������нǵ�����
	vector<Point2f> corners;					//���ڴ��浥��ͼ��ǵ�����
	vector<vector<Point3f> > object_Points;		//����궨���Ͻǵ����ά����
	int cali_sum = 15;							//�궨��Ҫ��Ч��Ƭ����
	int cali_temp = 0;							//�Ѿ���ȡ����Ч��Ƭ������
	int corner_num = 0;							//����ǵ�����
	Mat src, gray;
	Size board_size = Size(6, 7);				//���̸�ǵ�����
	Size square_size = Size(20, 20);			//ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С
	Size image_size = Size(640, 480);			//ͼ���С
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
	Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));		//�ڲξ���
	vector<int> point_counts;										//ÿ��ͼ���нǵ������  
	Mat distortion_coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));	//�������5������ϵ����k1,k2,p1,p2,k3 
	vector<Mat> tvecsMat;											//ÿ��ͼ���ƽ������
	vector<Mat> rvecsMat;											//ÿ��ͼ�����ת����

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
	
	cout << "��ʼ�궨" << endl;
	calibrateCamera(object_Points, Corner_Seq, image_size, intrinsic_matrix, distortion_coeffs, rvecsMat, tvecsMat);
	cout << "�궨����" << endl;

	std::cout << "��ʼ���涨����������������" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ����ÿ��ͼ�����ת���� */
	fout << "����ڲ�������" << endl;
	fout << intrinsic_matrix << endl << endl;
	fout << "����ϵ����\n";
	fout << distortion_coeffs << endl << endl << endl;
	for (int i = 0; i < cali_sum; i++)
	{
		fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
		fout << rvecsMat[i] << endl;
		/* ����ת����ת��Ϊ���Ӧ����ת���� */
		Rodrigues(rvecsMat[i], rotation_matrix);
		fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
		fout << rotation_matrix << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
		fout << tvecsMat[i] << endl << endl;
	}
	std::cout << "��ɱ���" << endl;
	fout << endl;

	destroyAllWindows();

	waitKey(0);

	cout << "��ʼ����У��" << endl;
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
	
	cout << "����У������" << endl;
	return 0;
}