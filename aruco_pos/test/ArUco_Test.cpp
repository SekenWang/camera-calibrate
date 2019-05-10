#include "aruco_dec.h"

int Frame_All = 0;

Mat imgsrc, imggray;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Target_Dec");
	ros::NodeHandle nh;

	ros::Rate rate(100);
	VideoCapture cap("./src/aruco_pos/video/test0.mp4");
	if(!cap.isOpened())
	{
		cout << "Cann't open the video!" << endl;
		exit(0);
	}
	/*******************************Initialization*******************************/
	vector<string>dicnamestr;
	dicnamestr.clear();
	dicnamestr.push_back(dicname);
	dicnamestr.push_back(dicname4launch1);
	dicnamestr.push_back(dicname4launch2);

	ArUco_Camera_Init(cameraMatrix, distCoeffs);
	ArUco_Load_Dictionary(dicnamestr, aruco_dic, Dictionary);
	/*******************************Initialization*******************************/
	cout << "Aruco_Dec: start" << endl;

	/*saving*/	
	ofstream fout("./src/aruco_pos/file/test0.txt");

	fout << "frame" << "\t\t\t\t" << "point" << "\t\t\t\t\t\t\t\t" << "position" << endl;
	/*saving*/
	while(ros::ok() && cap.grab())
	{
		cap >> imgsrc;
		ArUco_Detect(imgsrc, Dictionary, Corners, Ids);	//detection
		if (Ids.size() != 0)
		{
			cout << "Total target = " << Ids.size() << endl;
			if(Ids.size() > 1)
			{
				cout << "More that one!!!......" << endl;
			}
			else;
			ArUco_Uav_Pos_Cal(imgsrc,
							  Corners,
							  Ids,
							  cameraMatrix,
							  distCoeffs,
							  aruco_dic,
							  v_position,
							  v_eular,
							  v_quat);
			vector<Point2f> temp = Corners[0];
			cout << "<<<<<<<<-------->>>>>>>>" << endl;
			cout << "Point = { " << temp[0] << ", "
								 << temp[1] << ", "
								 << temp[2] << ","
								 << temp[3] <<" }"<< endl;
			cout << "Pos = [ " << v_position[0] << ", "
							   << v_position[1] << ", "
							   << v_position[2] <<" ]"<< endl;
			cout << "Quat = [" << v_quat[0] << ", "
							   << v_quat[1] << ", "
							   << v_quat[2] << ", "
							   << v_quat[3] << " ]" << endl;
			cout << "<<<<<<<<-------->>>>>>>>" << endl;

			fout << Frame_All << "\t\t";
			fout << temp[0] << "," << temp[1] << "," << temp[2] << "," << temp[3] << "\t\t";
			fout << v_position[0] << "," << v_position[1] << ","<< v_position[2] << endl;
		}
		else	//no aruco
		{
			fout << Frame_All << "\t\t\t\t\t\t\t\t\t\t";
			fout << "NO" << endl;
			cout << "No Aruco!" << endl;
		}
		imshow("imgsrc", imgsrc);
		if(waitKey(0) == 27)
			break;

		ros::spinOnce();
		rate.sleep();
		Frame_All++;
	}
	cout << "Aruco_Dec: stop" << endl;
	fout.close();
	destroyAllWindows();
	cap.release();
	return 0;
}
