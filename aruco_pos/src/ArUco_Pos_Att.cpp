#include "aruco_dec.h"

int Frame_All = 0;

Mat imgsrc, imggray;
ros::Subscriber video_sub;
ros::Publisher pos_pub;
aruco_pos::ArUco_Pos_Att aruco_pos_att;
mavros_msgs::CommandBool arming_client;
mavros_msgs::State current_state;

void Img_callback(sensor_msgs::Image img_msg)
{
	cv_bridge::CvImagePtr img_bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
	img_bridge_ptr->image.copyTo(imgsrc);
}

void SYS_State_Callback(const mavros_msgs::State& msg)
{
	current_state = msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Target_Dec");
	ros::NodeHandle nh;

	video_sub = nh.subscribe<sensor_msgs::Image>("img_pub", 10, Img_callback);
	pos_pub = nh.advertise<aruco_pos::ArUco_Pos_Att>("Global_Pos_Att", 10);

	ros::Subscriber SYS_State_Sub = nh.subscribe("mavros/state", 100, SYS_State_Callback);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

	ros::Rate rate(100);
	/*******************************Initialization*******************************/
	aruco_pos_att.name = "WSK";

	aruco_pos_att.header_cur.stamp = ros::Time::now();
	aruco_pos_att.header_cur.frame_id = "Global_Pos_Att";
	aruco_pos_att.header_cur.seq = Frame_All;

	aruco_pos_att.header_pre.stamp = ros::Time::now();
	aruco_pos_att.header_pre.frame_id = "Global_Pos_Att";
	aruco_pos_att.header_pre.seq = Frame_All;

	aruco_pos_att.header_pre2.stamp = ros::Time::now();
	aruco_pos_att.header_pre2.frame_id = "Global_Pos_Att";
	aruco_pos_att.header_pre2.seq = Frame_All;

	aruco_pos_att.Max_Limit_Exceeded_cur = false;
	aruco_pos_att.Max_Limit_Exceeded_pre = false;
	aruco_pos_att.Max_Limit_Exceeded_pre2 = false;

	aruco_pos_att.aruco_is_detect_cur = false;
	aruco_pos_att.aruco_is_detect_pre = false;
	aruco_pos_att.aruco_is_detect_pre2 = false;

	aruco_pos_att.aruco_total_num_cur = 0;
	aruco_pos_att.aruco_total_num_pre = 0;
	aruco_pos_att.aruco_total_num_pre2 = 0;

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
	time_t tname = time(0);
	char video_name[64], file_name[64];

	strftime(video_name,sizeof(video_name),"./src/aruco_pos/video/Video_%Y%m%d%H%M%S.mp4",localtime(&tname));
	VideoWriter writer(video_name, CV_FOURCC('M', 'J', 'P', 'G'), 60, Size(IMAGE_W, IMAGE_H), true);

	strftime(file_name,sizeof(file_name),"./src/aruco_pos/file/P_Q_%Y%m%d%H%M%S.csv",localtime(&tname));
	ofstream fout(file_name);

	fout << "local_position_x" << "," << "local_position_y" << "," << "local_position_z" << ","
		 << "q[0]" << "," << "q[1]" << "," << "q[2]" << "," << "q[3]" << endl;
	/*saving*/
	while(!imgsrc.data && ros::ok())
	{
		cout << "No data,waiting for image_publish..." << endl;
		ros::spinOnce();
		rate.sleep();
	}
	while(ros::ok())
	{
		if(imgsrc.data)
		{
			ArUco_Detect(imgsrc, Dictionary, Corners, Ids);	//detection
			if (Ids.size() != 0)
			{
				cout << "Total target = " << Ids.size() << endl;
				if(Ids.size() > 4)
				{
					cout << ">>>>>Max Limit Exceeded!!!!<<<<<" << endl;
					aruco_pos_att.Max_Limit_Exceeded_cur = true;
				}
				else
					aruco_pos_att.Max_Limit_Exceeded_cur = false;
				ArUco_Uav_Pos_Cal(imgsrc,
								  Corners,
								  Ids,
								  cameraMatrix,
								  distCoeffs,
								  aruco_dic,
								  v_position_bf,
								  v_eular,
								  v_quat);
				aruco_pos_att.header_cur.stamp = ros::Time::now();
				aruco_pos_att.header_cur.seq = Frame_All;

				aruco_pos_att.aruco_is_detect_cur = true;
				aruco_pos_att.aruco_total_num_cur = Ids.size();

				bandstop_filter(v_position_bf,v_position);

				aruco_pos_att.aruco_local_cur.position.x = v_position[0];
				aruco_pos_att.aruco_local_cur.position.y = v_position[1];
				aruco_pos_att.aruco_local_cur.position.z = v_position[2];
				aruco_pos_att.aruco_local_cur.orientation.x = v_quat[0];
				aruco_pos_att.aruco_local_cur.orientation.y = v_quat[1];
				aruco_pos_att.aruco_local_cur.orientation.z = v_quat[2];
				aruco_pos_att.aruco_local_cur.orientation.w = v_quat[3];

				cout << "<<<<<<<<-------->>>>>>>>" << endl;
				cout << "pos_x = " << v_position[0] << endl;
				cout << "pos_y = " << v_position[1] << endl;
				cout << "pos_z = " << v_position[2] << endl;
				cout << "qua_x = " << v_quat[0] << endl;
				cout << "qua_y = " << v_quat[1] << endl;
				cout << "qua_z = " << v_quat[2] << endl;
				cout << "qua_w = " << v_quat[3] << endl;
				cout << "<<<<<<<<-------->>>>>>>>" << endl;
				if(arming_client.exists())	//PX4 Connected
				{
					if(1)
					if((int)(current_state.armed))//If our uav needn't to be armed, add comments in this line
					{
						fout << v_position[0] << "," << v_position[1] << ","<< v_position[2] << ","
							 << v_quat[0] << "," << v_quat[1] << ","<< v_quat[2] << "," << v_quat[3] << endl;
						writer << imgsrc;
					}
					else
						cout << "Motor Not Armed" << endl;
				}
				else
				{
					fout << v_position[0] << "," << v_position[1] << ","<< v_position[2] << ","
						 << v_quat[0] << "," << v_quat[1] << ","<< v_quat[2] << "," << v_quat[3] << endl;
					writer << imgsrc;
				}
			}
			else	//no aruco
			{
				aruco_pos_att.header_cur.stamp = ros::Time::now();
				aruco_pos_att.header_cur.seq = Frame_All;

				aruco_pos_att.Max_Limit_Exceeded_cur = false;
				aruco_pos_att.aruco_is_detect_cur = false;
				aruco_pos_att.aruco_total_num_cur = 0;
				cout << "No Aruco!" << endl;
			}

			imshow("imgsrc", imgsrc);
			if(waitKey(5) == 27)
				break;
		}
		else	//no image data
		{
			aruco_pos_att.header_cur.stamp = ros::Time::now();
			aruco_pos_att.header_cur.seq = Frame_All;

			aruco_pos_att.Max_Limit_Exceeded_cur = false;
			aruco_pos_att.aruco_is_detect_cur = false;
			aruco_pos_att.aruco_total_num_cur = 0;
			cout << "Sorry! No image!" << endl;
			cout << "Please chesk your camera or camera node......" << endl;
		}

		pos_pub.publish(aruco_pos_att);
		Msg_Update(aruco_pos_att);

		ros::spinOnce();
		rate.sleep();
		Frame_All++;
	}
	cout << "Aruco_Dec: stop" << endl;
	fout.close();
	destroyAllWindows();
	writer.release();
	return 0;
}
