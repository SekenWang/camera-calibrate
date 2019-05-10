#include "aruco_dec.h"

Mat imgsrc;
ros::Subscriber video_sub;

void Img_callback(sensor_msgs::Image img_msg)
{
	cv_bridge::CvImagePtr img_bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
	img_bridge_ptr->image.copyTo(imgsrc);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Aruco_Video_Record");
	ros::NodeHandle nh;
	ros::Rate rate(100);
	video_sub = nh.subscribe<sensor_msgs::Image>("img_pub", 10, Img_callback);

	VideoWriter writer("./src/aruco_pos/video/test0.mp4",
					   CV_FOURCC('M', 'J', 'P', 'G'),
					   60,
					   Size(IMAGE_W, IMAGE_H),
					   true);

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
			writer << imgsrc;
			imshow("src", imgsrc);
			if(waitKey(5) == 27)
				break;
		}
		else;
		ros::spinOnce();
		rate.sleep();
	}
	writer.release();
	destroyAllWindows();
	return 0;
}
