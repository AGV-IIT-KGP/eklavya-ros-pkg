#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_pub");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 10);
	//cv::WImageBuffer3_b image( cvLoadImage("data1.jpg",	CV_LOAD_IMAGE_COLOR) );
	sensor_msgs::ImagePtr msg; 	// =	sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");
	
	CvCapture* capture=0;
    IplImage* frame=0;
    capture = cvCaptureFromCAM(1); // read AVI video    
	if( !capture )
		throw "Error when reading steam_avi";
		
    
    ros::Rate loop_rate(10);
	int counter=1;
	while (nh.ok()) {
		frame = cvQueryFrame( capture );
		if(!frame)
        {    
			throw "Unable to get frame";
			break;
		}
		ROS_INFO("%d", counter);
    /*            if (counter == 1)
                  image.SetIpl( cvLoadImage("data1.jpg", CV_LOAD_IMAGE_COLOR) );
                if (counter == 2)
                  image.SetIpl( cvLoadImage("data2.jpg", CV_LOAD_IMAGE_COLOR) );
                if (counter == 3)
                  image.SetIpl( cvLoadImage("data3.jpg", CV_LOAD_IMAGE_COLOR) );
                if (counter == 4)
                  image.SetIpl( cvLoadImage("data4.jpg", CV_LOAD_IMAGE_COLOR) );
                if (counter == 5)
                  image.SetIpl( cvLoadImage("data5.jpg", CV_LOAD_IMAGE_COLOR) );
    */
                counter++;
    if (counter > 5)
        counter = 1;            
		msg =	sensor_msgs::CvBridge::cvToImgMsg(frame, "bgr8");
		pub.publish(msg);
	//ros::spinOnce();
		loop_rate.sleep();
	}
	cvReleaseImage(&frame);
	return (0);
}
