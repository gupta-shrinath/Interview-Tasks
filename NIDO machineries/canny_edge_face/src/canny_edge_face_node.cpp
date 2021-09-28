#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace cv;
Mat src, src_gray;
Mat dst, detected_edges;
int lowThreshold = 0;
const int max_lowThreshold = 100;
const int ratio = 3;
const int kernel_size = 3;
const char* window_name = "Edge Map";

static void CannyThreshold(int, void*)
{
    blur( src_gray, detected_edges, Size(3,3) );
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    dst = Scalar::all(0);
    src.copyTo( dst, detected_edges);
    //imshow( window_name, dst );
}


void cameraImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try{
        //imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);        
        src = cv_bridge::toCvShare(msg, "bgr8")->image;
        dst.create( src.size(), src.type() );
        cvtColor( src, src_gray, COLOR_BGR2GRAY );
        // namedWindow( window_name, WINDOW_AUTOSIZE );
        // createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
        CannyThreshold(0, 0);
        ros::NodeHandle nodeHandle;
        image_transport::ImageTransport imageTransport(nodeHandle);
	    image_transport::Publisher edgeImagePublisher = imageTransport.advertise("canny_edge_image",1);
        waitKey(30);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
        ros::Rate loop_rate(5);
        while(nodeHandle.ok()) {
            edgeImagePublisher.publish(msg);
            loop_rate.sleep();
        }
    }
    catch (cv_bridge::Exception& e){
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
}
int main(int argc,char **argv) {
    ros::init(argc,argv,"canny_edge_face_subscriber");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("/cv_camera/image_raw/",1,cameraImageCallback);
    ros::spin();
	return 0;
}