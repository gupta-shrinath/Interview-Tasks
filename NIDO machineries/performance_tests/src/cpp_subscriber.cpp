#include "ros/ros.h"
#include "performance_tests/SuperAwesome.h"


void testcallback(const performance_tests::SuperAwesome::ConstPtr& msg) {
	ROS_INFO("Received [%s]",msg->message.c_str());
}

int main(int argc,char** argv) {
	ros::init(argc,argv,"subscriber_cpp");
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber = nodeHandle.subscribe("performance_tests",1000,testcallback);
	ros::spin();
	return 0;
}

