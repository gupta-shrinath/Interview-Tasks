#include "ros/ros.h"
#include <sstream>
#include "performance_tests/SuperAwesome.h"

int main(int argc,char **argv){
	ros::init(argc,argv,"publisher_cpp");
	ros::NodeHandle nodeHandle;
	ros::Publisher cppPublisher = nodeHandle.advertise<performance_tests::SuperAwesome>("performance_tests",1000);
	ros::Rate loop_rate(10);
	int count = 0;
	while(ros::ok()) {
		performance_tests::SuperAwesome msg;
		std::stringstream ss;
		ss<<"helloworld"<<count;
		msg.message = ss.str();
		ROS_INFO("%s",msg.message.c_str());
		cppPublisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
