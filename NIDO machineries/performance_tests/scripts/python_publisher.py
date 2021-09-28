import rospy
from std_msgs.msg import String
from performance_tests.msg import SuperAwesome

def talker():
    pub = rospy.Publisher('performance_tests', SuperAwesome, queue_size=10)
    rospy.init_node('python_publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = SuperAwesome()
        count = 0
        hello_str.message = "hello world %s" % rospy.get_time() 
        
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
