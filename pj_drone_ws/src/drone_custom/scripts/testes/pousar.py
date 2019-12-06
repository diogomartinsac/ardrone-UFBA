#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty

def pousar():
        pub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
        rospy.init_node('takeoff', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
          pub.publish(Empty())
          rate.sleep()

if __name__ == '__main__':
        try:
          pousar()
        except rospy.ROSInterruptException:
          pass