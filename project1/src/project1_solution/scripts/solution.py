#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts

rospy.init_node('solution')

def callback(msg):
#	print msg.a
#  	print msg.b
   	sum = msg.a + msg.b
	pub.publish(sum)	
  
pub = rospy.Publisher('sum', Int16, queue_size=10)
sub = rospy.Subscriber('two_ints', TwoInts, callback)

rospy.spin()