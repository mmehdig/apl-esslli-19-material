#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(string):
   pub.publish("I heared \"{0}\"".format(string.data))

rospy.init_node("feedbacker")
pub = rospy.Publisher("feedback_topic", String, queue_size=1)
sub = rospy.Subscriber("greeting_topic", String, callback)
rospy.spin()

