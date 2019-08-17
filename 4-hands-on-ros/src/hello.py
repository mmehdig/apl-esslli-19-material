#!/usr/bin/env python
import rospy
from std_msgs.msg import String
rospy.init_node("greeter")
rate = rospy.Rate(1) # it means 1hz
pub = rospy.Publisher("greeting_topic", String, queue_size=1)
while not rospy.is_shutdown():
   pub.publish("hello world {0}".format(rospy.get_time()))
   rate.sleep()
