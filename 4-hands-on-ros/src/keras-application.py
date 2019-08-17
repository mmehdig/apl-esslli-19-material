#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from keras.applications.resnet50 import ResNet50
from keras.preprocessing import image as kimage
from keras.applications.resnet50 import preprocess_input, decode_predictions
from keras import backend as K
import numpy as np
import os
from cv_bridge import CvBridge
import cv2

def img_to_array(img):
   return cv2.resize(CvBridge().imgmsg_to_cv2(img, "rgb8"), (224, 224))

dir_path = os.path.dirname(os.path.realpath(__file__))

model = ResNet50(weights='imagenet', include_top=True)

global h_object
global frame
h_object = "unknown"
frame = None

# This is just a warm-up call:
model.predict(np.zeros([1,224,224,3]))

rospy.init_node("greeter")
pub = rospy.Publisher("greeting_topic", String, queue_size=1) 

def callback(image_data):
   #rate = rospy.Rate(10) # 10hz
   #while not rospy.is_shutdown():
   x = img_to_array(image_data)
   frame = cv2.cvtColor(x, cv2.COLOR_RGB2BGR)
   cv2.imshow('frame',frame)
   keystroke = chr(cv2.waitKey(1) & 0xFF).lower()

   if keystroke == 'q':
      rospy.signal_shutdown("user hit q for quit")

   if keystroke == 'r':
      print("lets recognise")
      x = np.asarray(x, dtype=np.float32)
      x = np.expand_dims(x, axis=0)
      x = preprocess_input(x)
      preds = model.predict(x)
      h_object = decode_predictions(preds, top=3)[0][0][1]
      pub.publish("hello {0}".format(h_object))

sub = rospy.Subscriber("/camera/rgb/image_color", Image, callback)
rospy.spin()
