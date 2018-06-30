#!/usr/bin/env python

PKG = 'manhole_detector'
import roslib; roslib.load_manifest(PKG)
import rospy
import fileinput
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
import sys
from keras.models import load_model
import time
from cv_bridge import CvBridge, CvBridgeError
import cv2
from visualization_msgs.msg import Marker

class CNNDetector:    

  def depth_callback(self, img):
    bool_msg = Bool(False)
    depth_image = self.bridge.imgmsg_to_cv2(img, '32FC1')
    
    #downsampled = cv2.resize(depth_image, None, fx=0.25,fy=0.25)
    for i_ in range(0, 59):
      for j_ in range(0,79):
        if (np.isnan(depth_image[i_*4,j_*4])):
          #print "NAN"
          self.down_image[0,0,i_,j_] = 10.0;
        else:
          #print depth_image[i_*4, j_*4]
          self.down_image[0,0,i_,j_] = np.real(depth_image[i_*4,j_*4]);
          
    y = self.model.predict(self.down_image)
    print y
    if y[0,0]>= 0.9999999 or y[0,0]<=0.00000000000000001:
      return
    
    
    if y[0,0]>= self.thres:
      bool_msg.data = True
      marker = Marker()
      marker.header.frame_id = "/base_link"
      marker.header.stamp = rospy.Time.now()

      marker.type = marker.CYLINDER
      marker.action = marker.ADD
      marker.pose.orientation.w = 1
      marker.pose.position.x = 2.0
      marker.pose.position.y = 2.0

      t = rospy.Duration(2)
      marker.lifetime = t
      marker.scale.x = 1.6
      marker.scale.y = 1.6
      marker.scale.z = 1.6
      marker.color.a = 1.0
      marker.color.r = 1.0
      marker.color.g = 1.0
      marker.color.b = 1.0

      self.manhole_pub.publish(marker)
      
    else:
      bool_msg.data = False
    self.bool_pub.publish(bool_msg)
        
  def __init__(self, camera, filename):
    self.bridge = CvBridge()
    self.down_image = np.zeros((1,1,60,80), dtype=float)
    self.load_cnn(filename)
    np.set_printoptions(precision=3, threshold=10000, linewidth=10000)
    depth_image = camera + "/depth_registered/image_raw"
    depth_info = camera + "/depth_registered/camera_info"
    # Set up your subscriber and define its callback
    rospy.Subscriber(depth_image, Image, self.depth_callback)
    # Setup publisher
    self.bool_pub = rospy.Publisher('manhole',Bool, queue_size=2)
    self.manhole_pub = rospy.Publisher('manhole_marker', Marker, queue_size = 10)
    self.thres = 0.5
    
    
    
  def load_cnn(self, filename):
    self.model = load_model(filename)
    print "Loaded_model"
    print self.model.summary()
      
if __name__ == '__main__':
  if len(sys.argv) > 2:
    rospy.init_node("cnn_detector")
    detector = CNNDetector(sys.argv[1], sys.argv[2])
    if rospy.has_param('~thres'):
      detector.thres = rospy.get_param('~thres')
      print "New threshold: %f"%detector.thres
    # Spin until ctrl + c
    rospy.spin()
  else:
    print "usage: %s <camera> <cnn_file>" % sys.argv[0]
    sys.exit()