#!/usr/bin/env python

PKG = 'manhole_detector'
import roslib; roslib.load_manifest(PKG)
import rospy
import fileinput
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import numpy as np
import sys

class JoyDetectorFeeder:    
    
  def joy_callback(self, joy):
    self.manhole = joy.axes[5] < 0.5;
    print "New manhole: ", self.manhole
    
  def __init__(self):
    rospy.Subscriber("joy", Joy, self.joy_callback)
    # Setup publisher
    self.bool_pub = rospy.Publisher('manhole',Bool, queue_size = 2)
    self.manhole = False
    rate = rospy.Rate(5)
    bool_msg = Bool(False)
    while not rospy.is_shutdown():
      bool_msg.data = self.manhole
      self.bool_pub.publish(bool_msg)
      rate.sleep()
                
if __name__ == '__main__':
  rospy.init_node("fake_detector_labeler")
  detector = JoyDetectorFeeder()
  