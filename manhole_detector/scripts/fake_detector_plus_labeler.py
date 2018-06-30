#!/usr/bin/env python

PKG = 'manhole_detector'
import roslib; roslib.load_manifest(PKG)
import rospy
import fileinput
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
import numpy as np
import sys

class FakeDetectorLabeler:    
    
  def export_depth_image(msg, file_):
    #print len(msg.data)
    nparr = np.fromstring(msg.data, np.uint8)
    gray_image = cv2.imdecode(nparr[12:], cv2.CV_LOAD_IMAGE_GRAYSCALE) # CompressedDepth format adds 12 bits of garbage at the beginning
    #print gray_image
    downsampled = cv2.resize(gray_image, None, fx=0.25, fy=0.25)
    file_.write(np.array2string( downsampled).replace('[','').replace(']',''))

  def rgb_callback(self, img):
    self.seq = img.header.seq
    self.time = img.header.stamp.secs* 1000000000 + img.header.stamp.nsecs
    print "RGB Callback. Seq: %d"%self.seq
    bool_msg = Bool(False)
    id_msg = Int32(-1)
    pose_msg = Pose2D(float('nan'), float('nan'), float('nan'))
    
  def depth_callback(self, img):
    print "Depth Callback"
    #seq = img.header.seq
    #bool_msg = Bool(False)
    #for i in range(len(self.detected_vector)):
      #if self.detected_vector[i][0] < seq and self.detected_vector[i][1] > seq:
        ##bool_msg.data = True
        #break
    #self.bool_pub.publish(bool_msg)
        
  def __init__(self, camera, filename):
    np.set_printoptions(precision=3, threshold=10000, linewidth=10000)
    rgb_image = camera + "/rgb/image_raw/compressed"
    rgb_info = camera + "/rgb/camera_info"
    rospy.Subscriber(rgb_image, CompressedImage, self.rgb_callback)
    
    # Setup publisher
    self.bool_pub = rospy.Publisher('manhole',Bool, queue_size=2)
    #self.pose_pub = rospy.Publisher('position', Pose2D, queue_size=2)
    # Spin until ctrl + c
    a = raw_input('ENTER exit to halt program. Any other word to switch from manhole detected or not')
    manhole_msg = Bool(False)
    i = 0
    curr_vec = np.array([0, 0])
    curr_time = np.array([0, 0])
    self.seq = 0
    self.time = 0
    self.detected_manholes = np.array([[0, 0]])
    self.detected_time = np.array([[0, 0]])
    while a != 'exit':
      manhole_msg.data = not manhole_msg.data
      self.bool_pub.publish(manhole_msg)
      if manhole_msg.data:
        curr_vec[0] = self.seq
        curr_time[0] = self.time
      else:
        curr_vec[1] = self.seq
        curr_time[1] = self.time
        self.detected_manholes = np.vstack([self.detected_manholes, curr_vec])
        self.detected_time = np.vstack([self.detected_time, curr_time])
        
      a = raw_input('')
    np.savetxt(filename, self.detected_manholes)
    np.savetxt('time_'+filename, self.detected_time)
    
                
if __name__ == '__main__':
  if len(sys.argv) > 2:
    rospy.init_node("fake_detector_labeler")
    detector = FakeDetectorLabeler(sys.argv[1], sys.argv[2])
  else:
    print "usage: %s <camera> <output_vector_file>" % sys.argv[0]
    sys.exit()