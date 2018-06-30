#!/usr/bin/env python

PKG = 'manhole_detector'
import roslib; roslib.load_manifest(PKG)
import rospy
import fileinput
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import sys
import tf
import math

class FakeDetector:    

  # Export images to files:
  def export_rgb_image(msg, file_):
    nparr = np.fromstring(msg.data,np.uint8)
    gray_image = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    detect_manhole(gray_image, min_rad)
    downsampled = cv2.resize(gray_image, None, fx=0.25, fy=0.25)
    #cv2.imshow('img',downsampled)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    file_.write(np.array2string( downsampled).replace('[','').replace(']',''))
    
  def export_depth_image(msg, file_):
    #print len(msg.data)
    nparr = np.fromstring(msg.data, np.uint8)
    gray_image = cv2.imdecode(nparr[12:], cv2.CV_LOAD_IMAGE_GRAYSCALE) # CompressedDepth format adds 12 bits of garbage at the begining
    #print gray_image
    downsampled = cv2.resize(gray_image, None, fx=0.25, fy=0.25)
    file_.write(np.array2string( downsampled).replace('[','').replace(']',''))
    
    
  def pose_callback(self, pose):
    self.varx = pose.pose.covariance[0]
    self.vary = pose.pose.covariance[7]
    self.vara = pose.pose.covariance[35]
    self.covarxy = pose.pose.covariance[1]

  def rgb_callback(self, img):
    seq = img.header.seq
    #print "RGB Callback. Seq: %d"%seq
    bool_msg = Bool(False)
    id_msg = Int32(-1)
    pose_msg = Pose2D(float('nan'), float('nan'), float('nan'))
    
    for i in range(len(self.detected_vector)):
      if self.detected_vector[i][0] <= seq and self.detected_vector[i][1] >= seq:
        #  Prepare all data for publishing
        if len(self.detected_vector[i]) > 2:
          id_msg.data = self.detected_vector[i][2]
        bool_msg.data = True
        
        if len(self.detected_vector[i]) > 4:
          pose_msg.x = self.detected_vector[i][3]
          pose_msg.y = self.detected_vector[i][4]
          # save a file with the distance to the manhole
          try:
            (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            dist = math.sqrt( (trans[0] - pose_msg.x) ** 2 + (trans[1] - pose_msg.y) ** 2)
            (roll, pitch, yaw) = euler_from_quaternion(rot)
            text_ = '{0}  \t \t  {1} {2} {3} \t \t {4} {5} \t \t {6}\t{7} {8} {9} {10}\n'.format(id_msg.data, trans[0], trans[1], yaw, pose_msg.x, pose_msg.y, dist, self.varx, self.vary, self.vara, self.covarxy)
            
            self.stats_file.write(text_)
            print "Writed to file:{0}".format(text_)
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Exception catched while waiting for transform"
        break
    self.bool_pub.publish(bool_msg)
    self.id_pub.publish(id_msg)
    self.pose_pub.publish(pose_msg)
    
  def __init__(self, camera, filename, out_file):
    self.load_vector(filename)
    np.set_printoptions(precision=3, threshold=10000, linewidth=10000)
    rgb_image = camera + "/rgb/image_raw/compressed"
    rgb_info = camera + "/rgb/camera_info"
    rospy.Subscriber(rgb_image, CompressedImage, self.rgb_callback)
    rospy.Subscriber("/amcl_sewer_node/estimated_pose", PoseWithCovarianceStamped, self.pose_callback)
    
    # Setup publisher
    self.bool_pub = rospy.Publisher('manhole',Bool, queue_size=2)
    self.id_pub = rospy.Publisher('manhole_id', Int32, queue_size=2)
    self.pose_pub = rospy.Publisher('position', Pose2D, queue_size=2)
    
    # For statistics stuff
    self.listener = listener = tf.TransformListener()
    self.stats_file = open(out_file, "w")
    
    # Initialice covariances
    self.varx = 0
    self.vary = 0
    self.vara = 0
    self.covarxy = 0
    
    # Spin until ctrl + c
    rospy.spin()
    
    
    
  def load_vector(self, filename):
    self.detected_vector = np.loadtxt(filename)
    print "Loaded_vector"
    print self.detected_vector
      
if __name__ == '__main__':
  if len(sys.argv) > 2:
    rospy.init_node(sys.argv)
    out_file = "stats_python.txt"
    
    if len(sys.argv) > 3:
      out_file = sys.argv[3]
      
    print "Writing stats to: {0}".format(out_file)
    detector = FakeDetector(sys.argv[1], sys.argv[2], out_file)
  else:
    print "usage: %s <camera> <vector_file> [<stats file>]" % sys.argv[0]
    sys.exit()