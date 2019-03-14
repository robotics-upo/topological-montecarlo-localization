#!/usr/bin/env python

PKG = 'manhole_detector'
import roslib; roslib.load_manifest(PKG)
import rospy
import fileinput
from manhole_detector.msg import Manhole
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import sys
import tf
import math

class GroundTruth:    
    
  def pose_callback(self, pose):
    self.varx = pose.pose.covariance[0]
    self.vary = pose.pose.covariance[7]
    self.vara = pose.pose.covariance[35]
    self.covarxy = pose.pose.covariance[1]

  def rgb_callback(self, img):
    seq = img.header.seq
    #print "RGB Callback. Seq: %d"%seq
    msg_mh = Manhole()
    #print msg_mh
    
    try:
      (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
      (roll, pitch, yaw) = euler_from_quaternion(rot)
      now = rospy.get_rostime()
      self.traj_file.write('{0} \t {1} \t {2} \t {3} \t {4} \t {5} \t {6} \t {7}.{8} \n'.format(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], now.secs, format(now.nsecs, '0>9')))
      for i in range(len(self.detected_vector)):
	
	if self.detected_vector[i][0] <= seq and self.detected_vector[i][1] >= seq:
	  #  Prepare all data for publishing
	  if len(self.detected_vector[i]) > 2:
	    msg_mh.id = self.detected_vector[i][2]
	  
	  
	  if len(self.detected_vector[i]) > 4:
	    print msg_mh.local_pose.x
	    msg_mh.local_pose.x = self.detected_vector[i][3]
	    msg_mh.local_pose.y = self.detected_vector[i][4]
	    # save a file with the distance to the manhole
	    dist = math.sqrt( (trans[0] - msg_mh.local_pose.x) ** 2 + (trans[1] - msg_mh.local_pose.y) ** 2)
	    text_ = '{0}  \t \t  {1} {2} {3} \t \t {4} {5} \t \t {6}\t{7} {8} {9} {10}\n'.format(msg_mh.id, trans[0], trans[1], yaw, msg_mh.local_pose.x, msg_mh.local_pose.y, dist, self.varx, self.vary, self.vara, self.covarxy)
	    self.stats_file.write(text_)
	    print "Writed to file:{0}".format(text_)
	    self.bool_pub.publish(msg_mh)
	  break
      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print "Exception catched while waiting for transform"
    
    
  def __init__(self, camera, filename, out_file, out_file_2):
    self.load_vector(filename)
    np.set_printoptions(precision=3, threshold=10000, linewidth=10000)
    rgb_image = camera + "/rgb/image_raw/compressed"
    rospy.Subscriber(rgb_image, CompressedImage, self.rgb_callback)
    rospy.Subscriber("/amcl_sewer_node/estimated_pose", PoseWithCovarianceStamped, self.pose_callback)
    
    # Setup publisher
    self.bool_pub = rospy.Publisher('ground_truth',Manhole, queue_size=2)
        
    # For statistics stuff
    self.listener = tf.TransformListener()
    self.stats_file = open(out_file, "w")
    print ("Opening trajectory out file {0}".format(out_file_2))
    self.traj_file = open(out_file_2, "w")
    
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
    rospy.init_node("ground_truth_manhole")
    out_file = "stats_python.txt"
    out_file_2 = "traj_python.txt"
    
    print "Orden:{0}", sys.argv
    
    if len(sys.argv) > 3:
      out_file = sys.argv[3]
    if len(sys.argv) > 4:
      out_file_2 = sys.argv[4]
    
    print "Writing stats to: {0}".format(out_file)
    detector = GroundTruth(sys.argv[1], sys.argv[2], out_file, out_file_2)
  else:
    print "Usage: %s <camera> <vector_file> [<stats file>] [<traj_file>]" % sys.argv[0]
    sys.exit()