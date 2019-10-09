#include <ros/ros.h>
#include <string>
#include "particlefilter_sewer_fuse_odometry.hpp"

int main(int argc, char **argv)
{
  // Particle filter instance
  std::string node_name = "amcl_sewer_node";      
  ros::init(argc, argv, node_name);
  
  ros::NodeHandle lnh("~");

  ParticleFilter *pf = NULL;
  bool fused = false;
  if (!lnh.getParam("fused", fused)) {
    fused = false;
  }
  if (fused) {
    pf = new ParticleFilterFused(node_name);
    ROS_INFO("Fused mode");
  } else {
    pf = new ParticleFilter(node_name);
    ROS_INFO("Non Fused mode");
  }
  
  // Process data at given rate
  ros::spin();

  delete pf;

  return 0;
}




