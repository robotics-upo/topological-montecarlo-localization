#include <ros/ros.h>
#include <string>
#include "particlefilter_sewer.hpp"

int main(int argc, char **argv)
{
  // Particle filter instance
  std::string node_name = "amcl_sewer_node";      
  ros::init(argc, argv, node_name);  
  
  ParticleFilter pf(node_name);
  
  // Process data at given rate
  ros::spin();

  return 0;
}




