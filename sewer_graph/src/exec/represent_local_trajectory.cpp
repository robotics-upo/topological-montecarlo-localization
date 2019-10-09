// Publishes a local trajectory
#include <iostream>
#include "sewer_graph/trajectory.h"
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <functions/functions.h>

using namespace functions;
using namespace std;
using namespace sewer_graph;


int main(int argc, char **argv) {
  if (argc < 2) {
    cout << "Use: " << argv[0] << " <input_file> [<options>]\n";
    return -1;
  }
  
  ostringstream node_name;
  if(argc>2)
    node_name << "trajectory_pub" << argv[2];
  ros::init(argc, argv, node_name.str());
  ros::NodeHandle nh;
  ros::NodeHandle lnh("~");
  ros::Subscriber sub;
  ros::Publisher marker_pub;

  string frame;
  if (!lnh.getParam("frame", frame))
    frame = "map";

  marker_pub = nh.advertise<visualization_msgs::Marker>("trajectory_marker", 1);



  vector<vector<double> > M;
  visualization_msgs::Marker lines;
  lines.header.frame_id = frame;
  lines.header.stamp = ros::Time::now();
  // lines.ns = "sewer_graph";
  lines.action = visualization_msgs::Marker::ADD;
  lines.pose.orientation.w = 1.0;
  lines.id = 2;
  lines.type = visualization_msgs::Marker::LINE_STRIP;

  lines.scale.x = 0.5;
  lines.scale.y = 0.5;

  lines.color.r = 1.0;
  lines.color.g = 0;
  lines.color.b = 0;
  lines.color.a = 0.7;

  string color;
  if (lnh.getParam("color", color)) {
      ROS_INFO("Got color: %s", color.c_str());
      if (color == "b") {
        lines.color.b = 1;
        lines.color.r = 0;
      }
      if (color == "g") {
        lines.color.g = 1;
        lines.color.r = 0;
      }
      if (color == "k") {
        lines.color.r = 0;
      }
      if (color == "w") {
        lines.color.g = 1;
        lines.color.b = 1;
      }
      if (color == "y") {
        lines.color.g = 1;
      }
      if (color == "p") {
        lines.color.b = 1;
      }  

  }

  int downsample = 10;
  if (!lnh.getParam("downsample",downsample))
    downsample = 10;

  try {
    getMatrixFromFile(argv[1], M);

    for (unsigned int i = 0; i < M.size(); i+=downsample) {
        geometry_msgs::Point p;
        p.x = M[i][0];
        p.y = M[i][1];
        lines.points.push_back(p);
    }
  } catch (exception &e) {
    cerr << "Error while reading the file: " << argv[1] << "\t Exception content: " << e.what() << endl;
    return -2;
  }

  while(ros::ok()) {
      marker_pub.publish(lines);
      sleep(5);
  }

  return 0;
}

