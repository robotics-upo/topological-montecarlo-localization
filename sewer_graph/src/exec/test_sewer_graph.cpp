#include <sensor_msgs/NavSatFix.h>
#include "sewer_graph/sewer_graph.h"
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace sewer_graph;

int main(int argc, char ** argv) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <in_filename> [<out_filename>] [<output_kml_file>] [something if exporting kml only_manhole]\n";
    return -1;
  }
  string s(argv[1]);
  SewerGraph g(s);
  cout << g.toString_2() << endl; 
  
  if (argc > 2) {
    string f(argv[2]);
    if (g.writeGraph(f))
      cout << "Graph exported to " << f << endl;
    else
      cerr << "Could not export graph to " << f << endl;
  }
  
  if (argc > 3) {
    string kml_file(argv[3]);
    cout << "Exporting KML file to " << kml_file << endl;
    
    if (argc > 4) {
      g.exportKMLFile(kml_file, sewer_graph::MANHOLE);
      cout << "Exporting only manholes to KML\n";
    } else {
      g.exportKMLFile(kml_file);
    }
  }
  
  ros::init(argc, argv, "test_sewer");
  ros::NodeHandle nh;
  ros::NodeHandle lnh("~");
  
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 2, true);
  gps_pub.publish(g.getReferencePosition());

  string global_frame;
  if (!lnh.getParam("global_frame_id",global_frame)) {
    global_frame = "map";
  }
  
  ROS_INFO("Publishing markers");
  while (ros::ok()) {
    std::vector<visualization_msgs::Marker> vec = g.getMarkers(global_frame);
    for (unsigned int i = 0; i < vec.size(); i++) {
      marker_pub.publish(vec.at(i));
    }
    ros::spinOnce();
    gps_pub.publish(g.getReferencePosition());
    sleep(1);
  }
  
  return 0;
}