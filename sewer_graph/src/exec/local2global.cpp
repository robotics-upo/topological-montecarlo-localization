// Translates a UAV Navigation flight plan into a KML file

#include <functions/ArgumentData.h>
#include <functions/functions.h>
#include <iostream>
#include "sewer_graph/trajectory.h"
#include <string>

using namespace functions;
using namespace std;
using namespace sewer_graph;

int main(int argc, char **argv) {
  ArgumentData arg(argc, argv);
  
  if (argc < 5) {
    cout << "Use: " << arg[0] << " <input_file> <output_file> <lat> <lon>\n";
    return -1;
  }
  
  EarthLocation e(stof(arg[3]), stof(arg[4]));
  
  try {
    Trajectory traj(arg[1], e);
  
    int begin = 0;
    int end = traj.size();
  
    if ( arg.isOption("begin") ) {
      arg.getOption<int>("begin", begin);
    }
  
    if ( arg.isOption("end") ) {
      arg.getOption<int>("end", end);
    }
    string s;
    if (arg[2].find("kml") != std::string::npos) {
      cout << "Exporting kml file: " << arg[2] << endl;
      traj.exportKMLFile(arg[2]);
    } else {    
      cout << "Exporting text file: " << arg[2] << endl;
      if (!functions::writeStringToFile(arg.at(2), traj.toMatlab(NULL, begin, end)) ) {
	std::cerr << "Errors found while writing the global MATLAB file.\n";
	return -3;
      }
    }
  } catch (exception &e) {
//     cerr << "Could not read the file: " << arg[1] << endl;
    cerr << "Error while reading the file: " << arg[1] << "\t Exception content: " << e.what() << endl;
    return -2;
  }
  
  return 0;
}
  
