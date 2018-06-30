#ifndef SEWER_GRAPH_H
#define SEWER_GRAPH_H

#include <string>
#include "config.h"
#ifdef USE_KML
#include <kml/dom.h>
#endif

#include "functions/DegMinSec.h"
#include "functions/RealVector.h"
#include "simple_graph/SimpleGraph.h"
#include "sewer_graph/earthlocation.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/NavSatFix.h>

namespace sewer_graph {
  
  
enum SewerVertexType {
  MANHOLE,
  FORK,
  NORMAL,
  
  ALL = 255
};

// Types
struct SewerVertex {
  EarthLocation e;
  std::string comments;
  
  double x, y; // Local coordinates
  
  std::string toString() const;
  SewerVertexType type;
  
  
};

struct SewerEdge {
  double distance;
  double route; // Angle relative to N (in the sense of waterflow)
  
  long id;
  
  
  std::string section; // Can be T181, D1400, T168, T133, NT120A, T164, T130...
  
  std::string toString() const;
};
  
  


//! @brief This class is used to represent an Earth Location in terms of longitude and latitude.
//! Uses decimal degrees as internal representation of latitude and longitude.
//! The altitude is stored in meters above the sea level.
//! ToString method gives many representations.
class SewerGraph:public simple_graph::SimpleGraph<SewerVertex, SewerEdge>
{
public:
  SewerGraph() {
    
  }
  
   //! @brief Constructor from file
  SewerGraph(const std::string &filename); 
  
  //! @brief Loads graph from a file
  bool loadGraph(const std::string &filename);
  
  bool writeGraph(const std::string &filename);
  
  virtual void addEdge(int i, int j);
  
  virtual void addEdge(int i, int j, int type, int suffix = 0);
  
//   std::string toString() const;        Por ahora usamos la versión de SimpleGraph
  
  double getDistanceToClosestManhole(double x, double y) const;
  
  double getDistanceToClosestVertex(double x, double y, SewerVertexType type = ALL) const;
  
  int getClosestVertex(double x, double y, SewerVertexType type = ALL) const;
  
  double getDistanceToClosestEdge(double x, double y, int &id1, int &id2) const;
  
  double getClosestEdgeAngle(double x, double y) const;
  
  //! @brief Exports the trajectory information into KML format
  //! @param filename Output filename
  //! @retval true The information has been successfully saved
  //! @retval false Errors while saving information
  bool exportKMLFile(const std::string &filename) const;
  
  //! Grief Exports the Graph in RViz format
  std::vector<visualization_msgs::Marker> getMarkers(std::string ref_frame) const;
  
  sensor_msgs::NavSatFix getReferencePosition() const;
  
  void setCenter(const EarthLocation &c) {
    center = c;
  }
  
  EarthLocation getCenter() const {
    return center;
  }

protected:
  EarthLocation center;
  // KML stuff -------------------------------------------------
#ifdef USE_KML  
  kmldom::KmlFactory* factory;
  
  void addKMLStyle(kmldom::DocumentPtr& doc) const;
#endif
  // End of KML stuff -----------------------------------------------
  
  //! @brief Parses the type: the thousand digit indicates: if 0 --> T. if 1 --> D, if 2 --> NT
  //! @param type The number as readed from file
  //! @param type The suffix ASCII code (65 = A, 66 = B, 67 = C) (if zero, none)
  //! @return The string describing the section type
  std::string parseSectionType(int type, char suffix = 0) const;
};


}

#endif // EARTHLOCATION_H

