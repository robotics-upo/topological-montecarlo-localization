#include "sewer_graph/trajectory.h"
#include "sewer_graph/earthlocation.h"
#include "functions/DegMinSec.h"
#include "functions/functions.h"
#include <vector>
#include <iostream>

#ifdef USE_KML
#include "kml/base/file.h"
#include "kml/base/math_util.h"
#include "kml/dom.h"
#include "kml/engine.h"
#include "kml/convenience/convenience.h"
#include "kml/base/string_util.h"

using kmldom::CoordinatesPtr;
using kmldom::KmlFactory;
using kmldom::LineStringPtr;
using kmldom::PlacemarkPtr;
using kmldom::KmlPtr;
using kmlengine::KmlFile;
using kmlengine::KmlFilePtr;
#endif

using namespace std;
using functions::DegMinSec;

namespace sewer_graph {

Trajectory::Trajectory(const string& filename, const sewer_graph::EarthLocation& center, bool third_is_time): std::vector<EarthLocation>()
{
	init();
	
	if (!fromLocalTrajectory(filename, center, true, third_is_time)) {
	  string content = "Could not load the trajectory file: ";
	  content.append(filename);
	  
	  throw (std::runtime_error(content));
	  
	}
}

Trajectory::Trajectory(): vector<EarthLocation>() {
  init();
}
	
#ifdef USE_KML
bool Trajectory::exportKMLFile(const std::string &filename, int begin, int end, Trajectory::ExportType type) const {
  bool ret = true;
  
  checkBounds(begin, end);
  
  // Create document. This is necessary to handle styles
  kmldom::DocumentPtr doc = factory->CreateDocument();
  switch (type) {
    case Trajectory::LINE:
      doc->add_feature(getKMLPlaceMarck(filename, begin, end));  // kml takes ownership.
      break;
      
    case Trajectory::POINT:
    default:
      vector<PlacemarkPtr> v = getKMLPlaceMarcks(filename, begin, end);
      for (unsigned int i = 0; i < v.size(); i++) {
	doc->add_feature(v.at(i));
      }
  }

  if (style) {
    // Generates the styles and retale them to the doc if the option is choosen
    addKMLStyle(doc);
  }
  
  // Create the kml pointer with the document
  KmlPtr kml = factory->CreateKml();
  kml->set_feature(doc);
  
  // Now the file and serialize it to string
  KmlFilePtr kmlfile = KmlFile::CreateFromImport(kml);
  if (!kmlfile) {
    cerr << "error: could not create kml file" << endl;
    return false;
  }
  std::string kml_data;
  kmlfile->SerializeToString(&kml_data);
  
  // Once we get the string --> write it to a file
  if (!kmlbase::File::WriteStringToFile(kml_data, filename.c_str())) {
    cerr << "error: write of " << filename << " failed" << endl;
    ret = false;
  }
  
  return ret;
}
#endif

string Trajectory::toString() const {
  ostringstream os;
  
  os << "MTIME\tLAT LON ALT" << endl;
  
  for (unsigned int i = 0; i < size(); i++) {
    os << at(i).toString();
    os << endl;
  }
  
  return os.str();
}

string Trajectory::toMatlab(const EarthLocation *center, unsigned int begin, int end) const
{
  ostringstream os;
  unsigned int end_ = end;
  if (end < 0) {
    end_ = size();
  }
  
  for (unsigned int i = begin; i < end_; i++) {
    if (center == NULL) {
      os << at(i).toMatlab();
    } else {
      os << at(i).toMatlab(*center);
    }
    os << endl;
  }
  
  return os.str();
}

#ifdef USE_KML
PlacemarkPtr Trajectory::getKMLPlaceMarck(const std::string &place_name, int begin, int end) const {
  LineStringPtr linestring = getKMLLineString(begin, end);
  
  // Create the placemark containing the linestring
  PlacemarkPtr ret = factory->CreatePlacemark();
  ret->set_geometry(linestring);
  ret->set_name(place_name);
  ret->set_styleurl("#linestylemap");
    
  return ret;
}

vector<PlacemarkPtr> Trajectory::getKMLPlaceMarcks(const std::string &place_name, int begin, int end) const {
  vector<PlacemarkPtr> ret;
  for(unsigned int i = begin; i < end; i++) {
    ostringstream os;
    os << place_name << i + 1;
    PlacemarkPtr aux = at(begin).getKMLPlacemark(os.str());
    ret.push_back(aux);
  }
  return ret;
}

LineStringPtr Trajectory::getKMLLineString(int begin, int end) const {
  
  CoordinatesPtr coords = factory->CreateCoordinates();
  LineStringPtr linestring = factory->CreateLineString();

  checkBounds(begin, end);
  
  // First create the vector with the coordinates
  for (unsigned int i = begin; i < end; i++) {
    coords->add_latlngalt(at(i).getLatitude(), at(i).getLongitude(), at(i).getAltitude());
  }
  
  // Then assing them to the linestring
  linestring->set_coordinates(coords);
	
  return linestring;
}

void Trajectory::addKMLStyle(kmldom::DocumentPtr &doc) const
{
	// Create the style 
	kmldom::StylePtr normal = factory->CreateStyle();
  normal->set_id("normal");
  kmldom::LineStylePtr linestyle = factory->CreateLineStyle();
  linestyle->set_width(5.0);
	kmlbase::Color32 red("red");
	linestyle->set_color(red);
  normal->set_linestyle(linestyle);
  doc->add_styleselector(normal);
	
	kmldom::StyleMapPtr stylemap = factory->CreateStyleMap();
  stylemap->set_id("linestylemap");
  kmldom::PairPtr pair = factory->CreatePair();
  pair->set_key(kmldom::STYLESTATE_NORMAL);
  pair->set_styleurl("#linestyle");
  stylemap->add_pair(pair);
	
	doc->add_styleselector(stylemap);
}
#endif

void Trajectory::init() {
  style = false;
#ifdef USE_KML
  factory = KmlFactory::GetFactory();
#endif
}

Trajectory Trajectory::cut(int begin, int end) const {
  checkBounds(begin, end);
  Trajectory ret;
  
  double init_time = at(begin).getTime();
  
  for (int i = begin; i < end; i++) {
    
    ret.push_back(at(i));
  }
  
  ret.shift(0.0, 0.0, 0.0, -init_time);
  
  return ret;
}

void Trajectory::checkBounds(int &begin, int &end) const {
  if (end < 0 || end > (int)size()) {
    end = size(); 
  }
  
  if (begin < 0 || begin > (int)size()) {
    begin = 0;
  }
}

void Trajectory::distance(const Trajectory &traj, std::vector< double > &dist) {
  dist.clear(); // First clear the previous contents of dist vector
  
  for (int i = 0; i < traj.size() && i < size(); i++) {
    dist.push_back( at(i).distance(traj.at(i)) );
  }
}

void Trajectory::shift(double north, double east, double alt, double time) {
  for (int i = 0; i < size(); i++) {
    at(i).shift(north,east,alt, time);
  }
}

bool Trajectory::getStateFromTime(double time, EarthLocation &e) {
  bool found = false;
  
  if ( time < at(0).getTime() || size() == 0) {
    
    return false;
  }
  
  for (int i = 0; i < size() && !found ; i++) {
    if ( at(i).getTime() == time) {
      e = at(i);
      found = true;
    } else if (at(i).getTime() > time && i > 0) {
      // We have to interpolate
      EarthLocation &ant = at(i - 1);
      EarthLocation &pos = at(i);
      e = ant.interpolate(pos, time);
      found = true;
    }
   
  }
  
  return found;
}

bool Trajectory::fromLocalTrajectory(const string& filename, const EarthLocation &center, bool reverse, bool third_is_time)
{
  bool ret_val = true;
  
  vector<vector<double> > matrix;
  ret_val = functions::getMatrixFromFile(filename, matrix);
  
  if (ret_val) {
    ret_val = matrix.size() > 0 && matrix.at(0).size() >= 2;
    for (unsigned int i = 0; i < matrix.size() && ret_val; i++) {
      EarthLocation aux;
      std::vector<double> v = matrix.at(i);
      if (third_is_time) {
	v.resize(2);
      }
      aux.fromRelative(v, center, reverse);
      if (third_is_time && matrix[i].size() > 2) {
	aux.shift(0, 0 , 0, matrix[i][2]);
      }
      push_back(aux);
    }
  }
  
  return ret_val;
}

}
