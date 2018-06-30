#include "sewer_graph/general.h"
#ifdef USE_KML
#include <kml/base/file.h>
#include <kml/engine.h>

#include <iostream>

namespace UAVFlightPlan {

bool writeKMLToFile(const std::string &file, const kmldom::KmlPtr &kml) {
  bool ret_val = true;

  // Then the file
  kmlengine::KmlFilePtr kmlfile = kmlengine::KmlFile::CreateFromImport(kml);
  if (!kmlfile) {
    std::cerr << "error: could not create kml file" << std::endl;
    return false;
  }
    
  // And write it
  std::string kml_data;
  kmlfile->SerializeToString(&kml_data);
  if (!kmlbase::File::WriteStringToFile(kml_data, file.c_str())) {
    std::cerr << "error: write of " << file << " failed" << std::endl;
    ret_val = false;
  }

  return ret_val;
}
}
#endif
