#ifndef ______GENERAL__H______
#define ______GENERAL__H______

#include "config.h"
#ifdef USE_KML
  #include <kml/dom.h>
#endif
#include <string>

namespace sewer_graph {
  
  #ifdef USE_KML
  //! @brief Writes KML data to a file
  //! @retval true The write operation was performed successfully
  //! @retval false Errors were found
  bool writeKMLToFile(const std::string &file, const kmldom::KmlPtr &kml);
#endif
  
}


#endif
