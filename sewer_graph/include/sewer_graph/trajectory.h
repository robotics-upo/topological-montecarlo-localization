#include "earthlocation.h"
#include <vector>

namespace sewer_graph {

	class Trajectory:public std::vector<EarthLocation> {
	public:
	  
	  enum ExportType {
	    LINE, POINT
	  };
		//! @brief Constructor from UAV Navigation file
		//! @param filename Input filename
		Trajectory(const std::string &filename, const EarthLocation &center, bool third_is_time = true);
		
		//! @brief Default constructor. Creates an empty trajectory
		Trajectory();
		
		//! @brief Exports the trajectory information into KML format
		//! @param filename Output filename
		//! @param begin The first vector position to export
		//! @param end The last vector position to export
		//! @retval true The information has been successfully saved
		//! @retval false Errors while saving information
		bool exportKMLFile(const std::string &filename, int begin = 0, int end = -1, Trajectory::ExportType type = LINE) const;
		
		//! @brief Returns a placemark with the trajectory information in KML format
		//! @param place_name The name of the placemark
		//! @param begin The first vector position to export
		//! @param end The last vector position to export
		//! @return A smart pointer (see libkml details, but it has not to be freed) with the place
		kmldom::PlacemarkPtr getKMLPlaceMarck(const std::string &place_name, int begin = 0, int end = -1) const;

		//! @brief Returns a placemark with the trajectory information in KML format
		//! @param place_name The name of the placemark
		//! @param i The point to export
		//! @return A smart pointer (see libkml details, but it has not to be freed) with the place
		kmldom::PlacemarkPtr getKMLPointPlaceMarck(const std::string &place_name, int i) const;
		
		//! @brief Returns a placemark with the trajectory information in KML format as a sequence of points
		//! @param place_name The name of the placemark
		//! @param begin The first vector position to export
		//! @param end The last vector position to export
		std::vector<kmldom::PlacemarkPtr> getKMLPlaceMarcks(const std::string &place_name, int begin, int end) const;
		
		inline void setStyle(bool s) { style = s; }
		
		//! @brief Returns a trajectory cutted
		//! @param begin Begin of the cut
		//! @param end End of the cut
		//! @return A new trajectory that has the information between begin and end
		Trajectory cut(int begin, int end) const;
    
		//! @brief Calculates the distance between this trajectory and another.
		//! @param traj The other trajectory
		//! @param dist Out parameter: will save the distance between the two points
		void distance(const Trajectory &traj, std::vector< double > &dist);
		
		//! @brief Shifts the trajectory
		//! @param north Shift in north direction in meters
		//! @param east Shift in east direction in meters
		//! @param alt Altitude shift in meters
		//! @param time Time shift in seconds
		void shift(double north, double east, double alt = 0.0, double time = 0.0);
		
		//! @brief Gets the state that corresponds with a determinate mission time
		//! If not found, interpolates. If the end is reached, returns a negative value
		//! @param time The instant of time
		//! @return The approximate location at this time
		bool getStateFromTime(double time, EarthLocation &e);
		
		//! @brief Represents the info of the class with a std string
		//! @return The information
		std::string toString() const;
		
		//! @brief Represents the info of the class with a std string in matlab format
		//! @return The information
		std::string toMatlab(const EarthLocation *center = NULL, unsigned int begin = 0, int end = -1) const;
		
		  bool fromLocalTrajectory(const string& filename, const sewer_graph::EarthLocation& center, bool reverse = false, bool third_is_time = true);
		  
	protected:
		//! @brief Returns a linestring with the trajectory information in KML format
		//! @param begin The first vector position to export
		//! @param end The last vector position to export
		//! @return A smart pointer (see libkml details, but it has not to be freed) with the line
		kmldom::LineStringPtr getKMLLineString(int begin = 0, int end = -1) const;

		kmldom::KmlFactory* factory;
		
		bool style; // Tries or not to make a stylish KML
		
		void init();

		void addKMLStyle(kmldom::DocumentPtr& doc) const;

		//! @brief Checks the bounds adapting them to the vector size
		//! @param begin Begin bound
		//! @param end End bound
		void checkBounds(int &begin, int &end) const;

	};

}
