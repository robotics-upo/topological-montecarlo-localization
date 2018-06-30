#ifndef SIMPLE_VERTEX_H__
#define SIMPLE_VERTEX_H__

#include <string>
#include <sstream>
#include <list>
#include <map>

namespace simple_graph {

template <typename VertexType, typename EdgeType> class SimpleVertex {
  
  typedef typename std::map<int, EdgeType>::const_iterator map_iterator;
public:
  //! @brief Default constructor
  SimpleVertex();
	
  //! @brief Constructor with parameters
  SimpleVertex(int _index);
	
	//! @brief Constructor with parameters
	SimpleVertex(int _index, const VertexType &_content);
	
	//! @brief Destructor
	~SimpleVertex();
	
	 // //! @brief Returns the adjacency_list of SimpleVertex
//  	list <int> get_Neighbours();
	
	//! @brief Returns the vertex's index
	//! @return The index of the vertex
	int getIndex() const;
	
	//! @brief Adds a neighbour
	void addNeighbour(int index);
	
	//! @brief Adds a neighbour
	void addNeighbour(int index, const EdgeType &edge_content);
	
	//! @brief Returns the number of neighbours
	int nNeighbours() const;
	
	//! @brief Returns true if index is neighbour of the vertex
	bool isNeighbour(int index) const;
	
	bool getWeight(int neighbour_index, EdgeType &weight) const;
	
	inline std::list<int> getNeighbours() const {return adjacency_list;};
	
	//! @brief Returns a string that contains the info related to Simple Vertex class
	std::string toString() const;
        
        //! @brief Returns a string that contains the info related to Simple Vertex class (using tostring methods)
        std::string toString_2() const;
	
	VertexType getContent() const { return content;};
	
	private:
	std::list<int> adjacency_list; // Saves the identifiers of the adjacents vertices
	std::map<int, EdgeType> property_map; // Relates the adjacency list with the weights of the vertices
	
	int index; // Identifier
	
	VertexType content; // Content related to the vertex
};

// Returns the vertex's index
template <typename VertexType, typename EdgeType>
int SimpleVertex<VertexType, EdgeType>::getIndex() const {
	return index;
}

// Returns the number of neighbours
template <typename VertexType, typename EdgeType>
int SimpleVertex<VertexType, EdgeType>::nNeighbours() const {
	return adjacency_list.size();
}

// Returns true if index is neighbour of the vertex
template <typename VertexType, typename EdgeType>
inline bool SimpleVertex<VertexType, EdgeType>::isNeighbour(int index) const {
	bool ret_value=false;
	
	std::list <int>::const_iterator it;
	
	for (it = adjacency_list.begin();it!=adjacency_list.end() && !ret_value;it++) {
		ret_value = index == *it;
	}
	
	return ret_value;
}

template <typename VertexType, typename EdgeType>
bool SimpleVertex<VertexType, EdgeType>::getWeight(int neighbour_index, EdgeType& weight) const
{
  bool ret_val = isNeighbour(neighbour_index);
  
  if (ret_val) {
    map_iterator it = property_map.find(neighbour_index);
    ret_val = it != property_map.end();
    
    if (ret_val) {
      weight = it->second;
    }
  }
  
  return ret_val;
}


template <typename VertexType, typename EdgeType>
std::string SimpleVertex<VertexType, EdgeType>::toString() const
{
  std::ostringstream os;
	os << "Vertex number: "<< index << "\tNumber of neighbours:"<< nNeighbours()<<std::endl;
        os << "Content: " << content<< std::endl;
	
	std::list<int>::const_iterator it;
	os << "Neighbours:\t";
	for ( it = adjacency_list.begin(); it != adjacency_list.end(); it++ ) {
	  os << *it << " ";
	  map_iterator ed_it = property_map.find(*it);
	  if ( ed_it != property_map.end()) {
	    os << "Content: " << ed_it->second << " ";
	  }
	}
	os << std::endl;
	
	return os.str();
}

template <typename VertexType, typename EdgeType>
std::string SimpleVertex<VertexType, EdgeType>::toString_2() const
{
  std::ostringstream os;
        os << "Vertex number: "<< index << "\tNumber of neighbours:"<< nNeighbours()<<std::endl;
        os << "Content: " << content.toString() << std::endl;
        
        std::list<int>::const_iterator it;
        os << "Neighbours:\t";
        for ( it = adjacency_list.begin(); it != adjacency_list.end(); it++ ) {
          os << *it << " ";
          map_iterator ed_it = property_map.find(*it);
          if ( ed_it != property_map.end()) {
            os << "Content: " << ed_it->second.toString() << " ";
          }
        }
        os << std::endl;
        
        return os.str();
}

template <typename VertexType, typename EdgeType>
SimpleVertex<VertexType, EdgeType>::SimpleVertex() {
	index=0;
}

template <typename VertexType, typename EdgeType>
SimpleVertex<VertexType, EdgeType>::SimpleVertex(int _index) {
	index=_index;
}

template <typename VertexType, typename EdgeType>
SimpleVertex<VertexType, EdgeType>::SimpleVertex(int _index, const VertexType &_content)
{
  index = _index;
  content = _content;
}

template <typename VertexType, typename EdgeType>
SimpleVertex<VertexType, EdgeType>::~SimpleVertex()
{
  property_map.clear();
  adjacency_list.clear();
}

// Adds a neighbour
template <typename VertexType, typename EdgeType>
void SimpleVertex<VertexType, EdgeType>::addNeighbour(int index) {
  if (!isNeighbour(index)) {
    adjacency_list.push_back(index);
  }
}

template <typename VertexType, typename EdgeType>
void SimpleVertex<VertexType, EdgeType>::addNeighbour(int index, const EdgeType& edge_content)
{
  addNeighbour(index);
  property_map[index] = edge_content;
}

}

#endif