#ifndef SIMPLE_GRAPH_H__
#define SIMPLE_GRAPH_H__

#include "SimpleVertex.h"
#include <string>
#include <sstream>
#include <iostream>
#include <list>
#include <vector>

namespace simple_graph {

//! This class implements a simple directed graph
template <typename VertexType, typename EdgeType> class SimpleGraph {
	public:
	//! @brief Default constructor
	//! @param sym Indicates whether the graph will be symmetrical or not
	SimpleGraph(bool sym = false);
	
	//! @brief Constructs a graph with n_vertices and no edges
	//! @param n_vertices The number of vertices
	//! @param sym Indicates whether the graph will be symmetrical or not
	SimpleGraph(int n_vertices, bool sym = false);
	
	//! @brief Constructs a graph with n_vertices with content and no edges
	//! @param vertex_content The content associated with each vertex
	//! @param sym Indicates whether the graph will be symmetrical or not
	SimpleGraph(const std::list<VertexType> &vertex_content, bool sym = false);
	
	//! @brief Adds an edge starting in index1 and finishing in index2
	//! @retval true if the edge was added successfully
	//! @retval false The edge cannot be added
	bool addEdge(int index1, int index2);
	
	//! @brief Adds an edge starting in index1 and finishing in index2
	//! @retval true if the edge was added successfully
	//! @retval false The edge cannot be added
	bool addEdge(int index1, int index2, const EdgeType &edge_content);
	
	//! @brief Adds a vertex in the last place
	//! @return The id of the created vertex
	int addVertex();
	
	//! @brief Adds a vertex in the last place
	//! @param vertex_content Information associated to that vertex
	//! @return The id of the created vertex
	int addVertex(const VertexType &vertex_content);
	
	//! @brief Performs a depth-first-search in the graph to find a node target from a node root
	//! @param root Node in wich the search is started
	//! @param target Node to reach
	//! @retval true There is a path between root and target
	//! @retval false No connection between this two nodes
	bool depthFirstSearch(int root, int target) const;
	
	//! @brief Returns the number of vertices of the graph
	//! @return The number of vertices
	int nVertices() const;
	
	//! @brief Returns the number of edges of the graph
	//! @return The number of edges
	int nEdges() const;
	
	//! @brief Returns the number of edges that exit node x
	//! @return The number of edges
	int nEdges(int vertex_id) const;
	
	//! @brief Returns true if there is an edge that starts in index1 and finishes in index2
	//! @param index1 Index number of the first node
	//! @param index2 Index number of the second node
	bool isEdge(int index1, int index2) const;
	
	//! @brief Translates the information contained in the class into a string
        //! @note The type has to have implemented the operator << 
	std::string toString() const;
        
        //! @brief Translates the information contained in the class into a string
        //! @note The type has to have implemented the operator << 
        std::string toString_2() const;
	
	//! @brief Clears all info stored in the class
	void clear();
	
	virtual VertexType getVertexContent(int vertex_id) const { 
	  
	  return vertices.at(vertex_id).getContent();
	};
	
	inline bool getEdgeContent(int index1, int index2, EdgeType &content) const {
	  bool ret_val = false;
	  if (isEdge(index1, index2)) {
	    if (vertices[index1].getWeight(index2, content)) {
	      ret_val = true;
	    }
	  }
	  
	  return ret_val;
	}
	
	inline std::list<int> getEdges(int vertex_id) const {
	  return vertices[vertex_id].getNeighbours();
	}
	
	protected:
	// The vertices list, each vertex has its adjacency list
	std::vector<SimpleVertex<VertexType, EdgeType> > vertices;
	bool symmetric;
	
	//! @brief This method implements the depth-first-search
	//! @param root Node in wich the search is started
	//! @param target Node to reach
	//! @param visitado An array that contains the vertices that have been visited right now
	//! @retval true There is a path between root and target
	//! @retval false No connection between these two nodes
	bool DFS(int root, int target, bool *visitado) const;
};

template <typename VertexType, typename EdgeType>
SimpleGraph<VertexType, EdgeType>::SimpleGraph(bool sym)
{
  symmetric = sym;
}

template <typename VertexType, typename EdgeType>
SimpleGraph<VertexType, EdgeType>::SimpleGraph(int n_vertices, bool sym)
{
  symmetric = sym;
  for (int i=0; i<n_vertices;i++) {
    addVertex();
  }
}

template <typename VertexType, typename EdgeType>
SimpleGraph<VertexType, EdgeType>::SimpleGraph(const std::list<VertexType> &_vertices, bool sym)
{
  symmetric = sym;
  std::list<VertexType> &v_it = _vertices.begin();
  for (; v_it != _vertices.end();v_it++) {
    addVertex(*v_it);
  }
}

template <typename VertexType, typename EdgeType>
void SimpleGraph<VertexType, EdgeType>::clear() {
  vertices.clear();
}

template <typename VertexType, typename EdgeType>
std::string SimpleGraph<VertexType, EdgeType>::toString() const
{
  std::ostringstream os;
  
  os << "Printing graph.\tN_vertices = " << nVertices() << "\tN_edges = " << nEdges() << std::endl;

  for ( unsigned int i=0; i < vertices.size(); i++) {
    os << vertices[i].toString();
  }
	
  os << "End of the graph" << std::endl << std::endl;
	
  return os.str();
}

template <typename VertexType, typename EdgeType>
std::string SimpleGraph<VertexType, EdgeType>::toString_2() const
{
  std::ostringstream os;
  
  os << "Printing graph.\tN_vertices = " << nVertices() << "\tN_edges = " << nEdges() << std::endl;

  for ( unsigned int i=0; i < vertices.size(); i++) {
    os << vertices[i].toString_2();
  }
        
  os << "End of the graph" << std::endl << std::endl;
        
  return os.str();
}


template <typename VertexType, typename EdgeType>
bool SimpleGraph<VertexType, EdgeType>::addEdge(int index1, int index2)
{
  bool ret_value = false;
  int n_v = nVertices();
	
  
  // TODO: Allow self pointing edges??
  if (index1 < n_v && index2 < n_v && index1 >= 0 && index2 >= 0 && index1 != index2) {
    ret_value = true;
    vertices[index1].addNeighbour(index2);
    if (symmetric) {
      vertices[index2].addNeighbour(index1);
    }
  }

  return ret_value;
}

template <typename VertexType, typename EdgeType>
bool SimpleGraph<VertexType, EdgeType>::addEdge(int index1, int index2, const EdgeType &edge_content)
{
  bool ret_value = false;
  int n_v = nVertices();
	
  
  // TODO: Allow self pointing edges??
  if (index1 < n_v && index2 < n_v && index1 >= 0 && index2 >= 0 && index1 != index2) {
    ret_value = true;
    vertices[index1].addNeighbour(index2, edge_content);
    if (symmetric) {
      vertices[index2].addNeighbour(index1, edge_content);
    }
  }

  return ret_value;
}

template <typename VertexType, typename EdgeType>
int SimpleGraph<VertexType, EdgeType>::addVertex()
{
  int ret = nVertices();
  SimpleVertex<VertexType, EdgeType> aux( nVertices() );
  vertices.push_back(aux);
  
  return ret;
}

template <typename VertexType, typename EdgeType>
int SimpleGraph<VertexType, EdgeType>::addVertex(const VertexType& vertex_content)
{
  int ret = nVertices();
  SimpleVertex<VertexType, EdgeType> aux( nVertices(), vertex_content);
  vertices.push_back(aux);
  return ret;
}

template <typename VertexType, typename EdgeType>
bool SimpleGraph<VertexType, EdgeType>::isEdge(int index1, int index2) const
{
  return vertices[index1].isNeighbour(index2);
}

// --------------- Search functions ------------------
template <typename VertexType, typename EdgeType>
bool SimpleGraph<VertexType, EdgeType>::depthFirstSearch(int root, int target) const
{
  // First the initialization
#ifdef GRAPH_DEBUG
	cout << "SIMPLE_GRAPH::search("<< root<<", "<<target<<") --> Start\n";
#endif
  bool visitado[nVertices()];
  for (int i = 0; i < nVertices(); i++) {
    visitado[i] = false;
  }

  return DFS(root,target, visitado);
}

template <typename VertexType, typename EdgeType>
bool SimpleGraph<VertexType, EdgeType>::DFS(int root, int target, bool* visitado) const
{
  visitado[root] = true;
  std::list<int>::const_iterator it;

#ifdef GRAPH_DEBUG
  cout << "SIMPLE_GRAPH::DFS("<< root<<", "<<target<<") --> Start\n";
#endif
	
  if ( root == target ) { 
    // Exit condition
#ifdef GRAPH_DEBUG
    cout << "SIMPLE_GRAPH::DFS() --> Found!!! \n";
#endif
    return true;
  }
	
  // For each neighbour
  for (it = vertices[root].adjacency_list.begin() ; it != vertices[root].adjacency_list.end(); it++) {
    if ( visitado[*it] == false ) {
      // Begin a new search in that node
      if ( DFS( *it, target, visitado) ) {
	return true; // We have found it!!!!!!!!
      }
    }
  }

  return false;
}

template <typename VertexType, typename EdgeType>
int SimpleGraph<VertexType, EdgeType>::nEdges() const
{
  int ret_value = 0;
	
  for (int i = nVertices() - 1; i >= 0; i--) {
    ret_value += vertices[i].nNeighbours();
  }
	
  return ret_value;
}

template <typename VertexType, typename EdgeType>
int SimpleGraph<VertexType, EdgeType>::nVertices() const
{
  return (int)vertices.size();
}





template <typename VertexType, typename EdgeType>
int SimpleGraph<VertexType, EdgeType>::nEdges(int vertex_id) const
{
  return vertices.at(vertex_id).nNeighbours();
}

}

#endif