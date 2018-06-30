#include "SimpleGraph.h"

#include <iostream>

using namespace simple_graph;
using namespace std;

#include <string.h>

int main (int argc, char **argv) {
  bool symmetric = false;
  if (argc > 1) {
    if ( strcmp(argv[1], "symmetric") == 0) {
      cout << "Symmetric test.\n";
      symmetric = true;
    }
  }
  
  SimpleGraph<int, double> graph(symmetric);
  
  // Define the vertices

  graph.addVertex(0.0);
  graph.addVertex(1000);
  graph.addVertex(2000);
  graph.addVertex(-1);

  // Define the edges
  graph.addEdge(0, 1, 1000.0);
  graph.addEdge(0, 2, 1500.0);
  graph.addEdge(0, 3, 1000.0);
  graph.addEdge(1, 3, 2500.0);
  
  cout << graph.toString() << endl;
}