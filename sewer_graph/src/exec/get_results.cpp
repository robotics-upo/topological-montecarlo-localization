#include "sewer_graph/sewer_graph.h"
#include <iostream>
#include <ros/ros.h>
#include <functions/functions.h>
#include <functions/RealVector.h>

using namespace std;
using namespace sewer_graph;
using functions::RealVector;

int main(int argc, char ** argv) {
  if (argc < 6) {
    cerr << "Usage: " << argv[0] << " <graph_file> <ground_truth file> <base_file> <init> <end>\n";
    return -1;
  }
  string s(argv[1]);
  SewerGraph g(s);
  cout << g.toString_2() << endl; 

  string s2(argv[2]);
  vector<double> ground = functions::getVectorFromFile(s2);
  
  vector<vector<double> > mat, mean_mat, dev_mat;
  vector<double> empty;
  
  for (int j = 0; j < ground.size(); j++) {
    mean_mat.push_back(empty);
    dev_mat.push_back(empty);
  }
  
  
  cout << "One on one distances:\n";
  
  for (int i = atoi(argv[4]); i <= atoi(argv[5]); i++) {
    
    ostringstream os;
    os << argv[3] << i << ".txt";
    cout << "Stat file: " << os.str() << endl << endl;
    functions::getMatrixFromFile(os.str(), mat);
    RealVector v(2);
    RealVector manhole(2);
    
    for (int j = 0; j < ground.size(); j++) {
      
      manhole[0] = g.getVertexContent(ground[j]).x;
      manhole[1] = g.getVertexContent(ground[j]).y;
      v[0] = mat[j][0];
      v[1] = mat[j][1];
      
      mean_mat[j].push_back(manhole.distance(v));
      dev_mat[j].push_back(sqrt(mat[j][3]*mat[j][3] + mat[j][4]*mat[j][4]));
      
      cout << manhole.distance(v) << "\t" << dev_mat[j][dev_mat[j].size() - 1] << "\t" << mat[j][3] << "\t" << mat[j][4] << endl;
      
    }
  }
  
  cout << "\n\n -------------------- Mean results --------------------\n";
  
  for (int i = 0; i < mean_mat.size();i++) {
    cout << functions::printVector(mean_mat[i]) << endl;
//     cout << "Mean and std_dev o the devs near the manhole " << ground[i] << ":" << functions::mean(dev_mat[i]) << "\t" << functions::std_dev(dev_mat[i]) << endl;
  }
  
  cout << "\n\n -------------------- Dev results --------------------\n";
  
  for (int i = 0; i < mean_mat.size();i++) {
//     cout << "Mean and std_dev distance to manhole " << ground[i] << ":" << functions::mean(mean_mat[i]) << "\t" << functions::std_dev(mean_mat[i]) << endl;
    cout << ground[i] << ":" << functions::mean(dev_mat[i]) << "\t" << functions::std_dev(dev_mat[i]) << endl;
  }
  
  return 0;
}