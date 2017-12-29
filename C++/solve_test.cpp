#include <iostream>
#include <fstream>
#include "solve.h"

using namespace PSolver;
using namespace std;

int main(int argc, char** argv){
  bool use_manifold = false;
  if (argc>1) {
    if (! strcmp(argv[1], "-m")){
      cerr << "using manifolds" << endl;
      use_manifold  = true;
    } 
  } else 
    cerr << "using standard solver" << endl;

  //create a point vector;
  Point2fVector points;
  drawCircle(points, Eigen::Vector2f(0,0), 10, 100);
  drawLine(points, Eigen::Vector2f(-10,-10), Eigen::Vector2f(10,10), 100);

  // transform the other vector
  Point2fVector tpoints;
  // this is the transform that simulates the points as seen when the robot was in
  // x = 0.2, y=0.1, theta=0.1
  Eigen::Isometry2f myTransform=v2t( Eigen::Vector3f(0.1, 0.1, 0.1)).inverse();
  points.transform(tpoints, myTransform);

  // create a perfect correspondence set
  CorrespondenceVector corr(tpoints.size());
  for (size_t i = 0; i<tpoints.size(); i++){
    corr[i] = std::make_pair(i,i);
  }

  Eigen::Isometry2f T;
  T.setIdentity();
  
  for(int i=0; i<100; i++){
    float e = solve(T, points, tpoints, corr, use_manifold); 
    cerr << "iteration: " <<i << " error: " << e << " transform: "<< t2v(T).transpose() << endl;
  }

  return 0;
}
