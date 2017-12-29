#include "icp.h"
#include <iostream>
#include <fstream>

namespace PSolver {
  using namespace std;

    
  // does icp
  // returns the error
  float icp(Eigen::Isometry3f& initialT,
	    const Point3fVector& rpts,
	    const Point3fVector& cpts,
        Vector6fVector& Tvector,
        std::vector<float, Eigen::aligned_allocator<float> >& errorVector,
        int it) {

      // define the correspondence vector
      CorrespondenceVector corr;
      
      for(int i = 0; i<it; i++) {
      
          // do FLANN, find the correspondences
          findCorrespondences(initialT, rpts, cpts, corr);
          
          // solve the linear system
          float e = solve(initialT, rpts, cpts, corr);
          
          Vector6f temp = t2v(initialT);
         
          Tvector.push_back(temp);
          errorVector.push_back(e);
          
    }
    
      return 1;
  }

 }
