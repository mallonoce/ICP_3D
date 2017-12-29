#pragma once
#include "point2f_vector.h"
#include "point3f_vector.h"
#include "find_correspondences.h"
#include "solve.h"
#include "defs.h"

namespace PSolver {

    
  // Use this struct to handle the pointclouds in simpleViewer
    struct container{
        int counter;
        Point3fVector points , tpoints , drawPoints;
        Vector6fVector vector;
        Eigen::Isometry3f T, Tguess;
        int it=100;
        std::vector<float, Eigen::aligned_allocator<float> > errorVector;

    };
    
    
    
  // does icp
  // returns the error
  float icp(Eigen::Isometry3f& initialT,  // initial transform (will be modified)
	    const Point3fVector& rpts, // points that don't move
	    const Point3fVector& cpts, // points that move
        Vector6fVector& Tvector, // vector of all the transformations
        std::vector<float, Eigen::aligned_allocator<float> >& errorVector,
	    int it = 10 // how many iterations to do
        );
    
}
