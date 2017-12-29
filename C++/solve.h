#pragma once
#include "find_correspondences.h"


namespace PSolver {

  // constructs and solves the linear systems
  // returns the error
  float solve(Eigen::Isometry3f& T,  // initial transform (will be modified)
	      const Point3fVector& rpts, // points that don't move
	      const Point3fVector& cpts, // points that move
	      const CorrespondenceVector& corr // correspondences (pairs, first: fixed, second moving)
	      ); // use or not the local increments
}
