#pragma once
#include "distance_map.h"
#include "point3f_vector.h"
#include <flann/flann.hpp>

// Find the correspondences between points using FLANN

namespace PSolver {
  typedef std::pair<int, int> Correspondence;
  typedef std::vector<Correspondence> CorrespondenceVector;

  void findCorrespondences(const Eigen::Isometry3f& T,
                           const Point3fVector& rpts,
                           const Point3fVector& spts,
                           CorrespondenceVector& corr);

}

