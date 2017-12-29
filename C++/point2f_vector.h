#pragma once
#include "defs.h"

namespace PSolver {

  struct Point2fVector: public Vector2fVector {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    void transformInPlace(const Eigen::Isometry2f& T);
    void transform(Point2fVector& other, const Eigen::Isometry2f& T) const;
    void multiply(Point2fVector& other, const Eigen::Matrix3f& K) const;
  };


  void drawLine(Point2fVector& model, const Eigen::Vector2f& from, const Eigen::Vector2f& to, int steps);

  void drawCircle(Point2fVector& model, const Eigen::Vector2f& center, float radius, int steps); 

}
