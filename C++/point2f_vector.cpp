#include "point2f_vector.h"

namespace PSolver {

  using namespace std;

  void Point2fVector::transformInPlace(const Eigen::Isometry2f& T){
    Eigen::Matrix2f R=T.linear();
    for (iterator it=begin(); it!=end(); it++){
      *it = T*(*it);
    }
  }
  
  void Point2fVector::transform(Point2fVector& other, const Eigen::Isometry2f& T) const {
    other.resize(size());
    iterator oit=other.begin();
    for (const_iterator it = begin(); it!=end(); it++, oit++){
      *oit = T*(*it);
    }
  }

  void Point2fVector::multiply(Point2fVector& other, const Eigen::Matrix3f& K) const {
    other.resize(size());
    iterator oit=other.begin();
    const Eigen::Matrix2f linear = K.block<2,2>(0,0);
    const Eigen::Vector2f affine = K.block<2,1>(0,2);
    for (const_iterator it = begin(); it!=end(); it++, oit++){
      *oit = linear*(*it) + affine;
    }
  }


  void drawLine(Point2fVector& model, const Eigen::Vector2f& from, const Eigen::Vector2f& to, int steps){
    Eigen::Vector2f dp = to-from;
    float norm = dp.norm();
    Eigen::Vector2f t = dp.normalized();
    Eigen::Vector2f n;
    n<< t.y(), -t.x();
    
    Eigen::Vector2f dt = t*(norm/steps);
    for (int i=0; i<steps+1; i++) {
      Eigen::Vector2f p=from+dt*(float)i;
      model.push_back(p);
    }
  }

  void drawCircle(Point2fVector& model, const Eigen::Vector2f& center, float r, int steps){
    Eigen::Vector2f previousPoint;
    float da=(2*M_PI)/steps;
    float a = 0;
    for (size_t i=0; i<steps; i++){
      Eigen::Vector2f p(r*cos(a)+center.x(), r*sin(a)+center.y());
      if (i>0) {
	drawLine(model, previousPoint, p, 1);
      }
      previousPoint = p;
      a+=da;
    }
  } 
}
