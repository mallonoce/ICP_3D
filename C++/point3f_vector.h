#pragma once
#include "defs.h"

// Primitives to handle the 3D vectors and draw simple figures.

namespace PSolver {
    
    struct Point3fVector: public Vector3fVector {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        void transform(Point3fVector& other, const Eigen::Isometry3f& T) const;
        
    };
    

    void drawLine(Point3fVector& model, const Eigen::Vector3f& from, const Eigen::Vector3f& to, int steps);
    
    void drawCircle(Point3fVector& model, const Eigen::Vector3f& center, float radius, Eigen::Quaternionf q, int steps);
    
    void drawSpiral(Point3fVector& model, const float nbSteps);
        
    
}
