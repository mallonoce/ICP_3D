#include "point3f_vector.h"

namespace PSolver {
    
    using namespace std;
    

    
    void Point3fVector::transform(Point3fVector& dest, const Eigen::Isometry3f& T) const {
        dest.resize(size());
        iterator oit=dest.begin();
        const Eigen::Matrix3f linear = T.rotation();
        const Eigen::Vector3f affine = T.translation();        
        for (const_iterator it = begin(); it!=end(); it++, oit++){
            *oit = linear*(*it) + affine;


        }
    }
    

    void drawLine(Point3fVector& model, const Eigen::Vector3f& from, const Eigen::Vector3f& to, int steps){
        Eigen::Vector3f dp = to-from;
        float norm = dp.norm();
        Eigen::Vector3f t = dp.normalized();
        
        Eigen::Vector3f dt = t*(norm/steps);
        for (int i=0; i<steps+1; i++) {
            Eigen::Vector3f p=from+dt*(float)i;
            model.push_back(p);
        }
    }
    
    
    void drawCircle(Point3fVector& model, const Eigen::Vector3f& center, float r, Eigen::Quaternionf q, int steps){
        float da=(2*M_PI)/steps;
        float a = 0;
        for (size_t i=0; i<steps; i++){
            Eigen::Vector3f p(r*cos(a)+center.x(), r*sin(a)+center.y(), center.z());
            model.push_back(p);
            a+=da;
        }
        Eigen::Isometry3f T;
        T.setIdentity();
        T.linear() = q.toRotationMatrix();
        model.transform(model, T);
    } 

    
    void drawSpiral(Point3fVector& model, const float nbSteps){
        
        for (int i=0; i<nbSteps; ++i)
        {
            
            const float ratio = i/nbSteps;
            const float angle = 21.0*ratio;
            const float c = cos(angle);
            const float s = sin(angle);
            const float r1 = 10.0 - 8*ratio;
            const float r2 = 8 - 8*ratio;
            const float alt = ratio*10 - 5;

            Eigen::Vector3f p1(r1*c, alt, r1*s);
            Eigen::Vector3f p2(r2*c, alt+0.05f, r2*s);
            model.push_back(p1);
            model.push_back(p2);
        }
    }


    
}
