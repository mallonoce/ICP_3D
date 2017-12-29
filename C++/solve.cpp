#include <iostream>
#include <fstream>
#include <Eigen/Cholesky>
#include "solve.h"

namespace PSolver {
  using namespace std;

  void errorAndJacobian(Eigen::Vector3f& e,
			Matrix3_6f& J,
			const Eigen::Vector3f& pr,
			const Eigen::Vector3f& pc,
			const Eigen::Isometry3f& T){
      // Move the point
      Eigen::Vector3f Tpoint = T.rotation()*pc+T.translation();
      // compute the error
      e = pr - Tpoint;
      // compute the Jacobian
      J.block<3,3>(0,0) = -Eigen::Matrix3f::Identity();
      J.block<3,3>(0,3) = skew(Tpoint);// * (-2);
      
  }

  
  float buildLinearSystem(Matrix6f& H, Vector6f& b,
			  const Point3fVector& rpts,
			  const Point3fVector& cpts,
			  const CorrespondenceVector& corr,
			  const Eigen::Isometry3f& T){
      
      H.setZero();
      b.setZero();
      float error = 0;
    
      for (size_t i=0; i<corr.size(); i++){
          
          // use the correlations found with FLANN
          int ridx = corr[i].first;
          int cidx = corr[i].second;
          const Eigen::Vector3f& pr = rpts[ridx];
          const Eigen::Vector3f& pc = cpts[cidx];

        //No data association case
        //const Eigen::Vector3f& pr = rpts[i];
        //const Eigen::Vector3f& pc = cpts[i];
        
          Matrix3_6f J;
          Eigen::Vector3f e;
          errorAndJacobian(e,J,pr,pc,T);
        
          // Compute the error H and b
          error += e.transpose()*e;
          H+=J.transpose()*J;
          b+=J.transpose()*e;
    }
      // returns the error
      return error;
  }
  

 
  // returns the error before applying the solution
  float solve(Eigen::Isometry3f& T,
	      const Point3fVector& rpts,
	      const Point3fVector& cpts,
	      const CorrespondenceVector& corr){
      
      // define H and b
      Matrix6f H;
      Vector6f b;
      // compute all the elements of the system
      float error = buildLinearSystem(H, b, rpts, cpts,corr, T);
      // damp the H matrix if needed
      float damping = 1;
      H += damping*Matrix6f::Identity();
    
      // manifold case
      Vector6f dt = H.ldlt().solve(-b);
      Vector6f t = t2v(T);
      t+=dt;
      T = v2t(t);
  
      
      //Vector6f dt = H.ldlt().solve(-b);
      //Eigen::Isometry3f dT = v2t(dt);
      //T = dT*T;
   
      return error;
  }

    
    
}
 