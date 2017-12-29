#include "icp.h"
#include <iostream>
#include <fstream>

using namespace PSolver;
using namespace std;
		     
		     
int main(int, char**){


  //create a point vector;
  Point2fVector points;
  drawCircle(points, Eigen::Vector2f(0,0), 10, 100);
  drawLine(points, Eigen::Vector2f(-10,-10), Eigen::Vector2f(10,10), 100);

  // camera matrix that makes a pixel = 10 cm;
  Eigen::Matrix3f K;
  K << 
    10, 0, 200,
    0, 10, 200,
    0, 0,    1;

  // apply the camera projection
  Point2fVector imagePoints;
  points.multiply(imagePoints, K);
  
  IntImage srcIndices;
  srcIndices.create(400, 400);
  srcIndices = -1;
  
  
  // we scan the reprojected points and put in each pixel the index of the point in the array
  for (size_t i = 0; i< imagePoints.size(); i++){
    int x = imagePoints[i].x();
    int y = imagePoints[i].y();
    srcIndices.at<int>(x,y) = i;
  }

  // we compute the distance map
  DistanceMap dmap;
  IntImage dmapIndices;
  makeDistanceMap(dmap, dmapIndices, srcIndices, 40);
    
  CharImage dmapImage;
  dmap2img(dmapImage, dmap);
  
  Point2fVector tpoints;
  // this is the transform that simulates the points as seen when the robot was in
  // x = 0.2, y=0.1, theta=0.1
  Eigen::Isometry2f myTransform=v2t( Eigen::Vector3f(0.1, 0.1, 0.1)).inverse();
  points.transform(tpoints, myTransform);

  CorrespondenceVector corr;

  findCorrespondences(corr, 
		      dmapImage,
		      tpoints,
		      Eigen::Isometry2f::Identity(),
		      K);
  
  drawCorrespondences(dmapImage,
		      points,
		      tpoints,
		      corr,
		      Eigen::Isometry2f::Identity(),
		      K);
     
  cv::imwrite("/Users/mallonoce/Desktop/SLAM/Ls_slam_things/SLAM/corr.png", dmapImage);

  return 0;
}
