#include "distance_map.h"
#include "point2f_vector.h"
#include <iostream>
#include <fstream>
#include "icp.h"

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
  
  cv::imwrite("dmap.png", dmapImage);


  return 0;
}
