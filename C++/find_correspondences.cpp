#include "find_correspondences.h"
#include <iostream>
#include <fstream>

namespace PSolver {
  using namespace std;

    void findCorrespondences(const Eigen::Isometry3f& T,
                             const Point3fVector& rpts,
                             const Point3fVector& spts,
                             CorrespondenceVector& corr) {

  
    //number of nearest neighbors to find
    int nn = 1;
        
    // do an one shot transformation to move the points and evaluate the distances
    Point3fVector points;
    spts.transform(points, T);
    
    // translate our matrices in a FLANN compatible structure
    flann::Matrix<float> dataset(new float[rpts.size()*3], rpts.size(),  3);
    for (int i = 0; i  < rpts.size(); i++) {
        for (int j = 0; j < 3; j++) {
            dataset[i][j] = rpts.at(i)(j);
        }
    }
    
    flann::Matrix<float> query(new float[points.size()*3], points.size(),  3);
    for (int i = 0; i  < points.size(); i++) {
        for (int j = 0; j < 3; j++) {
            query[i][j] = points.at(i)(j);
        }
    }

    // allocate the space for the results
    flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
    flann::Matrix<float> dists(new float[query.rows*nn], query.rows, nn);
    
    // construct an randomized kd-tree index using 4 kd-trees
    flann::Index<flann::L2<float> > index(query, flann::KDTreeIndexParams(4));
    index.buildIndex();
        
    // do a knn search, using 128 checks
    index.knnSearch(dataset, indices, dists, nn, flann::SearchParams(128));
    corr.resize(spts.size());

    // associate the results
    for (size_t i = 0; i<spts.size(); i++) {
        corr[i]=std::make_pair(indices[i][0],i);
    }

    //deallocate the structures
    delete[] dataset.ptr();
    delete[] query.ptr();
    delete[] indices.ptr();
    delete[] dists.ptr();
    
    }
    

}
