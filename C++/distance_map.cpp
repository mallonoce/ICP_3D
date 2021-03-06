#include "distance_map.h"
#include <limits>
#include <deque>
#include <queue>
#include <set>
#include <vector>

namespace PSolver {
  using namespace std;

  struct MapCellComparator{
    inline bool operator()(const MapCell* m1, const MapCell* m2) const{
      return m1->distance>m2->distance || 
	(m1->distance==m2->distance && m1->r>m2->r) ||
	(m1->distance==m2->distance && m1->r==m2->r && m1->c>m2->c) ;
      return false;
    }
  };

  inline int findNeighbors(MapCell** neighbors, DistanceMap& dmap, MapCell* m){
    int r = m->r;
    int c = m->c;
    int rmin = r-1<0?0:r-1;
    int rmax = r+1>dmap.rows()-1?dmap.rows()-1:r+1;
    int cmin = c-1<0?0:c-1;
    int cmax = c+1>dmap.cols()-1?dmap.cols()-1:c+1;
    int k=0;
    for (int rr =rmin; rr<=rmax; rr++)
      for (int cc =cmin; cc<=cmax; cc++) {
    	if (rr!=r || cc!=c) 
    	  neighbors[k++]=&dmap(rr,cc);
      }

    return k;
  }

  struct QEntry{
    QEntry(MapCell* c=0, int d=std::numeric_limits<int>::max()) {
      cell = c;
      distance = d;
    }
    
    inline bool operator < (const QEntry& e) const {
      return e.distance < distance ;
    }

    int distance;
    MapCell* cell;

  };

  struct MapCellQueue : public std::priority_queue<QEntry> {
    typedef typename std::priority_queue<QEntry>::size_type size_type;
    MapCellQueue(size_type capacity = 0) { reserve(capacity); };
    void reserve(size_type capacity) { this->c.reserve(capacity); } 
    size_type capacity() const { return this->c.capacity(); } 
    MapCell* top() { return std::priority_queue<QEntry>::top().cell;}
    void push(MapCell* c) { return std::priority_queue<QEntry>::push(QEntry(c, c->distance));}
  };


  void makeDistanceMap(DistanceMap& dmap, IntImage& imap, const IntImage& indexImage, float maxDistance, const FloatImage& weights){
    // cerr << "A";
    //double t0 = getTime();
    maxDistance *=maxDistance;
    int rows = indexImage.rows;
    int cols = indexImage.cols;
    dmap.resize(rows,cols);
    imap.create(rows,cols);
    imap = -1;
    MapCellQueue q(rows*cols);
    //q.reserve(rows*cols);
    for (int r=0; r<indexImage.rows; r++)
      for (int c=0; c<indexImage.cols; c++){
	MapCell& cell = dmap(r,c);
	cell.r = r;
	cell.c = c;
	int w2 = 1;
	if (weights.rows && weights.cols) {
	  w2=weights.at<float>(r,c);
	}
	cell.weight = w2;
	cell.parent = 0;
	cell.distance = maxDistance;
	int idx = indexImage.at<int>(r,c);
	if (idx>-1){
	  cell.parent = &cell;
	  cell.distance = 0;
	  q.push(&cell);
	  imap.at<int>(r,c)=idx;
	}
      }
    MapCell * neighbors[8];
    int operations = 0;
    size_t maxQSize = q.size();
    // cerr << "startq: "  << maxQSize << endl;
    int currentDistance = 0;
    while (! q.empty()){
      MapCell* current = q.top();
      MapCell* parent = current->parent;
      int parentIndex = imap.at<int>(parent->r, parent->c);
      q.pop();
      if (current->distance<currentDistance)
	continue;
      currentDistance = current->distance;
      // // cerr << "current: " << current->r << " "  << current->c << " " << current->distance << " "
      //  	 << "parent: " << parent->r << " "  << parent->c << endl;
      int k = findNeighbors(neighbors, dmap, current);
      // // cerr << "neighbors: " << k<< endl;

      for (int i=0; i<k; i++){
	MapCell* children=  neighbors[i];
	int r = children->r;
	int c = children->c;
	int w = parent->weight;
	int dr = r-parent->r;
	int dc = c-parent->c;
	int d=w*(dr*dr+dc*dc);
	// // cerr << "children: " << children->r << " "  << children->c << " " << children->distance <<  " " << d << endl;
	operations++;
	if (d<maxDistance && (
			      ! children->parent && children->distance>d)) {
	  children->parent = parent;
	  imap.at<int>(r,c) = parentIndex;
	  children->distance = d;
	  q.push(children);
	}
      }
      maxQSize = maxQSize < q.size() ? q.size() : maxQSize;
    }
    //double t1 = getTime();
    //cerr << "# operations: " << operations << " maxQ: " << maxQSize << " time: " << t1-t0 << endl;
  }

  void dmap2img(CharImage& img, const DistanceMap& dmap){
    img.create(dmap.rows(), dmap.cols());
  
    float mdist = 0;
    for (int r=0; r<dmap.rows(); r++)
      for (int c=0; c<dmap.cols(); c++){
	const MapCell& cell = dmap(r,c);
	if (cell.distance == std::numeric_limits<int>::max())
	  continue;
	mdist = (mdist < cell.distance) ?  cell.distance : mdist;
      }
    mdist = sqrt(mdist);
    // cerr << "mdist=" << mdist;
    for (int r=0; r<dmap.rows(); r++)
      for (int c=0; c<dmap.cols(); c++){
	const MapCell& cell = dmap(r,c);
	float ndist = 127 * sqrt(cell.distance)/mdist;
	int v = 127-ndist;
	img.at<unsigned char>(r,c) = (unsigned char) v;
      }
  }


}
