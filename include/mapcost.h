#ifndef MAPCOST_H
#define MAPCOST_H

#include <cmath>

#include "constants.h"
#include "node3d.h"
namespace HybridAStar {

/*!
   \brief This map be created for recording grid parameters 
   To calculate cost_visted,or height,or cost_lateral_slope

*/
class Map {
 public:
  /// The default constructor for 2D array initialization.
  Map(): Map(0, 0, 0) {}
  /// Constructor for a node with the given arguments
  Map(float h, int v, int s) {
    this->h = h;
    this->v = v;
    this->s = s;
  }
  // GETTER METHODS
  /// get the h postion(z-axis)
  int getH() const { return h; }
  /// get cost-visted
  int getV() const { return v; }
  /// get the y position
  int getS() const { return s; }
  /// get visted
  bool isVisted() const { return ( v>0 ?1:0) ; }
  /// determine whether the node is closed

  // SETTER METHODS
  /// set the h postion(z-axis)
  void setH(const float& h) { this->h = h; }
  /// set the y position
  void setV(const int& v) { this->v += v; }
  /// set the cost-to-come (heuristic value)
  void setS(const int& s) { this->s = s; }

  // UPDATE METHODS
  /// Updates the cost-visted
  void updateV() {}; 
  /// Reset the cost-visted of all node
  void clearV(){};  


 private:
  /// the h postion(z-axis)
  float h; 
  /// the cost-visted
  int v;
  /// the cost-slide
  int s;


};
}
#endif