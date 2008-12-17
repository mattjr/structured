#include "quadtree.hpp"
#include <ostream>
 class terrain_data{
public:
  double z;
friend  std::ostream& operator << (std::ostream& os, const terrain_data &data)
  {
    return os<<data.z<<std::endl;
  }

};

typedef quadtree<terrain_data> terrain_tree;
typedef quadtree_node<terrain_data> terrain_node;
