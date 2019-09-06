#ifndef NODE_H
#define NODE_H
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>

class Node
{
private:
  grid_map::Index idx;
  grid_map::Position pos;
public:
  Node(grid_map::Index _idx, grid_map::Position _pos){
    idx=_idx;
    pos=_pos;
    hCost=0;
    gCost=INT_MAX;
    parent=NULL;
    heapIdx=-1;
  }
  //~Node();
  Node *parent;
  double hCost;
  double gCost;
  int heapIdx;
  int getX() const{ return idx(0);}
  int getY() const{ return idx(1);}
  double fCost(){ return hCost+gCost;}
  grid_map::Position getPos(){return pos;}
  grid_map::Index getIdx(){return idx;}
  bool compare(Node *node)
  { return ((fCost()==node->fCost())?(hCost<node->hCost):(fCost()<node->fCost()));}
  bool operator==(const Node& n) const
  {return getX()==n.getX()&&getY()==n.getY();}
};
namespace std
{
  template<> struct hash<Node*>
  {
    
    size_t operator()(Node* const& node) const noexcept
    {
      size_t const h1 ( std::hash<int>()(node->getX()));
      size_t const h2 ( std::hash<int>()(node->getY()) );
      return (53+h1)*53+h2; 
    }
  };
  template<> struct equal_to<Node*>
  {
    
    bool operator()(Node* const& lhs,Node* const& rhs) const noexcept
    {
      return lhs->getX()==rhs->getX()&&lhs->getY()==rhs->getY();
    }
  };
}
#endif
