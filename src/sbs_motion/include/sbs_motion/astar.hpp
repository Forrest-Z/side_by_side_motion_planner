#ifndef ASTAR_H
#define ASTAR_H
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include "Node.hpp"
#include <ros/ros.h>
namespace std
{
  template<> struct hash<grid_map::Index>
  {
    
    size_t operator()(grid_map::Index const& idx) const noexcept
    {
      size_t const h1 ( std::hash<int>()(idx(0)));
      size_t const h2 ( std::hash<int>()(idx(1)) );
      return (53+h1)*53+h2; 
    }
  };
  template<> struct equal_to<grid_map::Index>
  {
    
    bool operator()(grid_map::Index const& lhs,grid_map::Index const& rhs) const noexcept
    {
      return lhs(0)==rhs(0)&&lhs(1)==rhs(1);
    }
  };
}
class astar
{
public:
  astar(grid_map::GridMap map);
  ~astar();
  bool makePlan(grid_map::Position startPos, grid_map::Position endPos, ros::Publisher pub);
  std::vector<grid_map::Position> getVisits();
  std::vector<grid_map::Index> getPlan();
  void reset();
private:
  grid_map::GridMap map;
  Node *start,*end;
  grid_map::Matrix& data;
  std::unordered_map<grid_map::Index,Node*> grid;
  int sizeX, sizeY;
  std::vector<Node*> getNeighbours(Node *node);
  double getDist(Node *start, Node *end);
};

#endif
