#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <Eigen/Dense>
#include <unordered_set>
//#include <algorithm>
#include "astar.hpp"
#include "Node.hpp"
#include "heap.hpp"
#include <ros/ros.h>
astar::astar(grid_map::GridMap _map):data( _map["traversability"]){
  map=_map;
  sizeX=map.getSize()(0);
  sizeY=map.getSize()(1);
  
}
astar::~astar(){
  for(const auto& p: grid){
    delete p.second;
  }
  grid.clear();
}

void astar::reset(){
  for(const auto& p: grid){
    delete p.second;
  }
  grid.clear();
}

bool astar::makePlan(grid_map::Position startPos, grid_map::Position endPos, ros::Publisher pub){
  if(!map.isInside(startPos)||!map.isInside(endPos))
    return false;
  
  grid_map::Index startIdx,endIdx;
  map.getIndex(startPos,startIdx);
  map.getIndex(endPos,endIdx);
  start=new Node(startIdx,startPos);
  end= new Node(endIdx,endPos);
  start->gCost=0;
  start->hCost=getDist(start,end);
  end->hCost=0;  
  grid.insert({startIdx,start});
  grid.insert({endIdx,end});
  heap openSet(sizeX*sizeY);
  std::unordered_set<Node*> closedSet;
  openSet.add(start);
  openSet.add(end);
  map.clear("A*");
  while(!openSet.empty()){
    Node* current = openSet.pop();
    closedSet.insert(current);

    if ((*current)==(*end))
      return true;

    std::vector<Node*> neighbours=getNeighbours(current);
    for(Node* node:neighbours){
      if(closedSet.count(node)){
     	continue;
      }    
      double gCost=current->gCost+getDist(current,node);
      if(node->gCost<=gCost){
	continue;
      }
      map.at("A*", node->getIdx())=gCost;
      node->gCost=gCost;
      node->parent=current;
      if(!openSet.contains(node)){
	openSet.add(node);
      }
      else{
	openSet.update(node);
      }
    }
  }
  ROS_ERROR("No path");
  return false;
}

double astar::getDist(Node* start, Node* end){
  return (start->getPos()-end->getPos()).norm();
}

std::vector<Node*> astar::getNeighbours(Node* node){
  std::vector<Node*> neighbours;
  for(int x=-1; x<=1; x++){
    for(int y=-1; y<=1; y++){
      if(!(x==0^y==0))
	//if(x==0&&y==0)
	continue;
      int checkX=x+node->getX();
      int checkY=y+node->getY();
      if(checkX>=0&&checkX<sizeX&&checkY>=0&&checkY<sizeY){
	grid_map::Index idx(checkX,checkY);
	if(map.at("traversability",idx))
	  continue;
	if(!grid.count(idx)){
	  grid_map::Position pos;
	  map.getPosition(idx,pos);
	  Node *neighbour=new Node(idx,pos);
	  neighbour->hCost=getDist(node,end);
	  grid[idx]=neighbour;
	}
	neighbours.push_back(grid[idx]);
      }
    }
  }
  return neighbours;
}

std::vector<grid_map::Position> astar::getVisits(){
  std::vector<grid_map::Position> plan;
  for(const auto& p: grid){

    plan.push_back(p.second->getPos());
  }
  return plan;
}

std::vector<grid_map::Index> astar::getPlan(){
  std::vector<grid_map::Index> plan;
  Node* prev=end;
  while(prev!=NULL){

    plan.push_back(prev->getIdx());
    prev=prev->parent;
  }
  return plan;
}
