#ifndef TRAVER_H
#define TRAVER_H
#include <grid_map_core/GridMap.hpp>
#include <Eigen/Dense>
#include <unordered_set>
#include <queue>

static const grid_map::Index sides[]={grid_map::Index(-1,0),grid_map::Index(1,0),grid_map::Index(0,-1),grid_map::Index(0,1)};

template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

inline std::vector<grid_map::Index> findTraver(grid_map::GridMap map, grid_map::Position center){
  grid_map::Matrix& data = map["traversability"];
  std::unordered_set<Eigen::Vector2i,matrix_hash<Eigen::Vector2i>> visited;
  std::queue<grid_map::Index> queue;
  grid_map::Index start;
  map.getIndex(center,start);
  queue.push(start);
  visited.insert(start);
  while(!queue.empty()){
    grid_map::Index pos=queue.front();
    queue.pop();
    if(visited.count(pos))
      continue;
    visited.insert(pos);
    grid_map::Index e=queue.front();
    grid_map::Index w=queue.front();
    while(data(w(0),w(1))==0){
      w+=sides[0];
      visited.insert(w);
      grid_map::Index up=w+sides[3];
      if(data(up(0),up(1))==0&&!visited.count(up)){
	queue.push(up);
      }
      grid_map::Index down=w+sides[2];
      if(data(down(0),down(1))==0&&!visited.count(down)){
	queue.push(down);
      }
    }      
    while(data(e(0),e(1))==0){
      e+=sides[0];
      visited.insert(e);
      grid_map::Index up=e+sides[3];
      if(data(up(0),up(1))==0&&!visited.count(up)){
	queue.push(up);
      }
      grid_map::Index down=e+sides[2];
      if(data(down(0),down(1))==0&&!visited.count(down)){
	queue.push(down);
      }
    }
  }
  return std::vector<grid_map::Index>( visited.begin(), visited.end() );
}

#endif
