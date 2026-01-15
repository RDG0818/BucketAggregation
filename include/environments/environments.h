#pragma once
#include "node.h"

template<typename StateType> class GridEnvironment { 
// template may not be necessary for static polymorphism and inlining
public:

  void get_successors(uint32_t node_index, std::vector<uint32_t>& neighbors);
  uint32_t get_heuristic(unint32_t node_index);
  bool is_goal(uint32_t node_index);

  uint32_t get_start_node();

  NodePool& pool;

  std::vector<StateType> node_states; // node_index -> StateType

};

template<typename StateType> class PancakeEnvironment {

public:

  void get_successors(uint32_t node_index, std::vector<uint32_t>& neighbors);
  uint32_t get_heuristic(unint32_t node_index);
  bool is_goal(uint32_t node_index);

  uint32_t get_start_node();

  NodePool& pool;

  std::vector<StateType> node_states; // node_index -> StateType

};

template<typename StateType> class SlidingTileEnvironment {

public:

  void get_successors(uint32_t node_index, std::vector<uint32_t>& neighbors);
  uint32_t get_heuristic(unint32_t node_index);
  bool is_goal(uint32_t node_index);

  uint32_t get_start_node();

  NodePool& pool;

  std::vector<StateType> node_states; // node_index -> StateType

};
