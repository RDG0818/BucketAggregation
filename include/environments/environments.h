#pragma once
#include "node.h"

// NOTE: This is just for defining what the actual environments need to have for static polymorphism.
// NOTE: This class will never actually be used, and should be deleted once the actual environments are implemented.

template<typename StateType> class Environment {

public:

  void get_successors(uint32_t node_index, std::vector<uint32_t>& neighbors);
  uint32_t get_heuristic(unint32_t node_index);
  bool is_goal(uint32_t node_index);

  uint32_t get_start_node();

  NodePool& pool;

  std::vector<StateType> node_states; // node_index -> StateType

};