#include "environments/environments.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

DimacsEnvironment::DimacsEnvironment(const std::string& co_file, const std::string& gr_file, uint32_t start_node, uint32_t goal_node, uint32_t capacity)
  : start_id_(start_node), goal_id_(goal_node) {
  
  uint32_t num_nodes = 0;

  // Read .co file for coordinates
  std::ifstream co_in(co_file);
  if (!co_in.is_open()) {
    throw std::runtime_error("Could not open .co file: " + co_file);
  }

  std::string line;
  while (std::getline(co_in, line)) {
    if (line.empty() || line[0] == 'c') continue;
    
    std::stringstream ss(line);
    std::string type;
    ss >> type;
    
    if (type == "p") {
      std::string aux, sp, co;
      ss >> aux >> sp >> co >> num_nodes;
      coords_.resize(num_nodes + 1);
      adjacency_list_.resize(num_nodes + 1);
    } else if (type == "v") {
      uint32_t node_id;
      double x, y;
      ss >> node_id >> x >> y;
      if (node_id <= num_nodes) {
        coords_[node_id] = {x, y};
      }
    }
  }
  co_in.close();

  // Read .gr file for edges
  std::ifstream gr_in(gr_file);
  if (!gr_in.is_open()) {
    throw std::runtime_error("Could not open .gr file: " + gr_file);
  }

  while (std::getline(gr_in, line)) {
    if (line.empty() || line[0] == 'c') continue;
    
    std::stringstream ss(line);
    std::string type;
    ss >> type;
    
    if (type == "p") {
      std::string sp;
      uint32_t nodes, edges;
      ss >> sp >> nodes >> edges;
      if (nodes > num_nodes) {
        num_nodes = nodes;
        coords_.resize(num_nodes + 1);
        adjacency_list_.resize(num_nodes + 1);
      }
    } else if (type == "a") {
      uint32_t u, v;
      uint32_t c;
      ss >> u >> v >> c;
      if (u <= num_nodes && v <= num_nodes) {
        uint32_t cost = c;
        adjacency_list_[u].push_back({v, cost});
      }
    }
  }
  gr_in.close();

  pool_.reserve(capacity);
  pool_.resize_state_space(num_nodes + 1);
}

void DimacsEnvironment::get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors) {
  neighbors.clear();
  if (u_id < adjacency_list_.size()) {
    for (const auto& edge : adjacency_list_[u_id]) {
      neighbors.push_back(edge.to);
    }
  }
}
