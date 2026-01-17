// include/environments/node.h

#pragma once

#include <cstdint>
#include <limits>
#include <vector>

static constexpr uint32_t NODE_NULL = std::numeric_limits<uint32_t>::max();

struct Node { // Fits in a cache line
  uint32_t parent;
  uint32_t g;
  uint32_t h; 
  uint32_t queue_ref; // Can store the heap index, next node index, or a sentinel value
};

class NodePool {

public:

  void reserve(uint32_t capacity) { // preallocate memory
    nodes.reserve(capacity);
  }

  void clear() {
    nodes.clear(); 
  }

  uint32_t allocate(uint32_t parent, uint32_t g, uint32_t h) {
    Node n = {parent, g, h, NODE_NULL};
    nodes.push_back(n);
    return nodes.size() - 1;
  }

  Node& operator[](uint32_t h) { return nodes[h]; }; // Write access
  const Node& operator[](uint32_t h) const {return nodes[h]; } // Read access
  
  uint32_t size() const { return nodes.size(); }

private:

  std::vector<Node> nodes; // Nodes are stored contiguously

};

