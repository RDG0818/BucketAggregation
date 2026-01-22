#include "environments/node.h"

class BinaryHeap { // Stores node handles

public:

  BinaryHeap(NodePool& p) : pool_(p) {}
  void push(uint32_t handle, uint32_t priority);
  uint32_t pop(); // returns node handle
  void decrease_key(uint32_t handle, uint32_t new_priority);
  bool contains(uint32_t handle) const;
  bool empty() { return heap_.empty(); };

private:

  NodePool& pool_;

  struct HeapItem {
        uint32_t priority;
        uint32_t handle;
  };
  
  std::vector<HeapItem> heap_;
};