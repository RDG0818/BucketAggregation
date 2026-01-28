#pragma once

#include "environments/node.h"
#include "binary_heap.h"


class BucketHeap {

public:

  BucketHeap(NodePool& p) : pool_(p) {};

  void push(uint32_t handle);

  uint32_t pop();

  void decrease_key(uint32_t handle);

  void rebuild();

  bool contains(uint32_t handle) const;

  bool empty() const;

private:


  NodePool& pool_; 

}; 