#include "binary_heap.h"



void BinaryHeap::push(uint32_t handle, uint32_t priority) {
  heap_.push_back({priority, handle});
  int current_idx = heap_.size() - 1;  

  pool_[handle].queue_ref = current_idx;

  while (current_idx > 0) {
    int parent_idx = (current_idx - 1) / 2;
    if (heap_[current_idx].priority < heap_[parent_idx].priority) {
      std::swap(heap_[current_idx], heap_[parent_idx]);

      pool_[heap_[current_idx].handle].queue_ref = current_idx;
      pool_[heap_[parent_idx].handle].queue_ref = parent_idx;
            
      current_idx = parent_idx; 

    }
    else {
      break;
    }
  }
}

uint32_t BinaryHeap::pop() {
  uint32_t best_handle = heap_[0].handle;
  pool_[best_handle].queue_ref = NODE_NULL; // maybe change sentinel value
  HeapItem last_item = heap_.back();
  heap_.pop_back();

  if (heap_.empty()) {
    return best_handle;
  }

  heap_[0] = last_item;

  pool_[last_item.handle].queue_ref = 0;

  uint32_t idx = 0;
  uint32_t size = heap_.size();

  while (true) {
    uint32_t left_child = 2 * idx + 1;
    uint32_t right_child = 2 * idx + 2;
    uint32_t smallest = idx;

    if (left_child < size && heap_[left_child].priority < heap_[smallest].priority) {
      smallest = left_child;
    }

    if (right_child < size && heap_[right_child].priority < heap_[smallest].priority) {
      smallest = right_child;
    }

    if (smallest == idx) {
      break;
    }

    std::swap(heap_[idx], heap_[smallest]);

    pool_[heap_[idx].handle].queue_ref = idx;
    pool_[heap_[smallest].handle].queue_ref = smallest;

    idx = smallest;
  }

  return best_handle;
}

void BinaryHeap::decrease_key(uint32_t handle, uint32_t new_priority) {
  int current_idx = pool_[handle].queue_ref;
  heap_[current_idx].priority = new_priority;

  while (current_idx > 0) {
    int parent_idx = (current_idx - 1) / 2;
    if (heap_[current_idx].priority >= heap_[parent_idx].priority) { break; }

    std::swap(heap_[current_idx], heap_[parent_idx]);
    pool_[heap_[current_idx].handle].queue_ref = current_idx;
    pool_[heap_[parent_idx].handle].queue_ref = parent_idx;

    current_idx = parent_idx;
  }
}

bool BinaryHeap::contains(uint32_t handle) const {
    return pool_[handle].queue_ref != NODE_NULL; 
}