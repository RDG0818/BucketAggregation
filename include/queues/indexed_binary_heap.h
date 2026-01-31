// include/queues/indexed_binary_heap.h

#pragma once

#include "environments/node.h"
#include <vector>
#include <limits>
#include <algorithm>

template <typename PriorityType = double, typename Compare = std::greater<PriorityType>>
class IndexedBinaryHeap {
public:

  IndexedBinaryHeap() = default;

  void push(uint32_t id, PriorityType priority, uint32_t h = 0) {
    if (id >= id_to_index_.size()) {
      size_t new_size = std::max<size_t>(id + 1, id_to_index_.size() * 2);
      id_to_index_.resize(new_size, -1);
    }
    
    if (id_to_index_[id] != -1) {
      change_priority(id, priority);
      return;
    }

    int index = static_cast<int>(heap_.size());
    heap_.push_back({priority, id});
    id_to_index_[id] = index;
    sift_up(index);
  }

  uint32_t pop() {
    if (heap_.empty()) return NODE_NULL;

    uint32_t best_id = heap_[0].id;
    remove_at_index(0);
    return best_id;
  }

  uint32_t top() {
    if (heap_.empty()) return NODE_NULL;
    return heap_[0].id;
  }

  void remove(uint32_t id) {
    if (contains(id)) {
      remove_at_index(id_to_index_[id]);
    }
  }

  void change_priority(uint32_t id, PriorityType new_priority) {
    if (!contains(id)) {
      push(id, new_priority);
      return;
    }

    int index = id_to_index_[id];
    PriorityType old_priority = heap_[index].priority;
    heap_[index].priority = new_priority;

    if (Compare()(old_priority, new_priority)) {
      sift_up(index);
    } else {
      sift_down(index);
    }
  }

  template <typename Func>
  void rebuild (Func f) {
    for (auto& item : heap_) {
      item.priority = f(item.id);
    }
    if (heap_.size() > 1) {
      for (int i = (static_cast<int>(heap_.size()) - 2) / 2; i >= 0; i--) {
        sift_down(i);
      }
    }
  }

  bool contains(uint32_t id) const {
    return id < id_to_index_.size() && id_to_index_[id] != -1;
  }

  bool empty() const { return heap_.empty(); }
  void clear() { heap_.clear(); id_to_index_.clear(); }

private:

  struct HeapItem {
    PriorityType priority;
    uint32_t id;
  };

  std::vector<HeapItem> heap_;
  std::vector<int> id_to_index_; // Maps ID -> Heap Index. -1 if not in heap.

  void sift_up(int index) {
    while (index > 0) {
      int parent = (index - 1) / 2;
      if (Compare()(heap_[parent].priority, heap_[index].priority)) { 
        swap_nodes(index, parent);
        index = parent;
      } 
      else {
        break;
      }
    }
  }

  void sift_down(int index) {
    int size = static_cast<int>(heap_.size());
    while (true) {
      int left = 2 * index + 1;
      int right = 2 * index + 2;
      int best = index;

      if (left < size && Compare()(heap_[best].priority, heap_[left].priority))
        best = left;
      if (right < size && Compare()(heap_[best].priority, heap_[right].priority)) 
        best = right;

      if (best != index) {
        swap_nodes(index, best);
        index = best;
      } 
      else {
        break;
      }
    }
  }

  void remove_at_index(int index) {
    int last_index = static_cast<int>(heap_.size()) - 1;
    uint32_t removed_id = heap_[index].id;

    if (index != last_index) {
      swap_nodes(index, last_index);
      heap_.pop_back();
          
      // Re-balance the swapped element
      sift_up(index);
      sift_down(index);
    } 
    else {
      heap_.pop_back();
    }

    if (removed_id < id_to_index_.size()) {
      id_to_index_[removed_id] = -1;
    }
  }

  void swap_nodes(int i, int j) {
    std::swap(heap_[i], heap_[j]);
    id_to_index_[heap_[i].id] = i;
    id_to_index_[heap_[j].id] = j;
  }
};