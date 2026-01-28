#pragma once

#include "environments/node.h"

#include <algorithm>
#include <functional> // For std::less/greater
#include <vector>

template <typename PriorityType, typename Compare = std::greater<PriorityType>>
class BinaryHeap {

public:

  BinaryHeap() = default;

  void push(uint32_t id, PriorityType priority, uint32_t h = 0) {
    heap_.push_back({priority, id});
    std::push_heap(heap_.begin(), heap_.end(), HeapItemCompare());
  }

  uint32_t pop() {
    if (heap_.empty()) return NODE_NULL;

    std::pop_heap(heap_.begin(), heap_.end(), HeapItemCompare());
    uint32_t best_id = heap_.back().id;
    heap_.pop_back();

    return best_id;
  }

  uint32_t top() const {
    if (heap_.empty()) return NODE_NULL;
    return heap_.front().id;
  }

  bool empty() const { return heap_.empty(); }

  void clear() {
    heap_.clear();
  }

  template <typename Func>
  void rebuild(Func f) {
    for (auto& item : heap_) {
      item.priority = f(item.id);
    }

    std::make_heap(heap_.begin(), heap_.end(), HeapItemCompare());
  }

private:
  struct HeapItem {
    PriorityType priority;
    uint32_t id;
  };

  struct HeapItemCompare {
    bool operator()(const HeapItem& a, const HeapItem& b) const {
      return Compare()(a.priority, b.priority);
    }
  };

  std::vector<HeapItem> heap_;
};
