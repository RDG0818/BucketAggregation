// include/queues/indexed_d_ary_heap.h
//
// D-ary heap with O(1) decrease-key via an id→index lookup table.
// Supports arbitrary priority types and comparison orders (default: min-heap
// on double via std::greater, which puts the smallest priority at the root).
//
// Primarily used inside BucketHeap to track which primary f-buckets are
// non-empty, keyed by f-bucket index with a calculator-derived priority.
// Also used as a standalone open list for A*, ANA*, and DPS baselines.
//
// push() handles decrease-key implicitly: re-pushing an existing id updates
// its priority in place rather than inserting a duplicate.

#pragma once

#include "environments/node.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <cassert>

template <typename PriorityType = double, int D = 2, typename Compare = std::greater<PriorityType>>
class IndexedDaryHeap {
public:

  IndexedDaryHeap() = default;

  void push(uint32_t id, PriorityType priority, uint32_t h = 0) {
    // Guard against id == INF_COST: resizing id_to_index_ to 4B+ entries would OOM.
    assert(id != INF_COST && "Attempted to push INF_COST id onto the heap.");

    if (id >= id_to_index_.size()) {
      size_t new_size = std::max<size_t>(id + 1, id_to_index_.size() * 2);
      id_to_index_.resize(new_size, -1);
    }

    if (id_to_index_[id] != -1) {
      change_priority(id, priority, h);
      return;
    }

    int index = static_cast<int>(heap_.size());
    heap_.push_back({priority, id, h});
    id_to_index_[id] = index;
    sift_up(index);
  }

  uint32_t pop() noexcept {
    if (heap_.empty()) return NODE_NULL;
    uint32_t best_id = heap_[0].id;
    remove_at_index(0);
    return best_id;
  }

  uint32_t top() const noexcept {
    if (heap_.empty()) return NODE_NULL;
    return heap_[0].id;
  }

  PriorityType top_priority() const noexcept {
    if (heap_.empty()) return std::numeric_limits<PriorityType>::max();
    return heap_[0].priority;
  }

  void remove(uint32_t id) noexcept {
    if (contains(id)) {
      remove_at_index(id_to_index_[id]);
    }
  }

  void change_priority(uint32_t id, PriorityType new_priority, uint32_t new_h = 0) {
    if (!contains(id)) {
      push(id, new_priority, new_h);
      return;
    }

    int index = id_to_index_[id];
    auto old_item = heap_[index];
    heap_[index].priority = new_priority;
    heap_[index].h = new_h;

    if (is_better(heap_[index], old_item)) {
      sift_up(index);
    } else {
      sift_down(index);
    }
  }

  template <typename Func>
  void rebuild(Func f) {
    for (auto& item : heap_) {
      item.priority = f(item.id);
    }
    // Floyd's O(n) heapify: start from the last internal node and sift down.
    for (int i = (static_cast<int>(heap_.size()) - 1) / D; i >= 0; i--) {
      sift_down(i);
    }
  }

  bool contains(uint32_t id) const noexcept {
    return id < id_to_index_.size() && id_to_index_[id] != -1;
  }

  bool empty() const noexcept { return heap_.empty(); }
  void clear() noexcept { heap_.clear(); id_to_index_.clear(); }

private:
  struct HeapItem {
    PriorityType priority;
    uint32_t id;
    uint32_t h;
  };

  std::vector<HeapItem> heap_;
  std::vector<int> id_to_index_;

  // Returns true if a should be closer to the root than b.
  // Equivalent to: b is worse than a under Compare (i.e., Compare(b, a)).
  // With the default Compare = std::greater: smaller priority floats to root (min-heap).
  bool is_better(const HeapItem& a, const HeapItem& b) const noexcept {
    if (a.priority != b.priority) {
      return Compare()(b.priority, a.priority);
    }
    return a.h < b.h; // Tie-break: prefer smaller h
  }

  void sift_up(int index) noexcept {
    while (index > 0) {
      int parent = (index - 1) / D;
      if (is_better(heap_[index], heap_[parent])) {
        swap_nodes(index, parent);
        index = parent;
      } else {
        break;
      }
    }
  }

  void sift_down(int index) noexcept {
    size_t idx = static_cast<size_t>(index);
    size_t size = heap_.size();
    while (true) {
      size_t first_child = D * idx + 1;
      if (first_child >= size) break;

      size_t best_child = first_child;
      size_t limit = std::min(size, first_child + D);
      for (size_t i = first_child + 1; i < limit; ++i) {
        if (is_better(heap_[i], heap_[best_child])) {
          best_child = i;
        }
      }

      if (is_better(heap_[best_child], heap_[idx])) {
        swap_nodes(static_cast<int>(idx), static_cast<int>(best_child));
        idx = best_child;
      } else {
        break;
      }
    }
  }

  void remove_at_index(int index) noexcept {
    int last_index = static_cast<int>(heap_.size()) - 1;
    uint32_t removed_id = heap_[index].id;

    if (index != last_index) {
      swap_nodes(index, last_index);
      heap_.pop_back();
      // The swapped-in node's priority relative to its new neighbors is unknown,
      // so try both directions — only one will actually move it.
      if (!heap_.empty()) {
        sift_up(index);
        sift_down(index);
      }
    } else {
      heap_.pop_back();
    }

    if (removed_id < id_to_index_.size()) {
      id_to_index_[removed_id] = -1;
    }
  }

  void swap_nodes(int i, int j) noexcept {
    std::swap(heap_[i], heap_[j]);
    id_to_index_[heap_[i].id] = i;
    id_to_index_[heap_[j].id] = j;
  }
};
