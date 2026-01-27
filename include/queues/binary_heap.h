#pragma once

#include "environments/node.h"
#include <vector>
#include <functional> // For std::less/greater

template <typename PriorityType, typename Compare = std::less<PriorityType>>
class BinaryHeap {

public:
    BinaryHeap(NodePool& p) : pool_(p) {}

    void push(uint32_t handle, PriorityType priority) {
        heap_.push_back({priority, handle});
        pool_[handle].queue_ref = heap_.size() - 1;
        sift_up(heap_.size() - 1);
    }

    uint32_t pop() {
        if (empty()) {
            return NODE_NULL;
        }
        
        uint32_t best_handle = heap_[0].handle;
        pool_[best_handle].queue_ref = NODE_NULL;
        
        heap_[0] = heap_.back();
        heap_.pop_back();

        if (empty()) {
            return best_handle;
        }

        pool_[heap_[0].handle].queue_ref = 0;
        sift_down(0);

        return best_handle;
    }

    uint32_t top() {
        if (empty()) {
            return NODE_NULL;
        }

        return heap_[0].handle;
    }

    void decrease_key(uint32_t handle, PriorityType new_priority) {
        int current_idx = pool_[handle].queue_ref;
        // Ensure the handle is actually in the heap at the expected location
        if(current_idx >= heap_.size() || heap_[current_idx].handle != handle) return;

        heap_[current_idx].priority = new_priority;
        sift_up(current_idx);
    }

    bool contains(uint32_t handle) const {
        auto ref = pool_[handle].queue_ref;
        return ref != NODE_NULL && ref < heap_.size() && heap_[ref].handle == handle;
    }

    bool empty() const { return heap_.empty(); }

    template<typename Func>
    void rebuild(Func calculate_priority) {
        for (auto& item : heap_) {
            item.priority = calculate_priority(item.handle);
        }

        heapify();
    }

private:
    void heapify() {
        for (int i = (heap_.size() / 2) - 1; i >= 0; --i) {
            sift_down(i);
        }
    }

    void sift_up(int idx) {
        while (idx > 0) {
            int parent_idx = (idx - 1) / 2;
            if (comp_(heap_[idx].priority, heap_[parent_idx].priority)) {
                std::swap(heap_[idx], heap_[parent_idx]);
                pool_[heap_[idx].handle].queue_ref = idx;
                pool_[heap_[parent_idx].handle].queue_ref = parent_idx;
                idx = parent_idx;
            } else {
                break;
            }
        }
    }

    void sift_down(int idx) {
        uint32_t size = heap_.size();
        while (true) {
            uint32_t left_child = 2 * idx + 1;
            uint32_t right_child = 2 * idx + 2;
            uint32_t top_idx = idx;

            if (left_child < size && comp_(heap_[left_child].priority, heap_[top_idx].priority)) {
                top_idx = left_child;
            }

            if (right_child < size && comp_(heap_[right_child].priority, heap_[top_idx].priority)) {
                top_idx = right_child;
            }

            if (top_idx == idx) {
                break;
            }

            std::swap(heap_[idx], heap_[top_idx]);
            pool_[heap_[idx].handle].queue_ref = idx;
            pool_[heap_[top_idx].handle].queue_ref = top_idx;
            idx = top_idx;
        }
    }

    NodePool& pool_;
    Compare comp_;

    struct HeapItem {
        PriorityType priority;
        uint32_t handle;
    };

    std::vector<HeapItem> heap_;
};