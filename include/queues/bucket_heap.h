#pragma once

#include "environments/node.h"
#include "two_level_bucket_queue.h"
#include "binary_heap.h"

#include <vector>
#include <functional>

template <typename PriorityType, typename Compare = std::greater<PriorityType>>
class BucketHeap {

public:
    BucketHeap(NodePool& pool, uint32_t f_cap_hint = 1024) 
        : pool_(pool), 
          bucket_store_(f_cap_hint) {
    }

    void push(uint32_t id, PriorityType priority, uint32_t h) {
        uint32_t g = pool_.get_g(id);
        uint32_t f = g + h;

        size_t old_count = bucket_store_.get_node_count(f);
        
        bucket_store_.push(id, f, h);

        if (f >= current_priorities_.size()) {
            size_t new_size = std::max<size_t>(f + 1, current_priorities_.size() * 1.5);
            current_priorities_.resize(new_size, std::numeric_limits<PriorityType>::lowest());
        }
        
        // If the bucket was empty, or if the new priority is better (for max-heap), update.
        // The lazy push handles decrease-key.
        if (old_count == 0 || Compare()(priority, current_priorities_[f])) {
            primary_heap_.push(f, priority);
            current_priorities_[f] = priority;
        }
    }

    uint32_t pop() {
        while (!primary_heap_.empty()) {
            uint32_t f_cost = primary_heap_.top();
            PriorityType heap_priority = primary_heap_.top_priority();

            if (f_cost >= current_priorities_.size() || heap_priority != current_priorities_[f_cost]) {
                primary_heap_.pop();
                continue;
            }

            primary_heap_.pop();
            
            bucket_store_.set_f_min(f_cost);
            uint32_t id = bucket_store_.pop();

            if (bucket_store_.get_node_count(f_cost) > 0) {
                uint32_t new_h_min = bucket_store_.get_h_min(f_cost);
                uint32_t rep_id = bucket_store_.peek_top_node(f_cost, new_h_min);
                uint32_t rep_g = pool_.get_g(rep_id);

                PriorityType new_priority = priority_calculator_(rep_g, new_h_min);
                primary_heap_.push(f_cost, new_priority);
                current_priorities_[f_cost] = new_priority;
            }

            return id;
        }
        return NODE_NULL;
    }

    template<typename Func>
    void rebuild(Func new_priority_calculator) {
        priority_calculator_ = new_priority_calculator;

        primary_heap_.rebuild([&](uint32_t f_cost_index) {
            uint32_t h_min = bucket_store_.get_h_min(f_cost_index);
            if (h_min == std::numeric_limits<uint32_t>::max()) {
                return std::numeric_limits<PriorityType>::lowest();
            }
            uint32_t representative_node_id = bucket_store_.peek_top_node(f_cost_index, h_min);
            uint32_t g_cost = pool_.get_g(representative_node_id);
            
            PriorityType new_priority = new_priority_calculator(g_cost, h_min);
            if(f_cost_index >= current_priorities_.size()) {
                current_priorities_.resize(f_cost_index + 1, std::numeric_limits<PriorityType>::lowest());
            }
            current_priorities_[f_cost_index] = new_priority;
            return new_priority;
        });
    }

    bool empty() const { 
        // The primary heap can have stale entries, so we can't just check its size.
        // A full check is too slow. We rely on pop() to return NODE_NULL when truly empty.
        // For the main loop, `primary_heap_.empty()` is a good enough heuristic.
        return primary_heap_.empty(); 
    }

    void clear() {
        primary_heap_.clear();
        bucket_store_.clear();
        current_priorities_.clear();
    }
    
private:
    NodePool& pool_;
    TwoLevelBucketQueue bucket_store_;
    BinaryHeap<PriorityType, Compare> primary_heap_;
    std::vector<PriorityType> current_priorities_;

    std::function<PriorityType(uint32_t, uint32_t)> priority_calculator_;
};