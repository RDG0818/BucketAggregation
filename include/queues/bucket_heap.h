#pragma once

#include "environments/node.h"
#include "binary_heap.h"

#include <vector>
#include <memory>
#include <limits>
#include <functional>
#include <unordered_map>

template <typename PriorityType, typename Compare = std::less<PriorityType>>
class BucketHeap {

private:
  // NodeList stores pairs of {handle, generation}
  using NodeList = std::vector<std::pair<uint32_t, uint32_t>>; 
  using SecondaryBucketMap = std::vector<NodeList>; 

  struct PrimaryBucket {
    PriorityType dynamic_priority = std::numeric_limits<PriorityType>::lowest();
    uint32_t h_min = std::numeric_limits<uint32_t>::max();
    std::unique_ptr<SecondaryBucketMap> secondary_buckets;
    uint32_t node_count = 0;
  };

public:

  using PriorityFunc = std::function<PriorityType(uint32_t node_handle)>;

  BucketHeap(NodePool& p, PriorityFunc pf, uint32_t f_cap = 4096, uint32_t h_cap = 1024)
    : pool_(p),
      priority_func_(pf),
      heap_(bucket_node_pool_),
      f_capacity_(f_cap),
      h_capacity_(h_cap),
      count_(0)
    {
      buckets_.resize(f_capacity_);
      bucket_node_pool_.reserve(f_capacity_);
    };

  void set_priority_func(PriorityFunc pf) {
    priority_func_ = pf;
  }

  void push(uint32_t handle) {
    const bool is_new_node = !contains(handle);
    const auto& node_data = pool_[handle];
    const uint32_t f_cost = node_data.g + node_data.h;
    const uint32_t h_cost = node_data.h;

    resize_if_needed_(f_cost, h_cost);
    PrimaryBucket& bucket = buckets_[f_cost];

    if (!bucket.secondary_buckets) {
      bucket.secondary_buckets = std::make_unique<SecondaryBucketMap>(h_capacity_);
      bucket.h_min = h_cost;
      bucket.dynamic_priority = priority_func_(handle);
      heap_.push(f_cost, bucket.dynamic_priority);
    } 
    else {
      bool priority_changed = false;
      // If this new node sets a new h_min, or if it has the same h as the current h_min,
      // the bucket's priority might need to be updated.
      if (h_cost < bucket.h_min) {
        bucket.h_min = h_cost;
        priority_changed = true;
      } else if (h_cost == bucket.h_min) {
        // If the new node's priority is better than the current bucket priority,
        // it becomes the new representative.
        if (Compare()(priority_func_(handle), bucket.dynamic_priority)) {
          priority_changed = true;
        }
      }

      if (priority_changed) {
        bucket.dynamic_priority = priority_func_(handle);
        heap_.decrease_key(f_cost, bucket.dynamic_priority);
      }
    }

    (*bucket.secondary_buckets)[h_cost].push_back({handle, generations_[handle]});
    pool_[handle].queue_ref = f_cost;
    bucket.node_count++;

    if (is_new_node) {
        count_++;
    }
  }

  uint32_t pop() {
    while (!empty()) {
      uint32_t f_bucket_idx = heap_.pop();
      PrimaryBucket& bucket = buckets_[f_bucket_idx];

      while (bucket.node_count > 0) {
        NodeList& nodes = (*bucket.secondary_buckets)[bucket.h_min];

        while (!nodes.empty()) {
          auto node_entry = nodes.back();
          uint32_t handle = node_entry.first;
          uint32_t stored_generation = node_entry.second;

          nodes.pop_back();
          bucket.node_count--;

          if (stored_generation != generations_[handle]) {
            continue; // Stale node
          }

          count_--;
          pool_[handle].queue_ref = NODE_NULL;

          // If the h_min bucket is now empty, find the next one.
          if (nodes.empty()) {
            find_next_h_min_(bucket);
          }
          
          // If the bucket still has nodes, re-calculate its priority from its
          // h_min representative and push it back on the heap.
          if (bucket.node_count > 0) {
            // Must clean out stale nodes from the new h_min bucket front
            // until we find a valid representative.
            while(bucket.node_count > 0) {
                NodeList& rep_nodes = (*bucket.secondary_buckets)[bucket.h_min];
                while(!rep_nodes.empty()) {
                    auto& back_entry = rep_nodes.back();
                    if(back_entry.second == generations_[back_entry.first]) {
                        bucket.dynamic_priority = priority_func_(back_entry.first);
                        heap_.push(f_bucket_idx, bucket.dynamic_priority);
                        goto found_rep;
                    } else {
                        rep_nodes.pop_back();
                        bucket.node_count--;
                    }
                }
                find_next_h_min_(bucket);
            }
            found_rep:;
          }
          
          return handle;
        }

        find_next_h_min_(bucket);
      }
    }
    return NODE_NULL;
  }

  void decrease_key(uint32_t handle) {
    generations_[handle]++;
    push(handle);
  }

  void rebuild() {
    heap_.rebuild([&](uint32_t f_bucket_idx) {
      PrimaryBucket& bucket = buckets_[f_bucket_idx];
      if (bucket.node_count == 0) {
        return std::numeric_limits<PriorityType>::lowest();
      }

      // In a rebuild, we must find the true best priority for each bucket
      // according to the new priority function, based on its h_min node.
      // This requires cleaning stale nodes from h_min buckets until a valid rep is found.
       while(bucket.node_count > 0) {
          NodeList& rep_nodes = (*bucket.secondary_buckets)[bucket.h_min];
          while(!rep_nodes.empty()) {
              auto& back_entry = rep_nodes.back();
              if(back_entry.second == generations_[back_entry.first]) {
                  bucket.dynamic_priority = priority_func_(back_entry.first);
                  return bucket.dynamic_priority;
              } else {
                  rep_nodes.pop_back();
                  bucket.node_count--;
              }
          }
          find_next_h_min_(bucket);
      }
      return std::numeric_limits<PriorityType>::lowest();
    });
  }

  bool contains(uint32_t handle) const { return pool_[handle].queue_ref != NODE_NULL; }
  bool empty() const { return count_ == 0; }

private:

  void resize_if_needed_(uint32_t f_cost, uint32_t h_cost) {
    if (f_cost >= f_capacity_) {
      uint32_t new_f_capacity = f_capacity_ > 0 ? f_capacity_ : 1;
      while (f_cost >= new_f_capacity) {
        new_f_capacity *= 2;
      }
      f_capacity_ = new_f_capacity;

      buckets_.resize(f_capacity_);
      bucket_node_pool_.reserve(f_capacity_);
      while(f_cost >= bucket_node_pool_.size()){
        bucket_node_pool_.allocate(NODE_NULL, 0, 0);
      }
    }

    PrimaryBucket& bucket = buckets_[f_cost];
    if (bucket.secondary_buckets && h_cost >= bucket.secondary_buckets->size()) {
      uint32_t current_h_cap = bucket.secondary_buckets->size();
      uint32_t new_h_cap = current_h_cap > 0 ? current_h_cap : 1;
      while (h_cost >= new_h_cap) {
          new_h_cap *= 2;
      }
      bucket.secondary_buckets->resize(new_h_cap);
    }
  }

  void find_next_h_min_(PrimaryBucket& bucket) {
    uint32_t current_h = bucket.h_min + 1;
    while (current_h < bucket.secondary_buckets->size()) {
      if (!(*bucket.secondary_buckets)[current_h].empty()) {
        bucket.h_min = current_h;
        return;
      }
      current_h++;
    }
    bucket.h_min = std::numeric_limits<uint32_t>::max();
  }

  NodePool& pool_; 
  NodePool bucket_node_pool_;
  BinaryHeap<PriorityType, Compare> heap_;
  
  std::vector<PrimaryBucket> buckets_;
  std::unordered_map<uint32_t, uint32_t> generations_;
  PriorityFunc priority_func_;
  uint32_t f_capacity_;
  uint32_t h_capacity_;
  size_t count_;

};
