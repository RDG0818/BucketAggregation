#pragma once

#include <vector>
#include <cstdint>
#include <limits>
#include "utils/utils.h"

template <typename E, typename PQ>
class ANAStar {

public:
    ANAStar(E& env, PQ& priority_queue, utils::SearchStats* stats = nullptr) 
        : env_(env), priority_queue_(priority_queue), stats_(stats), G_upper_(std::numeric_limits<double>::max()) {};

    void solve() {
        uint32_t start_node_handle = env_.get_start_node();
        const auto& start_node_data = env_.get_pool()[start_node_handle];
        
        if (env_.is_goal(env_.get_state(start_node_handle))) {
            G_upper_ = start_node_data.g;
            if (stats_) stats_->solution_cost = G_upper_;
            return;
        }

        double start_e = calculate_e(start_node_data.g, start_node_data.h);
        priority_queue_.push(start_node_handle, start_e);

        std::vector<uint32_t> neighbors;

        while (!priority_queue_.empty()) {
            uint32_t current_node_handle = priority_queue_.pop();
            const auto& current_node_data = env_.get_pool()[current_node_handle];

            // If a node's optimistic cost is no better than our incumbent, it can't lead to an improved solution.
            if (current_node_data.g + current_node_data.h >= G_upper_) {
                continue;
            }
            if (stats_) {
                stats_->nodes_expanded++;
            }

            if (env_.is_goal(env_.get_state(current_node_handle))) {
                if (current_node_data.g < G_upper_) {
                    // Found a new, better solution. Update the upper bound.
                    G_upper_ = current_node_data.g;
                    if (stats_) stats_->solution_cost = G_upper_;

                    // Rebuild the entire heap with new e-values based on the new G_upper
                    auto calculator = [&](uint32_t handle) -> double {
                        const auto& node = env_.get_pool()[handle];
                        return calculate_e(node.g, node.h);
                    };
                    priority_queue_.rebuild(calculator);
                }
                // Continue searching for even better solutions
                continue;
            }

            neighbors.clear();
            env_.get_successors(current_node_handle, neighbors);

            for (uint32_t neighbor_handle : neighbors) {
                const auto& neighbor_data = env_.get_pool()[neighbor_handle];
                
                // Pruning Step: Don't expand nodes that can't possibly beat the incumbent solution.
                if (neighbor_data.g + neighbor_data.h >= G_upper_) {
                    continue;
                }

                double neighbor_e = calculate_e(neighbor_data.g, neighbor_data.h);

                if (priority_queue_.contains(neighbor_handle)) {
                    // A better path to this node was found, so its e-value increases.
                    // The 'decrease_key' method performs the required sift-up in the max-heap.
                    priority_queue_.decrease_key(neighbor_handle, neighbor_e);
                } else {
                    priority_queue_.push(neighbor_handle, neighbor_e);
                }
            }
        }
    }

private:
    // Calculates the e-value for a given node's g and h values.
    double calculate_e(uint32_t g, uint32_t h) const {
        if (h == 0) {
            // Goal nodes should always have the highest priority
            return std::numeric_limits<double>::max();
        }
        if (g >= G_upper_) {
            // Nodes that are already more expensive than the best solution have no promise
            return std::numeric_limits<double>::lowest();
        }
        return (G_upper_ - g) / static_cast<double>(h);
    }

    E& env_;
    PQ& priority_queue_;
    utils::SearchStats* stats_;
    double G_upper_;
};
