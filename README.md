# Improvements on a Bucket-Based Priority Queue 

BucketHeap is a high-performance C++ research framework designed for analyzing and benchmarking heuristic search algorithms (A*, ANA*, AWA*) and priority queue data structures. The project emphasizes cache locality, memory efficiency, and instruction-level parallelism to achieve maximum node throughput.

Unlike standard object-oriented search implementations that suffer from pointer chasing and memory fragmentation, BucketHeap employs a Data-Oriented Design (DOD) philosophy. It utilizes Structure of Arrays (SoA), custom memory allocators, and flat data structures to keep the CPU cache hot and minimize allocation overhead.

🏗️ Core Architecture & Separation of Concerns

The framework strictly enforces a separation of concerns to maximize modularity and performance. Each component has a single, well-defined responsibility.
1. The Solver (Algorithm)

    Responsibility: Orchestrates the search logic (A*, ANA*, etc.).

    Behavior: It is the only component that understands F=G+H. It calculates priorities and decides when to push/pop from the queue. It handles duplicate detection (via NodePool checks) and goal checks.

    Statelessness: Solvers hold no data between runs aside from configuration.

2. The Environment (Graph Definition)

    Responsibility: Defines the graph topology (Successors) and edge costs (C(u,v)).

    Statelessness: Environments are stateless regarding the search progress. They do not store G values, parent pointers, or "Closed List" flags. They only store static map data (e.g., the grid layout or the tile configuration).

    Optimizations:

        Implicit Graphs: For domains like Grid, no node objects are created; state IDs are direct indices into memory.

        Explicit Graphs: For domains like Sliding Tile, states are mapped to dense integer IDs on-the-fly.

3. The NodePool (State Memory Manager)

    Responsibility: The centralized "Source of Truth" for search data.

    Data Layout: Uses Structure of Arrays (SoA) instead of Array of Structures (AoS).

        vector<uint32_t> g_costs

        vector<uint32_t> parents

        vector<uint8_t> flags (visited status)

    Optimization: This layout ensures that accessing g_costs[id] during node generation pulls only relevant integer data into the cache line, maximizing cache utilization and allowing SIMD optimizations.

4. The Priority Queue (Open List)

    Responsibility: A "dumb" container that orders integers based on a priority value.

    Lazy Deletion: Queues do not support decrease_key or contains. If a better path to a node is found, it is simply pushed again. The Solver filters out stale entries upon popping by comparing the popped cost against the NodePool.

    Optimization: This removes the need for expensive hash map lookups inside the queue operations.

🛠️ Utilities

    SearchStats: A comprehensive struct tracking micro-benchmarks (enqueue/dequeue times in nanoseconds) and macro-metrics (nodes expanded, solution cost).

    ProfiledQueue: A wrapper class that uses std::chrono and SFINAE (Substitution Failure Is Not An Error) to transparently profile any queue implementation without modifying the algorithm code.

## Overview

### Algorithms

**A\***: The standard for optimal pathfinding.

    Implementation: Standard best-first search.

    Optimizations: Uses an optimized heuristic inlining and the SoA NodePool for fast G-value lookups.

**Anytime A\***: An anytime variant that quickly finds a suboptimal solution and iteratively improves it.

    Behavior: Uses a "window" size (implicitly handled by the queue or weight) to prune nodes that cannot improve upon the current best solution (incumbent).

    Optimizations: Implements incumbent pruning: any node with F≥Gbest​ is discarded immediately.

3. ANA* (Anytime Non-parametric A*)

An anytime algorithm that removes the need for tuning parameters (like weights).

    Priority Function: Maximizes E(n)=h(n)Gupper​−g(n)​. This effectively greedily expands nodes most likely to improve the current solution.

    Rebuild Mechanism: When a better solution (Gupper​) is found, the priority of every node in the open list changes.

    Optimization: Instead of clearing and re-inserting, the queue supports an in-place O(N) rebuild() operation that recalculates priorities and heapifies the backing vector in a single pass.



### Data Structures

1. Binary Heap (BinaryHeap)

A standard array-based binary heap.

    Type: Templated for Min-Heap (A*) or Max-Heap (ANA*).

    Optimizations:

        3-Argument push_heap: Uses custom comparators to avoid object overhead.

        Bulk Rebuild: Supports the O(N) rebuild required by ANA*.

2. Bucket Queue (BucketQueue)

A fast, integer-based priority queue for domains with small, integer edge costs.

    Structure: A vector<vector<uint32_t>>. Index i holds all nodes with Priority i.

    Optimizations:

        Geometric Resizing: Resizes by 1.5x/2.0x to amortize allocation costs.

        LIFO Access: Buckets act as Stacks (push_back/pop_back). This ensures the most recently generated nodes (hot in cache) are expanded first.

        Monotonic Min Pointer: Tracks the minimum bucket index to ensure O(1) amortized pop.

3. Two-Level Bucket Queue (TwoLevelBucketQueue)

A specialized queue designed to handle Tie-Breaking by H efficiently.

    Structure: A vector of Primary Buckets (indexed by F), each containing a vector of Secondary Buckets (indexed by H).

    Optimization:

        Implicit Tie-Breaking: By popping from the lowest non-empty H-bucket within the lowest F-bucket, it guarantees expanding nodes closest to the goal first.

        Memory Efficiency: Uses std::unique_ptr for Primary Buckets. Empty F-ranges consume almost zero memory (just null pointers).

        Embedded Vectors: Secondary buckets are embedded directly in the Primary struct to avoid double-indirection.


### Environments

1. Grid Environment (Implicit)

A standard 4-connected grid map.

    Type: Implicit Graph (States are not stored; ID is just y * width + x).

    Optimizations:

        Direct Indexing: No hash map lookups. Successors are computed via simple integer arithmetic.

        Pre-computed Heuristics: Manhattan distance is inlined.

2. Sliding Tile Puzzle (Explicit)

The classic 15-puzzle.

    Type: Explicit Graph (States are 64-bit integers representing tile positions).

    Optimizations:

        Bit-Packing: States are packed into uint64_t for efficient storage and copying.

3. Pancake Puzzle (Explicit)

A permutation puzzle where a spatula flips a prefix of the sequence.

    Type: Explicit Graph.

    Optimizations: Uses the Gap Heuristic to estimate distance to the sorted state.


### Architecture

### Utilities

## Prerequisites

```bash
git submodule update --init --recursive
```

## Installation

## Future Work

- Templated Testing Environment
  - Templated Heuristic
  - 15 Puzzle
  - 5000x5000 Grid Pathfinding (Add Integer/Real Valued Weight Costs)
  - 48 Pancake Problem
- Template for priority queues
  - Bucket Queue
  - Two-Level Bucket Queue
  - Binary Heap
  - Bucket Heap
  - Modified Bucket Heap
- Template for search algorithms
  - A*
  - WA*
  - ANA*
- Hybrid Expansion Strategy Demo
- Modifying Memory Management
- Track Parameters
  - Number of non-empty primary vs. secondary buckets
  - Number of node re-openings and re-expansions
- Visualizations?

TODO: 
Figure out full structure/where things are handled in codebase
figure out common tricks for cache locality


## License
This project is licensed under the MIT License - see the `License.md` file for details.