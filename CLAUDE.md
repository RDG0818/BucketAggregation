# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
git submodule update --init --recursive  # First-time setup
make          # Build (runs cmake + make in ./build)
make clean    # Clean build artifacts
make rebuild  # Clean + rebuild

./build/main   # Run full benchmark suite
./build/tests  # Run all unit tests
```

Build uses Release mode with `-O3 -march=native -flto -g -fno-omit-frame-pointer`. C++20 required.

## Architecture

This is a research implementation of bucket-based priority queues for heuristic search, extending the paper ["A Bucket-Based Priority Queue for Bounded-Suboptimal and Anytime A* Search"](https://ojs.aaai.org/index.php/SOCS/article/view/35976).

### Priority Queue Hierarchy (`include/queues/`)

The queues form a progression of complexity, all supporting a common interface (`push`, `pop`, `top`, `rebuild`, `empty`):

- **`binary_heap.h`** — STL-based heap; baseline comparison
- **`bucket_queue.h`** — Integer-indexed O(1) push; LIFO within buckets; good for dense integer costs
- **`two_level_bucket_queue.h`** — Primary buckets by f-cost, secondary by h-cost; uses a `BlockPool` of 512-byte blocks (126 node IDs each) for memory efficiency; tracks `h_min` per f-bucket; accepts `alpha`/`beta` params to aggregate secondary/primary buckets (nodes with cost h land in bucket `floor(h/alpha)`, etc.)
- **`indexed_d_ary_heap.h`** — D-ary heap (default D=2) with O(1) decrease-key via id→index map; exposes `rebuild()` for bulk reprioritization
- **`bucket_heap.h`** — Hybrid: `TwoLevelBucketQueue` stores nodes, `IndexedDAryHeap` tracks f-buckets; priority function injected via template `PriorityCalculator`; accepts `alpha`/`beta` aggregation params and `use_h_max` flag (passed to the internal queue)

### Search Algorithms (`include/algorithms/`)

All algorithms are templated on `<Environment, Queue>`. Key algorithms:

- **`a_star.h`** — Standard best-first search on f = g + h
- **`anytime_a_star.h`** — Iteratively tightens weight w; uses incumbent pruning
- **`ana_star.h`** — ANA*: priority E(n) = (G_upper − g(n)) / h(n); dynamically adjusts greediness; requires `rebuild()` on every solution improvement — this is the primary motivation for the bucket-based structures
- **`dps.h`** — Dynamic Potential Search

### Search Environments (`include/environments/`, `src/`)

All environments share an interface: `reset_search()`, `get_successors()`, `get_heuristic()`, `is_goal()`, `get_edge_cost()`, `get_pool()`.

- **Grid** — 4-connected W×H grid, 20% obstacles, Manhattan distance heuristic
- **Sliding Tile** — 15-puzzle, Manhattan distance heuristic, `phmap::flat_hash_map` for state hashing
- **Pancake** — N=48 pancake sorting, gap heuristic, `phmap::flat_hash_map` for caching
- **MSA** — Multiple sequence alignment environment

### Node Management (`include/environments/node.h`)

`NodePool` manages all node state for a search. Uses per-search iteration counters to avoid resetting the pool between searches — `is_generated()` / `is_closed()` check against the current iteration ID rather than clearing arrays. Stores g-costs, parent pointers, and open/closed status.

### Metrics (`include/utils/utils.h`)

`SearchStats` tracks timing per queue operation (enqueue, dequeue, rebuild, decrease-key), node counts, stale pops, and memory peak. `QueueDetailedMetrics` captures internal bucket state and h-value distributions for analysis. `src/main.cpp` runs all algorithm × queue × environment combinations and outputs CSV + formatted tables.

### Template Pattern

Algorithms and queues are connected via templates. The `PriorityCalculator` template parameter allows injecting custom f/h weighting into the bucket heap without modifying the data structure — critical for reusing the same structure across A*, Weighted A*, and ANA*.
