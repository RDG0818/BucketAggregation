# Bucket Aggregation in Bucket-Based Priority Queues for Heuristic Search

Research implementation accompanying the paper **"Bucket Aggregation in Bucket-Based Priority Queues for Heuristic Search"** (SoCS 2026), extending [Fereday & Hansen (SoCS 2024)](https://ojs.aaai.org/index.php/SOCS/article/view/35976).

## The Paper

Bucket-based priority queues are highly efficient open lists for heuristic search, but their performance degrades when the h-cost distribution is sparse, or when nodes cluster at a small number of distinct h-values spread over a wide range. This leads to many empty secondary buckets, wasting memory and causing cache misses during reordering.

This paper introduces bucket aggregation: grouping h-costs into intervals of width α so that node $n$ is placed in secondary bucket $\lfloor h(n)/\alpha \rfloor$ instead of bucket $h(n)$. A corresponding parameter $\beta$ aggregates primary (f-cost) buckets.

> **Suggestion for diagram here:** A side-by-side illustration showing sparse h-buckets (α=1) vs. aggregated h-buckets (α>1) for the same node set, highlighting the reduction in empty buckets.

### Aggregation and Correctness

For A\*, using the lower bound of each bucket interval as the $h$-representative preserves admissibility:

$$h^{(b)}(n) = \lfloor h(n)/\alpha \rfloor \cdot \alpha \leq h(n)$$

For Anytime Nonparametric A\* (ANA\*) and Dynamic Potential Search (DPS), using the upper bound of the interval as the h-representative:

$$h^{(b)}_{\max}(n) = (\lfloor h(n)/\alpha \rfloor + 1) \cdot \alpha - 1$$

preserves the relative ordering of the priority function, preventing the queue from silently misordering nodes during rebuilds.

### Key Results

Aggregation delivers substantial cache efficiency gains on sparse-h domains:

| Domain | Metric | Baseline | α=16 |
|---|---|---|---|
| Non-uniform grid | Page faults | 234K | 8K |
| Non-uniform grid | LLC misses | 109M | 40M |
| Heavy sliding tile | LLC misses | — | ~3× reduction |

---

## Build & Run

```bash
git submodule update --init --recursive   # First-time setup
make                                       # Build (Release, -O3 -march=native -flto)
make rebuild                               # Clean + rebuild
./build/main                              # Full benchmark suite
./build/tests                             # Unit tests
```

**Memory note:** On memory-constrained systems, use `--capacity N` to cap the node pool size (default: 1,000,000). The pool grows dynamically as needed.

```bash
./build/main --capacity 500000
```

### Hardware Performance Counters

On Linux, the benchmark runner uses `perf_event_open` to collect per-benchmark hardware and software counters, displayed as a detail line under each result:

```
  ANA* with BucketHeap (a=16, D=2)          100    1,234,567       42.318      1024
    enq 1,234,567 @ 45ns  deq 1,234,567 @ 38ns  ...  |  overhead 0.12 ms
    LLC  40.1M miss / 109.2M ref (36.7%)  |  br-miss  1.2M / 45.6M (2.7%)  |  pg-flt  8.0K min  234 maj  |  IPC 2.31
```

| Counter | Type | Requires paranoid ≤ 1? |
|---|---|---|
| LLC miss rate | Hardware | Yes |
| Branch miss rate | Hardware | Yes |
| IPC (instructions/cycle) | Hardware | Yes |
| Page faults (minor) | Software | No |
| Page faults (major) | Software | No |

Page faults are available unconditionally. Hardware counters require `kernel.perf_event_paranoid <= 1`:

```bash
sudo sysctl kernel.perf_event_paranoid=1
```

If a counter is unavailable its value is zero and it is omitted from the display. All eight counters (`perf_llc_misses`, `perf_llc_refs`, `perf_branch_misses`, `perf_branches`, `perf_cycles`, `perf_instructions`, `perf_faults_minor`, `perf_faults_major`) are always present as columns in CSV output.

---

## Architecture

### Priority Queue Hierarchy

All queues share a common interface (`push`, `pop`, `empty`, `clear`, `rebuild`) and are templated on environment and algorithm type.

| Queue | File | Description |
|---|---|---|
| `BinaryHeap` | `binary_heap.h` | STL-based baseline; O(log n) push/pop; O(n) rebuild |
| `BucketQueue` | `bucket_queue.h` | Integer-indexed; O(1) amortized push/pop; LIFO within buckets; A* only |
| `TwoLevelBucketQueue` | `two_level_bucket_queue.h` | Primary buckets by f, secondary by h; pool-allocated 512-byte blocks |
| `IndexedDaryHeap` | `indexed_d_ary_heap.h` | D-ary heap with O(1) decrease-key via id→index map; O(n) rebuild |
| `BucketHeap` | `bucket_heap.h` | **Hybrid:** `TwoLevelBucketQueue` stores nodes, `IndexedDaryHeap` tracks non-empty f-buckets |

> **Suggestion for diagram here:** A structural diagram of `BucketHeap` showing the two-layer decomposition — the `IndexedDaryHeap` on top tracking f-bucket indices with computed priorities, feeding into the `TwoLevelBucketQueue` with its f→h→block chain.

#### TwoLevelBucketQueue

The core node-storage layer. Each primary bucket (indexed by $\lfloor f/\beta \rfloor$) holds a vector of secondary buckets (indexed by $\lfloor h/\alpha \rfloor$). Secondary buckets store node IDs in 512-byte blocks (126 IDs each), managed by a slab allocator (`BlockPool`) that recycles blocks via a free list to amortize allocation cost.

```
f-buckets:  [f=10] [f=11] [f=12] ...
                |
           h-buckets:  [h=0] [h=1] [h=2] ...
                                |
                          Block -> Block -> ...
                          [id id id ...]
```

Pop is O(1) amortized: a lazy cursor advances past empty f-buckets; within each f-bucket, `h_min` tracks the minimum non-empty secondary bucket.

#### BucketHeap

Wraps `TwoLevelBucketQueue` with an `IndexedDaryHeap` that tracks which primary f-buckets are non-empty. The heap is keyed by f-bucket index with a priority dependent on the non-admissible algorithm-specific weighting (e.g., ANA*'s $E(n)$).

`rebuild()` recomputes all heap priorities in O(n) without touching the bucket structure, making it highly efficient for algorithms that reorder the open list frequently.

#### Aggregation Parameters

Both queues accept `alpha` (secondary bucket width) and `beta` (primary bucket width):

- **α=1, β=1**: Standard bucket heap (no aggregation)
- **α>1**: Multiple h-values share one secondary bucket → fewer buckets, better cache locality
- **β>1**: Multiple f-values share one primary bucket → reduces primary heap size

The `use_h_max` flag on `BucketHeap` switches the h-representative from lower bound (safe for A*) to upper bound (needed for ANA*/DPS correctness under aggregation).

---

### Search Algorithms

All algorithms are templated on `<Environment, Queue>`.

| Algorithm | File | Priority | Use case |
|---|---|---|---|
| A* | `a_star.h` | $f(n) = g(n) + h(n)$ | Optimal search |
| Anytime A* | `anytime_a_star.h` | $f(n) = g(n) + w \cdot h(n)$ | Fast suboptimal + iterative improvement |
| ANA* | `ana_star.h` | $E(n) = (G_{upper} - g(n))\, /\, h(n)$ | Anytime; no fixed weight; auto-adjusts greediness |
| DPS | `dps.h` | $U_D(n) = (\varepsilon \cdot f_{min} - g(n))\, /\, h(n)$ | Dynamic Potential Search; bounded suboptimal |

**ANA\*** and **DPS** call `rebuild()` every time they find a new incumbent or detect that $f_{min}$ has increased. 

---

### Search Environments

| Environment | States | Heuristic | Notes |
|---|---|---|---|
| Grid (uniform) | $W \times H$ grid | Manhattan distance | 4-connected, 20% obstacles, costs in $U\{1,10\}$ |
| Grid (random/non-uniform) | $W \times H$ grid | Manhattan distance | Sparse h-distribution; primary aggregation target |
| Sliding Tile (standard) | Korf-100 instances | Manhattan distance | 15-puzzle ($4 \times 4$) |
| Sliding Tile (heavy) | Korf-100 instances | Weighted Manhattan | Edge cost = tile number; large, sparse h-values |
| Pancake (standard) | $N=48$ permutations | Gap heuristic | Classic combinatorial; $N-1$ successors |
| Pancake (heavy) | $N=48$ permutations | Weighted gap | Edge cost = max pancake flipped; amplifies sparsity |
| MSA-5 / MSA-6 | Sequence alignment | Gap-based | Multiple sequence alignment; 5 or 6 sequences |

---

## License

MIT License — see `License.md`.
