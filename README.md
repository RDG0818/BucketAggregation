# Bucket Aggregation in Bucket-Based Priority Queues for Heuristic Search

Companion implementation for [Bucket Aggregation in Bucket-Based Priority Queues for Heuristic Search](docs/GoodwinSoCS26.pdf), extending [Fereday & Hansen (SoCS 2025)](https://ojs.aaai.org/index.php/SOCS/article/view/35976).

## Build & Run

Requires a C++20 compiler, CMake, and Make.

```bash
git submodule update --init --recursive
make
make rebuild
./build/main
./build/tests
```

Run `./build/main -h` to see all available CLI options.

To reproduce the paper's aggregation sweeps, pass per-domain `alpha`/`beta`/`D` ranges and `--metrics`, e.g.:

```bash
./build/main -e grid -l anastar_agg_bucket --grid-alphas 1,25,50,100,200 --grid-betas 1 --grid-ds 2 --metrics -o results.csv
```

On memory-constrained systems, use `--capacity N` to cap the node pool size (default: 1,000,000). The pool grows dynamically as needed.

```bash
./build/main --capacity 500000
```

For hardware profiling, page faults are available unconditionally. Hardware counters require `kernel.perf_event_paranoid <= 1`:

```bash
sudo sysctl kernel.perf_event_paranoid=1
```

## Features

- **Priority Queues**: binary heap, bucket queue, two-level bucket queue, indexed d-ary heap, and a hybrid bucket heap combining the two
- **Search Algorithms**: A*, Weighted/Anytime A*, ANA*, and Dynamic Potential Search
- **Environments**: uniform and non-uniform grids, sliding tile puzzles (standard and edge-weighted), pancake sorting (standard and edge-weighted), and multiple sequence alignment

## The Paper

Bucket-based priority queues are highly efficient open lists for heuristic search, but their performance degrades when the h-cost distribution is sparse, or when nodes cluster at a small number of distinct h-values spread over a wide range. This leads to many empty secondary buckets, wasting memory and causing cache misses during reordering.

We introduce bucket aggregation: grouping h-costs into intervals of width α so that node $n$ is placed in secondary bucket $\lfloor h(n)/\alpha \rfloor$ instead of bucket $h(n)$. A corresponding parameter $\beta$ aggregates primary (f-cost) buckets. Full derivation and correctness proofs are in the paper.

Priority queue overhead (in milliseconds) for A* (above) and ANA* (below) using a binary heap, a standard bucket queue ($\alpha$ = 1), and a bucket queue/heap with secondary bucket aggregation based on the indicated aggregation factor $\alpha$.

![Priority Queue Overhead Graph](images/combined_graph.png "Priority Queue Overhead")

## Citation

```bibtex
@inproceedings{goodwin2026bucket,
  title     = {Bucket Aggregation in Bucket-Based Priority Queues for Heuristic Search},
  author    = {Goodwin, Ryan D. and Hansen, Eric A. and Fereday, Garrett M.},
  booktitle = {Proceedings of the 19th International Symposium on Combinatorial Search (SoCS 2026)},
  year      = {2026},
  note      = {To appear}
}
```

## License

MIT License — see `LICENSE`.
