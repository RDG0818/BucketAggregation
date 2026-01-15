import heapq
import math
from typing import Optional
from collections import defaultdict

class SearchNode:
    """A simple representation of a search node for the demo."""
    def __init__(self, state, g_score=math.inf, h_score=math.inf):
        self.state = state
        self.g_score = g_score
        self.h_score = h_score

    @property
    def f_score(self):
        return self.g_score + self.h_score

    def __lt__(self, other):
        return self.f_score < other.f_score

class ANABucketHeapReal:
    """
    A simplified Bucket Heap implementation tailored specifically for the
    Anytime Non-parametric A* (ANA*) algorithm.
    """

    def __init__(self, alpha=1.0, beta=1.0):
        """Initializes the ANABucketHeap."""
        # The cost of the best solution found so far (the incumbent 'C').
        self.incumbent_cost = math.inf
        self.alpha = alpha
        self.beta = beta
        
        # The "control layer": A min-heap storing tuples of (-potential, min_h_key, f_key).
        self._heap = []

        # The "foundation": A two-level dictionary for the bucket queue.
        # self._buckets[f_key][h_key] = [node1, node2, ...]
        self._buckets = defaultdict(lambda: defaultdict(list))

        # Tracks metadata for each f-bucket: {f_key: (best_potential, min_h_key)}
        self._bucket_metadata = {}
        
        self._num_nodes = 0

    def __len__(self):
        return self._num_nodes

    def _calculate_potential(self, g, h):
        """The hardcoded potential function for ANA*."""
        if h == 0:  # Avoid division by zero at the goal
            return math.inf
        # If g >= C, potential is <= 0 and the node would be pruned anyway.
        return (self.incumbent_cost - g) / h

    def push(self, node: SearchNode):
        """
        Adds a node to the priority queue and updates the bucket's metadata
        on the heap.
        """
        f_key = math.floor(node.f_score / self.alpha)
        h_key = math.floor(node.h_score / self.beta)

        # Add the node to the internal bucket structure
        self._buckets[f_key][h_key].append(node)
        self._num_nodes += 1

        # Adding a node can change the bucket's best potential or its min_h_key.
        # The most robust way to ensure the heap is correct is to trigger a
        # recalculation for the affected bucket.
        self._recalculate_and_update_bucket(f_key)

    def pop(self) -> Optional[SearchNode]:
        """
        Pops the node with the highest potential, using min_h_key as a tie-breaker.
        This is the standard behavior for ANA*.
        """
        if not self._heap:
            return None

        while self._heap:
            neg_potential, min_h_key, f_key = heapq.heappop(self._heap)
            potential = -neg_potential

            # Staleness check: if the popped metadata is worse than the current
            # metadata for this bucket, it's stale, so skip it.
            current_potential, current_min_h = self._bucket_metadata.get(f_key, (-math.inf, math.inf))
            if (-potential, min_h_key) < (-current_potential, current_min_h):
                continue
            
            # Check if the bucket is actually empty
            if f_key not in self._buckets or not self._buckets[f_key]:
                if f_key in self._bucket_metadata:
                    del self._bucket_metadata[f_key]
                continue

            # The actual min_h_key in the bucket might have changed since the
            # heap entry was created. We always pop from the current min_h_key.
            min_h_key_in_bucket = min(self._buckets[f_key].keys())
            
            # Remove the node from the underlying bucket structure
            bucket = self._buckets[f_key][min_h_key_in_bucket]
            node = bucket.pop()
            self._num_nodes -= 1

            if not bucket:
                del self._buckets[f_key][min_h_key_in_bucket]
                if not self._buckets[f_key]:
                    del self._buckets[f_key]
                    # Bucket is fully empty, remove metadata
                    if f_key in self._bucket_metadata:
                        del self._bucket_metadata[f_key]
            
            # After popping, the bucket's metadata might need an update.
            self._recalculate_and_update_bucket(f_key)
            
            return node
        
        return None

    def _recalculate_and_update_bucket(self, f_key):
        """
        After a node is removed, find the new best potential and min_h_key in an
        f-bucket and update the heap with the new metadata.
        """
        if f_key not in self._buckets or not self._buckets[f_key]:
            # The bucket was completely emptied, so nothing to do.
            if f_key in self._bucket_metadata:
                del self._bucket_metadata[f_key]
            return

        new_best_potential = -math.inf
        min_h_key_in_bucket = min(self._buckets[f_key].keys())

        for h_key, nodes in self._buckets[f_key].items():
            for node in nodes:
                potential = self._calculate_potential(node.g_score, node.h_score)
                if potential > new_best_potential:
                    new_best_potential = potential
        
        if new_best_potential > -math.inf:
            self._bucket_metadata[f_key] = (new_best_potential, min_h_key_in_bucket)
            heapq.heappush(self._heap, (-new_best_potential, min_h_key_in_bucket, f_key))

    def update_incumbent(self, new_incumbent_cost):
        """
        Updates the incumbent cost 'C' and re-calculates the potential and min_h_key
        for all buckets, completely rebuilding the heap for correct ordering.
        """
        self.incumbent_cost = new_incumbent_cost
        
        self._heap = []
        self._bucket_metadata = {}
        for f_key, h_dict in self._buckets.items():
            if not h_dict: continue # Skip empty f-buckets

            best_potential_in_bucket = -math.inf
            min_h_key_in_bucket = min(h_dict.keys())

            for h_key, nodes in h_dict.items():
                for node in nodes:
                    potential = self._calculate_potential(node.g_score, node.h_score)
                    if potential > best_potential_in_bucket:
                        best_potential_in_bucket = potential
            
            if best_potential_in_bucket > -math.inf:
                self._bucket_metadata[f_key] = (best_potential_in_bucket, min_h_key_in_bucket)
                heapq.heappush(self._heap, (-best_potential_in_bucket, min_h_key_in_bucket, f_key))


if __name__ == '__main__':
    # --- DEMO ---
    # 1. Initialize the ANABucketHeapReal with alpha and beta values.
    # alpha defines the bucket width for f-costs, beta for h-costs.
    pq = ANABucketHeapReal(alpha=10.0, beta=10.0)
    print(f"Initialized ANABucketHeapReal with alpha={pq.alpha}, beta={pq.beta}")


    # 2. Set the initial incumbent cost and add nodes.
    pq.incumbent_cost = 100
    
    # f-scores: 90, 95, 90. With alpha=10, all go into bucket floor(9x/10) = 9.
    # h-scores: 80, 75, 60. With beta=10, they go into buckets 8, 7, and 6.
    nodes = [
        SearchNode("A", g_score=10, h_score=80), # f=90, u=(100-10)/80 = 1.125
        SearchNode("B", g_score=20, h_score=75), # f=95, u=(100-20)/75 = 1.066
        SearchNode("C", g_score=30, h_score=60), # f=90, u=(100-30)/60 = 1.166
    ]
    for n in nodes:
        pq.push(n)

    print(f"\nQueue has {len(pq)} nodes.")
    print("Buckets structure:", dict(pq._buckets))
    print("-" * 20)

    # 3. Pop a node. All nodes are in f-key bucket 9.
    # The highest potential is C's (u=1.166).
    # It's in h-key bucket 6, which is the lowest h-key, so it gets popped.
    node = pq.pop()
    print(f"Popped: Node {node.state} (g={node.g_score}, h={node.h_score}, u={pq._calculate_potential(node.g_score, node.h_score):.3f})")
    print(f"Queue now has {len(pq)} nodes.")
    print("-" * 20)

    # 4. A better solution is found! Update the incumbent cost.
    print("Found a better solution! Updating incumbent cost to 92.")
    pq.update_incumbent(92)
    print("-" * 20)
    # The potentials of the remaining nodes (A and B) are automatically recalculated.
    # New potentials:
    # A: u=(92-10)/80 = 1.025
    # B: u=(92-20)/75 = 0.96

    # 5. Pop the next node. It should be "A" (new highest potential u=1.025).
    node = pq.pop()
    print(f"Popped: Node {node.state} (g={node.g_score}, h={node.h_score}, u={pq._calculate_potential(node.g_score, node.h_score):.3f})")
    print(f"Queue now has {len(pq)} nodes.")
