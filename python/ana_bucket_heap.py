import heapq
from typing import Optional
import math
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

class ANABucketHeap:
    """
    A simplified Bucket Heap implementation tailored specifically for the
    Anytime Non-parametric A* (ANA*) algorithm.
    """

    def __init__(self):
        """Initializes the ANABucketHeap."""
        self.incumbent_cost = math.inf
        
        # The "control layer": A min-heap storing tuples of (-potential, min_h, f_cost).
        self._heap = []

        # The "foundation": A two-level dictionary for the bucket queue.
        self._buckets = defaultdict(lambda: defaultdict(list))

        # Tracks metadata for each f-bucket: {f_cost: (best_potential, min_h)}
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
        f = math.ceil(node.f_score)
        h = math.ceil(node.h_score)

        # Add the node to the internal bucket structure
        self._buckets[f][h].append(node)
        self._num_nodes += 1

        # Adding a node can change the bucket's best potential or its min_h.
        # The most robust way to ensure the heap is correct is to trigger a
        # recalculation for the affected bucket.
        self._recalculate_and_update_bucket(f)

    def pop(self) -> Optional[SearchNode]:
        """
        Pops the node with the highest potential, using min_h as a tie-breaker.
        This is the standard behavior for ANA*.
        """
        if not self._heap:
            return None

        while self._heap:
            neg_potential, min_h, f_cost = heapq.heappop(self._heap)
            potential = -neg_potential

            # Staleness check: if the popped metadata is worse than the current
            # metadata for this bucket, it's stale, so skip it.
            # We compare (-potential, min_h) with (-current_potential, current_min_h)
            current_potential, current_min_h = self._bucket_metadata.get(f_cost, (-math.inf, math.inf))
            if (-potential, min_h) < (-current_potential, current_min_h):
                continue
            
            # Check if the bucket is actually empty
            if f_cost not in self._buckets or not self._buckets[f_cost]:
                if f_cost in self._bucket_metadata:
                    del self._bucket_metadata[f_cost]
                continue

            # The actual min_h in the bucket might have changed since the
            # heap entry was created. We always pop from the current min_h.
            min_h_in_bucket = min(self._buckets[f_cost].keys())
            
            # Remove the node from the underlying bucket structure
            bucket = self._buckets[f_cost][min_h_in_bucket]
            node = bucket.pop()
            self._num_nodes -= 1

            if not bucket:
                del self._buckets[f_cost][min_h_in_bucket]
                if not self._buckets[f_cost]:
                    del self._buckets[f_cost]
                    # Bucket is fully empty, remove metadata
                    if f_cost in self._bucket_metadata:
                        del self._bucket_metadata[f_cost]
            
            # After popping, the bucket's metadata might need an update.
            # Recalculate and push the new metadata to the heap.
            self._recalculate_and_update_bucket(f_cost)
            
            return node
        
        return None

    def _recalculate_and_update_bucket(self, f_cost):
        """
        After a node is removed, find the new best potential and min_h in an
        f-bucket and update the heap with the new metadata.
        """
        if f_cost not in self._buckets or not self._buckets[f_cost]:
            # The bucket was completely emptied, so nothing to do.
            if f_cost in self._bucket_metadata:
                del self._bucket_metadata[f_cost]
            return

        new_best_potential = -math.inf
        min_h_in_bucket = min(self._buckets[f_cost].keys())

        for h_c, nodes in self._buckets[f_cost].items():
            for node in nodes:
                potential = self._calculate_potential(node.g_score, node.h_score)
                if potential > new_best_potential:
                    new_best_potential = potential
        
        if new_best_potential > -math.inf:
            self._bucket_metadata[f_cost] = (new_best_potential, min_h_in_bucket)
            heapq.heappush(self._heap, (-new_best_potential, min_h_in_bucket, f_cost))


    def update_incumbent(self, new_incumbent_cost):
        """
        Updates the incumbent cost 'C' and re-calculates the potential and min_h
        for all buckets, completely rebuilding the heap for correct ordering.
        """
        self.incumbent_cost = new_incumbent_cost
        
        self._heap = []
        self._bucket_metadata = {}
        for f_cost, h_dict in self._buckets.items():
            if not h_dict: continue # Skip empty f-buckets

            best_potential_in_bucket = -math.inf
            min_h_in_bucket = min(h_dict.keys())

            for h_cost, nodes in h_dict.items():
                for node in nodes:
                    potential = self._calculate_potential(node.g_score, node.h_score)
                    if potential > best_potential_in_bucket:
                        best_potential_in_bucket = potential
            
            if best_potential_in_bucket > -math.inf:
                self._bucket_metadata[f_cost] = (best_potential_in_bucket, min_h_in_bucket)
                heapq.heappush(self._heap, (-best_potential_in_bucket, min_h_in_bucket, f_cost))
