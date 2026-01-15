import math

class BinaryHeap:

    def __init__(self, potential_func):
        """
        Initializes the BinaryHeap.

        Args:
            potential_func (callable): A function that takes (g, h, C) and
                                       returns the node's potential.
        """
        self._heap = []
        self.potential_func = potential_func
        self.incumbent_cost = math.inf

    def __len__(self):
        return len(self._heap)

    def push(self, node):
        """
        Adds a node to the heap in O(log N) time.
        The item stored is a tuple: (-potential, h_score, node).
        """
        potential = self.potential_func(node.g_score, node.h_score, self.incumbent_cost)
        self._heap.append((-potential, node.h_score, node))
        self._sift_up(len(self._heap) - 1)

    def pop(self):
        """
        Removes and returns the node with the highest potential in O(log N) time.
        """
        if not self._heap:
            return None

        self._swap(0, len(self._heap) - 1)
        _, _, node = self._heap.pop()

        if self._heap:
            self._sift_down(0)
            
        return node

    def rebuild(self):
        """
        Re-calculates all potentials and rebuilds the heap from scratch in O(N) time.
        """
        new_heap_list = []
        # Iterate over a copy of the heap's items, as it will be replaced
        for _, _, node in list(self._heap):
            potential = self.potential_func(node.g_score, node.h_score, self.incumbent_cost)
            new_heap_list.append((-potential, node.h_score, node))
        
        self._heap = new_heap_list
        self._build_heap() # O(N) heapify operation

    def update_incumbent(self, new_incumbent_cost):
        """Updates the incumbent and triggers a full heap rebuild."""
        self.incumbent_cost = new_incumbent_cost
        self.rebuild()

    # --- Internal Heap Helper Methods ---

    def _swap(self, i, j):
        self._heap[i], self._heap[j] = self._heap[j], self._heap[i]

    def _sift_up(self, i):
        parent = (i - 1) // 2
        # Corrected: Compare the entire tuple for lexicographical ordering
        while i > 0 and self._heap[i] < self._heap[parent]:
            self._swap(i, parent)
            i = parent
            parent = (i - 1) // 2

    def _sift_down(self, i):
        size = len(self._heap)
        while True:
            left = 2 * i + 1
            right = 2 * i + 2
            smallest = i

            # Corrected: Compare the entire tuple for lexicographical ordering
            if left < size and self._heap[left] < self._heap[smallest]:
                smallest = left
            
            # Corrected: Compare the entire tuple for lexicographical ordering
            if right < size and self._heap[right] < self._heap[smallest]:
                smallest = right
            
            if smallest == i:
                break
            
            self._swap(i, smallest)
            i = smallest

    def _build_heap(self):
        """Builds a heap from an existing list in O(N) time."""
        n = len(self._heap)
        for i in range(n // 2 - 1, -1, -1):
            self._sift_down(i)
