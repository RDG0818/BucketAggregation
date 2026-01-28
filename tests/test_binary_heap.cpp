#include <gtest/gtest.h>
#include "queues/binary_heap.h"
#include "environments/node.h" // Only needed for Rebuild test simulation
#include <vector>
#include <algorithm>
#include <limits>

class BinaryHeapTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Defaulting to a min-heap of ints
    heap = new BinaryHeap<int, std::greater<int>>();
  }

  void TearDown() override {
    delete heap;
  }

  BinaryHeap<int, std::greater<int>>* heap;
};

TEST_F(BinaryHeapTest, IsEmptyInitially) {
  EXPECT_TRUE(heap->empty());
}

TEST_F(BinaryHeapTest, PushAndEmpty) {
  heap->push(1, 10);
  EXPECT_FALSE(heap->empty());
}

TEST_F(BinaryHeapTest, PushAndPopSingle) {
  uint32_t id = 5;
  heap->push(id, 10);
  uint32_t popped_id = heap->pop();
  EXPECT_EQ(popped_id, id);
  EXPECT_TRUE(heap->empty());
}

TEST_F(BinaryHeapTest, PushAndPopMultiple) {
  uint32_t h1 = 1;
  uint32_t h2 = 2;
  uint32_t h3 = 3;

  heap->push(h2, 20);
  heap->push(h3, 30);
  heap->push(h1, 10);

  // Min-Heap: 10, 20, 30
  EXPECT_EQ(heap->pop(), h1);
  EXPECT_EQ(heap->pop(), h2);
  EXPECT_EQ(heap->pop(), h3);
  EXPECT_TRUE(heap->empty());
}

TEST_F(BinaryHeapTest, LazyDeletionBehavior) {
  uint32_t id = 1;
  
  // Push same ID with different priorities
  heap->push(id, 30); // Old bad path
  heap->push(id, 10); // New better path

  // Should pop the better one first
  EXPECT_EQ(heap->pop(), id); // priority 10
  
  EXPECT_EQ(heap->pop(), id); // priority 30
  EXPECT_TRUE(heap->empty());
}

TEST_F(BinaryHeapTest, DuplicatePriorities) {
  uint32_t h1 = 1;
  uint32_t h2 = 2;
  uint32_t h3 = 3;

  heap->push(h1, 20);
  heap->push(h2, 10);
  heap->push(h3, 10);

  std::vector<uint32_t> popped;
  popped.push_back(heap->pop());
  popped.push_back(heap->pop());
  
  // Check that h2 and h3 are popped before h1
  bool h2_found = (popped[0] == h2 || popped[1] == h2);
  bool h3_found = (popped[0] == h3 || popped[1] == h3);
  
  EXPECT_TRUE(h2_found && h3_found);
  EXPECT_EQ(heap->pop(), h1);
}

TEST(BinaryHeapMaxHeapTest, PushAndPopMultiple) {
  // std::less creates a Max-Heap because our HeapItemCompare logic uses Compare()(a, b)
  // If Compare is std::less, it returns true if a < b.
  // In STL heap, that puts the "largest" element at the top.
  BinaryHeap<double, std::less<double>> max_heap;

  uint32_t h1 = 1;
  uint32_t h2 = 2;
  uint32_t h3 = 3;

  max_heap.push(h2, 20.5);
  max_heap.push(h3, 5.5);
  max_heap.push(h1, 30.1);

  EXPECT_EQ(max_heap.pop(), h1); // 30.1 (Highest)
  EXPECT_EQ(max_heap.pop(), h2); // 20.5
  EXPECT_EQ(max_heap.pop(), h3); // 5.5
  EXPECT_TRUE(max_heap.empty());
}

// --- Test Rebuild Feature ---

TEST(BinaryHeapRebuildTest, RebuildsCorrectlyForMaxHeap) {
  // Simulate Data Store
  struct MockNode { double g; double h; };
  std::vector<MockNode> nodes(100);
  
  nodes[1] = {10, 20}; // h1
  nodes[2] = {30, 10}; // h2
  nodes[3] = {50, 25}; // h3
  
  BinaryHeap<double, std::less<double>> heap; // Max Heap

  double g_upper = 100.0;
  
  // Initial priorities E = (G - g) / h
  // h1: (100-10)/20 = 4.5
  // h2: (100-30)/10 = 7.0
  // h3: (100-50)/25 = 2.0
  
  heap.push(1, 4.5);
  heap.push(2, 7.0);
  heap.push(3, 2.0);

  // Initial Max: h2 (7.0)
  EXPECT_EQ(heap.top(), 2);

  // --- Trigger Rebuild ---
  double new_g_upper = 50.0;
  
  // New E:
  // h1: (50-10)/20 = 2.0
  // h2: (50-30)/10 = 2.0
  // h3: (50-50)/25 = 0.0
  
  auto calculator = [&](uint32_t id) -> double {
      return (new_g_upper - nodes[id].g) / nodes[id].h;
  };
  
  heap.rebuild(calculator);

  // New Max should be h1 or h2 (tie at 2.0), h3 is last (0.0)
  
  std::vector<uint32_t> popped;
  popped.push_back(heap.pop());
  popped.push_back(heap.pop());

  EXPECT_TRUE((popped[0] == 1 && popped[1] == 2) || (popped[0] == 2 && popped[1] == 1));
  EXPECT_EQ(heap.pop(), 3);
}