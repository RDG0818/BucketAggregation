#include <gtest/gtest.h>
#include <vector>
#include <map>
#include "queues/indexed_binary_heap.h" 

class IndexedBinaryHeapTest : public ::testing::Test {
protected:
    IndexedBinaryHeap<double> heap;

    void SetUp() override {
        heap.clear();
    }
};

TEST_F(IndexedBinaryHeapTest, BasicPushPop) {
  heap.push(1, 10.0);
  heap.push(2, 5.0);
  heap.push(3, 20.0);

  ASSERT_FALSE(heap.empty());
  
  // Expect 5.0 (ID 2) -> 10.0 (ID 1) -> 20.0 (ID 3)
  EXPECT_EQ(heap.pop(), 2); 
  EXPECT_EQ(heap.pop(), 1); 
  EXPECT_EQ(heap.pop(), 3); 
  
  EXPECT_TRUE(heap.empty());
  EXPECT_EQ(heap.pop(), NODE_NULL);
}

TEST_F(IndexedBinaryHeapTest, UpdatesExistingNodes) {
  heap.push(1, 10.0);
  heap.push(2, 20.0);

  // Update ID 1 to be worse (increase key)
  heap.push(1, 30.0); 

  // Update ID 2 to be better (decrease key)
  heap.push(2, 5.0);  

  // Heap should NOT contain duplicates.
  // Order should be: ID 2 (5.0), ID 1 (30.0)
  EXPECT_EQ(heap.pop(), 2);
  EXPECT_EQ(heap.pop(), 1);
  EXPECT_TRUE(heap.empty()); 
}

TEST_F(IndexedBinaryHeapTest, ChangePriority) {
  heap.push(1, 10.0);
  heap.push(2, 20.0);

  // Decrease Key
  heap.change_priority(2, 5.0);
  EXPECT_EQ(heap.pop(), 2); // Should come out first now

  // Increase Key (on remaining node)
  heap.change_priority(1, 50.0);
  EXPECT_EQ(heap.pop(), 1);
}

TEST_F(IndexedBinaryHeapTest, RemoveNode) {
  heap.push(1, 10.0);
  heap.push(2, 20.0);
  heap.push(3, 30.0);

  // Remove the middle element
  heap.remove(2);

  EXPECT_FALSE(heap.contains(2));
  EXPECT_EQ(heap.pop(), 1);
  EXPECT_EQ(heap.pop(), 3);
}

TEST_F(IndexedBinaryHeapTest, Rebuild) {
  heap.push(1, 10.0);
  heap.push(2, 20.0);
  heap.push(3, 30.0);

  // Define a new scoring function (e.g., ANA* changing G_upper)
  // Let's reverse the priorities: 1->100, 2->50, 3->10
  std::map<uint32_t, double> new_scores;
  new_scores[1] = 100.0;
  new_scores[2] = 50.0;
  new_scores[3] = 10.0;

  auto calculator = [&](uint32_t id) {
    return new_scores[id];
  };

  heap.rebuild(calculator);

  // New order should be 3 (10.0) -> 2 (50.0) -> 1 (100.0)
  EXPECT_EQ(heap.pop(), 3);
  EXPECT_EQ(heap.pop(), 2);
  EXPECT_EQ(heap.pop(), 1);
}

TEST_F(IndexedBinaryHeapTest, LargeScale) {
  int N = 1000;
  for (int i = 0; i < N; ++i) {
    heap.push(i, (double)(N - i));
  }

  uint32_t last_id = -1;
  for (int i = 0; i < N; ++i) {
    uint32_t id = heap.pop();
    if (i > 0) {
      EXPECT_LT(id, last_id);
    }
    last_id = id;
  }
}

TEST(IndexedMaxHeapTest, MaxHeapBehavior) {
  // Pass std::less to make it a Max Heap
  IndexedBinaryHeap<double, std::less<double>> max_heap;

  max_heap.push(1, 10.0);
  max_heap.push(2, 50.0); // Should be top
  max_heap.push(3, 20.0);

  EXPECT_EQ(max_heap.pop(), 2); // 50.0
  EXPECT_EQ(max_heap.pop(), 3); // 20.0
  EXPECT_EQ(max_heap.pop(), 1); // 10.0
}