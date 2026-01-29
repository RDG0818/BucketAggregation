#include <gtest/gtest.h>
#include <limits>
#include <vector>
#include <map>
#include "queues/bucket_heap.h"
#include "environments/node.h"

struct MinCostCalculator {
  double operator()(uint32_t f, uint32_t h_min) const {
    return static_cast<double>(f + h_min);
  }
};

struct GradientCalculator {
  double G_upper = 10000.0;

  double operator()(uint32_t f, uint32_t h_min) const {
    if (h_min == 0) return -std::numeric_limits<double>::max(); // Goal is best
    if (f >= G_upper) return std::numeric_limits<double>::max(); // Prune

    // Maximizing (G - f) / h is equivalent to Minimizing -(G - f) / h
    return -((G_upper - f) / static_cast<double>(h_min));
  }
};

class BucketHeapTest : public ::testing::Test {
protected:
    MinCostCalculator min_calc;
    BucketHeap<MinCostCalculator> heap{min_calc};

    void SetUp() override {
        heap.clear();
    }
};

TEST_F(BucketHeapTest, StartsEmpty) {
    EXPECT_TRUE(heap.empty());
    EXPECT_EQ(heap.pop(), NODE_NULL);
}

TEST_F(BucketHeapTest, SingleBucketPushPop) {

    heap.push(1, 10, 5);
    heap.push(2, 10, 2);

    EXPECT_FALSE(heap.empty());

    // Should pop Node 2 first because it has better H (resulting in better priority)
    EXPECT_EQ(heap.pop(), 2);
    EXPECT_EQ(heap.pop(), 1);
    EXPECT_TRUE(heap.empty());
}

TEST_F(BucketHeapTest, MultiBucketOrdering) {
    heap.push(1, 10, 10);

    heap.push(2, 20, 1);

    heap.push(3, 5, 10);

    EXPECT_EQ(heap.pop(), 3);
    EXPECT_EQ(heap.pop(), 1);
    EXPECT_EQ(heap.pop(), 2);
}

TEST_F(BucketHeapTest, PriorityImprovementUpdatesHeap) {
    heap.push(1, 10, 10);

    heap.push(2, 15, 20);

    
    heap.push(3, 15, 1);

    // The heap should have re-ordered. F=15 should now be top.
    // Within F=15, Node 3 (h=1) should come before Node 2 (h=20).
    
    EXPECT_EQ(heap.pop(), 3); // From F=15
    EXPECT_EQ(heap.pop(), 1); // From F=10 (Pri 20)
    EXPECT_EQ(heap.pop(), 2); // From F=15 (Pri 35)
}

TEST_F(BucketHeapTest, PriorityWorseningOnPop) {
    // Test that when we pop the best H from a bucket, the bucket's priority drops correctly.
    
    // Bucket A: F=10. Contains H=5 (Pri 15) and H=20 (Pri 30).
    heap.push(1, 10, 20);
    heap.push(2, 10, 5);

    // Bucket B: F=20. Contains H=1 (Pri 21).
    heap.push(3, 20, 1);

    // 1. First pop should be Node 2 (F=10, Pri 15).
    EXPECT_EQ(heap.pop(), 2);

    // NOW: Bucket A's best remaining H is 20, so its Priority becomes 10+20=30.
    // Bucket B's Priority is 21.
    // Bucket B should now jump ahead of Bucket A.

    EXPECT_EQ(heap.pop(), 3); // Bucket B
    EXPECT_EQ(heap.pop(), 1); // Bucket A (now worse)
}

TEST_F(BucketHeapTest, BucketExhaustionRemovesFromHeap) {
    heap.push(1, 10, 5);
    heap.push(2, 20, 5);

    EXPECT_EQ(heap.pop(), 1); 
    // F=10 is now empty. It should be removed from the primary heap.
    // If it wasn't, the next pop might try to access it or crash.
    
    EXPECT_EQ(heap.pop(), 2);
    EXPECT_TRUE(heap.empty());
}

// -------------------------------------------------------------------------
// COMPLEX / ALGORITHMIC TESTS
// -------------------------------------------------------------------------

TEST_F(BucketHeapTest, RebuildReshufflesPriorities) {
  // This test simulates ANA* where we change G_upper and rebuild the heap.
  
  GradientCalculator ana_calc;
  BucketHeap<GradientCalculator> ana_heap(ana_calc);

  // Setup:
  // Node A: F=10, H=10. 
  // Node B: F=20, H=2. 
  
  // Initial G_upper = 100.
  // Pri A = -((100 - 10) / 10) = -9.0
  // Pri B = -((100 - 20) / 2)  = -40.0
  // Min-Heap (std::greater) pops smallest. -40 < -9. So B comes first.
  ana_calc.G_upper = 100.0;
  
  ana_heap.push(1, 10, 10);
  ana_heap.push(2, 20, 2);

  // Verify initial order (B first)
  // We assume peek/internal check would show B, but let's change parameters before popping.

  // CHANGE PARAMETERS:
  // Tighten bounds drastically. G_upper = 25.
  // Pri A = -((25 - 10) / 10) = -1.5
  // Pri B = -((25 - 20) / 2)  = -2.5
  // -2.5 < -1.5. B is still better? Wait.
  
  // Let's make A better.
  // G_upper = 30.
  // Pri A = -((30 - 10) / 10) = -2.0
  // Pri B = -((30 - 20) / 2)  = -5.0 (B is still "smaller/better" in negative land... wait)
  
  // Let's look at positive gradients (E):
  // E_A = (30-10)/10 = 2.
  // E_B = (30-20)/2 = 5.
  // ANA* pops LARGEST E. 
  // In our Min-Heap of NEGATIVES, we pop SMALLEST (-E).
  // -5 is smaller than -2. So B comes out (E=5). Correct.
  
  // To flip it, we need A to have higher gradient.
  // G_upper = 1000.
  // E_A = (1000 - 10)/10 = 99.
  // E_B = (1000 - 20)/2 = 490.
  // B is very strong. Low H makes high gradient.
  
  // Use simple math calculator to test Rebuild mechanics clearly.
  struct RebuildCalc {
      double multiplier = 1.0;
      double operator()(uint32_t f, uint32_t) const { return f * multiplier; }
  } simple_calc;
  
  BucketHeap<RebuildCalc> test_heap(simple_calc);
  
  test_heap.push(1, 10, 1); // Pri = 10
  test_heap.push(2, 20, 1); // Pri = 20
  
  // Normal order: 1 then 2.
  
  // Flip order by making multiplier negative (simulating huge priority shift)
  simple_calc.multiplier = -1.0;
  test_heap.rebuild();
  
  // Now:
  // 1 -> -10
  // 2 -> -20 (Smaller, should come first)
  
  EXPECT_EQ(test_heap.pop(), 2);
  EXPECT_EQ(test_heap.pop(), 1);
}

TEST_F(BucketHeapTest, InterleavedPushPop) {
  heap.push(1, 10, 10);
  heap.push(2, 10, 5);  // Improves F=10 priority
  heap.pop();           // Pops 2. F=10 degrades to H=10.
  
  heap.push(3, 8, 20);  // New F=8 bucket. Pri=28. (Worse than F=10, Pri=20)
  
  EXPECT_EQ(heap.pop(), 1);
  
  EXPECT_EQ(heap.pop(), 3);
}

TEST_F(BucketHeapTest, ClearResetsEverything) {
  heap.push(1, 10, 5);
  heap.clear();
  
  EXPECT_TRUE(heap.empty());
  
  heap.push(2, 20, 5);
  EXPECT_EQ(heap.pop(), 2);
}