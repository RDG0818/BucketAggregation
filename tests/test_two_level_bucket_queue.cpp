#include <gtest/gtest.h>
#include "queues/two_level_bucket_queue.h"
#include <vector>
#include <algorithm>
#include <limits>

class TwoLevelBucketQueueTest : public ::testing::Test {
protected:
  void SetUp() override {
    queue = new TwoLevelBucketQueue(1, 1, 100);
  }

  void TearDown() override {
    delete queue;
  }

  TwoLevelBucketQueue* queue;
};

TEST_F(TwoLevelBucketQueueTest, IsEmptyInitially) {
  EXPECT_TRUE(queue->empty());
  EXPECT_EQ(queue->pop(), std::numeric_limits<uint32_t>::max());
}

TEST_F(TwoLevelBucketQueueTest, PushAndPopSingle) {
  queue->push(1, 10, 5); // id=1, f=10, h=5
  EXPECT_FALSE(queue->empty());
  EXPECT_EQ(queue->pop(), 1);
  EXPECT_TRUE(queue->empty());
}

TEST_F(TwoLevelBucketQueueTest, TieBreakingByH) {
  // Three nodes with SAME F-cost (10), but DIFFERENT H-costs
  uint32_t n1 = 1; // f=10, h=8
  uint32_t n2 = 2; // f=10, h=2 (Best H)
  uint32_t n3 = 3; // f=10, h=5

  queue->push(n1, 10, 8);
  queue->push(n2, 10, 2);
  queue->push(n3, 10, 5);

  // Should pop n2 first (lowest H), then n3, then n1
  EXPECT_EQ(queue->pop(), n2);
  EXPECT_EQ(queue->pop(), n3);
  EXPECT_EQ(queue->pop(), n1);
  EXPECT_TRUE(queue->empty());
}

TEST_F(TwoLevelBucketQueueTest, MixedPriorities) {
  // Nodes with different F costs
  uint32_t n1 = 1; // f=20, h=5
  uint32_t n2 = 2; // f=10, h=5 (Best F)
  
  queue->push(n1, 20, 5);
  queue->push(n2, 10, 5);

  // Should pop lowest F first
  EXPECT_EQ(queue->pop(), n2);
  EXPECT_EQ(queue->pop(), n1);
}

TEST_F(TwoLevelBucketQueueTest, LIFOInsideSameFH) {
  // If F and H are identical, it should behave as a LIFO stack
  uint32_t n1 = 1;
  uint32_t n2 = 2;
  uint32_t n3 = 3;

  // All F=10, H=5
  queue->push(n1, 10, 5);
  queue->push(n2, 10, 5);
  queue->push(n3, 10, 5);

  EXPECT_EQ(queue->pop(), n3);
  EXPECT_EQ(queue->pop(), n2);
  EXPECT_EQ(queue->pop(), n1);
}

TEST_F(TwoLevelBucketQueueTest, AutoResizeF) {
  // Push beyond initial F capacity
  uint32_t n1 = 1;
  queue->push(n1, 2000, 5); // f=2000 > 100

  EXPECT_FALSE(queue->empty());
  EXPECT_EQ(queue->pop(), n1);
}

TEST_F(TwoLevelBucketQueueTest, AutoResizeH) {
  // Push beyond initial H capacity (vector usually starts small)
  // We assume the inner H-vector grows as needed.
  uint32_t n1 = 1;
  queue->push(n1, 10, 5000); // f=10, h=5000

  EXPECT_EQ(queue->pop(), n1);
}

TEST_F(TwoLevelBucketQueueTest, LazyDeletionBehavior) {
  // Simulate updating a node's priority
  uint32_t n1 = 1;

  // Old path: f=20, h=10
  queue->push(n1, 20, 10);
  
  // New better path: f=15, h=5
  queue->push(n1, 15, 5);

  // Should pop the better one first
  EXPECT_EQ(queue->pop(), n1); // 15
  
  // Stale one remains
  EXPECT_EQ(queue->pop(), n1); // 20
  EXPECT_TRUE(queue->empty());
}

TEST_F(TwoLevelBucketQueueTest, Clear) {
  queue->push(1, 10, 5);
  queue->push(2, 20, 10);
  
  queue->clear();
  
  EXPECT_TRUE(queue->empty());
  EXPECT_EQ(queue->pop(), std::numeric_limits<uint32_t>::max());
}

TEST_F(TwoLevelBucketQueueTest, EmptySkipLogic) {
  // Test that pop correctly skips empty buckets to find the next min
  queue->push(1, 10, 5);
  queue->push(2, 100, 5); 
  
  EXPECT_EQ(queue->pop(), 1);
  // Queue now has a huge gap between 10 and 100.
  // Pop needs to scan forward.
  EXPECT_EQ(queue->pop(), 2);
}

TEST_F(TwoLevelBucketQueueTest, PopFromSpecificBucket) {

  uint32_t n_f10_h5 = 1;
  uint32_t n_f10_h2 = 2; // Best in F=10 bucket
  uint32_t n_f20_h1 = 3; // Best in F=20 bucket (but worse globally by F)

  queue->push(n_f10_h5, 10, 5);
  queue->push(n_f10_h2, 10, 2);
  queue->push(n_f20_h1, 20, 1);

  EXPECT_EQ(queue->pop_from(20), n_f20_h1);

  EXPECT_EQ(queue->pop_from(20), std::numeric_limits<uint32_t>::max());

  EXPECT_EQ(queue->pop_from(10), n_f10_h2);
  EXPECT_EQ(queue->pop_from(10), n_f10_h5);

  EXPECT_EQ(queue->pop_from(10), std::numeric_limits<uint32_t>::max());

  EXPECT_TRUE(queue->empty());

  EXPECT_EQ(queue->pop_from(999), std::numeric_limits<uint32_t>::max());
}