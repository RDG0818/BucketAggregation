#include <gtest/gtest.h>
#include "queues/bucket_queue.h"
#include <vector>
#include <algorithm>
#include <limits>

class BucketQueueTest : public ::testing::Test {
protected:
  void SetUp() override {
    queue = new BucketQueue(100);
  }

  void TearDown() override {
    delete queue;
  }

  BucketQueue* queue;
};

TEST_F(BucketQueueTest, IsEmptyInitially) {
  EXPECT_TRUE(queue->empty());
}

TEST_F(BucketQueueTest, PushAndEmpty) {
  uint32_t id = 1;
  queue->push(id, 10);
  EXPECT_FALSE(queue->empty());
}

TEST_F(BucketQueueTest, PushAndPopSingle) {
  uint32_t id = 1;
  queue->push(id, 10);
  uint32_t popped_id = queue->pop();
  EXPECT_EQ(popped_id, id);
  EXPECT_TRUE(queue->empty());
}

TEST_F(BucketQueueTest, PushAndPopMultiple) {
  uint32_t h1 = 1; // prio 10
  uint32_t h2 = 2; // prio 20
  uint32_t h3 = 3; // prio 5

  queue->push(h1, 10);
  queue->push(h2, 20);
  queue->push(h3, 5);

  // Should pop lowest priority first
  EXPECT_EQ(queue->pop(), h3); // 5
  EXPECT_EQ(queue->pop(), h1); // 10
  EXPECT_EQ(queue->pop(), h2); // 20
  EXPECT_TRUE(queue->empty());
}

TEST_F(BucketQueueTest, DuplicatePrioritiesLIFO) {
  // Verify LIFO behavior (Stack) for same priority
  uint32_t h1 = 1;
  uint32_t h2 = 2;
  uint32_t h3 = 3;

  queue->push(h1, 10);
  queue->push(h2, 10);
  queue->push(h3, 10);

  // LIFO Order: h3, h2, h1
  EXPECT_EQ(queue->pop(), h3);
  EXPECT_EQ(queue->pop(), h2);
  EXPECT_EQ(queue->pop(), h1);
  
  EXPECT_TRUE(queue->empty());
}

TEST_F(BucketQueueTest, LazyDeletionBehavior) {
  // Replaces DecreaseKey
  uint32_t h1 = 1;

  // Push h1 with cost 20
  queue->push(h1, 20);
  
  // Found better path: Push h1 with cost 5
  queue->push(h1, 5);

  EXPECT_FALSE(queue->empty());
  
  // Should pop the better one (5) first
  EXPECT_EQ(queue->pop(), h1); 
  
  // The queue will still contain the stale entry (20).
  // The *Solver* (A*) handles discarding it by checking G-values.
  // The Queue itself just behaves as a dumb container.
  EXPECT_FALSE(queue->empty());
  
  // Pop the stale entry
  EXPECT_EQ(queue->pop(), h1);
  EXPECT_TRUE(queue->empty());
}

TEST_F(BucketQueueTest, AutoResize) {
  uint32_t h1 = 1;
  uint32_t h2 = 2;

  queue->push(h1, 10);
  
  // This priority is > initial capacity (100)
  // Should trigger resize
  queue->push(h2, 200);

  EXPECT_FALSE(queue->empty());
  EXPECT_EQ(queue->pop(), h1);
  EXPECT_EQ(queue->pop(), h2);
  EXPECT_TRUE(queue->empty());
}

TEST_F(BucketQueueTest, Clear) {
  queue->push(1, 10);
  queue->push(2, 20);
  
  EXPECT_FALSE(queue->empty());
  
  queue->clear();
  
  EXPECT_TRUE(queue->empty());
  EXPECT_EQ(queue->pop(), std::numeric_limits<uint32_t>::max());
}