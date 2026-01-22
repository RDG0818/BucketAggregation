#include <gtest/gtest.h>
#include "queues/two_level_bucket_queue.h"
#include "environments/node.h"
#include <vector>
#include <algorithm>

class TwoLevelBucketQueueTest : public ::testing::Test {
protected:
    void SetUp() override {
        pool.reserve(100);
        queue = new TwoLevelBucketQueue(pool);
    }

    void TearDown() override {
        delete queue;
    }

    NodePool pool;
    TwoLevelBucketQueue* queue;
};

TEST_F(TwoLevelBucketQueueTest, IsEmptyInitially) {
    EXPECT_TRUE(queue->empty());
}

TEST_F(TwoLevelBucketQueueTest, PushAndEmpty) {
    uint32_t handle = pool.allocate(NODE_NULL, 0, 0);
    queue->push(handle, 10);
    EXPECT_FALSE(queue->empty());
}

TEST_F(TwoLevelBucketQueueTest, PushAndPopSingle) {
    uint32_t handle = pool.allocate(NODE_NULL, 0, 0);
    queue->push(handle, 10);
    uint32_t popped_handle = queue->pop();
    EXPECT_EQ(popped_handle, handle);
    EXPECT_TRUE(queue->empty());
}

TEST_F(TwoLevelBucketQueueTest, PushAndPopMultiple) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0); // prio 10
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0); // prio 20
    uint32_t h3 = pool.allocate(NODE_NULL, 0, 0); // prio 5

    queue->push(h1, 10);
    queue->push(h2, 20);
    queue->push(h3, 5);

    EXPECT_EQ(queue->pop(), h3);
    EXPECT_EQ(queue->pop(), h1);
    EXPECT_EQ(queue->pop(), h2);
    EXPECT_TRUE(queue->empty());
}

TEST_F(TwoLevelBucketQueueTest, Contains) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);

    queue->push(h1, 10);

    EXPECT_TRUE(queue->contains(h1));
    EXPECT_FALSE(queue->contains(h2));

    queue->pop(); // h1 is popped
    EXPECT_FALSE(queue->contains(h1));
}

TEST_F(TwoLevelBucketQueueTest, DecreaseKey) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h3 = pool.allocate(NODE_NULL, 0, 0);

    queue->push(h1, 10);
    queue->push(h2, 20);
    queue->push(h3, 30);

    // Now, decrease key for h3 to 5. This leaves a stale entry at 30.
    queue->decrease_key(h3, 5);

    EXPECT_FALSE(queue->empty());
    EXPECT_EQ(queue->pop(), h3); // Should pop h3 first (prio 5)
    EXPECT_EQ(queue->pop(), h1); // prio 10
    EXPECT_EQ(queue->pop(), h2); // prio 20
    
    // The queue now contains one stale entry for h3 at priority 30.
    // `empty()` is based on `count_`, which was incremented 4 times.
    // 3 pops occurred, so count is 1.
    EXPECT_FALSE(queue->empty()); 

    // `contains` should be false for all since their queue_ref is NODE_NULL
    EXPECT_FALSE(queue->contains(h1));
    EXPECT_FALSE(queue->contains(h2));
    EXPECT_FALSE(queue->contains(h3));
    
    // The final pop should clear the stale entry and return NODE_NULL
    EXPECT_EQ(queue->pop(), NODE_NULL);
    EXPECT_TRUE(queue->empty());
}


TEST_F(TwoLevelBucketQueueTest, DuplicatePriorities) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h3 = pool.allocate(NODE_NULL, 0, 0);

    queue->push(h1, 10);
    queue->push(h2, 20);
    queue->push(h3, 10);

    // Because we push_back and pop_back, items with the same priority are LIFO
    EXPECT_EQ(queue->pop(), h3);
    EXPECT_EQ(queue->pop(), h1);
    EXPECT_EQ(queue->pop(), h2);
    EXPECT_TRUE(queue->empty());
}

TEST_F(TwoLevelBucketQueueTest, HighPriorities) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);

    queue->push(h1, 10);
    // This priority will go into a different high-level bucket
    // LOW_LEVEL_BITS is 10 (size 1024)
    queue->push(h2, 1024 + 10); 

    EXPECT_FALSE(queue->empty());
    EXPECT_EQ(queue->pop(), h1);
    EXPECT_EQ(queue->pop(), h2);
    EXPECT_TRUE(queue->empty());
}

TEST_F(TwoLevelBucketQueueTest, ComplexDecreaseKeyAndContains) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    queue->push(h1, 1000);

    EXPECT_TRUE(queue->contains(h1));
    
    queue->decrease_key(h1, 800);
    EXPECT_TRUE(queue->contains(h1));
    
    queue->decrease_key(h1, 500);
    EXPECT_TRUE(queue->contains(h1));

    uint32_t popped = queue->pop();
    EXPECT_EQ(popped, h1);

    // After the valid node is popped, `contains` should be false.
    EXPECT_FALSE(queue->contains(h1));
    
    // The queue still contains stale nodes, so it's not empty.
    EXPECT_FALSE(queue->empty());

    // The next pop will process the stale nodes and eventually return NODE_NULL
    popped = queue->pop();
    EXPECT_EQ(popped, NODE_NULL);

    // After all stale nodes are cleared, the queue is finally empty.
    EXPECT_TRUE(queue->empty());
}

TEST_F(TwoLevelBucketQueueTest, BoundaryPriorities) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h3 = pool.allocate(NODE_NULL, 0, 0);

    const uint32_t boundary = 1 << 10; // 1024

    queue->push(h1, boundary - 1); // low_idx 1023, high_idx 0
    queue->push(h2, boundary);     // low_idx 0,    high_idx 1
    queue->push(h3, 0);             // low_idx 0,    high_idx 0

    EXPECT_EQ(queue->pop(), h3);
    EXPECT_EQ(queue->pop(), h1);
    EXPECT_EQ(queue->pop(), h2);
    EXPECT_TRUE(queue->empty());
}
