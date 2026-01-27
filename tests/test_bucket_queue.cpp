#include <gtest/gtest.h>
#include "queues/bucket_queue.h"
#include "environments/node.h"
#include <vector>
#include <algorithm>

class BucketQueueTest : public ::testing::Test {
protected:
    void SetUp() override {
        pool.reserve(100);
        // Use a small initial capacity to test resizing logic
        queue = new BucketQueue(pool, 100);
    }

    void TearDown() override {
        delete queue;
    }

    NodePool pool;
    BucketQueue* queue;
};

TEST_F(BucketQueueTest, IsEmptyInitially) {
    EXPECT_TRUE(queue->empty());
}

TEST_F(BucketQueueTest, PushAndEmpty) {
    uint32_t handle = pool.allocate(NODE_NULL, 0, 0);
    queue->push(handle, 10);
    EXPECT_FALSE(queue->empty());
}

TEST_F(BucketQueueTest, PushAndPopSingle) {
    uint32_t handle = pool.allocate(NODE_NULL, 0, 0);
    queue->push(handle, 10);
    uint32_t popped_handle = queue->pop();
    EXPECT_EQ(popped_handle, handle);
    EXPECT_TRUE(queue->empty());
}

TEST_F(BucketQueueTest, PushAndPopMultiple) {
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

TEST_F(BucketQueueTest, Contains) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);

    queue->push(h1, 10);

    EXPECT_TRUE(queue->contains(h1));
    EXPECT_FALSE(queue->contains(h2));

    queue->pop(); // h1 is popped
    EXPECT_FALSE(queue->contains(h1));
}

TEST_F(BucketQueueTest, DecreaseKey) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h3 = pool.allocate(NODE_NULL, 0, 0);

    queue->push(h1, 10);
    queue->push(h2, 20);
    queue->push(h3, 30);

    // Now, decrease key for h3
    queue->decrease_key(h3, 5);

    EXPECT_FALSE(queue->empty());
    EXPECT_EQ(queue->pop(), h3); // Should pop h3 first
    EXPECT_EQ(queue->pop(), h1);
    EXPECT_EQ(queue->pop(), h2);
    EXPECT_TRUE(queue->empty());  // ignores the stale node
    
    EXPECT_FALSE(queue->contains(h1));
    EXPECT_FALSE(queue->contains(h2));
    EXPECT_FALSE(queue->contains(h3));
}

TEST_F(BucketQueueTest, DuplicatePriorities) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h3 = pool.allocate(NODE_NULL, 0, 0);

    queue->push(h1, 10);
    queue->push(h2, 20);
    queue->push(h3, 10);

    std::vector<uint32_t> first_two;
    first_two.push_back(queue->pop());
    first_two.push_back(queue->pop());

    bool h1_found = (first_two[0] == h1 || first_two[1] == h1);
    bool h3_found = (first_two[0] == h3 || first_two[1] == h3);
    EXPECT_TRUE(h1_found);
    EXPECT_TRUE(h3_found);

    EXPECT_EQ(queue->pop(), h2);
    EXPECT_TRUE(queue->empty());
}

TEST_F(BucketQueueTest, AutoResize) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);

    queue->push(h1, 10);
    // This priority is > initial capacity (100)
    queue->push(h2, 200);

    EXPECT_FALSE(queue->empty());
    EXPECT_EQ(queue->pop(), h1);
    EXPECT_EQ(queue->pop(), h2);
    EXPECT_TRUE(queue->empty());
}

TEST_F(BucketQueueTest, ComplexDecreaseKeyAndContains) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    queue->push(h1, 10);

    EXPECT_TRUE(queue->contains(h1));
    
    queue->decrease_key(h1, 8);
    EXPECT_TRUE(queue->contains(h1));
    
    queue->decrease_key(h1, 5);
    EXPECT_TRUE(queue->contains(h1));

    uint32_t popped = queue->pop();
    EXPECT_EQ(popped, h1);

    // After the valid node is popped, `contains` should be false.
    EXPECT_FALSE(queue->contains(h1));
    
    EXPECT_TRUE(queue->empty());

    // The next pop will process the stale nodes and eventually return NODE_NULL
    // as there are no more valid nodes.
    popped = queue->pop();
    EXPECT_EQ(popped, NODE_NULL);

    EXPECT_TRUE(queue->empty());
}
