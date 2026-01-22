#include <gtest/gtest.h>
#include "queues/binary_heap.h"
#include "environments/node.h"
#include <vector>
#include <algorithm>

class BinaryHeapTest : public ::testing::Test {
protected:
    void SetUp() override {
        pool.reserve(100);
        heap = new BinaryHeap(pool);
    }

    void TearDown() override {
        delete heap;
    }

    NodePool pool;
    BinaryHeap* heap;
};

TEST_F(BinaryHeapTest, IsEmptyInitially) {
    EXPECT_TRUE(heap->empty());
}

TEST_F(BinaryHeapTest, PushAndEmpty) {
    uint32_t handle = pool.allocate(NODE_NULL, 0, 0);
    heap->push(handle, 10);
    EXPECT_FALSE(heap->empty());
}

TEST_F(BinaryHeapTest, PushAndPopSingle) {
    uint32_t handle = pool.allocate(NODE_NULL, 0, 0);
    heap->push(handle, 10);
    uint32_t popped_handle = heap->pop();
    EXPECT_EQ(popped_handle, handle);
    EXPECT_TRUE(heap->empty());
}

TEST_F(BinaryHeapTest, PushAndPopMultiple) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h3 = pool.allocate(NODE_NULL, 0, 0);

    heap->push(h2, 20);
    heap->push(h3, 30);
    heap->push(h1, 10);

    EXPECT_EQ(heap->pop(), h1);
    EXPECT_EQ(heap->pop(), h2);
    EXPECT_EQ(heap->pop(), h3);
    EXPECT_TRUE(heap->empty());
}

TEST_F(BinaryHeapTest, Contains) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);

    heap->push(h1, 10);

    EXPECT_TRUE(heap->contains(h1));
    EXPECT_FALSE(heap->contains(h2));

    heap->pop();
    EXPECT_FALSE(heap->contains(h1));
}

TEST_F(BinaryHeapTest, DecreaseKey) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h3 = pool.allocate(NODE_NULL, 0, 0);

    heap->push(h1, 10);
    heap->push(h2, 20);
    heap->push(h3, 30);
    
    // h3 has priority 30, move it to 5
    heap->decrease_key(h3, 5);
    EXPECT_EQ(heap->pop(), h3);
    EXPECT_EQ(heap->pop(), h1);
    EXPECT_EQ(heap->pop(), h2);
}

TEST_F(BinaryHeapTest, DecreaseKeyToSame) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    heap->push(h1, 10);
    heap->decrease_key(h1, 10);
    EXPECT_EQ(heap->pop(), h1);
}

TEST_F(BinaryHeapTest, DuplicatePriorities) {
    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h3 = pool.allocate(NODE_NULL, 0, 0);

    heap->push(h1, 20);
    heap->push(h2, 10);
    heap->push(h3, 10);

    std::vector<uint32_t> popped;
    popped.push_back(heap->pop());
    popped.push_back(heap->pop());
    
    // Check that h2 and h3 are popped before h1
    EXPECT_TRUE((popped[0] == h2 && popped[1] == h3) || (popped[0] == h3 && popped[1] == h2));
    EXPECT_EQ(heap->pop(), h1);
}
