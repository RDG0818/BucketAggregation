#include <gtest/gtest.h>
#include "queues/binary_heap.h"
#include "environments/node.h"
#include <vector>
#include <algorithm>
#include <limits>

class BinaryHeapTest : public ::testing::Test {
protected:
    void SetUp() override {
        pool.reserve(100);
        // Defaulting to a min-heap of ints for existing tests
        heap = new BinaryHeap<int, std::less<int>>(pool);
    }

    void TearDown() override {
        delete heap;
    }

    NodePool pool;
    BinaryHeap<int, std::less<int>>* heap;
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

// --- Tests for New Templated Features ---

TEST(BinaryHeapMaxHeapTest, PushAndPopMultiple) {
    NodePool pool;
    pool.reserve(100);
    BinaryHeap<double, std::greater<double>> max_heap(pool);

    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h3 = pool.allocate(NODE_NULL, 0, 0);

    max_heap.push(h2, 20.5);
    max_heap.push(h3, 5.5);
    max_heap.push(h1, 30.1);

    EXPECT_EQ(max_heap.pop(), h1); // Highest priority
    EXPECT_EQ(max_heap.pop(), h2);
    EXPECT_EQ(max_heap.pop(), h3); // Lowest priority
    EXPECT_TRUE(max_heap.empty());
}

TEST(BinaryHeapMaxHeapTest, IncreaseKey) {
    NodePool pool;
    pool.reserve(100);
    BinaryHeap<double, std::greater<double>> max_heap(pool);

    uint32_t h1 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h2 = pool.allocate(NODE_NULL, 0, 0);
    uint32_t h3 = pool.allocate(NODE_NULL, 0, 0);

    max_heap.push(h1, 10.0);
    max_heap.push(h2, 20.0);
    max_heap.push(h3, 5.0);

    // Increase the priority of h3 from 5.0 to 25.0, making it the highest.
    // Note: The method is called decrease_key for API consistency.
    max_heap.decrease_key(h3, 25.0);

    // New order should be h3, h2, h1
    EXPECT_EQ(max_heap.pop(), h3);
    EXPECT_EQ(max_heap.pop(), h2);
    EXPECT_EQ(max_heap.pop(), h1);
}

TEST(BinaryHeapRebuildTest, RebuildsCorrectlyForMaxHeap) {
    NodePool pool;
    pool.reserve(100);
    BinaryHeap<double, std::greater<double>> heap(pool);

    // Setup nodes with g and h values for e-value calculation
    uint32_t h1 = pool.allocate(NODE_NULL, 10, 20); // g=10, h=20
    uint32_t h2 = pool.allocate(NODE_NULL, 30, 10); // g=30, h=10
    uint32_t h3 = pool.allocate(NODE_NULL, 50, 25); // g=50, h=25

    double g_upper = 100.0;
    
    // Initial priorities with G=100: h1=4.5, h2=7.0, h3=2.0
    heap.push(h1, (g_upper - 10) / 20.0);
    heap.push(h2, (g_upper - 30) / 10.0);
    heap.push(h3, (g_upper - 50) / 25.0);

    // Check initial order is h2, h1, h3
    ASSERT_EQ(heap.pop(), h2);
    heap.push(h2, (g_upper - 30) / 10.0); // push back to test again

    // --- Trigger Rebuild ---
    double new_g_upper = 50.0;
    auto calculator = [&](uint32_t handle) -> double {
        const auto& node = pool[handle];
        if (node.h == 0) return std::numeric_limits<double>::max();
        return (new_g_upper - node.g) / static_cast<double>(node.h);
    };
    heap.rebuild(calculator);

    // New priorities with G=50: h1=2.0, h2=2.0, h3=0.0
    // New order should be h1 and h2 (in any order), then h3.
    std::vector<uint32_t> popped;
    popped.push_back(heap.pop());
    popped.push_back(heap.pop());

    // Check that h1 and h2 are the first two popped
    EXPECT_TRUE((popped[0] == h1 && popped[1] == h2) || (popped[0] == h2 && popped[1] == h1));
    
    // The last one must be h3
    EXPECT_EQ(heap.pop(), h3);
    EXPECT_TRUE(heap.empty());
}