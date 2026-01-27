#include "gtest/gtest.h"
#include "queues/bucket_heap.h"
#include "environments/node.h"

#include <vector>
#include <functional>
#include <limits>
#include <unordered_map>

class BucketHeapTest : public ::testing::Test {
protected:
    void SetUp() override {
        pool = NodePool();
    }
    NodePool pool;
};

TEST_F(BucketHeapTest, IsEmptyInitially) {
    BucketHeap<double, std::greater<double>> heap(pool, [](uint32_t){ return 0.0; });
    ASSERT_TRUE(heap.empty());
}

TEST_F(BucketHeapTest, PushPopSingle) {
    std::unordered_map<uint32_t, double> priorities;
    auto priority_func = [&](uint32_t handle) { return priorities.at(handle); };
    BucketHeap<double, std::greater<double>> heap(pool, priority_func);
    
    ASSERT_TRUE(heap.empty());

    uint32_t handle = pool.allocate(NODE_NULL, 10, 5); // f=15
    priorities[handle] = 5.0;
    heap.push(handle);

    ASSERT_FALSE(heap.empty());
    uint32_t popped_handle = heap.pop();

    ASSERT_EQ(popped_handle, handle);
    ASSERT_TRUE(heap.empty());
}

TEST_F(BucketHeapTest, PushPopMultipleOrdered) {
    std::unordered_map<uint32_t, double> priorities;
    auto priority_func = [&](uint32_t handle) { return priorities.at(handle); };
    BucketHeap<double, std::greater<double>> heap(pool, priority_func);

    std::vector<uint32_t> handles;
    for (int i = 0; i < 5; ++i) {
        handles.push_back(pool.allocate(NODE_NULL, 10, i));
        priorities[handles[i]] = 10.0 - i;
        heap.push(handles[i]);
    }
    
    // Using a max-heap, so higher priorities (10, 9, 8...) pop first.
    // This corresponds to nodes with smaller h-costs (0, 1, 2...)
    ASSERT_EQ(heap.pop(), handles[0]);
    ASSERT_EQ(heap.pop(), handles[1]);
    ASSERT_EQ(heap.pop(), handles[2]);
    ASSERT_EQ(heap.pop(), handles[3]);
    ASSERT_EQ(heap.pop(), handles[4]);
    ASSERT_TRUE(heap.empty());
}

TEST_F(BucketHeapTest, PushPopMultipleUnordered) {
    std::unordered_map<uint32_t, double> priorities;
    auto priority_func = [&](uint32_t handle) { return priorities.at(handle); };
    BucketHeap<double, std::greater<double>> heap(pool, priority_func);

    uint32_t h1 = pool.allocate(NODE_NULL, 10, 5); // f=15
    uint32_t h2 = pool.allocate(NODE_NULL, 10, 2); // f=12
    uint32_t h3 = pool.allocate(NODE_NULL, 12, 3); // f=15

    priorities[h1] = 3.0;
    priorities[h2] = 5.0;
    priorities[h3] = 1.0;

    heap.push(h1);
    heap.push(h2);
    heap.push(h3);

    // Expected pop order:
    // 1. Bucket f=12 has priority 5.0 (from h2).
    // 2. Bucket f=15 has priority 1.0 (from h3, because h=3 is the new h_min).
    // 3. So, h2 is popped first.
    // 4. Then from bucket f=15, h3 is popped (as it's the h_min node).
    // 5. Then h1 is popped.
    ASSERT_EQ(heap.pop(), h2);
    ASSERT_EQ(heap.pop(), h3);
    ASSERT_EQ(heap.pop(), h1);
    ASSERT_TRUE(heap.empty());
}

TEST_F(BucketHeapTest, DecreaseKey) {
    std::unordered_map<uint32_t, double> priorities;
    auto priority_func = [&](uint32_t handle) { return priorities.at(handle); };
    BucketHeap<double, std::greater<double>> heap(pool, priority_func);

    uint32_t h1 = pool.allocate(NODE_NULL, 10, 5); // f=15
    uint32_t h2 = pool.allocate(NODE_NULL, 15, 3); // f=18
    
    priorities[h1] = 10.0;
    priorities[h2] = 5.0;

    heap.push(h1); // Bucket 15 gets prio 10.0
    heap.push(h2); // Bucket 18 gets prio 5.0

    // Pop h1 first
    ASSERT_EQ(heap.pop(), h1);

    // Now, let's update h2's priority and re-push it.
    // This is a valid scenario according to the dissertation's constraint,
    // as h2 is the only node in its bucket, making it the h_min node.
    priorities[h2] = 20.0;
    heap.decrease_key(h2); // Re-pushes h2 with new priority.

    // Add another node to make it interesting
    uint32_t h3 = pool.allocate(NODE_NULL, 10, 8); // f=18
    priorities[h3] = 15.0; // h=8, prio=15
    heap.push(h3); // h_min of bucket 18 is still 3 (from h2), so bucket prio remains 20.

    // Bucket 18 has priority 20 (from h2).
    // Pop should return h2.
    ASSERT_EQ(heap.pop(), h2);
    // Next pop should be h3
    ASSERT_EQ(heap.pop(), h3);
    ASSERT_TRUE(heap.empty());
}

TEST_F(BucketHeapTest, Rebuild) {
    auto priority_func = [&](uint32_t handle) {
        const auto& node = pool[handle];
        return (50.0 - node.g) / node.h;
    };
    BucketHeap<double, std::greater<double>> heap(pool, priority_func);

    uint32_t h1 = pool.allocate(NODE_NULL, 10, 8); // f=18
    uint32_t h2 = pool.allocate(NODE_NULL, 20, 4); // f=24
    uint32_t h3 = pool.allocate(NODE_NULL, 30, 2); // f=32
    
    heap.push(h1); // prio = (50-10)/8 = 5.0
    heap.push(h2); // prio = (50-20)/4 = 7.5
    heap.push(h3); // prio = (50-30)/2 = 10.0
    
    // --- Trigger Rebuild ---
    auto new_priority_func = [&](uint32_t handle) {
        const auto& node = pool[handle];
        return (35.0 - node.g) / node.h;
    };
    heap.set_priority_func(new_priority_func);
    heap.rebuild();

    // NEW prio(h1) = (35-10)/8 = 3.125
    // NEW prio(h2) = (35-20)/4 = 3.75
    // NEW prio(h3) = (35-30)/2 = 2.5
    
    ASSERT_EQ(heap.pop(), h2);
    ASSERT_EQ(heap.pop(), h1);
    ASSERT_EQ(heap.pop(), h3);
    ASSERT_TRUE(heap.empty());
}

TEST_F(BucketHeapTest, MultipleBuckets) {
    std::unordered_map<uint32_t, double> priorities;
    auto priority_func = [&](uint32_t handle) { return priorities.at(handle); };
    BucketHeap<double, std::greater<double>> heap(pool, priority_func);

    uint32_t h1 = pool.allocate(NODE_NULL, 5, 5);  // f=10
    uint32_t h2 = pool.allocate(NODE_NULL, 15, 5); // f=20

    priorities[h1] = 1.0;
    priorities[h2] = 5.0;
    heap.push(h1);
    heap.push(h2);

    // The heap should pop from the bucket with higher priority, which is h2's bucket.
    ASSERT_EQ(heap.pop(), h2);
    ASSERT_EQ(heap.pop(), h1);
    ASSERT_TRUE(heap.empty());
}

TEST_F(BucketHeapTest, HMinTracking) {
    auto calc = [&](uint32_t handle) {
        return 100.0 / pool[handle].h;
    };
    BucketHeap<double, std::greater<double>> heap(pool, calc);
    
    // All nodes go into the same f-bucket (f=20)
    uint32_t h1 = pool.allocate(NODE_NULL, 10, 10); // prio = 10
    uint32_t h2 = pool.allocate(NODE_NULL, 15, 5);  // prio = 20

    heap.push(h1);
    // The bucket's h_min is 10, dynamic priority is 10.
    
    heap.push(h2);
    // h2 has a smaller h, so it becomes the new h_min for the bucket.
    // The bucket's dynamic priority should be updated to 20.
    
    // The popped node should be h2, as it determines the bucket's priority.
    ASSERT_EQ(heap.pop(), h2);
    
    // After h2 is popped, h1 is the only one left.
    // The bucket should be re-inserted into the heap with h1's priority.
    ASSERT_EQ(heap.pop(), h1);
    ASSERT_TRUE(heap.empty());
}
