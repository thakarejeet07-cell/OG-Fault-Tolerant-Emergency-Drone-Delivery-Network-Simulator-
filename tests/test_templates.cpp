#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

#include "util/ThreadSafeQueue.hpp"
#include "util/MathUtils.hpp"

using namespace simulator::mesh::util;

// ============================================================================
// ThreadSafeQueue Tests
// ============================================================================

TEST(TemplateTest, ThreadSafeQueueBasicFIFO) {
    ThreadSafeQueue<int> q;
    EXPECT_TRUE(q.empty());
    EXPECT_EQ(q.size(), 0u);

    q.push(10);
    q.push(20);
    q.push(30);

    EXPECT_FALSE(q.empty());
    EXPECT_EQ(q.size(), 3u);

    int val = 0;
    EXPECT_TRUE(q.pop(val));
    EXPECT_EQ(val, 10);

    EXPECT_TRUE(q.try_pop(val));
    EXPECT_EQ(val, 20);

    EXPECT_TRUE(q.pop(val));
    EXPECT_EQ(val, 30);

    EXPECT_FALSE(q.try_pop(val));
    EXPECT_TRUE(q.empty());
}

TEST(TemplateTest, ThreadSafeQueueMultiThreadedPushPop) {
    ThreadSafeQueue<int> q;
    constexpr int NUM_PRODUCERS = 4;
    constexpr int NUM_CONSUMERS = 4;
    constexpr int ITEMS_PER_PRODUCER = 1000;
    const int TOTAL_ITEMS = NUM_PRODUCERS * ITEMS_PER_PRODUCER;

    std::atomic<int> itemsConsumed{0};
    std::atomic<long long> sumProduced{0};
    std::atomic<long long> sumConsumed{0};

    // Consumers
    std::vector<std::thread> consumers;
    for (int i = 0; i < NUM_CONSUMERS; ++i) {
        consumers.emplace_back([&]() {
            int val = 0;
            while (q.pop(val)) {
                itemsConsumed.fetch_add(1, std::memory_order_relaxed);
                sumConsumed.fetch_add(val, std::memory_order_relaxed);
            }
        });
    }

    // Producers
    std::vector<std::thread> producers;
    for (int i = 0; i < NUM_PRODUCERS; ++i) {
        producers.emplace_back([&, i]() {
            for (int j = 1; j <= ITEMS_PER_PRODUCER; ++j) {
                int item = i * 10000 + j;
                sumProduced.fetch_add(item, std::memory_order_relaxed);
                q.push(item);
            }
        });
    }

    for (auto& t : producers) {
        t.join();
    }

    // Wait until consumers have processed all items, then stop the queue
    while (itemsConsumed.load(std::memory_order_relaxed) < TOTAL_ITEMS) {
        std::this_thread::yield();
    }
    q.stop();

    for (auto& t : consumers) {
        t.join();
    }

    EXPECT_EQ(itemsConsumed.load(), TOTAL_ITEMS);
    EXPECT_EQ(sumConsumed.load(), sumProduced.load());
}

// ============================================================================
// clampTo<T> Tests
// ============================================================================

TEST(TemplateTest, ClampToInt) {
    EXPECT_EQ(clampTo<int>(5, 0, 10), 5);
    EXPECT_EQ(clampTo<int>(-3, 0, 10), 0);
    EXPECT_EQ(clampTo<int>(15, 0, 10), 10);
}

TEST(TemplateTest, ClampToFloatAndDouble) {
    EXPECT_FLOAT_EQ(clampTo<float>(5.5f, 0.0f, 10.0f), 5.5f);
    EXPECT_FLOAT_EQ(clampTo<float>(-1.2f, 0.0f, 10.0f), 0.0f);
    EXPECT_FLOAT_EQ(clampTo<float>(12.3f, 0.0f, 10.0f), 10.0f);

    EXPECT_DOUBLE_EQ(clampTo<double>(5.5, 0.0, 10.0), 5.5);
    EXPECT_DOUBLE_EQ(clampTo<double>(-1.2, 0.0, 10.0), 0.0);
    EXPECT_DOUBLE_EQ(clampTo<double>(12.3, 0.0, 10.0), 10.0);
}

TEST(TemplateTest, ClampToFloatNaNHandling) {
    // NaN should clamp safely to lo
    float nanVal = std::numeric_limits<float>::quiet_NaN();
    float clamped = clampTo<float>(nanVal, 0.0f, 100.0f);
    EXPECT_FLOAT_EQ(clamped, 0.0f);

    double nanDouble = std::numeric_limits<double>::quiet_NaN();
    double clampedD = clampTo<double>(nanDouble, 5.0, 50.0);
    EXPECT_DOUBLE_EQ(clampedD, 5.0);
}
